/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_regulator.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <gnu/dts/include/dt-bindings/mfd/qcom-rpm.h>

#include "syscon_if.h"
#include "qcom_rpm_if.h"


#define RPM_REQUEST_TIMEOUT	(5 * hz)

#define STATUS_BASE		0x0000
#define CTRL_BASE		0x0400
#define REQ_BASE		0x0600
#define ACK_BASE		0x0800


/* Status page registers */
#define RPM_STATUS_VERSION_MAJOR	 	 0
#define RPM_STATUS_VERSION_MINOR	 	 1
#define RPM_STATUS_VERSION_BUILD		 2
#define RPM_STATUS_SUPPORTED_RESOURCES_0	 3
#define RPM_STATUS_SUPPORTED_RESOURCES_1	 4
#define RPM_STATUS_SUPPORTED_RESOURCES_2	 5
#define RPM_STATUS_RESERVED_SUPPORTED_RESOURCES_0 6
#define RPM_STATUS_SEQUENCE			 7

/* Control page registers */
#define RPM_CTRL_VERSION_MAJO		 0
#define RPM_CTRL_VERSION_MINOR		 1
#define RPM_CTRL_VERSION_BUILD		 2
#define RPM_CTRL_REQ_CONTEXT		 3
#define RPM_CTRL_REQ_SELECTOR		11
#define RPM_CTRL_ACK_CONTEXT		15
#define RPM_CTRL_ACK_SELECTOR		23
#define RPM_SELECT_SIZE			5

#define RPM_ACK_DONE		(1U <<  0)
#define RPM_ACK_NOTIFICATION	(1U << 30)
#define RPM_ACK_REJECTED	(1U << 31)


#define RPM_LOCK(_sc)			mtx_lock(&(_sc)->mtx)
#define	RPM_UNLOCK(_sc)			mtx_unlock(&(_sc)->mtx)
#define RPM_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "rpm", MTX_DEF)
#define RPM_LOCK_DESTROY(_sc)		mtx_destroy(&_sc->mtx);
#define RPM_ASSERT_LOCKED(_sc)		mtx_assert(&_sc->mtx, MA_OWNED);
#define RPM_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);


#define	WR4(_sc, _base, _r, _v)						\
	    bus_write_4((_sc)->mem_res, (_base) + 4 * (_r), (_v))
#define	RD4(_sc, _base,_r)						\
	    bus_read_4((_sc)->mem_res, (_base) + 4 * (_r))

struct rpm_res {
	uint32_t	target_id;
	uint32_t	status_id;
	uint32_t	select_id;
	uint32_t	size;
};

struct rpm_dev_desc {
	uint32_t 	version;
	struct rpm_res	*table;
	int		table_size;
};

struct rpm_softc {
	struct simplebus_softc	simplebus_sc;
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
	struct resource		*ack_ires;
	struct resource		*err_ires;
	struct resource		*wakeup_ires;

	void			*ack_ih;
	void			*err_ih;
	void			*wakeup_ih;

	int			busy;
	struct rpm_res		*res_table;
	int			res_table_size;
	uint32_t		last_ack;
	device_t		syscon_dev;
	uint32_t		pic_reg;
	uint32_t		pic_mask;
};

/*					req, status, select, req size */
static struct rpm_res apq8064_table[] = {
	[QCOM_RPM_CXO_CLK] =		{ 25,   9,  5,  1},
	[QCOM_RPM_PXO_CLK] =		{ 26,  10,  6,  1},
	[QCOM_RPM_APPS_FABRIC_CLK] =	{ 27,  11,  8,  1},
	[QCOM_RPM_SYS_FABRIC_CLK] =	{ 28,  12,  9,  1},
	[QCOM_RPM_MM_FABRIC_CLK] =	{ 29,  13, 10,  1},
	[QCOM_RPM_DAYTONA_FABRIC_CLK] =	{ 30,  14, 11,  1},
	[QCOM_RPM_SFPB_CLK] =		{ 31,  15, 12,  1},
	[QCOM_RPM_CFPB_CLK] =		{ 32,  16, 13,  1},
	[QCOM_RPM_MMFPB_CLK] =		{ 33,  17, 14,  1},
	[QCOM_RPM_EBI1_CLK] =		{ 34,  18, 16,  1},
	[QCOM_RPM_APPS_FABRIC_HALT] =	{ 35,  19, 18,  1},
	[QCOM_RPM_APPS_FABRIC_MODE] =	{ 37,  20, 19,  1},
	[QCOM_RPM_APPS_FABRIC_IOCTL] =	{ 40,  21, 20,  1},
	[QCOM_RPM_APPS_FABRIC_ARB] =	{ 41,  22, 21, 12},
	[QCOM_RPM_SYS_FABRIC_HALT] =	{ 53,  23, 22,  1},
	[QCOM_RPM_SYS_FABRIC_MODE] =	{ 55,  24, 23,  1},
	[QCOM_RPM_SYS_FABRIC_IOCTL] =	{ 58,  25, 24,  1},
	[QCOM_RPM_SYS_FABRIC_ARB] =	{ 59,  26, 25, 30},
	[QCOM_RPM_MM_FABRIC_HALT] =	{ 89,  27, 26,  1},
	[QCOM_RPM_MM_FABRIC_MODE] =	{ 91,  28, 27,  1},
	[QCOM_RPM_MM_FABRIC_IOCTL] =	{ 94,  29, 28,  1},
	[QCOM_RPM_MM_FABRIC_ARB] =	{ 95,  30, 29, 21},
	[QCOM_RPM_PM8921_SMPS1] =	{116,  31, 30,  2},
	[QCOM_RPM_PM8921_SMPS2] =	{118,  33, 31,  2},
	[QCOM_RPM_PM8921_SMPS3] =	{120,  35, 32,  2},
	[QCOM_RPM_PM8921_SMPS4] =	{122,  37, 33,  2},
	[QCOM_RPM_PM8921_SMPS5] =	{124,  39, 34,  2},
	[QCOM_RPM_PM8921_SMPS6] =	{126,  41, 35,  2},
	[QCOM_RPM_PM8921_SMPS7] =	{128,  43, 36,  2},
	[QCOM_RPM_PM8921_SMPS8] =	{130,  45, 37,  2},
	[QCOM_RPM_PM8921_LDO1] =	{132,  47, 38,  2},
	[QCOM_RPM_PM8921_LDO2] =	{134,  49, 39,  2},
	[QCOM_RPM_PM8921_LDO3] =	{136,  51, 40,  2},
	[QCOM_RPM_PM8921_LDO4] =	{138,  53, 41,  2},
	[QCOM_RPM_PM8921_LDO5] =	{140,  55, 42,  2},
	[QCOM_RPM_PM8921_LDO6] =	{142,  57, 43,  2},
	[QCOM_RPM_PM8921_LDO7] =	{144,  59, 44,  2},
	[QCOM_RPM_PM8921_LDO8] =	{146,  61, 45,  2},
	[QCOM_RPM_PM8921_LDO9] =	{148,  63, 46,  2},
	[QCOM_RPM_PM8921_LDO10] =	{150,  65, 47,  2},
	[QCOM_RPM_PM8921_LDO11] =	{152,  67, 48,  2},
	[QCOM_RPM_PM8921_LDO12] =	{154,  69, 49,  2},
	[QCOM_RPM_PM8921_LDO13] =	{156,  71, 50,  2},
	[QCOM_RPM_PM8921_LDO14] =	{158,  73, 51,  2},
	[QCOM_RPM_PM8921_LDO15] =	{160,  75, 52,  2},
	[QCOM_RPM_PM8921_LDO16] =	{162,  77, 53,  2},
	[QCOM_RPM_PM8921_LDO17] =	{164,  79, 54,  2},
	[QCOM_RPM_PM8921_LDO18] =	{166,  81, 55,  2},
	[QCOM_RPM_PM8921_LDO19] =	{168,  83, 56,  2},
	[QCOM_RPM_PM8921_LDO20] =	{170,  85, 57,  2},
	[QCOM_RPM_PM8921_LDO21] =	{172,  87, 58,  2},
	[QCOM_RPM_PM8921_LDO22] =	{174,  89, 59,  2},
	[QCOM_RPM_PM8921_LDO23] =	{176,  91, 60,  2},
	[QCOM_RPM_PM8921_LDO24] =	{178,  93, 61,  2},
	[QCOM_RPM_PM8921_LDO25] =	{180,  95, 62,  2},
	[QCOM_RPM_PM8921_LDO26] =	{182,  97, 63,  2},
	[QCOM_RPM_PM8921_LDO27] =	{184,  99, 64,  2},
	[QCOM_RPM_PM8921_LDO28] =	{186, 101, 65,  2},
	[QCOM_RPM_PM8921_LDO29] =	{188, 103, 66,  2},
	[QCOM_RPM_PM8921_CLK1] =	{190, 105, 67,  2},
	[QCOM_RPM_PM8921_CLK2] =	{192, 107, 68,  2},
	[QCOM_RPM_PM8921_LVS1] =	{194, 109, 69,  1},
	[QCOM_RPM_PM8921_LVS2] =	{195, 110, 70,  1},
	[QCOM_RPM_PM8921_LVS3] =	{196, 111, 71,  1},
	[QCOM_RPM_PM8921_LVS4] =	{197, 112, 72,  1},
	[QCOM_RPM_PM8921_LVS5] =	{198, 113, 73,  1},
	[QCOM_RPM_PM8921_LVS6] =	{199, 114, 74,  1},
	[QCOM_RPM_PM8921_LVS7] =	{200, 115, 75,  1},
	[QCOM_RPM_PM8821_SMPS1] =	{201, 116, 76,  2},
	[QCOM_RPM_PM8821_SMPS2] =	{203, 118, 77,  2},
	[QCOM_RPM_PM8821_LDO1] =	{205, 120, 78,  2},
	[QCOM_RPM_PM8921_NCP] =		{207, 122, 80,  2},
	[QCOM_RPM_CXO_BUFFERS] =	{209, 124, 81,  1},
	[QCOM_RPM_USB_OTG_SWITCH] =	{210, 125, 82,  1},
	[QCOM_RPM_HDMI_SWITCH] =	{211, 126, 83,  1},
	[QCOM_RPM_DDR_DMM] =		{212, 127, 84,  2},
	[QCOM_RPM_VDDMIN_GPIO] =	{215, 131, 89,  1},
};

struct rpm_dev_desc apq8064_desc = {
	.version = 3,
	.table = apq8064_table,
	.table_size = nitems(apq8064_table)
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,rpm-apq8064",		(intptr_t)&apq8064_desc},
	{NULL,				0},
};

static void
dump_page(struct rpm_softc *sc, uint32_t base, uint32_t size)
{
	int i;

	printf("\nDump page at 0x%08X", base);
	size /= 4;

	for (i = 0; i < size; i++) {
		if ((i % 8) == 0) {
			printf("\n0x%04X:", i);

		}
		printf("0x%08X ", RD4(sc, base, i));
	}
	printf("\n");
}


static void
rpm_clear_response(struct rpm_softc *sc)
{
	int i;

	for (i = 0; i < RPM_SELECT_SIZE; i++)
		WR4(sc, CTRL_BASE, RPM_CTRL_ACK_SELECTOR + i, 0);
	WR4(sc, CTRL_BASE, RPM_CTRL_ACK_CONTEXT, 0);
}

static int
ack_intr(void *arg)
{
	struct rpm_softc *sc;
	uint32_t reg;

	sc = arg;
	RPM_LOCK(sc);
	reg = RD4(sc, CTRL_BASE, RPM_CTRL_ACK_CONTEXT);
	rpm_clear_response(sc);

printf("%s: Got irq: 0x%08X\n", __func__, reg);
	if (reg & RPM_ACK_NOTIFICATION) {
		device_printf(sc->dev, "RPM got unknown ack.\n");
	} else {
		sc->last_ack = reg;
		wakeup(&sc->last_ack);
	}
	RPM_UNLOCK(sc);

	return (FILTER_HANDLED);
}

static int
err_intr(void *arg)
{
	struct rpm_softc *sc;

	sc = arg;

	SYSCON_WRITE_4(sc->syscon_dev, sc->dev, sc->pic_reg, sc->pic_mask);
	device_printf(sc->dev, "RPM got fatal interrupt.\n");
	return (FILTER_HANDLED);
}

static int
wakeup_intr(void *arg)
{
	/* nnthing to do yet*/
	return (FILTER_HANDLED);
}

static int
rpm_write_cmd(device_t dev, device_t consumer, int ctx, int res_id,
    uint32_t *buf, int count)
{
	struct rpm_softc * sc;

	const struct rpm_res *node;
	uint32_t sel_mask[RPM_SELECT_SIZE] = { 0 };
	int rv , i;

//printf("%s: Enter: ctx:%d, res_id:%d\n", __func__, ctx, res_id);
	sc = device_get_softc(dev);

	if ((res_id < 0 || res_id >= sc->res_table_size))
		return (EINVAL);

	node = sc->res_table + res_id;
	if (node->size != count)
		return (EINVAL);

	RPM_LOCK(sc);
	while (sc->busy)
		msleep(sc, &sc->mtx, PZERO, "rpm_req", hz / 5);
//printf(" target_id: %d, status_id: %d, select_id: %d\n", node->target_id, node->status_id, node->select_id);

	sc->busy = 1;

	for (i = 0; i < node->size; i++)
		WR4(sc, REQ_BASE, node->target_id + i, buf[i]);

	sel_mask[node->select_id / 32] =
	    (1 << (node->select_id % 32));

	for (i = 0; i < nitems(sel_mask); i++)
		WR4(sc, CTRL_BASE, RPM_CTRL_REQ_SELECTOR + i, sel_mask[i]);

	WR4(sc, CTRL_BASE, RPM_CTRL_REQ_CONTEXT, 1 << ctx);
	RD4(sc, CTRL_BASE, RPM_CTRL_REQ_CONTEXT);

	SYSCON_WRITE_4(sc->syscon_dev, sc->dev, sc->pic_reg, sc->pic_mask);

//printf("%s: WFI\n", __func__);
	if (cold) {
		rv = 0;
		for (;;) {
			sc->last_ack = RD4(sc, CTRL_BASE, RPM_CTRL_ACK_CONTEXT);
			if (sc->last_ack != 0)  {
//dump_page(sc, STATUS_BASE, 0x400);
//dump_page(sc, CTRL_BASE, 0x200);
//dump_page(sc, REQ_BASE, 0x200);
//dump_page(sc, ACK_BASE, 0x200);
				rpm_clear_response(sc);
				break;
			}
		}
	} else {
		rv = msleep(&sc->last_ack, &sc->mtx, PZERO, "rpm",
		    RPM_REQUEST_TIMEOUT);
	}
	if (rv != 0)
		rv = ETIMEDOUT;
	else if (sc->last_ack & RPM_ACK_REJECTED)
		rv = EIO;
//printf("%s: END: 0x%08X\n", __func__, sc->last_ack);

	sc->busy = 0;
	RPM_UNLOCK(sc);
	wakeup(sc);

	return (rv);
}

static int
rpm_read_status(device_t dev, device_t consumer, int ctx, int res_id,
    uint32_t *buf, int count)
{
	struct rpm_softc * sc;

	const struct rpm_res *node;
	uint32_t seq_begin;
	uint32_t seq_end;
	int i;

	sc = device_get_softc(dev);

	if ((res_id < 0 || res_id >= sc->res_table_size))
		return (EINVAL);
	node = sc->res_table + res_id;
#if 0
	if (node->size != count)
		return (EINVAL);
#endif
	seq_begin = RD4(sc, STATUS_BASE, RPM_STATUS_SEQUENCE);

	for (i = 0; i < count; i++)
		buf[i] = RD4(sc, STATUS_BASE, node->status_id + i);

	seq_end = RD4(sc, STATUS_BASE, RPM_STATUS_SEQUENCE);
	return ((seq_begin != seq_end || (seq_begin & 0x01)) ? EINPROGRESS : 0);
}

static int
rpm_parse_syscon(struct rpm_softc *sc)
{
	phandle_t cnode;
	pcell_t *cells;
	int ncells;

	/* Get data from FDT */
	cnode = ofw_bus_get_node(sc->dev);
	cells = NULL;
	ncells = OF_getencprop_alloc(cnode, "qcom,ipc",  sizeof(*cells),
	    (void **)&cells);
	if (ncells <= 2) {
		if (ncells > 0)
			free(cells, M_OFWPROP);
		return (ERANGE);
	}
	sc->syscon_dev = OF_device_from_xref(cells[0]);
	if (sc->syscon_dev == NULL) {
		free(cells, M_OFWPROP);
		return (ENXIO);
	}
	sc->pic_reg = cells[1];
	sc->pic_mask = (1U << cells[2]);
	free(cells, M_OFWPROP);
	return 0;
}

static int
rpm_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "APQ8064 RPM subsystem");
	return (BUS_PROBE_DEFAULT);
}

static int
rpm_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
rpm_attach(device_t dev)
{
	struct rpm_softc * sc;
	int rid;
	phandle_t node;
	uint32_t fw_major, fw_minor, fw_build;
	struct rpm_dev_desc *desc;


	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(sc->dev);
	desc = (struct rpm_dev_desc *)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;

	RPM_LOCK_INIT(sc);

	if (rpm_parse_syscon(sc) != 0) {
		device_printf(dev, "Cannot parse syscon node\n");
		goto fail;
	}

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource\n");
		goto fail;
	}

	rid = 0;
	sc->ack_ires = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->ack_ires == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource\n");
		goto fail;
	}
	rid = 1;
	sc->err_ires = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->err_ires == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource\n");
		goto fail;
	}
	rid = 2;
	sc->wakeup_ires = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->wakeup_ires == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource\n");
		goto fail;
	}

	rpm_clear_response(sc);
	fw_major = RD4(sc, STATUS_BASE, RPM_STATUS_VERSION_MAJOR);
	fw_minor = RD4(sc, STATUS_BASE, RPM_STATUS_VERSION_MINOR);
	fw_build = RD4(sc, STATUS_BASE, RPM_STATUS_VERSION_BUILD);
	if (bootverbose)
		device_printf(sc->dev, "firmware: %u.%u (build %u)\n",
		    fw_major, fw_minor, fw_build);
	if (fw_major != desc->version) {
		device_printf(dev,
		    "Unexpected major version of firmware %d != %d\n",
		    fw_major, desc->version);
		goto fail;
	}
	sc->res_table = desc->table;
	sc->res_table_size = desc->table_size;

	if ((bus_setup_intr(dev, sc->ack_ires, INTR_TYPE_MISC | INTR_MPSAFE,
	    ack_intr, NULL, sc, &sc->ack_ih))) {
		device_printf(dev,
		    "Cannot to register interrupt handler\n");
		goto fail;
	}
	if ((bus_setup_intr(dev, sc->err_ires, INTR_TYPE_MISC | INTR_MPSAFE,
	    err_intr, NULL, sc, &sc->err_ih))) {
		device_printf(dev,
		    "WCannot to register interrupt handler\n");
		goto fail;
	}
	if ((bus_setup_intr(dev, sc->wakeup_ires, INTR_TYPE_MISC| INTR_MPSAFE,
	    wakeup_intr, NULL, sc, &sc->wakeup_ih))) {
		device_printf(dev,
		    "Cannot to register interrupt handler\n");
		goto fail;
	}

	simplebus_init(dev, 0);
	for (node = OF_child(node); node > 0;
	    node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);
	return (bus_generic_attach(dev));
fail:
	return (ENXIO);
}

static device_method_t qcom_rpm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rpm_probe),
	DEVMETHOD(device_attach,	rpm_attach),
	DEVMETHOD(device_detach,	rpm_detach),

	/* RPM interface */
	DEVMETHOD(qcom_rpm_write_cmd,	rpm_write_cmd),
	DEVMETHOD(qcom_rpm_read_status,	rpm_read_status),

	DEVMETHOD_END
};

static devclass_t qcom_rpm_devclass;
DEFINE_CLASS_1(qcom_rpm, qcom_rpm_driver, qcom_rpm_methods,
    sizeof(struct rpm_softc), simplebus_driver);
EARLY_DRIVER_MODULE(qcom_rpm, simplebus,  qcom_rpm_driver, qcom_rpm_devclass,
    0, 0, 73);
