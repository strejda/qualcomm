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

#include "qcom_ssbi_if.h"


#define SSBI_REQUEST_TIMEOUT	(5 * hz)

#define SSBI_LOCK(_sc)			mtx_lock(&(_sc)->mtx)
#define	SSBI_UNLOCK(_sc)			mtx_unlock(&(_sc)->mtx)
#define SSBI_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "ssbi", MTX_DEF)
#define SSBI_LOCK_DESTROY(_sc)		mtx_destroy(&_sc->mtx);
#define SSBI_ASSERT_LOCKED(_sc)		mtx_assert(&_sc->mtx, MA_OWNED);
#define SSBI_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);

#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, r, v)
#define	RD4(sc, r)	bus_read_4((sc)->mem_res, r)

#define SSBI2_CMD			0x0000
#define  CMD_RDWRN				(1 << 24)
#define  CMD_REG_ADDR_MASK			0x7fff
#define  CMD_REG_ADDR_SHIFT			8
#define  CMD_REG_DATA_MASK			0xff
#define  CMD_REG_DATA_SHIFT			0

#define SSBI2_RD_STATUS			0x0004
#define  RD_STATUS_TRANS_DONE			(1 << 27)
#define  RD_STATUS_TRANS_DENIED			(1 << 26)
#define  RD_STATUS_TRANS_PROG			(1 << 25)
#define  RD_STATUS_REG_DATA_MASK		0xff
#define  RD_STATUS_REG_DATA_SHIFT		0

#define SSBI2_TIMEOUT_US		100



enum ssbi_ctrl_type {
	SSBI_CTRL_SSBI,
	SSBI_CTRL_SSBI2,
	SSBI_CTRL_PMIC_ARBITER,
};

struct ssbi_softc {
	struct simplebus_softc	simplebus_sc;
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
	enum ssbi_ctrl_type	type;
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,ssbi",	1},
	{NULL,		0},
};

/*
 * See ssbi_wait_mask for an explanation of the time and the
 * busywait.
 */
static int
ssbi_pmic_exec(struct ssbi_softc *sc, uint32_t cmd, uint8_t *buf)
{
	int i;
	uint32_t sts;

	WR4(sc, SSBI2_CMD, cmd);

	for (i = 0; i < SSBI2_TIMEOUT_US; i++) {
		sts = RD4(sc, SSBI2_RD_STATUS);

		if (sts & RD_STATUS_TRANS_DENIED)
			return (EPERM);

		if (sts & RD_STATUS_TRANS_DONE) {
			if (buf != NULL)
				*buf = (sts  >> RD_STATUS_REG_DATA_SHIFT) &
				    RD_STATUS_REG_DATA_MASK;
			return (0);
		}
		DELAY(1);
	}

	return (ETIMEDOUT);
}

static int
ssbi_pmic_read(struct ssbi_softc *sc, uint32_t addr, uint8_t *buf, int len)
{
	uint32_t cmd;
	int rv;

	cmd = CMD_RDWRN | (addr & CMD_REG_ADDR_MASK) << CMD_REG_ADDR_SHIFT;
	while (len > 0) {
		rv = ssbi_pmic_exec(sc, cmd, buf);
		if (rv)
			return (rv);
		buf++;
		len--;
	}

	return (0);
}

static int
ssbi_pmic_write(struct ssbi_softc *sc, uint32_t addr, uint8_t *buf, int len)
{
	uint32_t cmd;
	int rv;

	cmd = (addr & CMD_REG_ADDR_MASK) << CMD_REG_ADDR_SHIFT;
	while (len > 0) {
		rv = ssbi_pmic_exec(sc,
		    cmd | (*buf & CMD_REG_DATA_MASK) << CMD_REG_DATA_SHIFT,
		    NULL);
		if (rv)
			return (rv);
		buf++;
		len--;
	}

	return (0);
}

static int
ssbi_read(device_t dev, device_t consumer, uint32_t addr, uint8_t *buf, int len)
{
	struct ssbi_softc *sc;
	int rv;

	sc= device_get_softc(dev);
	SSBI_LOCK(sc);
	/* XXX read varians */
	rv = ssbi_pmic_read(sc, addr, buf, len);
	SSBI_UNLOCK(sc);

	return (rv);
}

static int
ssbi_write(device_t dev, device_t consumer, uint32_t addr, uint8_t *buf,
    int len)
{
	struct ssbi_softc *sc;
	int rv;

	sc= device_get_softc(dev);
	SSBI_LOCK(sc);
	/* XXX write varians */
	rv = ssbi_pmic_write(sc, addr, buf, len);
	SSBI_UNLOCK(sc);

	return (rv);
}

static enum ssbi_ctrl_type
ssbi_get_ctrl_mode(device_t dev, phandle_t node)
{
	char *tmpstr;
	int rv;
	enum ssbi_ctrl_type ret;

	ret = SSBI_CTRL_SSBI;
	rv = OF_getprop_alloc(node, "qcom,controller-type", 1,
	    (void **)&tmpstr);
	if (rv <= 0)
		return (ret);

	if (strcmp(tmpstr, "ssbi") == 0)
		ret = SSBI_CTRL_SSBI;
	else if (strcmp(tmpstr, "ssbi2") == 0)
		ret = SSBI_CTRL_SSBI2;
	else if (strcmp(tmpstr, "pmic-arbiter") == 0)
		ret = SSBI_CTRL_PMIC_ARBITER;
	else
		device_printf(dev, "Unknown controller mode: %s\n", tmpstr);
	free(tmpstr, M_OFWPROP);
	return (ret);
}

static int
ssbi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "APQ8064 SSBI driver");
	return (BUS_PROBE_DEFAULT);
}

static int
ssbi_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
ssbi_attach(device_t dev)
{
	struct ssbi_softc * sc;
	int rid;
	phandle_t node;



	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(sc->dev);

	SSBI_LOCK_INIT(sc);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource\n");
		goto fail;
	}

	sc->type = ssbi_get_ctrl_mode(dev, node);
	simplebus_init(dev, 0);
	for (node = OF_child(node); node > 0; node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);
	return (bus_generic_attach(dev));
fail:
	return (ENXIO);
}

static device_method_t qcom_ssbi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ssbi_probe),
	DEVMETHOD(device_attach,	ssbi_attach),
	DEVMETHOD(device_detach,	ssbi_detach),

	/* SSBI interface */
	DEVMETHOD(qcom_ssbi_write,	ssbi_write),
	DEVMETHOD(qcom_ssbi_read,	ssbi_read),

	DEVMETHOD_END
};

static devclass_t qcom_ssbi_devclass;
DEFINE_CLASS_1(qcom_ssbi, qcom_ssbi_driver, qcom_ssbi_methods,
    sizeof(struct ssbi_softc), simplebus_driver);
EARLY_DRIVER_MODULE(qcom_ssbi, simplebus,  qcom_ssbi_driver, qcom_ssbi_devclass,
    0, 0, 73);
