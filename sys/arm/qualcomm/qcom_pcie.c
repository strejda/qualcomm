/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Portions of this software were developed by Oleksandr Rybalko
 * under sponsorship from the FreeBSD Foundation.
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

/*
 * Qualcomm intergated PCI/PCI-Express controller driver.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/gpio.h>
#include <sys/interrupt.h>

#include <machine/fdt.h>
#include <machine/intr.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_phy.h>
#include <dev/fdt/fdt_regulator.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/fdt/fdt_common.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include <machine/devmap.h>
#include <machine/resource.h>
#include <machine/bus.h>

#include <gnu/dts/include/dt-bindings/gpio/gpio.h>

#include "gpiobus_if.h"
#include "ofw_bus_if.h"
#include "pcib_if.h"


//#define QCOM_PCI_MSI

#define LOCK(_sc)			mtx_lock(&(_sc)->mtx)
#define	UNLOCK(_sc)			mtx_unlock(&(_sc)->mtx)
#define LOCK_INIT(_sc)			mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "qcom_pcie", MTX_DEF)
#define LOCK_DESTROY(_sc)		mtx_destroy(&_sc->mtx);
#define ASSERT_LOCKED(_sc)		mtx_assert(&_sc->mtx, MA_OWNED);
#define ASSERT_UNLOCKED(_sc)		mtx_assert(&_sc->mtx, MA_NOTOWNED);

#define PCIE20_MSI_PHY_ADDR		0xa0000000


/* Standard PCIe 2.0 config space */
#define PCIE20_CAP				0x70
#define PCIE20_CAP_LINKCTRLSTATUS		(PCIE20_CAP + 0x10)

/* Synopsis DesignWare PCIe 2.0 core extensions */
#define PCIE20_AXI_MSTR_RESP_COMP_CTRL0 	0x818
#define PCIE20_AXI_MSTR_RESP_COMP_CTRL1 	0x81c
#define PCIE20_MSI_CTRL_ADDR			0x820
#define PCIE20_MSI_CTRL_UPPER_ADDR		0x824
#define PCIE20_MSI_CTRL_INTR_EN			0x828
#define PCIE20_MSI_CTRL_INTR_MASK		0x82C
#define PCIE20_MSI_CTRL_INTR_STATUS		0x830
#define  PCIE20_MSI_CTRL_REGS 				8

#define PCIE20_PLR_IATU_VIEWPORT		0x900
#define  IATU_VIEWPORT_REGION_INBOUND			(0x1 << 31)
#define  IATU_VIEWPORT_REGION_OUTBOUND			(0x0 << 31)
#define  IATU_VIEWPORT_REGION_INDEX1			(0x1 <<  0)
#define  IATU_VIEWPORT_REGION_INDEX0			(0x0 <<  0)

#define PCIE20_PLR_IATU_CTRL1			0x904
#define  IATU_CTRL1_TYPE_MEM				(0x0 << 0)
#define  IATU_CTRL1_TYPE_IO				(0x2 << 0)
#define  IATU_CTRL1_TYPE_CFG0				(0x4 << 0)
#define  IATU_CTRL1_TYPE_CFG1				(0x5 << 0)
#define PCIE20_PLR_IATU_CTRL2			0x908
#define  IATU_CTRL2_ENABLE				(0x1 << 31)
#define  IATU_CTRL2_BAR_MODE_ENABLE			(0x1 << 30)
#define PCIE20_PLR_IATU_LBAR			0x90C
#define PCIE20_PLR_IATU_UBAR			0x910
#define PCIE20_PLR_IATU_LAR			0x914
#define PCIE20_PLR_IATU_LTAR			0x918
#define PCIE20_PLR_IATU_UTAR			0x91c
#define PCIE20_IATU_BUS(b)				(((b) & 0x00ff) << 24)
#define PCIE20_IATU_DEV(d)				(((d) & 0x001f) << 19)
#define PCIE20_IATU_FUN(f)				(((f) & 0x0007) << 16)
#define PCIE20_IATU_REG(r)				(((r) & 0xffff) <<  0)

/* PARF - PCIe Auxiliary Register File (aka PCIe phy) */
#define PCIE20_PARF_PCS_DEEMPH			0x034
#define  PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN1(x)	(((x) & 0x3F) << 16)
#define  PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN2_3_5DB(x)	(((x) & 0x3F) <<  8)
#define  PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN2_6DB(x)	(((x) & 0x3F) <<  0)

#define PCIE20_PARF_PCS_SWING			0x038
#define  PCIE20_PARF_PCS_SWING_TX_SWING_FULL(x)		(((x) & 0x7F) <<  8)
#define  PCIE20_PARF_PCS_SWING_TX_SWING_LOW(x)		(((x) & 0x7F) <<  0)

#define PCIE20_PARF_PHY_CTRL			0x040
#define  PCIE20_PARF_PHY_CTRL_PHY_TX0_TERM_OFFST(x)	(((x) & 0x1F) << 16)
#define  PCIE20_PARF_PHY_CTRL_PHY_LOS_LEVEL(x)		(((x) & 0x1F) <<  8)
#define  PCIE20_PARF_PHY_CTRL_PHY_RTUNE_REQ		(1 << 4)
#define  PCIE20_PARF_PHY_CTRL_PHY_TEST_BURNIN		(1 << 2)
#define  PCIE20_PARF_PHY_CTRL_PHY_TEST_BYPASS		(1 << 1)
#define  PCIE20_PARF_PHY_CTRL_PHY_TEST_PWR_DOWN		(1 << 0)

#define PCIE20_PARF_PHY_REFCLK			0x04C
#define  PCIE20_PARF_PHY_REFCLK_REF_SSP_EN		(1 << 16)
#define  PCIE20_PARF_PHY_REFCLK_REF_USE_PAD		(1 << 12)
#define  PCIE20_PARF_PHY_REFCLK_REFCLK_DIV2		(1 << 8)
#define  PCIE20_PARF_PHY_REFCLK_MPLL_MULTIPLIER(x)	(((x) & 0x7F) <<  0)

#define PCIE20_PARF_CONFIG_BITS			0x050
#define PCIE20_PARF_CONFIG_BITS_PHY_RX0_EQ(x)		(((x) & 0x7) << 24)

/* ELBI - External Local Bus Interface registers */
#define PCIE20_ELBI_SYS_CTRL			0x004
#define  PCIE20_ELBI_SYS_CTRL_INIT_RST			(1 << 6)
#define  PCIE20_ELBI_SYS_CTRL_UNLOCK_MSG		(1 << 5)
#define  PCIE20_ELBI_SYS_CTRL_PME_TURNOFF_MSG		(1 << 4)
#define  PCIE20_ELBI_SYS_CTRL_LTSSM_EN			(1 << 0)

/* Number of MSI interrupts */
#define MSI_IRQ_NUM			(PCIE20_MSI_CTRL_REGS * 32)
/* Start of MSI interrupt numbers */
#define MSI_IRQ				(NIRQ - MSI_IRQ_NUM)

#define DEBUG
#ifdef DEBUG
#define debugf(fmt, args...) do { printf(fmt,##args); } while (0)
#else
#define debugf(fmt, args...)
#endif
#define	CORE_RD4(sc, reg)	\
    bus_read_4((sc)->core_mem_res, (reg))
#define CORE_WR4(sc, reg, value)	\
    bus_write_4((sc)->core_mem_res, (reg), (value))

#define	PARF_RD4(sc, reg)	\
    bus_read_4((sc)->parf_mem_res, (reg))
#define PARF_WR4(sc, reg, value)	\
    bus_write_4((sc)->parf_mem_res, (reg), (value))
#define	ELBI_RD4(sc, reg)	\
    bus_read_4((sc)->elbi_mem_res, (reg))
#define ELBI_WR4(sc, reg, value)	\
    bus_write_4((sc)->elbi_mem_res, (reg), (value))

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
	{"qcom,pcie-ipq8064",	1},
	{NULL,		 	0},
};

struct ofw_pci_range {
	uint32_t		pci_flags;
	uint64_t		pci;
	uint64_t		host;
	uint64_t		size;
};

struct qcom_pcib_softc {
	device_t		dev;
	struct mtx		mtx;
	struct ofw_bus_iinfo	pci_iinfo;

	struct rman		mem_rman;
	struct rman		io_rman;

	struct resource		*core_mem_res;
	struct resource		*elbi_mem_res;
	struct resource		*parf_mem_res;

	struct resource		*msi_irq_res;
	int			irq_num[4];
	void 			*msi_intr_cookie;

	struct ofw_pci_range	mem_range;
	struct ofw_pci_range	io_range;
	struct ofw_pci_range	cfg_range;
	struct resource		*cfg_mem_res;

	/* FDT resources */
	phy_t			phy;
	clk_t			clk_core;
	clk_t			clk_iface;
	clk_t			clk_phy;
	reset_t			reset_axi;
	reset_t			reset_ahb;
	reset_t			reset_por;
	reset_t			reset_pci;
	reset_t			reset_phy;
	struct gpiobus_pin	*gpio_reset;
	regulator_t		supply_vdd;
	regulator_t		supply_avdd;
	regulator_t		supply_pcie_clk;
	regulator_t		supply_pcie_ext3v3;
	int			ext_phy_ref_clk;

	uint32_t		msi_bitmap;

	int			busnr;		/* Host bridge bus number */
};


static int
ofw_pci_fill_ranges(phandle_t node, struct ofw_pci_range **ranges)
{
	int host_address_cells = 1, pci_address_cells = 3, size_cells = 2;
	cell_t *base_ranges;
	ssize_t nbase_ranges;
	int nranges;
	int i, j, k;

	OF_getencprop(OF_parent(node), "#address-cells", &host_address_cells,
	    sizeof(host_address_cells));
	OF_getencprop(node, "#address-cells", &pci_address_cells,
	    sizeof(pci_address_cells));
	OF_getencprop(node, "#size-cells", &size_cells, sizeof(size_cells));

	nbase_ranges = OF_getproplen(node, "ranges");
	if (nbase_ranges <= 0)
		return (-1);
	nranges = nbase_ranges / sizeof(cell_t) /
	    (pci_address_cells + host_address_cells + size_cells);


	*ranges = malloc(nranges * sizeof(struct ofw_pci_range),
	    M_DEVBUF, M_WAITOK);
	base_ranges = malloc(nbase_ranges, M_DEVBUF, M_WAITOK);
	OF_getencprop(node, "ranges", base_ranges, nbase_ranges);

	for (i = 0, j = 0; i < nranges; i++) {
		(*ranges)[i].pci_flags = base_ranges[j++];
		(*ranges)[i].pci = 0;
		for (k = 0; k < pci_address_cells - 1; k++) {
			(*ranges)[i].pci <<= 32;
			(*ranges)[i].pci |= base_ranges[j++];
		}
		(*ranges)[i].host = 0;
		for (k = 0; k < host_address_cells; k++) {
			(*ranges)[i].host <<= 32;
			(*ranges)[i].host |= base_ranges[j++];
		}
		(*ranges)[i].size = 0;
		for (k = 0; k < size_cells; k++) {
			(*ranges)[i].size <<= 32;
			(*ranges)[i].size |= base_ranges[j++];
		}
	}

	free(base_ranges, M_DEVBUF);
	return (nranges);
}

static int is_link_up(struct qcom_pcib_softc *sc)
{
	return (CORE_RD4(sc, PCIE20_CAP_LINKCTRLSTATUS) & (1 << 29));
}

static void
setup_cfg_region(struct qcom_pcib_softc *sc, uint32_t cfg_type,
	uint32_t bus, uint32_t slot, uint32_t func)
{
	uint32_t base;

	base = PCIE20_IATU_BUS(bus) | PCIE20_IATU_DEV(slot) |
	    PCIE20_IATU_FUN(func);

	CORE_WR4(sc, PCIE20_PLR_IATU_VIEWPORT, 0);
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL1, cfg_type);
	CORE_WR4(sc, PCIE20_PLR_IATU_LTAR, base);
}

static uint32_t
qcom_pcib_read_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes)
{
	struct qcom_pcib_softc *sc;
	u_int off;
	struct resource	*res;
	uint32_t val;

	sc = device_get_softc(dev);
#if 0
if (bus > 0)
	printf("%s: %d %d %d %x %d\n", __func__,
	       bus, slot, func, reg, bytes);
#endif
	LOCK(sc);
	/* Root complex */
	if (bus == sc->busnr) {
		/* Root complex */
		if ((slot > 0) || (func > 0)) {
			UNLOCK(sc);
			return (0xFFFFFFFF);
		}
		off = reg;
		res = sc->core_mem_res;
	} else if (bus == 1) {
		if (!is_link_up(sc) || (slot > 0)) {
			UNLOCK(sc);
			return (0xFFFFFFFF);
		}
		setup_cfg_region(sc, IATU_CTRL1_TYPE_CFG0, bus, slot, func);
		off =  PCIE20_IATU_REG(reg);
		res = sc->cfg_mem_res;
	} else {
		if (!is_link_up(sc)) {
			UNLOCK(sc);
			return (0xFFFFFFFF);
		}
		setup_cfg_region(sc, IATU_CTRL1_TYPE_CFG0, bus, slot, func);
		off =  PCIE20_IATU_REG(reg);
		res = sc->cfg_mem_res;
	}

	val = bus_read_4(res, off & ~3);
#if 0
	if ((bus > 0) && (slot >1))
		printf("%s: %d %d %d %x %d < 0x%08X\n", __func__,
		       bus, slot, func, reg, bytes, val);
#endif

	if ((bus == sc->busnr) && ((off & ~3) == PCIR_REVID)) {
		val = (val & 0xFFFF) | (PCIC_BRIDGE << 24) |
		    (PCIS_BRIDGE_PCI << 16);
	}

	switch (bytes) {
	case 4:
		break;
	case 2:
		if (off & 3)
			val >>= 16;
		val &= 0xffff;
		break;
	case 1:
		val >>= ((off & 3) << 3);
		val &= 0xff;
		break;
	}
	UNLOCK(sc);

#if 0
	if (bus > 0)
	printf("  %08x\n", val);
#endif
	return val;
}

static void
qcom_pcib_write_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, uint32_t val, int bytes)
{
	struct qcom_pcib_softc *sc;
	uint32_t off;
	struct resource	*res;
	uint32_t val2;

	sc = device_get_softc(dev);
#if 0
if (bus > 0)
printf("%s: %d %d %d %x %d %08x\n", __func__,
       bus, slot, func, reg, bytes, val);
#endif
	LOCK(sc);
	/* Root complex */
	if (bus == sc->busnr) {
		/* Root complex */
		if ((slot > 0) || (func > 0)) {
			UNLOCK(sc);
			return;
		}
		off = reg;
		res = sc->core_mem_res;
	} else if (bus == 1) {
		if (!is_link_up(sc) || (slot > 0)) {
			UNLOCK(sc);
			return;
		}
		setup_cfg_region(sc, IATU_CTRL1_TYPE_CFG0, bus, slot, func);
		off =  PCIE20_IATU_REG(reg);
		res = sc->cfg_mem_res;
	} else {
		if (!is_link_up(sc)) {
			UNLOCK(sc);
			return;
		}
		setup_cfg_region(sc, IATU_CTRL1_TYPE_CFG1, bus, slot, func);
		off =  PCIE20_IATU_REG(reg);
		res = sc->cfg_mem_res;
	}

	switch (bytes) {
	case 4:
		bus_write_4(res, off, val);
		break;
	case 2:
		val2 = bus_read_4(res, off & ~3);
		val2 &= ~(0xffff << ((off & 3) << 3));
		val2 |= ((val & 0xffff) << ((off & 3) << 3));
		bus_write_4(res, off & ~3, val2);
		break;
	case 1:
		val2 = bus_read_4(res, off & ~3);
		val2 &= ~(0xff << ((off & 3) << 3));
		val2 |= ((val & 0xff) << ((off & 3) << 3));
		bus_write_4(res, off & ~3, val2);
		break;
	}
	UNLOCK(sc);
#if 0
	printf(" done\n");
#endif
}

static int
qcom_pcib_route_interrupt(device_t bus, device_t dev, int pin)
{
	struct qcom_pcib_softc *sc;

	sc = device_get_softc(bus);
	pin--;
	device_printf(bus, "route pin %d for device %d.%d to %d\n",
		      pin, pci_get_slot(dev), pci_get_function(dev),
		     sc->irq_num[pin]);

	return (sc->irq_num[pin]);
}

#ifdef QCOM_PCI_MSI

static int
qcom_pcib_map_msi(device_t dev, device_t child, int irq, uint64_t *addr,
    uint32_t *data)
{
	struct qcom_pcib_softc *sc;

	sc = device_get_softc(dev);
	irq = irq - MSI_IRQ;

	/* validate parameters */
	if (isclr(&sc->msi_bitmap, irq)) {
		device_printf(dev, "invalid MSI 0x%x\n", irq);
		return (EINVAL);
	}

	*addr = PCIE20_MSI_PHY_ADDR;
	*data = irq;

	debugf("%s: irq: %d addr: %jx data: %x\n", __func__, irq, *addr, *data);

	return (0);
}


static int
qcom_pcib_alloc_msi(device_t dev, device_t child, int count,
    int maxcount __unused, int *irqs)
{
	struct qcom_pcib_softc *sc;
	int start, i;

	sc = device_get_softc(dev);

	if (powerof2(count) == 0 || count > MSI_IRQ_NUM)
		return (EINVAL);

	LOCK(sc);

	for (start = 0; (start + count) < MSI_IRQ_NUM; start++) {
		for (i = start; i < start + count; i++) {
			if (isset(&sc->msi_bitmap, i))
				break;
		}
		if (i == start + count)
			break;
	}

	if ((start + count) == MSI_IRQ_NUM) {
		UNLOCK(sc);
		return (ENXIO);
	}

	for (i = 0; i < count; i++) {
		setbit(&sc->msi_bitmap, start + i);
		irqs[i] = MSI_IRQ + start + i;
	}

	UNLOCK(sc);
	debugf("%s: start: %d count: %d irq: %d (%d)\n", __func__,
	    start, count, irqs[0], MSI_IRQ);
	return (0);
}

static int
qcom_pcib_release_msi(device_t dev, device_t child, int count, int *irqs)
{
	struct qcom_pcib_softc *sc;
	int i;

	sc = device_get_softc(dev);
	LOCK(sc);

	for (i = 0; i < count; i++)
		clrbit(&sc->msi_bitmap, irqs[i] - MSI_IRQ);

	UNLOCK(sc);
	return (0);
}

static void
qcom_pci_run_irqs(struct qcom_pcib_softc *sc, struct intr_event *ie)
{
	struct intr_handler *ih;

	TAILQ_FOREACH(ih, &ie->ie_handlers, ih_next) {
		if (ih->ih_filter != NULL) {
			ih->ih_filter(ih->ih_argument);
		} else 	if (ih->ih_handler != NULL) {
			ih->ih_handler(ih->ih_argument);
		}
	}
}

extern struct intr_event *intr_events[NIRQ];
static int
qcom_pci_intr(void *arg)
{
	struct qcom_pcib_softc *sc;
	uint32_t val;
	int i, j, rv;
	struct intr_event *event;

	sc = (struct qcom_pcib_softc *)arg;
	rv = FILTER_STRAY;
	for (i = 0; i < PCIE20_MSI_CTRL_REGS; i++) {
		val = CORE_RD4(sc, PCIE20_MSI_CTRL_INTR_STATUS + 12 * i);
		if (val == 0)
			continue;
		for (j = 0; j < 32; j++) {
			if ( val & (1 << j)) {
//printf("Interrupt: reg: %d, bit: %d, val: 0x%08X\n", i, j, val);
				event = intr_events[i * 32 + j + MSI_IRQ];
//printf(" irq: %d, event: %p\n", i * 32 + j + MSI_IRQ, event);
				if (event != NULL)
					qcom_pci_run_irqs(sc, event);
				CORE_WR4(sc,
				    PCIE20_MSI_CTRL_INTR_STATUS + 12 * i,
				    (1 << j));
				rv = FILTER_HANDLED;
			}
		}
	}

	return (rv);
}
#endif /* QCOM_PCI_MSI */

/*
 * Resource manager
 */
static int
qcom_pcib_rman_init(struct qcom_pcib_softc *sc)
{
	int err;
	char buf[64];

	/*
	 * Memory management.
	 */
	sc->mem_rman.rm_type = RMAN_ARRAY;
	snprintf(buf, sizeof(buf), "%s memory space",
	    device_get_nameunit(sc->dev));
	sc->mem_rman.rm_descr = strdup(buf, M_DEVBUF);
	err = rman_init(&sc->mem_rman);
	if (err)
		return (err);

	sc->io_rman.rm_type = RMAN_ARRAY;
	snprintf(buf, sizeof(buf), "%s I/O space",
	    device_get_nameunit(sc->dev));
	sc->io_rman.rm_descr = strdup(buf, M_DEVBUF);
	err = rman_init(&sc->io_rman);
	if (err) {
		rman_fini(&sc->mem_rman);
		return (err);
	}

	err = rman_manage_region(&sc->mem_rman, sc->mem_range.host,
	    sc->mem_range.host + sc->mem_range.size - 1);
	if (err)
		goto error;
	err = rman_manage_region(&sc->io_rman, sc->io_range.pci,
	    sc->io_range.pci + sc->io_range.size - 1);
	if (err)
		goto error;
	return (0);

error:
	rman_fini(&sc->mem_rman);
	rman_fini(&sc->io_rman);
	return (err);
}

static struct rman *
qcom_pcib_rman(struct qcom_pcib_softc *sc, int type)
{

	switch (type) {
	case SYS_RES_IOPORT:
		return (&sc->io_rman);
	case SYS_RES_MEMORY:
		return (&sc->mem_rman);
	default:
		break;
	}

	return (NULL);
}

static struct resource *
qcom_pcib_alloc_resource(device_t dev, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct qcom_pcib_softc *sc;
	struct rman *rm;
	struct resource *res;

	debugf("%s: %d start %lx end %lx count %lx\n", __func__,
	    type, start, end, count);
	sc = device_get_softc(dev);

#if defined(NEW_PCIB) && defined(PCI_RES_BUS)
	if (type ==  PCI_RES_BUS) {
		  return (pci_domain_alloc_bus(0, child, rid, start, end, count,
					       flags));
	}
#endif

	rm = qcom_pcib_rman(sc, type);

	if (rm == NULL) {
res = BUS_ALLOC_RESOURCE(device_get_parent(dev), dev,
		    type, rid, start, end, count, flags);
debugf("%s: %d start %lx end %lx count %lx <- %p\n", __func__,
	    type, start, end, count, res);

return (res);
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), dev,
		    type, rid, start, end, count, flags));
	}

 	if (bootverbose) {
		device_printf(dev,
		    "rman_reserve_resource: start=%#lx, end=%#lx, count=%#lx\n",
		    start, end, count);
	}

	res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (res == NULL)
		goto fail;

	rman_set_rid(res, *rid);

	if (flags & RF_ACTIVE) {
		if (bus_activate_resource(child, type, *rid, res)) {
			rman_release_resource(res);
			goto fail;
		}
	}
	return (res);

fail:
	if (bootverbose) {
		device_printf(dev, "%s FAIL: type=%d, rid=%d, "
		    "start=%016lx, end=%016lx, count=%016lx, flags=%x\n",
		    __func__, type, *rid, start, end, count, flags);
	}

	return (NULL);
}

static int
qcom_pcib_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{
	struct qcom_pcib_softc *sc;
	struct rman *rm;

	sc = device_get_softc(dev);
	debugf("%s: %d rid %x\n",  __func__, type, rid);

#if defined(NEW_PCIB) && defined(PCI_RES_BUS)
	if (type == PCI_RES_BUS)
		return (pci_domain_release_bus(0, child, rid, res));
#endif

	rm = qcom_pcib_rman(sc, type);
	if (rm != NULL) {
		KASSERT(rman_is_region_manager(res, rm), ("rman mismatch"));
		rman_release_resource(res);
	}

	return (bus_generic_release_resource(dev, child, type, rid, res));
}

static int
qcom_pcib_adjust_resource(device_t dev, device_t child, int type,
			    struct resource *res, u_long start, u_long end)
{
	struct qcom_pcib_softc *sc;
	struct rman *rm;

	sc = device_get_softc(dev);
	debugf("%s: %d start %lx end %lx \n", __func__,
	    type, start, end);

#if defined(NEW_PCIB) && defined(PCI_RES_BUS)
	if (type == PCI_RES_BUS)
		return (pci_domain_adjust_bus(0, child, res, start, end));
#endif

	rm = qcom_pcib_rman(sc, type);
	if (rm != NULL)
		return (rman_adjust_resource(res, start, end));
	return (bus_generic_adjust_resource(dev, child, type, res, start, end));
}


/*
 * IVARs
 */
static int
qcom_pcib_read_ivar(device_t dev, device_t child, int which, uintptr_t *result)
{
	struct qcom_pcib_softc *sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_BUS:
		*result = sc->busnr;
		return (0);
	case PCIB_IVAR_DOMAIN:
		*result = device_get_unit(dev);
		return (0);
	}

	return (ENOENT);
}

static int
qcom_pcib_write_ivar(device_t dev, device_t child, int which, uintptr_t value)
{
	struct qcom_pcib_softc *sc = device_get_softc(dev);

	switch (which) {
	case PCIB_IVAR_BUS:
		sc->busnr = value;
		return (0);
	}

	return (ENOENT);
}

static int
qcom_pcib_maxslots(device_t dev)
{
	return (2);
}

static int
qcom_pcib_parse_fdt(struct qcom_pcib_softc *sc, phandle_t node)
{
	int err;
	if (OF_hasprop(node, "qcom,external-phy-refclk"))
		sc->ext_phy_ref_clk = 1;

	sc->supply_vdd = fdt_regulator_get_by_name(sc->dev, "vdd-supply");
	if (sc->supply_vdd == NULL) {
		device_printf(sc->dev, "Cannot get \"vdd\" regulator\n");
		return (ENXIO);
	}
	sc->supply_pcie_clk = fdt_regulator_get_by_name(sc->dev,
	    "pcie-clk-supply");
	if (sc->supply_pcie_clk == NULL) {
		device_printf(sc->dev, "Cannot get \"pcie-clk\" regulator\n");
		return (ENXIO);
	}
	sc->supply_avdd = fdt_regulator_get_by_name(sc->dev, "avdd-supply");
	if (sc->supply_avdd == NULL) {
		device_printf(sc->dev, "Cannot get \"avdd\" regulator\n");
		return (ENXIO);
	}
	sc->supply_pcie_ext3v3 = fdt_regulator_get_by_name(sc->dev,
	    "ext-3p3v-supply");
	if (sc->supply_pcie_ext3v3 == NULL) {
		device_printf(sc->dev, "Cannot get \"ext-3p3v\" regulator\n");
		return (ENXIO);
	}

	err = ofw_gpiobus_parse_gpios(sc->dev, "reset-gpio", &sc->gpio_reset);
	if (err != 1) {
		device_printf(sc->dev, "Cannot get \"reset-gpio\" gpio\n");
		return (err);
	}

	err = clk_get_by_ofw_name(node, "iface", &sc->clk_iface);
	if (err != 0) {
		device_printf(sc->dev, "Cannot get \"iface\" clock\n");
		return (err);
	}
	err = clk_get_by_ofw_name(node, "phy", &sc->clk_phy);
	if (err != 0) {
		device_printf(sc->dev, "Cannot get \"phy\" clock\n");
		return (err);
	}
	err = clk_get_by_ofw_name(node, "core", &sc->clk_core);
	if (err != 0) {
		device_printf(sc->dev, "Cannot get \"core\" clock\n");
		return (err);
	}

	sc->reset_axi = fdt_reset_get_by_name(sc->dev, "axi");
	if (sc->reset_axi == NULL) {
		device_printf(sc->dev, "Cannot get \"axi\" reset\n");
		return (ENXIO);
	}
	sc->reset_ahb = fdt_reset_get_by_name(sc->dev, "ahb");
	if (sc->reset_ahb == NULL) {
		device_printf(sc->dev, "Cannot get \"ahb\" reset\n");
		return (ENXIO);
	}
	sc->reset_por = fdt_reset_get_by_name(sc->dev, "por");
	if (sc->reset_por == NULL) {
		device_printf(sc->dev, "Cannot get \"por\" reset\n");
		return (ENXIO);
	}
	sc->reset_pci = fdt_reset_get_by_name(sc->dev, "pci");
	if (sc->reset_pci == NULL) {
		device_printf(sc->dev, "Cannot get \"pci\" reset\n");
		return (ENXIO);
	}
	sc->reset_phy = fdt_reset_get_by_name(sc->dev, "phy");
	if (sc->reset_phy == NULL) {
		device_printf(sc->dev, "Cannot get \"phy\" reset\n");
		return (ENXIO);
	}

	return 0;
}

static int
gpio_set(struct gpiobus_pin *pin, int val)
{
	int err;

	val = val ? 1: 0;
	if (pin->flags & GPIO_ACTIVE_LOW)
		val ^= 1;
	err = GPIO_PIN_SETFLAGS(pin->dev, pin->pin, GPIO_PIN_OUTPUT);
	return (GPIO_PIN_SET(pin->dev, pin->pin, val));
}

static int
qcom_pcie_vreg_on(struct qcom_pcib_softc *sc)
{
	int err;

	/* enable regulators */
	err = fdt_regulator_enable(sc->dev, sc->supply_vdd);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable power supply \"vdd\"\n");
		return (err);
	}

	err = fdt_regulator_enable(sc->dev, sc->supply_pcie_clk);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable power supply \"pcie clk\"\n");
		return (err);
	}

	err = fdt_regulator_enable(sc->dev, sc->supply_avdd);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable power supply \"avdd\"\n");
		return (err);
	}

	err = fdt_regulator_enable(sc->dev, sc->supply_pcie_ext3v3);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable power supply \"pcie ext3v3\"\n");
		return (err);
	}

	return (0);
}

static void
qcom_pcib_host_init(struct qcom_pcib_softc *sc)
{
	uint64_t end, i;

	/* Setup IATU region 0 to config type 0, outbound */
	CORE_WR4(sc, PCIE20_PLR_IATU_VIEWPORT, 0);
	CORE_RD4(sc, PCIE20_PLR_IATU_VIEWPORT);

	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL1, IATU_CTRL1_TYPE_CFG0);
	end = sc->cfg_range.host + sc->cfg_range.size - 1;
	CORE_WR4(sc, PCIE20_PLR_IATU_LBAR, (uint32_t)sc->cfg_range.host);
	CORE_WR4(sc, PCIE20_PLR_IATU_UBAR, (uint32_t)(sc->cfg_range.host >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_LAR, (uint32_t)end);
	CORE_WR4(sc, PCIE20_PLR_IATU_LTAR, (uint32_t)sc->cfg_range.pci);
	CORE_WR4(sc, PCIE20_PLR_IATU_UTAR, (uint32_t)(sc->cfg_range.pci >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL2, IATU_CTRL2_ENABLE);

	/* Setup IATU region 1 to IO space, outbound  */
	CORE_WR4(sc, PCIE20_PLR_IATU_VIEWPORT, 1);
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL1, IATU_CTRL1_TYPE_IO);
	end = sc->io_range.host + sc->io_range.size -1;
	CORE_WR4(sc, PCIE20_PLR_IATU_LBAR, (uint32_t)sc->io_range.host);
	CORE_WR4(sc, PCIE20_PLR_IATU_UBAR, (uint32_t)(sc->io_range.host >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_LAR, (uint32_t)end);
	CORE_WR4(sc, PCIE20_PLR_IATU_LTAR, (uint32_t)sc->io_range.pci);
	CORE_WR4(sc, PCIE20_PLR_IATU_UTAR, (uint32_t)(sc->io_range.pci >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL2, IATU_CTRL2_ENABLE);
#if 0
	/* Setup IATU region 2 to memory space */
	CORE_WR4(sc, PCIE20_PLR_IATU_VIEWPORT, 2);
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL1, IATU_CTRL1_TYPE_MEM);
	end = sc->mem_range.pci + sc->mem_range.size -1;
	CORE_WR4(sc, PCIE20_PLR_IATU_LBAR, (uint32_t)sc->mem_range.host);
	CORE_WR4(sc, PCIE20_PLR_IATU_UBAR, (uint32_t)(sc->mem_range.host >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_LAR, (uint32_t)end);
	CORE_WR4(sc, PCIE20_PLR_IATU_LTAR, (uint32_t)sc->mem_range.pci);
	CORE_WR4(sc, PCIE20_PLR_IATU_UTAR, (uint32_t)(sc->mem_range.pci >> 32));
	CORE_WR4(sc, PCIE20_PLR_IATU_CTRL2, IATU_CTRL2_ENABLE);
#endif
	/* Setup Remote Read Request Size to 1024 */
	CORE_WR4(sc, PCIE20_AXI_MSTR_RESP_COMP_CTRL0, 0x03);
	CORE_WR4(sc, PCIE20_AXI_MSTR_RESP_COMP_CTRL1, 0x01);


	/* Initialize MSI base address and enable all vectors */
	CORE_WR4(sc, PCIE20_MSI_CTRL_ADDR, PCIE20_MSI_PHY_ADDR);
	CORE_WR4(sc, PCIE20_MSI_CTRL_UPPER_ADDR, 0);
	for (i = 0; i < PCIE20_MSI_CTRL_REGS; i++)
		CORE_WR4(sc, PCIE20_MSI_CTRL_INTR_EN + (i * 12), 0xFFFFFFFF);
}


static int
qcom_pcib_phy_init(struct qcom_pcib_softc *sc)
{
	int err, i;
	uint32_t val;

	/* Set external reset */
	err = gpio_set(sc->gpio_reset, 1);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset external\n");
		return (ENXIO);
	}

	/* Enable power */
	err = qcom_pcie_vreg_on(sc);
	if (err != 0) {
		return (ENXIO);
	}

	/* Assert all resets */
	err = fdt_reset_assert(sc->dev, sc->reset_ahb);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset PARF\n");
		return (ENXIO);
	}
	err = fdt_reset_assert(sc->dev, sc->reset_phy);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset PHY\n");
		return (ENXIO);
	}
	err = fdt_reset_assert(sc->dev, sc->reset_pci);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset PCI\n");
		return (ENXIO);
	}
	err = fdt_reset_assert(sc->dev, sc->reset_por);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset POR\n");
		return (ENXIO);
	}
	err = fdt_reset_assert(sc->dev, sc->reset_axi);
	if (err != 0) {
		device_printf(sc->dev, "Cannot reset AXI\n");
		return (ENXIO);
	}

	/* Enable clocks */
	err = clk_enable(sc->clk_iface);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable iface clock: %d\n", err);
		return (ENXIO);
	}
	err = clk_enable(sc->clk_phy);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable phy clock: %d\n", err);
		return (ENXIO);
	}	err = clk_enable(sc->clk_core);
	if (err != 0) {
		device_printf(sc->dev, "Cannot enable core clock: %d\n", err);
		return (ENXIO);
	}

	/*
	 * Initialize PARF
	 */
	/* Release PARF reset */
	err = fdt_reset_clear(sc->dev, sc->reset_ahb);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset PARF\n");
		return (ENXIO);
	}
	DELAY(10);

	/* Release power down and tests */
	val = PARF_RD4(sc, PCIE20_PARF_PHY_CTRL);
	val &= ~(PCIE20_PARF_PHY_CTRL_PHY_TEST_PWR_DOWN |
	    PCIE20_PARF_PHY_CTRL_PHY_TEST_BYPASS |
	    PCIE20_PARF_PHY_CTRL_PHY_TEST_BURNIN);
	PARF_WR4(sc, PCIE20_PARF_PHY_CTRL, val);

	val = PARF_RD4(sc, PCIE20_PARF_PHY_CTRL);
	val |= PCIE20_PARF_PHY_CTRL_PHY_TX0_TERM_OFFST(7);
	PARF_WR4(sc, PCIE20_PARF_PHY_CTRL, val);

	PARF_WR4(sc, PCIE20_PARF_PCS_DEEMPH,
	    PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN1(0x18) |
	    PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN2_3_5DB(0x18) |
	    PCIE20_PARF_PCS_DEEMPH_TX_DEEMPH_GEN2_6DB(0x22));
	PARF_WR4(sc, PCIE20_PARF_PCS_SWING,
	    PCIE20_PARF_PCS_SWING_TX_SWING_FULL(0x78) |
	    PCIE20_PARF_PCS_SWING_TX_SWING_LOW(0x78));
	PARF_WR4(sc, PCIE20_PARF_CONFIG_BITS,
	    PCIE20_PARF_CONFIG_BITS_PHY_RX0_EQ(4));

	/* Setup external referenc if requested */
	val = PARF_RD4(sc, PCIE20_PARF_PHY_REFCLK);
	if (sc->ext_phy_ref_clk)
		val |= PCIE20_PARF_PHY_REFCLK_REF_USE_PAD;
	else
		val = ~PCIE20_PARF_PHY_REFCLK_REF_USE_PAD;
	val |= PCIE20_PARF_PHY_REFCLK_REF_SSP_EN;
	PARF_WR4(sc, PCIE20_PARF_PHY_REFCLK, val);
	PARF_RD4(sc, PCIE20_PARF_PHY_REFCLK);

	/* Release all resets */
	err = fdt_reset_clear(sc->dev, sc->reset_ahb);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset PARF\n");
		return (ENXIO);
	}
	err = fdt_reset_clear(sc->dev, sc->reset_phy);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset PHY\n");
		return (ENXIO);
	}
	err = fdt_reset_clear(sc->dev, sc->reset_pci);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset PCI\n");
		return (ENXIO);
	}
	err = fdt_reset_clear(sc->dev, sc->reset_por);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset POR\n");
		return (ENXIO);
	}
	err = fdt_reset_clear(sc->dev, sc->reset_axi);
	if (err != 0) {
		device_printf(sc->dev, "Cannot unreset AXI\n");
		return (ENXIO);
	}
	DELAY(10);

	/* Release external reset */
	err = gpio_set(sc->gpio_reset, 0);
	if (err != 0) {
		device_printf(sc->dev, "Cannot relese external reset\n");
		return (ENXIO);
	}
	DELAY(1000);

	/* Start link training */
	val = ELBI_RD4(sc, PCIE20_ELBI_SYS_CTRL);
	val |= PCIE20_ELBI_SYS_CTRL_LTSSM_EN;
	ELBI_WR4(sc, PCIE20_ELBI_SYS_CTRL, val);

	/* Wait for link */
	for(i = 100; i > 0; i--) {
		if (CORE_RD4(sc, PCIE20_CAP_LINKCTRLSTATUS) & (1 << 29))
			break;
		DELAY(1000);
	}
	if ( i <= 0)
		device_printf(sc->dev, "Cannot estabilish link\n");
#if 1
	printf("CORE:  0x%08X, 0x%08X\n", CORE_RD4(sc, 0), CORE_RD4(sc, 4));
	printf("ELBI:  0x%08X, 0x%08X, 0x%08X\n", ELBI_RD4(sc, 0), ELBI_RD4(sc, 4), ELBI_RD4(sc, 8));
	printf("PARF:  0x%08X, 0x%08X\n", PARF_RD4(sc, 0), PARF_RD4(sc, 4));

#endif
	return (0);
}

static int
qcom_pcib_decode_ranges(struct qcom_pcib_softc *sc,
    struct ofw_pci_range *ranges, int nranges)
{
	int i;

	for (i = 0; i < nranges; i++) {
		if ((ranges[i].pci_flags & OFW_PCI_PHYS_HI_SPACEMASK) ==
		    OFW_PCI_PHYS_HI_SPACE_CONFIG) {
			if (sc->cfg_range.size != 0) {
				device_printf(sc->dev,
				    "Duplicated config range found in DT\n");
				return (ENXIO);
			}
			sc->cfg_range = ranges[i];
		}
		if ((ranges[i].pci_flags & OFW_PCI_PHYS_HI_SPACEMASK) ==
		    OFW_PCI_PHYS_HI_SPACE_IO) {
			if (sc->io_range.size != 0) {
				device_printf(sc->dev,
				    "Duplicated IO range found in DT\n");
				return (ENXIO);
			}
			sc->io_range = ranges[i];
		}
		if ((ranges[i].pci_flags & OFW_PCI_PHYS_HI_SPACEMASK) ==
		    OFW_PCI_PHYS_HI_SPACE_MEM32) {
			if (sc->mem_range.size != 0) {
				device_printf(sc->dev,
				    "Duplicated memory range found in DT\n");
				return (ENXIO);
			}
			sc->mem_range = ranges[i];
		}
	}
	if ((sc->cfg_range.size == 0) || (sc->io_range.size == 0) ||
	    (sc->mem_range.size == 0)) {
		device_printf(sc->dev,
		    " Not all required rnages are found in DT\n");
		return (ENXIO);
	}
	return (0);
}

static int
qcom_pcib_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm PCI/PCI-E Controller");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

static int
qcom_pcib_attach(device_t dev)
{
	struct qcom_pcib_softc *sc;
	phandle_t node;
	uint32_t unit;
	int i, err, rid, nranges;
	struct ofw_pci_range *ranges;
	struct resource *res;

	sc = device_get_softc(dev);
	sc->dev = dev;
	unit = fdt_get_unit(dev);
	node = ofw_bus_get_node(dev);
	ofw_bus_setup_iinfo(node, &sc->pci_iinfo, sizeof(pcell_t));

	LOCK_INIT(sc);

	/* Parse FDT data */
	err = qcom_pcib_parse_fdt(sc, node);
	if (err != 0)
		goto out;

	nranges = ofw_pci_fill_ranges(node, &ranges);
	if (nranges != 3) {
		device_printf(sc->dev, "Invalid number of ranges: %d\n",
		    nranges);
		err = ENXIO;
		goto out;
	}
printf("Got %d ranges \n", nranges);

	/*
	 * Retrieve our mem-mapped registers range.
	 */
	rid = 0;
	sc->core_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->core_mem_res == NULL) {
		device_printf(dev, "Cannot allocate PCIE20 register\n");
		err = ENXIO;
		goto out;
	}

	rid = 1;
	sc->elbi_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->elbi_mem_res == NULL) {
		device_printf(dev, "Cannot allocate ELBI register\n");
		err = ENXIO;
		goto out;
	}

	rid = 2;
	sc->parf_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->parf_mem_res == NULL) {
		device_printf(dev, "Cannot allocate PARF memory\n");
		err = ENXIO;
		goto out;
	}

	/*
	 * Get MSI and PCI interrupts info
	 */
	err = ofw_bus_find_string_index(node, "interrupt-names", "msi", &rid);
	if (err != 0) {
		device_printf(dev, "Cannot get MSI interrupt\n");
		err = ENXIO;
		goto out;
	}
	sc->msi_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->msi_irq_res == NULL) {
		device_printf(dev, "Cannot allocate MSI IRQ resource\n");
		err = ENXIO;
		goto out;
	}
	for (i = 0; i < nitems(sc->irq_num); i++) {
		rid = i;
		res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, 0);
		if (res == NULL) {
			device_printf(dev, "Cannot allocate IRQ resources\n");
			err = ENXIO;
			goto out;
		}
		sc->irq_num[i] = rman_get_start(res);
		 bus_release_resource(dev, SYS_RES_IRQ, i, res);
	}

	/* Memory management */
	err = qcom_pcib_decode_ranges(sc, ranges, nranges);
	if (err)
		goto out;
	err = qcom_pcib_rman_init(sc);
	if (err)
		goto out;
	free(ranges, M_DEVBUF);
	qcom_pcib_phy_init(sc);
	qcom_pcib_host_init(sc);

	sc->cfg_mem_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    (u_long)sc->cfg_range.host,
	    (u_long)(sc->cfg_range.host + sc->cfg_range.size - 1),
	    sc->cfg_range.size,
	     RF_ACTIVE);
	if (sc->cfg_mem_res == NULL) {
		device_printf(dev, "Cannot allocate config space memory\n");
		err = ENXIO;
		goto out;
	}
#ifdef QCOM_PCI_MSI
	if (bus_setup_intr(dev, sc->msi_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
			   qcom_pci_intr, NULL, sc, &sc->msi_intr_cookie)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		err = ENXIO;
		goto out;
	}
#endif

	device_add_child(dev, "pci", -1);
#if 0
	/* Enable all ABCD interrupts */
	pcib_write_irq_mask(sc, 0);
#endif

	return (bus_generic_attach(dev));

out:
	printf("%s: error\n", __func__);
breakpoint();
	return (err);
}


static device_method_t qcom_pcib_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			qcom_pcib_probe),
	DEVMETHOD(device_attach,		qcom_pcib_attach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,		qcom_pcib_read_ivar),
	DEVMETHOD(bus_write_ivar,		qcom_pcib_write_ivar),
	DEVMETHOD(bus_alloc_resource,		qcom_pcib_alloc_resource),
	DEVMETHOD(bus_adjust_resource,		qcom_pcib_adjust_resource),
	DEVMETHOD(bus_release_resource,		qcom_pcib_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),

	/* pcib interface */
	DEVMETHOD(pcib_maxslots,		qcom_pcib_maxslots),
	DEVMETHOD(pcib_read_config,		qcom_pcib_read_config),
	DEVMETHOD(pcib_write_config,		qcom_pcib_write_config),
	DEVMETHOD(pcib_route_interrupt,		qcom_pcib_route_interrupt),

#ifdef QCOM_PCI_MSI
	DEVMETHOD(pcib_alloc_msi,		qcom_pcib_alloc_msi),
	DEVMETHOD(pcib_release_msi,		qcom_pcib_release_msi),
	DEVMETHOD(pcib_map_msi,			qcom_pcib_map_msi),
#endif

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_compat,		ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,		ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,		ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,		ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,		ofw_bus_gen_get_type),

	DEVMETHOD_END
};

static driver_t qcom_pcib_driver = {
	"pcib",
	qcom_pcib_methods,
	sizeof(struct qcom_pcib_softc),
};

devclass_t pcib_devclass;
DRIVER_MODULE(pcib, simplebus, qcom_pcib_driver, pcib_devclass, 0, 0);
