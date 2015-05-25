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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/fdt_phy.h>
#include <dev/fdt/fdt_regulator.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "ulpi.h"

#define	OTG_HS_AHB_BURST		0x0090
#define	OTG_HS_AHB_MODE			0x0098
#define OTG_HS_GENCONFIG		0x009c
#define  GENCONFIG_TXFIFO_IDLE_FORCE_DISABLE	(1 << 4)
#define OTG_HS_GENCONFIG_2		0x00a0
#define  GENCONFIG_2_SESS_VLD_CTRL_EN		(1 << 7)

#define	OTG_HS_USBCMD			0x0140
#define  USBCMD_RST				(1 << 1)
#define USBCMD_SESS_VLD_CTRL			(1 << 25)

#define	OTG_HS_USBINTR			0x0148
#define USB_ULPI_VIEWPORT		0x0170
#define  ULPI_RUN				(1 << 30)
#define  ULPI_WRITE				(1 << 29)
#define  ULPI_READ				(0 << 29)
#define  ULPI_ADDR(n)				(((n) & 255) << 16)
#define  ULPI_WR_DATA(n)			((n) & 255)
#define  ULPI_RD_DATA(n)			(((n) >> 8) & 255)

#define OTG_HS_PORTSC			0x0184
#define  PORTSC_PTS_MASK			(3 << 30)
#define  PORTSC_PTS_SERIAL			(3 << 30)
#define  PORTSC_PTS_ULPI			(2 << 30)
#define  PORTSC_PTS_UTMI			(0 << 30)
#define  PORTSC_PHCD				(1 << 23)

#define	OTG_HS_OTGSC			0x01A4
#define	OTG_HS_USBMODE			0x01A8

#define OTG_HS_PHY_CTRL			0x0240
#define  PHY_CTRL_PHY_POR			(1 << 0)

#define ULPI_MISC_A			0x096
#define ULPI_MISC_A_VBUSVLDEXTSEL		(1 << 1)
#define ULPI_MISC_A_VBUSVLDEXT			(1 << 0)

#define	LINK_RESET_TIMEOUT_USEC		250000
#define ULPI_IO_TIMEOUT_USEC		10000

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN = 0,
	USB_DR_MODE_DEVICE,
	USB_DR_MODE_HOST,
	USB_DR_MODE_OTG
};

struct usbphy_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void 			*intr_cookie;

	regulator_t		supply_vddcx;
	regulator_t		supply_v3p3;
	regulator_t		supply_v1p8;
	clk_t			clk_core;
	clk_t			clk_alt_core;
	clk_t			clk_iface;
	reset_t			reset_link;
	enum usb_dr_mode	dr_mode;

};

static struct ofw_compat_data compat_data[] = {
	{"qcom,usb-otg-ci",		1},
	{NULL,				0},
};

#define RD4(sc, offs)		bus_read_4(sc->mem_res, offs)
#define WR4(sc, offs, val)	bus_write_4(sc->mem_res, offs, val)

static int
ulpi_read(struct usbphy_softc *sc, uint32_t reg)
{
	int cnt;
	uint32_t val;

	/* initiate read operation */
	WR4(sc, USB_ULPI_VIEWPORT, ULPI_RUN | ULPI_READ | ULPI_ADDR(reg));

	/* wait for completion */
	cnt = ULPI_IO_TIMEOUT_USEC;
	while (cnt > 0) {
		if ((RD4(sc, USB_ULPI_VIEWPORT) & ULPI_RUN) == 0)
			break;
		DELAY(1);
		cnt--;
	}

	if (cnt <= 0) {
		device_printf(sc->dev, "%s: timeout\n", __func__);
		return (ETIMEDOUT);
	}

	val = RD4(sc, USB_ULPI_VIEWPORT);
	return (ULPI_RD_DATA(val));
}

static int
ulpi_write(struct usbphy_softc *sc, uint32_t reg, uint32_t val)
{

	int cnt;

	/* initiate write operation */
	WR4(sc, USB_ULPI_VIEWPORT, ULPI_RUN | ULPI_WRITE | ULPI_ADDR(reg) |
	    ULPI_WR_DATA(val));

	/* wait for completion */
	cnt = ULPI_IO_TIMEOUT_USEC;
	while (cnt >0) {
		if ((RD4(sc, USB_ULPI_VIEWPORT) & ULPI_RUN) == 0)
			break;
		DELAY(1);
		cnt--;
	}

	if (cnt <= 0) {
		device_printf(sc->dev, "%s: timeout\n", __func__);
		return (ETIMEDOUT);
	}
	return (0);
}

static int
usbphy_link_reset(struct usbphy_softc *sc)
{
	uint32_t val;
	int rv;

	rv = fdt_reset_assert(sc->dev, sc->reset_link);
	if (rv)
		return (rv);
	DELAY(20000);

	rv = fdt_reset_clear(sc->dev, sc->reset_link);
	if (rv)
		return (rv);

	/* Select upli mode */
	val = RD4(sc, OTG_HS_PORTSC);
	val &= ~PORTSC_PTS_MASK;
	val |= PORTSC_PTS_SERIAL;
	WR4(sc, OTG_HS_PORTSC, val);

	return (0);
}

static int
usbphy_otg_reset(struct usbphy_softc *sc)
{
	int cnt;

	cnt = LINK_RESET_TIMEOUT_USEC / 10;
	WR4(sc, OTG_HS_USBCMD, USBCMD_RST);
	while (cnt > 0) {
		if ((RD4(sc, OTG_HS_USBCMD) & USBCMD_RST) == 0)
			break;
		DELAY(10);
		cnt--;
	}
	if (cnt <= 0)
		return (ETIMEDOUT);

	/* select ULPI phy and clear other status/control bits in PORTSC */
	WR4(sc, OTG_HS_PORTSC, PORTSC_PTS_ULPI);

	WR4(sc, OTG_HS_AHB_BURST, 0x00);
	WR4(sc, OTG_HS_AHB_MODE, 0x08);

	return (0);
}

static void
usbphy_phy_reset(struct usbphy_softc *sc)
{
	uint32_t val;

	/* Pulse PHY POR */
	val = RD4(sc, OTG_HS_PHY_CTRL);
	val &= ~PHY_CTRL_PHY_POR;
	WR4(sc, OTG_HS_PHY_CTRL, val | PHY_CTRL_PHY_POR);
	DELAY(100);
	WR4(sc, OTG_HS_PHY_CTRL, val);
}

static int
usbphy_init(struct usbphy_softc *sc)
{
	uint32_t val;

	usbphy_phy_reset(sc);

	/* Fix for HW errata - rx buffer collision issue */
	val = RD4(sc, OTG_HS_GENCONFIG);
	val &= ~GENCONFIG_TXFIFO_IDLE_FORCE_DISABLE;
	WR4(sc, OTG_HS_GENCONFIG, val);

 	val = ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT;
	ulpi_write(sc, ULPI_SET(ULPI_MISC_A), val);

 	val = RD4(sc, OTG_HS_GENCONFIG_2);
	val |= GENCONFIG_2_SESS_VLD_CTRL_EN;
 	WR4(sc, OTG_HS_GENCONFIG_2, val);

 	val = RD4(sc, OTG_HS_USBCMD);
 	val |= USBCMD_SESS_VLD_CTRL;
 	WR4(sc, OTG_HS_USBCMD, val);

	val = ulpi_read(sc, ULPI_FUNC_CTRL);
	val &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	val |= ULPI_FUNC_CTRL_OPMODE_NORMAL;
	ulpi_write(sc, ULPI_FUNC_CTRL, val);

	return (0);
}

static int
usbphy_reset(struct usbphy_softc *sc)
{
	int rv;

	rv = usbphy_link_reset(sc);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot reset link\n");
		return (rv);
	}

	rv = usbphy_otg_reset(sc);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot reset OTG\n");
		return (rv);
	}

	DELAY(1000);
	usbphy_phy_reset(sc);

	return (0);
}


static int
usbphy_phy_cfg(device_t dev, int id, int val)
{
	struct usbphy_softc *sc;
	int rv = 0;

	sc = device_get_softc(dev);

	if (val != 0) {
		rv = usbphy_init(sc);
	}

	return (rv);
}

static int
usbphy_init_fdt(struct usbphy_softc *sc, phandle_t node)
{
	int rv;

	rv = clk_set_freq(sc->clk_core, 60000000, 1);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot set \"core\" clock\n");
		return (rv);
	}
	rv = clk_enable(sc->clk_core);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable \"core\" clock\n");
		return (rv);
	}
	rv = clk_enable(sc->clk_iface);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable \"iface\" clock\n");
		return (rv);
	}
	if (sc->clk_alt_core != NULL) {
		rv = clk_enable(sc->clk_alt_core);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot enable \"alt_core\""
			   " clock\n");
			return (rv);
		}
	}

	rv = fdt_regulator_enable(sc->dev, sc->supply_vddcx);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable \"vddcx\" supply");
		return (rv);
	}
	rv = fdt_regulator_enable(sc->dev, sc->supply_v3p3);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable \"v3p3\" supply");
		return (rv);
	}
	rv = fdt_regulator_enable(sc->dev, sc->supply_v1p8);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable \"v1p8\" supply");
		return (rv);
	}
	return (0);
}

static enum usb_dr_mode
usb_get_dr_mode(device_t dev, phandle_t node, char *name)
{
	char *tmpstr;
	int rv;
	enum usb_dr_mode ret;

	rv = OF_getprop_alloc(node, name, 1, (void **)&tmpstr);
	if (rv <= 0)
		return (USB_DR_MODE_UNKNOWN);

	ret = USB_DR_MODE_UNKNOWN;
	if (strcmp(tmpstr, "device") == 0)
		ret = USB_DR_MODE_DEVICE;
	else if (strcmp(tmpstr, "host") == 0)
		ret = USB_DR_MODE_HOST;
	else if (strcmp(tmpstr, "otg") == 0)
		ret = USB_DR_MODE_OTG;
	else
		device_printf(dev, "Unknown dr mode: %s\n", tmpstr);
	free(tmpstr, M_OFWPROP);
	return (ret);
}

static int
usbphy_parse_fdt(struct usbphy_softc *sc, phandle_t node)
{
	int rv;

	sc->supply_vddcx = fdt_regulator_get_by_name(sc->dev, "vddcx-supply");
	if (sc->supply_vddcx == NULL) {
		device_printf(sc->dev, "Cannot get \"vddcx\" regulator\n");
		return (ENXIO);
	}
	sc->supply_v3p3 = fdt_regulator_get_by_name(sc->dev, "v3p3-supply");
	if (sc->supply_v3p3 == NULL) {
		device_printf(sc->dev, "Cannot get \"v3p3\" regulator\n");
		return (ENXIO);
	}
	sc->supply_v1p8 = fdt_regulator_get_by_name(sc->dev, "v1p8-supply");
	if (sc->supply_v1p8 == NULL) {
		device_printf(sc->dev, "Cannot get \"v1p8\" regulator\n");
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(node, "core", &sc->clk_core);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get \"core\" clock\n");
		return (rv);
	}
	rv = clk_get_by_ofw_name(node, "iface", &sc->clk_iface);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get \"iface\" clock\n");
		return (rv);
	}
	clk_get_by_ofw_name(node, "alt_core", &sc->clk_alt_core);

	sc->reset_link = fdt_reset_get_by_name(sc->dev, "link");
	if (sc->reset_link == NULL) {
		device_printf(sc->dev, "Cannot get \"link\" reset\n");
		return (ENXIO);
	}

	/* dr_mode */
	sc->dr_mode = usb_get_dr_mode(sc->dev, node, "dr_mode");
	return (0);
}

static int
usbphy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm USB phy");
	return (BUS_PROBE_DEFAULT);
}

static int
usbphy_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
usbphy_attach(device_t dev)
{
	struct usbphy_softc *sc;
	int rid, rv;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		rv = ENXIO;
		goto fail;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate IRQ resource\n");
		rv = ENXIO;
		goto fail;
	}

	node = ofw_bus_get_node(dev);

	rv = usbphy_parse_fdt(sc, node);
	if (rv != 0)
		goto fail;

	rv = usbphy_init_fdt(sc, node);
	if (rv != 0)
		goto fail;

	if (sc->dr_mode == USB_DR_MODE_UNKNOWN)
		sc->dr_mode = USB_DR_MODE_HOST;

	WR4(sc, OTG_HS_OTGSC, 0);
	WR4(sc, OTG_HS_USBINTR, 0);
#if 0
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, usbphy_intr, sc, &sc->intr_cookie)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		rv = ENXIO;
		goto fail;
	}
#endif
	usbphy_reset(sc);

	fdt_phy_register_provider(dev);

	return (0);

fail:
	return (rv);
}


static device_method_t qcom_otgphy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		usbphy_probe),
	DEVMETHOD(device_attach,	usbphy_attach),
	DEVMETHOD(device_detach,	usbphy_detach),

	/* fdt_phy interface */
	DEVMETHOD(fdt_phy_set,		usbphy_phy_cfg),

	DEVMETHOD_END
};

static driver_t qcom_otgphy_driver = {
	"qcom_otgphy",
	qcom_otgphy_methods,
	sizeof(struct usbphy_softc),
};

static devclass_t qcom_otgphy_devclass;

EARLY_DRIVER_MODULE(qcom_otgphy, simplebus, qcom_otgphy_driver,
    qcom_otgphy_devclass, 0, 0, 74);


