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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * EHCI driver for qcom SoCs.
 */
#include "opt_bus.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/rman.h>

#include <dev/fdt/fdt_phy.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/ehci.h>
#include <dev/usb/controller/ehcireg.h>
#include "usbdevs.h"

#include <machine/bus.h>
#include <machine/resource.h>

#include "opt_platform.h"

#define	QCOM_EHCI_REG_OFF	0x100
#define	QCOM_EHCI_REG_SIZE	0x100

#define	OTG_HS_AHB_BURST	0x090
#define	OTG_HS_AHB_MODE		0x098
#define	OTG_HS_USBMODE		0x1A8

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
	{"qcom,ehci-host",	1},
	{NULL,		 	0},
};

struct qcom_ehci_softc {
	ehci_softc_t	ehci_softc;
	device_t	dev;
	struct resource	*mem_res;	/* EHCI core regs. */
	struct resource	*irq_res;	/* EHCI core IRQ. */
	int		usb_alloc_called;
	phy_t		phy;
};

static int
qcom_ehci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm EHCI controller");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

static int
qcom_ehci_detach(device_t dev)
{
	struct qcom_ehci_softc *sc;
	ehci_softc_t *esc;

	sc = device_get_softc(dev);

	esc = &sc->ehci_softc;
	if (sc->phy != NULL)
		fdt_phy_release(sc->phy);
	if (esc->sc_bus.bdev != NULL)
		device_delete_child(dev, esc->sc_bus.bdev);
	if (esc->sc_flags & EHCI_SCFLG_DONEINIT)
		ehci_detach(esc);
	if (esc->sc_intr_hdl != NULL)
		bus_teardown_intr(dev, esc->sc_irq_res,
		    esc->sc_intr_hdl);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0,
		    sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0,
		    sc->mem_res);
	if (sc->usb_alloc_called)
		usb_bus_mem_free_all(&esc->sc_bus, &ehci_iterate_hw_softc);

	/* During module unload there are lots of children leftover */
	device_delete_children(dev);

	return (0);
}

static int
qcom_ehci_attach(device_t dev)
{
	struct qcom_ehci_softc *sc;
	ehci_softc_t *esc;
	int err, rid;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	esc = &sc->ehci_softc;
	err = 0;

	/* Allocate bus_space resources. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		err = ENXIO;
		goto out;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate IRQ resources\n");
		err = ENXIO;
		goto out;
	}

	node = ofw_bus_get_node(dev);

	sc->phy = fdt_phy_get_by_property(sc->dev, "usb-phy");
	if (sc->phy == NULL) {
		device_printf(sc->dev, "Cannot get \"usb-phy\" phy\n");
		return (ENXIO);
	}
	err = fdt_phy_enable(sc->dev, sc->phy);
	if (err != 0) {
		device_printf(dev, "Cannot enable phy: %d\n", err);
		err = ENXIO;
		goto out;
	}

	esc->sc_io_tag = rman_get_bustag(sc->mem_res);
	esc->sc_bus.parent = dev;
	esc->sc_bus.devices = esc->sc_devices;
	esc->sc_bus.devices_max = EHCI_MAX_DEVICES;
	esc->sc_bus.dma_bits = 32;

	/* Allocate all DMA memory. */
	err = usb_bus_mem_alloc_all(&esc->sc_bus, USB_GET_DMA_TAG(dev),
	    &ehci_iterate_hw_softc);
	sc->usb_alloc_called = 1;
	if (err != 0) {
		device_printf(dev, "usb_bus_mem_alloc_all() failed\n");
		err = ENOMEM;
		goto out;
	}

	/*
	 * Set handle to USB related registers subregion used by
	 * generic EHCI driver.
	 */
	err = bus_space_subregion(esc->sc_io_tag,
	    rman_get_bushandle(sc->mem_res),
	    QCOM_EHCI_REG_OFF, QCOM_EHCI_REG_SIZE, &esc->sc_io_hdl);
	if (err != 0) {
		device_printf(dev, "bus_space_subregion() failed\n");
		err = ENXIO;
		goto out;
	}

	/* Setup interrupt handler. */
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_BIO, NULL,
	    (driver_intr_t *)ehci_interrupt, esc, &esc->sc_intr_hdl);
	if (err != 0) {
		device_printf(dev, "Could not setup IRQ\n");
		goto out;
	}

	/* Add USB bus device. */
	esc->sc_bus.bdev = device_add_child(dev, "usbus", -1);
	if (esc->sc_bus.bdev == NULL) {
		device_printf(dev, "Could not add USB device\n");
		goto out;
	}
	device_set_ivars(esc->sc_bus.bdev, &esc->sc_bus);

	esc->sc_id_vendor = USB_VENDOR_QUALCOMM3;
	strlcpy(esc->sc_vendor, "Qualcomm", sizeof(esc->sc_vendor));

	/* Setup bus parameters and core mode */
	bus_write_4(sc->mem_res, OTG_HS_AHB_BURST, 0);
	bus_write_4(sc->mem_res, OTG_HS_AHB_MODE, 0);
	bus_write_4(sc->mem_res, OTG_HS_USBMODE, 0x13); /* Host mode */

	/* Set flags that affect ehci_init() behavior. */
	esc->sc_flags |=  EHCI_SCFLG_DONTRESET | EHCI_SCFLG_NORESTERM | EHCI_SCFLG_IAADBUG;
	err = ehci_init(esc);
	if (err != 0) {
		device_printf(dev, "USB init failed, usb_err_t=%d\n",
		    err);
		goto out;
	}
	esc->sc_flags |= EHCI_SCFLG_DONEINIT;


	/* Probe the bus. */
	err = device_probe_and_attach(esc->sc_bus.bdev);
	if (err != 0) {
		device_printf(dev,
		    "device_probe_and_attach() failed\n");
		goto out;
	}

	return (0);

out:
	qcom_ehci_detach(dev);
	return (err);
}

static device_method_t ehci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, qcom_ehci_probe),
	DEVMETHOD(device_attach, qcom_ehci_attach),
	DEVMETHOD(device_detach, qcom_ehci_detach),
	DEVMETHOD(device_suspend, bus_generic_suspend),
	DEVMETHOD(device_resume, bus_generic_resume),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),

	DEVMETHOD_END
};

static driver_t ehci_driver = {
	"ehci",
	ehci_methods,
	sizeof(struct qcom_ehci_softc)
};

static devclass_t ehci_devclass;

DRIVER_MODULE(ehci, simplebus, ehci_driver, ehci_devclass, 0, 0);
MODULE_DEPEND(ehci, usb, 1, 1, 1);