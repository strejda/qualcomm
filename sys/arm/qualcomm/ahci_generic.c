/*-
 * Copyright (c) 2009-2012 Alexander Motin <mav@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/fdt.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ahci/ahci.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/fdt_phy.h>
#include <dev/fdt/fdt_regulator.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define GEN_AHCI_MAX_CLKS	16


#define	WR4(_sc, _r, _v)	bus_write_4((_sc)->ctlr.r_mem, (_r), (_v))
#define	RD4(_sc, _r)		bus_read_4((_sc)->ctlr.r_mem, (_r))

static struct ofw_compat_data compat_data[] = {
	{"generic-ahci", 	1},
	{NULL,			0}
};

struct  assigned_clk {
	clk_t			clk;
	uint64_t		freq;
};


struct gen_ahci_sc {
	struct ahci_controller	ctlr;	/* Must be first */
	device_t		dev;
	int			port_mask;
	regulator_t		supply_target;
	phy_t			phy;
	clk_t			clks[GEN_AHCI_MAX_CLKS];
	struct assigned_clk	assigned_clks[GEN_AHCI_MAX_CLKS];

};

static int
get_fdt_resources(struct gen_ahci_sc *sc, phandle_t node)
{
	int i, rv;

	sc->supply_target = fdt_regulator_get_by_name(sc->dev, "target-supply");
	if (sc->supply_target == NULL) {
		device_printf(sc->dev, "Cannot get \"target\" regulator\n");
		return (ENXIO);
	}

	sc->phy = fdt_phy_get_by_name(sc->dev, "sata-phy");
	if (sc->phy == NULL) {
		device_printf(sc->dev, "Cannot get \"sata\" phy\n");
		return (ENXIO);
	}

	for (i = 0; i < GEN_AHCI_MAX_CLKS; i++) {
		rv = clk_get_by_ofw_index(node, i, sc->clks + i);
		if (rv != 0)
			break;
	}
	return (0);
}

static int
enable_fdt_resources(struct gen_ahci_sc *sc)
{
	int i, rv;

	rv = fdt_regulator_enable(sc->dev, sc->supply_target);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable  \"target\" regulator\n");
		return (rv);
	}

	/* XXX Use assigned clock */
	rv = clk_set_freq(sc->clks[3], 100000000, 1);
	rv = clk_set_freq(sc->clks[4], 100000000, 1);

	for (i = 0; (i < GEN_AHCI_MAX_CLKS) && (sc->clks[i] != NULL); i++) {
		rv = clk_enable(sc->clks[i]);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot enable clock\n");
			return (rv);
		}
	}

	rv = fdt_phy_enable(sc->dev, sc->phy);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable phy\n");
		return (rv);
	}

	return (0);
}

static int
gen_ahci_ctlr_reset(device_t dev)
{
	struct gen_ahci_sc *sc;
	int rv;
	uint32_t val;
	uint32_t max_ports;

	sc = device_get_softc(dev);

	rv = ahci_ctlr_reset(dev);
	if (rv != 0)
		return (0);

	/* Praper write once registers */
	val  = RD4(sc, AHCI_CAP);
	val |= AHCI_CAP_SSS;
	WR4(sc, AHCI_CAP, val);

	max_ports = val & AHCI_CAP_NPMASK;
	val = sc->port_mask;
	val &= (max_ports - 1);
	WR4(sc, AHCI_PI, val);
	return (0);
}

static int
gen_ahci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc_copy(dev, "AHCI SATA controller");
	return (BUS_PROBE_DEFAULT);
}

static int
gen_ahci_attach(device_t dev)
{
	struct gen_ahci_sc *sc;
	struct ahci_controller *ctlr;
	phandle_t node;
	int rv;
//	int	error, i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	ctlr = &sc->ctlr;
	node = ofw_bus_get_node(dev);

	ctlr->r_rid = 0;
	ctlr->r_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &ctlr->r_rid, RF_ACTIVE);
	if (ctlr->r_mem == NULL)
		return (ENXIO);


	rv = get_fdt_resources(sc, node);
	if (rv != 0) {
		device_printf(sc->dev, "Failed to allocate FDT resource(s)\n");
		goto fail;
	}

	rv = enable_fdt_resources(sc);
	if (rv != 0) {
		device_printf(sc->dev, "Failed to enable FDT resource(s)\n");
		goto fail;
	}

//	ctlr->quirks =  AHCI_Q_FORCE_PI | AHCI_Q_RESTORE_CAP | AHCI_Q_EDGEIS;

//	ctlr->vendorid = pci_get_vendor(dev);
//	ctlr->deviceid = pci_get_device(dev);
//	ctlr->subvendorid = pci_get_subvendor(dev);
//	ctlr->subdeviceid = pci_get_subdevice(dev);

	/* Setup controller defaults. */
	ctlr->msi = 0;
	ctlr->numirqs = 1;
	ctlr->ccc = 1;
	sc->port_mask = 1;

	/* Reset controller */
	if ((rv = gen_ahci_ctlr_reset(dev)) != 0) {
		goto fail;
	}

	rv = ahci_attach(dev);
	return (rv);

fail:
	/* XXX FDT  stuff */
	if (ctlr->r_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, ctlr->r_rid,
		    ctlr->r_mem);
	return (rv);
}

static int
gen_ahci_detach(device_t dev)
{

	ahci_detach(dev);
	return (0);
}

static int
gen_ahci_suspend(device_t dev)
{
	struct gen_ahci_sc *sc = device_get_softc(dev);

	bus_generic_suspend(dev);
	/* Disable interupts, so the state change(s) doesn't trigger */
	ATA_OUTL(sc->ctlr.r_mem, AHCI_GHC,
	     ATA_INL(sc->ctlr.r_mem, AHCI_GHC) & (~AHCI_GHC_IE));
	return (0);
}

static int
gen_ahci_resume(device_t dev)
{
	int res;

	if ((res = gen_ahci_ctlr_reset(dev)) != 0)
		return (res);
	ahci_ctlr_setup(dev);
	return (bus_generic_resume(dev));
}

devclass_t genahci_devclass;
static device_method_t genahci_methods[] = {
	DEVMETHOD(device_probe,		gen_ahci_probe),
	DEVMETHOD(device_attach,	gen_ahci_attach),
	DEVMETHOD(device_detach,	gen_ahci_detach),
	DEVMETHOD(device_suspend,	gen_ahci_suspend),
	DEVMETHOD(device_resume,	gen_ahci_resume),
	DEVMETHOD(bus_print_child,	ahci_print_child),
	DEVMETHOD(bus_alloc_resource,	ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,	ahci_release_resource),
	DEVMETHOD(bus_setup_intr,	ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,	ahci_teardown_intr),
	DEVMETHOD(bus_child_location_str, ahci_child_location_str),
	DEVMETHOD(bus_get_dma_tag,	ahci_get_dma_tag),

	DEVMETHOD_END
};
static driver_t genahci_driver = {
	"ahci",
	genahci_methods,
	sizeof(struct gen_ahci_sc)
};
DRIVER_MODULE(genahci, simplebus, genahci_driver, genahci_devclass, NULL, NULL);
