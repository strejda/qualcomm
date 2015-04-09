/*-
 * Copyright (c) 2015 Michal Meloun <meloun@miracle.cz>
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
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>


#include <dev/clk/clk.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>

struct amba_softc {
	struct simplebus_softc simplebus_sc;
	clk_t pclk;
};

static struct ofw_compat_data compat_data[] = {
	{"arm,amba-bus", 	1},
	{NULL,			0}
};

/* XXX Export this function fro simplebus */
static int
simplebus_fill_ranges(phandle_t node, struct simplebus_softc *sc)
{
	int host_address_cells;
	cell_t *base_ranges;
	ssize_t nbase_ranges;
	int err;
	int i, j, k;

	err = OF_searchencprop(OF_parent(node), "#address-cells",
	    &host_address_cells, sizeof(host_address_cells));
	if (err <= 0)
		return (-1);

	nbase_ranges = OF_getproplen(node, "ranges");
	if (nbase_ranges < 0)
		return (-1);
	sc->nranges = nbase_ranges / sizeof(cell_t) /
	    (sc->acells + host_address_cells + sc->scells);
	if (sc->nranges == 0)
		return (0);

	sc->ranges = malloc(sc->nranges * sizeof(sc->ranges[0]),
	    M_DEVBUF, M_WAITOK);
	base_ranges = malloc(nbase_ranges, M_DEVBUF, M_WAITOK);
	OF_getencprop(node, "ranges", base_ranges, nbase_ranges);

	for (i = 0, j = 0; i < sc->nranges; i++) {
		sc->ranges[i].bus = 0;
		for (k = 0; k < sc->acells; k++) {
			sc->ranges[i].bus <<= 32;
			sc->ranges[i].bus |= base_ranges[j++];
		}
		sc->ranges[i].host = 0;
		for (k = 0; k < host_address_cells; k++) {
			sc->ranges[i].host <<= 32;
			sc->ranges[i].host |= base_ranges[j++];
		}
		sc->ranges[i].size = 0;
		for (k = 0; k < sc->scells; k++) {
			sc->ranges[i].size <<= 32;
			sc->ranges[i].size |= base_ranges[j++];
		}
	}

	free(base_ranges, M_DEVBUF);
	return (sc->nranges);
}

static int
amba_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "ARM ABMA Bus");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
amba_attach(device_t dev)
{
	struct		amba_softc *sc;
	phandle_t	node;
	int		rv;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	/* Enable bus clock */
	rv = clk_get_by_ofw_name(node, "apb_pclk", &sc->pclk);
	if (rv == 0) {
		rv = clk_enable(sc->pclk);
		if (rv != 0) {
			device_printf(dev, "Cannot enable AMBA bus clock: %d\n",
			    rv);
			goto fail;
		}
	}
	simplebus_init(dev, 0);

	if (simplebus_fill_ranges(node, &sc->simplebus_sc) < 0)
		device_printf(dev, "missing \"ranges\" property\n");

	for (node = OF_child(node); node > 0; node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);
	return (bus_generic_attach(dev));
fail:
	if (sc->pclk != NULL)
		clk_release(sc->pclk);
	return (ENXIO);
}

static int
amba_detach(device_t dev)
{
	struct		amba_softc *sc;

	sc = device_get_softc(dev);

	if (sc->pclk != NULL)
		clk_release(sc->pclk);
	sc->pclk = NULL;
	return (0);
}

static device_method_t amba_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		amba_probe),
	DEVMETHOD(device_attach,	amba_attach),
	DEVMETHOD(device_detach,	amba_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(amba, amba_driver, amba_methods,
    sizeof(struct amba_softc), simplebus_driver);
static devclass_t amba_devclass;
EARLY_DRIVER_MODULE(amba, simplebus, amba_driver, amba_devclass, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(amba, 1);
