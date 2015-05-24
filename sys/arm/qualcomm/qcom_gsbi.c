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
 *
 * $FreeBSD$
 */

/*
 * Qualcomm GSBI driver
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>


#define GSBI_CTRL_REG		0x0000
#define GSBI_PROTOCOL_SHIFT	4
#define GSBI_PROTOCOL_MASK	0x0F
#define GSBI_CRCI_SHIFT		0
#define GSBI_CRCI_MASK		0x01

#define	WR4(_sc, _r, _v)						\
	    bus_write_4((_sc)->mem_res, (_r), (_v))

struct qcom_gsbi_softc {
	struct simplebus_softc simplebus_sc; /* Must be first */
	device_t		dev;
	struct resource		*mem_res;
	clk_t			clk;
	uint32_t		mode;
	uint32_t		crci;

};

static struct ofw_compat_data compat_data[] = {
	{"qcom,gsbi-v1.0.0", 	1},
	{NULL,			0}
};

static int
qcom_gsbi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm GSBI");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
qcom_gsbi_detach(device_t dev)
{
	struct qcom_gsbi_softc *sc;

	sc = device_get_softc(dev);

	if (sc->clk != NULL)
		clk_release(sc->clk);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return(0);
}

static int
qcom_gsbi_attach(device_t dev)
{
	struct qcom_gsbi_softc *sc;
	int rid, rv;
	phandle_t	node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	rv = OF_getencprop(node, "qcom,mode", &sc->mode, sizeof(sc->mode));
	if (rv <= 0) {
		device_printf(dev, "Cannot get mode property: %d\n", rv);
		return (ENXIO);
	}
	rv = OF_getencprop(node, "qcom,crci", &sc->crci, sizeof(sc->crci));
	if (rv <= 0)
		sc->crci = 0;

	/* Allocate bus_space resources. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		qcom_gsbi_detach(dev);
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(node, "iface", &sc->clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get interface clock: %d\n", rv);
		qcom_gsbi_detach(dev);
		return (ENXIO);
	}
	rv = clk_enable(sc->clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable interface clock: %d\n", rv);
		qcom_gsbi_detach(dev);
		return (ENXIO);
	}
	WR4(sc, GSBI_CTRL_REG,
	    ((sc->mode & GSBI_PROTOCOL_MASK) << GSBI_PROTOCOL_SHIFT) |
	    ((sc->crci & GSBI_CRCI_MASK) << GSBI_CRCI_MASK));

	simplebus_init(dev, 0);
	for (node = OF_child(node); node > 0;
	    node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);
	return (bus_generic_attach(dev));
}

static device_method_t qcom_gsbi_methods[] = {
	DEVMETHOD(device_probe,		qcom_gsbi_probe),
	DEVMETHOD(device_attach,	qcom_gsbi_attach),
	DEVMETHOD(device_detach,	qcom_gsbi_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(qcom_gsbi, qcom_gsbi_driver, qcom_gsbi_methods,
    sizeof(struct qcom_gsbi_softc), simplebus_driver);
static devclass_t qcom_gsbi_devclass;
DRIVER_MODULE(qcom_gsbi, simplebus,  qcom_gsbi_driver, qcom_gsbi_devclass,
    0, 0);
MODULE_VERSION(qcom_gsb, 1);
