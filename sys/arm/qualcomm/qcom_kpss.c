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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/clk/clk_mux.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#define APCS_CPU_AUX_CLK_SEL	0x14
#define APCS_L2_AUX_CLK_SEL	0x28


#define	TYPE_KPSS_ACC1		1
#define	TYPE_KPSS_GCC		2
static struct ofw_compat_data compat_data[] = {
	{"qcom,kpss-acc-v1",	TYPE_KPSS_ACC1},
	{"qcom,kpss-gcc", 	TYPE_KPSS_GCC},
	{NULL,		 	0},
};

struct qcom_kpss_softc {
	device_t		dev;
	struct resource *	mem_res;
	struct mtx		mtx;
	struct clkdom 		*clkdom;
	int			type;
};
#define PLIST(x) static const char *x[]

static const char *mux_map[] = {"pxo", NULL, NULL, "pll8_vote"};
static struct clk_mux_def clk_mux;

static int
qcom_kpss_clkdev_read(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct qcom_kpss_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	*val = bus_read_4(sc->mem_res, addr);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
qcom_kpss_clkdev_write(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct qcom_kpss_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	bus_write_4(sc->mem_res, addr, val);
	mtx_unlock(&sc->mtx);
	return (0);
}


static int
qcom_kpss_clkdev_modify(device_t dev, bus_addr_t addr, uint32_t clear_mask,
    uint32_t set_mask)
{
	struct qcom_kpss_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	reg = bus_read_4(sc->mem_res, addr);
	reg &= clear_mask;
	reg |= set_mask;
	bus_write_4(sc->mem_res, addr, reg);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
register_clocks(struct qcom_kpss_softc *sc)
{
	phandle_t node;
	char *tmpstr;
	int rv;

	node = ofw_bus_get_node(sc->dev);

	sc->clkdom = clkdom_create(sc->dev);
	if (sc->clkdom == NULL) {
		device_printf(sc->dev, "Cannot create clock domain.\n");
		return (ENXIO);
	}
	if (sc->type == TYPE_KPSS_ACC1) {
		rv = OF_getprop_alloc(node, "clock-output-names", 1,
		    (void **)&tmpstr);
		if (rv <= 0) {
			device_printf(sc->dev,
			    "Missing \"clock-output-names\" property.\n");
			return (ENXIO);
		}
		clk_mux.clkdef.name = tmpstr;
		clk_mux.offset = APCS_CPU_AUX_CLK_SEL;
	} else {
		clk_mux.clkdef.name = "acpu_l2_aux";
		clk_mux.offset = APCS_L2_AUX_CLK_SEL;
	}
	clk_mux.clkdef.parent_names = mux_map;
	clk_mux.clkdef.parents_num = nitems(mux_map);
	clk_mux.shift = 0;
	clk_mux.width = 2;

	rv = clknode_mux_register(sc->clkdom, &clk_mux);
	if (rv != 0) {
 		device_printf(sc->dev, "Cannot register clock mux.\n");
		free(tmpstr, M_OFWPROP);
		return (ENXIO);
	}

	clkdom_finit(sc->clkdom);
	free(tmpstr, M_OFWPROP);
	return (0);
}

static int
qcom_kpss_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm KPSS clock");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
qcom_kpss_attach(device_t dev)
{
	struct qcom_kpss_softc *sc;
	int rid, rv;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	/* Resource setup. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!sc->mem_res) {
		device_printf(dev, "Cannot allocate memory resource\n");
		rv = ENXIO;
		goto fail;
	}

	rv = register_clocks(sc);
	if (rv != 0) {
		device_printf(dev, "Cannot registers clock(s)\n");
		rv = ENXIO;
		goto fail;
	}

	return (0);

fail:
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (rv);
}

static int
qcom_kpss_detach(device_t dev)
{

	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t qcom_kpss_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_kpss_probe),
	DEVMETHOD(device_attach,	qcom_kpss_attach),
	DEVMETHOD(device_detach,	qcom_kpss_detach),
	/* clkdev  interface*/
	DEVMETHOD(clkdev_read,		qcom_kpss_clkdev_read),
	DEVMETHOD(clkdev_write,		qcom_kpss_clkdev_write),
	DEVMETHOD(clkdev_modify,	qcom_kpss_clkdev_modify),
	DEVMETHOD_END
};

static devclass_t qcom_kpss_devclass;

static driver_t qcom_kpss_driver = {
	"qcom_kpss",
	qcom_kpss_methods,
	sizeof(struct qcom_kpss_softc),
};

EARLY_DRIVER_MODULE(qcom_kpss, simplebus, qcom_kpss_driver,
    qcom_kpss_devclass, 0, 0, BUS_PASS_TIMER+1);

