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

#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_mux.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/cpu.h>
/*
 * Pictrure taken from post in linux kernel list
 *
 *				 secondary
 *	 +-----+                    +
 *	 | QSB |-------+------------|\
 *	 +-----+       |            | |-+
 *		       |    +-------|/  |
 *		       |    |       +   |
 *	 +------+      |    |           |
 *	 | PLL8 |---+-------+           |   primary
 *	 +------+   |  |                |     +
 *		    |  |                +-----|\       +------+
 *	 +-------+  |  |                      | \      |      |
 *	 | HFPLL |----------+-----------------|  |-----| CPU0 |
 *	 +-------+  |  |    |                 |  |     |      |
 *		    |  |    | +-----+         | /      +------+
 *		    |  |    +-| / 2 |---------|/
 *		    |  |      +-----+         +
 *		    |  |         secondary
 *		    |  |            +
 *		    |  +------------|\
 *		    |               | |-+
 *		    +---------------|/  |   primary
 *				    +   |     +
 *					+-----|\       +------+
 *	 +-------+                            | \      |      |
 *	 | HFPLL |----------------------------|  |-----| CPU1 |
 *	 +-------+          |                 |  |     |      |
 *			    | +-----+         | /      +------+
 *			    +-| / 2 |---------|/
 *			      +-----+         +
 *
 *
 */
#define DUMMY_CLK_ID		9999
#define SOC_CORES		4

#define	TYPE_KRAIT_CC_V1	1
#define	TYPE_KRAIT_CC_V2	2
static struct ofw_compat_data compat_data[] = {
	{"qcom,krait-cc-v1",	TYPE_KRAIT_CC_V1},
/*	{"qcom,krait-cc-v2", 	TYPE_KRAIT_CC_V2}, */
	{NULL,		 	0},
};

struct qcom_krait_cc_softc {
	device_t		dev;
	struct mtx		mtx;
	struct clkdom 		*clkdom;
	int			type;
};

#define PLIST(x) static const char *x[]
#define FRATE(_id, cname, _freq)					\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parents_num = 0,					\
	.clkdef.flags = CLK_FLAGS_STATIC,				\
	.freq = _freq,							\

/* Fake QSB clock */
static struct clk_fixed_def qsb_clk = {
	FRATE(DUMMY_CLK_ID, "qsb", 1)
};

/* Temporary L2_AUX clock */
static struct clk_fixed_def acpu_l2_aux_clk = {
	FRATE(DUMMY_CLK_ID, "acpu_l2_aux", 1)
};

/* Muxes */
PLIST(pri_mux_srcs) = {NULL, NULL, NULL, NULL};
static struct clk_mux_def pri_mux;
PLIST(sec_mux_srcs) = {NULL, NULL, NULL, NULL};
struct clk_mux_def sec_mux;
PLIST(div_srcs) = {NULL};
static struct clk_div_def pll_div;


static void
set_cp15_l2_reg(uint32_t addr, uint32_t val)
{

	/* l2cpselr */
	__asm volatile ("mcr p15, 3, %0, c15, c0, 6" : : "r" (addr));
	isb();
	/* l2cpdr" */
	__asm volatile ("mcr p15, 3, %0, c15, c0, 7" : : "r" (val));
	isb();
//printf("%s: addr: 0x%08X val: 0x%08X\n", __func__, addr, val);
}

static uint32_t
get_cp15_l2_reg(uint32_t addr)
{
	uint32_t val;

	/* l2cpselr */
	__asm volatile ("mcr p15, 3, %0, c15, c0, 6" : : "r" (addr));
	isb();
	/* l2cpdr" */
	__asm volatile ("mrc p15, 3, %0, c15, c0, 7" : "=r" (val));
//printf("%s: addr: 0x%08X val: 0x%08X\n", __func__, addr, val);
	return (val);
}

static int
register_pri_mux(struct qcom_krait_cc_softc *sc, int id, char *name,
   bus_addr_t offset)
{
	char mux_name[32];
	char mux_src1[32];
	char mux_src2[32];
	char mux_src3[32];

	sprintf(mux_name, "krait%s_pri_mux", name);
	sprintf(mux_src1, "krait%s_sec_mux", name);
	sprintf(mux_src2, "hfpll%s", name);
	sprintf(mux_src3, "hfpll%s_div", name);
	pri_mux_srcs[0] = mux_src1;
	pri_mux_srcs[1] = mux_src2;
	pri_mux_srcs[2] = mux_src3;

	pri_mux.clkdef.id = id;
	pri_mux.clkdef.name = mux_name;
	pri_mux.clkdef.parent_names = pri_mux_srcs;
	pri_mux.clkdef.parents_num = nitems(pri_mux_srcs);
	pri_mux.offset = offset;
	pri_mux.offset = offset;
	pri_mux.width = 2;
	pri_mux.shift = 0;

	return (clknode_mux_register(sc->clkdom, &pri_mux));
}

static int
register_sec_mux(struct qcom_krait_cc_softc *sc, int id, char *name,
   bus_addr_t offset)
{
	char mux_name[32];
	char mux_src1[32];

	sprintf(mux_name, "krait%s_sec_mux", name);
	sprintf(mux_src1, "acpu%s_aux", name);
	sec_mux_srcs[0] = "qsb";
	sec_mux_srcs[2] = mux_src1;

	sec_mux.clkdef.id = DUMMY_CLK_ID;
	sec_mux.clkdef.name = mux_name;
	sec_mux.clkdef.parent_names = sec_mux_srcs;
	sec_mux.clkdef.parents_num = nitems(sec_mux_srcs);
	sec_mux.offset = offset;
	sec_mux.width = 2;
	sec_mux.shift = 2;
	return ( clknode_mux_register(sc->clkdom, &sec_mux));
}

static int
register_divider(struct qcom_krait_cc_softc *sc, int id, char *name,
   bus_addr_t offset)
{
	char div_name[32];
	char div_src[32];

	sprintf(div_src, "hfpll%s", name);
	sprintf(div_name, "hfpll%s_div", name);
	div_srcs[0] = div_src;

	pll_div.clkdef.id = DUMMY_CLK_ID;
	pll_div.clkdef.name = div_name;
	pll_div.clkdef.parent_names = div_srcs;
	pll_div.clkdef.parents_num =  nitems(div_srcs);
	pll_div.offset = offset;
	pll_div.i_width = 2;
	pll_div.i_shift = 6;
	return(clknode_div_register(sc->clkdom, &pll_div));
}

static int
register_clk(struct qcom_krait_cc_softc *sc, int id, char *name,
   bus_addr_t offset)
{
	int rv;

	rv = register_pri_mux(sc, id, name, offset);
	if (rv != 0)
		return (rv);
	rv = register_sec_mux(sc, id, name, offset);
	if (rv != 0)
		return (rv);
	rv = register_divider(sc, id, name, offset);
	if (rv != 0)
		return (rv);

	return (0);
}


static int
fixup_clk(struct qcom_krait_cc_softc *sc, char *name, uint64_t freq)
{
	int rv;
	char pri_mux_name[32];
	char sec_mux_name[32];
	char hfpll_name[32];
	struct clknode *clknode;

	/* Switch to safe mode */
	sprintf(pri_mux_name, "krait%s_pri_mux", name);
	sprintf(sec_mux_name, "krait%s_sec_mux", name);
	sprintf(hfpll_name, "hfpll%s", name);

	/* Switch to safe slow clock first */
	clknode = clknode_find_by_name(sc->clkdom, sec_mux_name);
	KASSERT(clknode != NULL, ("%s", sec_mux_name));
	rv = clknode_set_parent_by_name(clknode, "qsb");

	clknode = clknode_find_by_name(sc->clkdom, pri_mux_name);
	KASSERT(clknode != NULL, ("%s", pri_mux_name));
	rv = clknode_set_parent_by_name(clknode, sec_mux_name);

	/* Set HFPLL to base startup frequency and enable it  */
	clknode = clknode_find_by_name(sc->clkdom, hfpll_name);
	KASSERT(clknode != NULL, ("%s", hfpll_name));
	rv = clknode_set_freq(clknode, freq, 1, 1);
	rv = clknode_enable(clknode);

	/* And switch to HFPLL */
	clknode = clknode_find_by_name(sc->clkdom, pri_mux_name);
	KASSERT(clknode != NULL, ("%s", pri_mux_name));
	rv = clknode_set_parent_by_name(clknode, hfpll_name);

	return (0);
}

static int
register_clocks(struct qcom_krait_cc_softc *sc)
{
	phandle_t node;
	char cpuname[3];
	int i, rv;

	node = ofw_bus_get_node(sc->dev);

	sc->clkdom = clkdom_create(sc->dev);
	if (sc->clkdom == NULL) {
		device_printf(sc->dev, "Cannot create clock domain.\n");
		return (ENXIO);
	}

	rv = clknode_fixed_register(sc->clkdom, &qsb_clk);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot register QSB clock.\n");
		return (rv);
	}

	/* XXX FIXME: how to limit CPU count here ?, mp_ncpus ? */
	for (i = 0; i < SOC_CORES; i++) {
		sprintf(cpuname, "%d", i);
		rv = register_clk(sc, i, cpuname, 0x4501 + i * 0x1000);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot register CPU clock.\n");
			return (rv);
		}
	}

	/* See DTS file - AUX ACC is in collision with syscon */
	rv = clknode_fixed_register(sc->clkdom, &acpu_l2_aux_clk);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot register APU_L2_AUX clock.\n");
		return (rv);
	}
	/* L2 apps clock */
	rv = register_clk(sc, -1, "_l2", 0x500);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot register L2 apps clock.\n");
		return (rv);
	}

	clkdom_finit(sc->clkdom);
	fixup_clk(sc, "_l2", 1188000000);
	for (i = 1; i < SOC_CORES; i++) {
		sprintf(cpuname, "%d", i);
		rv = fixup_clk(sc, cpuname, 918000000);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot fixup CPU clock.\n");
			return (rv);
		}
	}
	clkdom_finit(sc->clkdom);

	return (0);
}

static int
qcom_krait_cc_clkdev_read(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct qcom_krait_cc_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	*val = get_cp15_l2_reg(addr);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
qcom_krait_cc_clkdev_write(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct qcom_krait_cc_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	set_cp15_l2_reg(addr, val);
	mtx_unlock(&sc->mtx);
	return (0);
}


static int
qcom_krait_cc_clkdev_modify(device_t dev, bus_addr_t addr, uint32_t clear_mask,
    uint32_t set_mask)
{
	struct qcom_krait_cc_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	reg = get_cp15_l2_reg(addr);
	reg &= clear_mask;
	reg |= set_mask;
	set_cp15_l2_reg(addr, reg);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
qcom_krait_cc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm CPU clock");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
qcom_krait_cc_attach(device_t dev)
{
	struct qcom_krait_cc_softc *sc;
	int rv;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;


	rv = register_clocks(sc);
	if (rv != 0) {
		device_printf(dev, "Cannot registers clock(s)\n");
		return (ENXIO);
	}

	return (0);
}

static int
qcom_krait_cc_detach(device_t dev)
{

	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t qcom_krait_cc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_krait_cc_probe),
	DEVMETHOD(device_attach,	qcom_krait_cc_attach),
	DEVMETHOD(device_detach,	qcom_krait_cc_detach),
	/* clkdev  interface*/
	DEVMETHOD(clkdev_read,		qcom_krait_cc_clkdev_read),
	DEVMETHOD(clkdev_write,		qcom_krait_cc_clkdev_write),
	DEVMETHOD(clkdev_modify,	qcom_krait_cc_clkdev_modify),


	DEVMETHOD_END
};

static devclass_t qcom_krait_cc_devclass;

static driver_t qcom_krait_cc_driver = {
	"qcom_krait_cc",
	qcom_krait_cc_methods,
	sizeof(struct qcom_krait_cc_softc),
};

EARLY_DRIVER_MODULE(qcom_krait_cc, simplebus, qcom_krait_cc_driver,
    qcom_krait_cc_devclass, 0, 0, BUS_PASS_TIMER + 2);

