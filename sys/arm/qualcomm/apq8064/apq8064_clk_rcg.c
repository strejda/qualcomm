/*-
 * Copyright (c) 2014 Michal Meloun <meloun@miracle.cz>
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
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <dev/clk/clk.h>

#include "apq8064_clk.h"


/*
 * Root clock generator
 *
 * M/N:D counter
 * input multiplexer
 * enable gate
 *
 */

/* APPS MD register bits */
#define RCG_MD_M_VAL_SHIFT		16	/* M */
#define RCG_MD_D_VAL_SHIFT		0	/* NOT(2 * D) */

/* APPS NS register bits */
#define RCG_NS_N_VAL_SHIFT		16	/* NOT(N - M) */
#define RCG_NS_CLK_ROOT_ENA		(1 << 11)
#define RCG_NS_CLK_INV			(1 << 10)
#define RCG_NS_CLK_BRANCH_ENA		(1 << 9)
#define RCG_NS_MNCNTR_EN		(1 << 8)
#define RCG_NS_MNCNTR_RST		(1 << 7)
#define RCG_NS_MNCNTR_MODE_BYPASS 	0
#define RCG_NS_MNCNTR_MODE_SWALLOW 	1
#define RCG_NS_MNCNTR_MODE_DUAL_EDGE 	2
#define RCG_NS_MNCNTR_MODE_SINGLE_EDGE 	3
#define RCG_NS_MNCNTR_MODE_MASK 	0x3
#define RCG_NS_MNCNTR_MODE_SHIFT	5
#define RCG_NS_PRE_DIV_SEL_MASK		0x3
#define RCG_NS_PRE_DIV_SEL_SHIFT	0x3
#define RCG_NS_SRC_SEL_MASK		0x7
#define RCG_NS_SRC_SEL_SHIFT		0


struct rcg_sc {
	struct mtx	*mtx;
	struct resource	*mem_res;
	uint32_t	md_reg;
	uint32_t	ns_reg;
	uint32_t	mask;
};

static int
apg8064_rcg_recalc(struct clknode *clk, uint64_t *freq)
{
	struct rcg_sc *sc;
	uint32_t ns, md;

	uint32_t pre_div, mode, m, n;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	ns = RD4(sc, sc->ns_reg);
	if (sc->md_reg != 0)
		md = RD4(sc, sc->md_reg);
	DEVICE_UNLOCK(sc);

	/* No divider */
	if (sc->md_reg == 0)
		return (0);

	pre_div = (ns >> RCG_NS_PRE_DIV_SEL_SHIFT) & RCG_NS_PRE_DIV_SEL_MASK;

	/* Get m and n values */
	m = (md >> RCG_MD_M_VAL_SHIFT) & sc->mask;
	n = (~ns >> RCG_NS_N_VAL_SHIFT) & sc->mask;
	n += m;
	mode = (ns >> RCG_NS_MNCNTR_MODE_SHIFT) & RCG_NS_MNCNTR_MODE_MASK;
	*freq /= pre_div + 1;
#if 0
if (mode != 0) {
	int d = ((~md >> RCG_MD_D_VAL_SHIFT) & sc->mask);
printf("  md: 0x%08X (0x%08X), ns: 0x%08X (0x%08X), mask: 0x%08X \n", md, sc->md_reg, ns, sc->ns_reg, sc->mask);
printf("  m: 0x%08X, n: 0x%08X, d: 0x%08X, pre_div: 0x%08X,  mode: 0x%08X\n", m, n, d, pre_div, mode);
printf("  m: %d, n: %d, pre_div: %d,  mode: %d, mn ena: %d, rst: %d, bena: %d, rena: %d\n", m, n, pre_div, mode, !!(ns & RCG_NS_MNCNTR_EN), !!(ns & RCG_NS_MNCNTR_RST),  !!(ns & RCG_NS_CLK_BRANCH_ENA), !!(ns & RCG_NS_CLK_ROOT_ENA));
}
#endif
	switch (mode) {
	case RCG_NS_MNCNTR_MODE_BYPASS:
		break;

	case RCG_NS_MNCNTR_MODE_SWALLOW:
	case RCG_NS_MNCNTR_MODE_DUAL_EDGE:
	case RCG_NS_MNCNTR_MODE_SINGLE_EDGE:
		*freq *= m;
		*freq /= n;
		break;
	}
	return (0);
}

static int
apg8064_rcg_init(struct clknode *clk, device_t dev)
{
	struct rcg_sc *sc;
	uint32_t	ns;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	ns = RD4(sc, sc->ns_reg);
	DEVICE_UNLOCK(sc);

	clknode_init_parent_idx(clk,
	    (ns >> RCG_NS_SRC_SEL_SHIFT) & RCG_NS_SRC_SEL_MASK);
	return(0);
}

static int
apg8064_rcg_set_mux(struct clknode *clk, int idx)
{
	struct rcg_sc *sc;
	uint32_t	ns;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	ns = RD4(sc, sc->ns_reg);
	ns &= ~(RCG_NS_SRC_SEL_MASK << RCG_NS_SRC_SEL_SHIFT);
	ns |= (idx & RCG_NS_SRC_SEL_MASK) << RCG_NS_SRC_SEL_SHIFT;
	WR4(sc, sc->ns_reg, ns);
	DEVICE_UNLOCK(sc);
	return (0);
}


/* XXXX Minimal implementation  - must be fixed  */
static int
apg8064_rcg_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
   uint64_t *prec, int *stop)
{
	struct rcg_sc *sc;
	uint32_t n, m, rn, rm, rd, ns, md, prediv;

	sc = clknode_get_softc(clk);

	/* Integer only divider  */
	if (sc->md_reg == 0) {
		/* XXX not now */
		*stop = 1;
		return (1);
	}

	/* XXX checks here */
	if (fin < *fout)
		*fout = fin;

	/* Compute divider */
	m = *fout;
	n = fin;
	prediv = 0;
	while (((m & 1) == 0) && ((m & 1) == 0)) {
		m >>= 1;
		n >>= 1;
	}
	while (((m & ~sc->mask) != 0) || ((n & ~sc->mask) != 0)) {
		m >>= 1;
		n >>= 1;
	}
	if (((fin * m) / n) != *fout)
		n++;
	while (((m & ~sc->mask) != 0) || ((n & ~sc->mask) != 0)) {
		m >>= 1;
		n >>= 1;
	}

	/* Transform to hw values */
	rd = ~n;
	rm = m;
	rn = ~(n - m);

	/* Write to hw */
	DEVICE_LOCK(sc);
	ns = RD4(sc, sc->ns_reg);
	WR4(sc, sc->ns_reg, ns | RCG_NS_MNCNTR_RST);
	ns &= ~(RCG_NS_PRE_DIV_SEL_MASK << RCG_NS_PRE_DIV_SEL_SHIFT);
	ns &= ~(sc->mask << RCG_NS_N_VAL_SHIFT);
	ns &= ~(RCG_NS_MNCNTR_MODE_MASK << RCG_NS_MNCNTR_MODE_SHIFT);

	md = RD4(sc, sc->md_reg);
	md &= ~(sc->mask << RCG_MD_M_VAL_SHIFT);
	md &= ~(sc->mask << RCG_MD_D_VAL_SHIFT);

	ns |= prediv << RCG_NS_PRE_DIV_SEL_SHIFT;
	ns |= rn << RCG_NS_N_VAL_SHIFT;
	ns |= RCG_NS_MNCNTR_MODE_DUAL_EDGE << RCG_NS_MNCNTR_MODE_SHIFT;
	ns |= RCG_NS_MNCNTR_EN;
	ns |= RCG_NS_CLK_ROOT_ENA;

	md |= (rm  & sc->mask) << RCG_MD_M_VAL_SHIFT;
	md |= (rd  & sc->mask) << RCG_MD_D_VAL_SHIFT;
	WR4(sc, sc->md_reg, md);
	WR4(sc, sc->ns_reg, ns| RCG_NS_MNCNTR_RST);
	WR4(sc, sc->ns_reg, ns);
	DEVICE_UNLOCK(sc);

	*fout = fin * m;
	*fout /= n;
	*stop = 1;
	return (0);
}

static clknode_method_t apg8064_rcg_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		apg8064_rcg_init),
	CLKNODEMETHOD(clknode_recalc_freq,	apg8064_rcg_recalc),
	CLKNODEMETHOD(clknode_set_freq,		apg8064_rcg_set_freq),
	CLKNODEMETHOD(clknode_set_mux,		apg8064_rcg_set_mux),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(apg8064_rcg, apg8064_rcg_class, apg8064_rcg_methods,
   sizeof(struct rcg_sc), clknode_class);

int
apg8064_rcg_register(struct clkdom *clkdom, struct clk_rcg_def *clkdef,
    struct mtx *dev_mtx, struct resource *mem_res)
{
	struct clknode *clk;
	struct rcg_sc *sc;

	clk = clknode_create(clkdom, &apg8064_rcg_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->mtx = dev_mtx;
	sc->mem_res = mem_res;

	sc->md_reg = clkdef->md_reg;
	sc->ns_reg = clkdef->ns_reg;
	sc->mask = clkdef->mask;
	clknode_register(clkdom, clk);
	return (0);
}
