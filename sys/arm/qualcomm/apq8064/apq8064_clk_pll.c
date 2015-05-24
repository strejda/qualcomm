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



/* PLL MODE register bits */
#define PLL_VOTE_FSM_RESET	(1 << 21)
#define PLL_VOTE_FSM_ENA	(1 << 20)
#define PLL_BIAS_COUNT_SHIFT	14
#define PLL_BIAS_COUNT_MASK	0x3f
#define PLL_LOCK_COUNT_SHIFT	8
#define PLL_LOCK_COUNT_MASK	0x3f
#define PLL_REF_XO_SEL_SHIFT	4
#define PLL_REF_XO_SEL_MASK	0x03
#define PLL_PLLTEST		(1 << 3)
#define PLL_RESET_N		(1 << 2)
#define PLL_BYPASSNL		(1 << 1)
#define PLL_OUTCTRL		(1 << 0)

/* PLL CONFIG register bits */
#define EARLY_OUT_ENA		(1 << 27)
#define CLK33_OUT_SEL		(1 << 26)
#define CLK33_OUT_ENA		(1 << 25)
#define OUT_SEL			(1 << 24)
#define MAIN_OUT_ENA		(1 << 23)
#define MN_ACCUM_ENA		(1 << 22)
#define PLLOUT_DIVIDE_SHIFT	20
#define PLLOUT_DIVIDE_MASK	0x03
#define PRE_DIVIDE		(1 << 19)
#define PLLOUT_VCO_SHIFT	16
#define PLLOUT_VCO_MASK		0x03

/* PLL STATUS register bits */
#define PLL_ACTIVE_FLAG		(1 << 16)
#define PLL_D_SHIFT		0
#define PLL_D_MASK		0xffff

struct pll_sc {
	struct clknode	*clknode;
	uint32_t	l_reg;
	uint32_t	m_reg;
	uint32_t	n_reg;
	uint32_t	cfg_reg;
	uint32_t	mode_reg;
	uint32_t	status_reg;
	uint32_t	status_mask;
	uint32_t	post_div_mask;
	uint32_t	post_div_shift;
	const struct pll_freq_tbl *freq_tbl;
};

static const
struct pll_freq_tbl *find_freq(const struct pll_freq_tbl *f, unsigned long rate)
{
	if (!f)
		return NULL;

	for (; f->freq; f++)
		if (rate <= f->freq)
			return f;

	return NULL;
}

static void
clk_pll_enable(struct pll_sc *sc)
{
	uint32_t mode;

	mode = RD4(sc, sc->mode_reg);
printf("%s: PLL mode: 0x%08X(0x%08X), lock: 0x%08X (0x%08X)\n", __func__, mode, sc->mode_reg, RD4(sc, 0x3420) & (1 << 3), RD4(sc, 0x3420));
	/* Do nothing if PLL is already enabed */
	if ((mode & (PLL_OUTCTRL | PLL_RESET_N | PLL_BYPASSNL)) ==
	    (PLL_OUTCTRL | PLL_RESET_N | PLL_BYPASSNL))
		return;
	/* Do nothing if PLL is controlled by FSM voting */
	if (mode & PLL_VOTE_FSM_ENA)
		return;

	/* Disable bypass first */
	mode |= PLL_BYPASSNL;
 	WR4(sc, sc->mode_reg, mode);
	DELAY(10);

	/* Remove PLL form reset */
	mode |= PLL_RESET_N;
 	WR4(sc, sc->mode_reg, mode);
	DELAY(50);

	/* Finally, enable output */
	mode |= PLL_OUTCTRL;
 	WR4(sc, sc->mode_reg, mode);

DELAY(50);
 printf("%s: PLL mode: 0x%08X, lock: 0x%08X (0x%08X)\n", __func__, mode, RD4(sc, 0x3420) & (1 << 3), RD4(sc, 0x3420));

}

static void
clk_pll_disable(struct pll_sc *sc)
{
	uint32_t mode;

	mode = RD4(sc, sc->mode_reg);
	/* Do nothing if PLL is controlled by FSM voting */
	if (mode & PLL_VOTE_FSM_ENA)
		return;

	/* Disable output first */
	mode &= ~PLL_OUTCTRL;
	WR4(sc, sc->mode_reg, mode);
	DELAY(10);

	/* Then set bypass and reset */
	mode &= ~(PLL_RESET_N | PLL_BYPASSNL);
	WR4(sc, sc->mode_reg, mode);
	DELAY(10);
}

static int
apg8064_pll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct pll_sc *sc;
	uint32_t l, m, n, cfg, mode;
	uint64_t rate;
	uint64_t tmp;

	sc = clknode_get_softc(clk);

	l = RD4(sc, sc->l_reg) & 0x3FF;
	m = RD4(sc, sc->m_reg) & 0x7FFFF;
	n = RD4(sc, sc->n_reg) & 0x7FFFF;
	cfg = RD4(sc, sc->cfg_reg);
	mode = RD4(sc, sc->mode_reg);

printf("Parent: %lld, l: 0x%08X, m: 0x%08X, n: 0x%08X,  mode: 0x%08X,  cfg: 0x%08X\n", *freq, l, m, n, mode, cfg);
printf("  l: 0x%08X, m: 0x%08X, n: 0x%08X,  cfg: 0x%08X\n", sc->l_reg, sc->m_reg, sc->n_reg, sc->cfg_reg);
//	if (not_locked)
//		return (EINVAL);

	/* VCO = REF * [L+(M/N)] / (2 * P) */
	rate = *freq * l;
	if (n) {
		tmp = *freq;
		tmp *= m;
		tmp /= n;
		rate += tmp;
	}
	if (sc->post_div_mask) {
		cfg >>= sc->post_div_shift;
		cfg &= sc->post_div_mask;
		rate /= cfg + 1;
	}
	*freq = rate;
	return (0);
}

static int
apg8064_pll_init(struct clknode *clk, device_t dev)
{
	struct pll_sc *sc;
	uint32_t mode;

	sc = clknode_get_softc(clk);

	mode =  RD4(sc, sc->mode_reg);

	clknode_init_parent_idx(clk,
	    (mode >> PLL_REF_XO_SEL_SHIFT) & PLL_REF_XO_SEL_MASK);
	return(0);
}

static int
apg8064_pll_set_gate(struct clknode *clk, int enable)
{
	struct pll_sc *sc;

	sc = clknode_get_softc(clk);
	if (enable)
		clk_pll_enable(sc);
	else
		clk_pll_disable(sc);

	return (0);
}

static void
configure_pll(struct clknode *clk, const struct clk_pll_cfg *cfg,
    uint32_t bias_cnt, uint32_t lock_cnt)
{
	struct pll_sc *sc;
	uint32_t val;

	sc = clknode_get_softc(clk);

	if ((RD4(sc, sc->status_reg) & PLL_ACTIVE_FLAG) != 0)
		return;

	/* PLL registers */
	WR4(sc, sc->l_reg, cfg->l_val);
	WR4(sc, sc->m_reg, cfg->m_val);
	WR4(sc, sc->n_reg, cfg->n_val);

	/* merge config bits */
	val = RD4(sc, sc->cfg_reg);

	val &= ~cfg->mn_ena_mask;
	val |= cfg->mn_ena_val;

	val &= ~cfg->main_output_mask;
	val |= cfg->main_output_val;

	val &= ~cfg->pre_div_mask;
	val |= cfg->pre_div_val;

	val &= ~cfg->post_div_mask;
	val |= cfg->post_div_val;

	val &= ~cfg->vco_mask;
	val |= cfg->vco_val;
	WR4(sc, sc->cfg_reg, val);

	if (cfg->fsm_mode) {
		val = RD4(sc, sc->mode_reg);

		val &= ~PLL_VOTE_FSM_RESET;
		WR4(sc, sc->mode_reg, val);

		val &= ~(PLL_BIAS_COUNT_MASK << PLL_BIAS_COUNT_SHIFT);
		val |= bias_cnt << PLL_BIAS_COUNT_SHIFT;
		WR4(sc, sc->mode_reg, val);

		val &= ~(PLL_LOCK_COUNT_MASK << PLL_LOCK_COUNT_SHIFT);
		val |= lock_cnt << PLL_LOCK_COUNT_SHIFT;
		WR4(sc, sc->mode_reg, val);

		val |= PLL_VOTE_FSM_ENA;
		WR4(sc, sc->mode_reg, val);

	}
}

static clknode_method_t apg8064_pll_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		apg8064_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	apg8064_pll_recalc),
	CLKNODEMETHOD(clknode_set_gate,		apg8064_pll_set_gate),
//	CLKNODEMETHOD(clknode_set_freq,		apg8064_pll_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(apg8064_pll, apg8064_pll_class, apg8064_pll_methods,
   sizeof(struct pll_sc), clknode_class);



int
apg8064_pll_register(struct clkdom *clkdom, struct clk_pll_def *clkdef)
{
	struct clknode *clk;
	struct pll_sc *sc;

	clk = clknode_create(clkdom, &apg8064_pll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->l_reg = clkdef->l_reg;
	sc->m_reg = clkdef->m_reg;
	sc->n_reg = clkdef->n_reg;
	sc->cfg_reg = clkdef->cfg_reg;
	sc->mode_reg = clkdef->mode_reg;
	sc->status_reg = clkdef->status_reg;
	sc->status_mask = clkdef->status_mask;
	sc->post_div_mask = clkdef->post_div_mask;
	sc->post_div_shift = clkdef->post_div_shift;
	sc->freq_tbl = clkdef->freq_tbl;
	clknode_register(clkdom, clk);
	if (clkdef->cfg != NULL)
		configure_pll(clk, clkdef->cfg, 1, 8);
	return (0);
}
