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
 * PLL freqency VCO = REF * [L+(M/N)] / (2 * P)
 *
 *
 * PLL	  DS Name Type   Source   Freq (MHz)	VDD)	Notes		 Subsystem
 * PLL0	  GPLL0	  SR	 PXO	   800.000	0.945	FABRIC/DDR PLL	 RPM
 * PLL1	  MMPLL0  SR	 PXO	  1332.000	1.050	MM FABRIC	 Multimedia
 * PLL2	  MMPLL1  SR	 PXO	   800.000	0.945	Display		 Multimedia
 * PLL3	  GPLL01  SR2	 PXO	  1200.000	1.050	QDS		 Apps
 * PLL4	  PLL4    SR	 PXO	   393.216	0.945	LPA		 LPA
 	 			   491.520
 * PLL5	  MPLL0	  SR	 CXO	   288.000	0.945	GPS PLL		 Modem
 * PLL8	  PLL8	  SR	 PXO	   384.000	0.945	Peripherals,
 *							RPM, and modem	 Shared
 * PLL9	  SCPLL0  HF	 PXO	Up to 2000	1.050	Apps Core0	 Apps
 * PLL10  SCPLL1  HF	 PXO	Up to 2000	1.050	Apps Core1	 Apps
 * PLL11  EBI1PLL SR2	 PXO	  1066.000	1.050	Turbo FABRIC/DDR RPM
 * PLL12  SCL2PLL HF	 PXO	Up to 1700	1.050	Apps L2		 Apps
 * PLL13  WCNPLL  SR2	 WCNXO	   960.000	0.945	WCNPLL		 WCNSS
   		  	 CXO - used if WCNXO absent
 * PLL14  PLL14   SR	 PXO	   480.000	0.945	HSIC PLL	 Apps
 * PLL15  MMPLL3  SR	 PXO	   975.000	0.945	Graphics PLL	 Apps
 * PLL16  SCPLL2  HF	 PXO	Up to 2000	1.050	Apps Core2	 Apps
 * PLL17  SCPLL3  HF	 PXO 	Up to 2000	1.050	Apps Core3	 Apps
 *
 */

/* PLL MODE register bits */
#define PLL_PLLTEST		(1 << 3)
#define PLL_RESET_N		(1 << 2)
#define PLL_BYPASSNL		(1 << 1)
#define PLL_OUTCTRL		(1 << 0)

/* PLL CONFIG register bits */
#define MN_ACCUM_ENA		(1 << 27)

#define EARLY_OUT_ENA		(1 << 27)
#define MAIN_OUT_ENA		(1 << 23)
#define PLLOUT_DIVIDE_SHIFT	20
#define PLLOUT_DIVIDE_MASK	0x03
#define PRE_DIVIDE		(1 << 19)
#define PLLOUT_VCO_SHIFT	16
#define PLLOUT_VCO_MASK		0x03

/* PLL STATUS register bits */

struct hfpll_sc {
	struct clknode	*clknode;

	uint32_t	l_reg;
	uint32_t	m_reg;
	uint32_t	n_reg;
	uint32_t	mode_reg;
	uint32_t	droop_reg;
	uint32_t	cfg_reg;
	uint32_t	status_reg;

	uint32_t	user_val;
	uint32_t	droop_val;
	uint32_t	cfg_val;

	int		inited;

};

static void
clk_hfpll_init_once(struct hfpll_sc *sc)
{

	if (sc->inited)
		return;

	/* Configure PLL parameters for integer mode. */
	if (sc->cfg_val)
		WR4(sc, sc->cfg_reg, sc->cfg_val);
	WR4(sc, sc->m_reg, 0);
	WR4(sc, sc->n_reg, 1);

	if (sc->droop_reg)
		WR4(sc, sc->droop_reg, sc->droop_val);

	sc->inited = 1;
}


static void
clk_hfpll_enable(struct hfpll_sc *sc)
{
	uint32_t mode;

	mode = RD4(sc, sc->mode_reg);

	/* Do nothing if PLL is already enabed */
	if ((mode & (PLL_OUTCTRL | PLL_RESET_N | PLL_BYPASSNL)) ==
	    (PLL_OUTCTRL | PLL_RESET_N | PLL_BYPASSNL))
		return;

	clk_hfpll_init_once(sc);

	/* Disable bypass first */
	mode |= PLL_BYPASSNL;
 	WR4(sc, sc->mode_reg, mode);
	DELAY(10);

	/* Remove PLL form reset */
	mode |= PLL_RESET_N;
 	WR4(sc, sc->mode_reg, mode);
	DELAY(100);

	/* Finally, enable output */
	mode |= PLL_OUTCTRL;
 	WR4(sc, sc->mode_reg, mode);
}

static void
clk_hfpll_disable(struct hfpll_sc *sc)
{
	uint32_t mode;

	mode = RD4(sc, sc->mode_reg);

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
apg8064_hfpll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct hfpll_sc *sc;
	uint32_t l, m, n, cfg, mode;
	uint64_t rate;
	uint64_t tmp;

	sc = clknode_get_softc(clk);

	l = RD4(sc, sc->l_reg) & 0x3FF;
	m = RD4(sc, sc->m_reg) & 0x7FFFF;
	n = RD4(sc, sc->n_reg) & 0x7FFFF;
	cfg = RD4(sc, sc->cfg_reg);
	mode = RD4(sc, sc->mode_reg);
printf("%s\n", __func__);
printf(" Parent: %lld, l: 0x%08X, m: 0x%08X, n: 0x%08X,  mode: 0x%08X,  cfg: 0x%08X\n", *freq, l, m, n, mode, cfg);
printf("  l: 0x%08X, m: 0x%08X, n: 0x%08X,  cfg: 0x%08X\n", sc->l_reg, sc->m_reg, sc->n_reg, sc->cfg_reg);
//	if (not_locked)
//		return (EINVAL);

	/* VCO = REF * [L+(M/N)] / (2 * P) */
	rate = *freq * l;
	if (n && (cfg & MN_ACCUM_ENA)) {
		tmp = *freq;
		tmp *= m;
		tmp /= n;
		rate += tmp;
	}
	*freq = rate;
	return (0);
}

static int
apg8064_hfpll_init(struct clknode *clk, device_t dev)
{
	struct hfpll_sc *sc;
	uint32_t mode;

	sc = clknode_get_softc(clk);
	mode =  RD4(sc, sc->mode_reg);
	clknode_init_parent_idx(clk, 0);
	return(0);
}

static int
apg8064_hfpll_set_gate(struct clknode *clk, int enable)
{
	struct hfpll_sc *sc;

	sc = clknode_get_softc(clk);
	if (enable)
		clk_hfpll_enable(sc);
	else
		clk_hfpll_disable(sc);

	return (0);
}

#if 0
static void
configure_hfpll(struct clknode *clk, const struct clk_hfpll_cfg *cfg,
    uint32_t bias_cnt, uint32_t lock_cnt)
{
	struct hfpll_sc *sc;
	uint32_t val;

	sc = clknode_get_softc(clk);

	if ((RD4(sc, sc->status_reg) & HFPLL_ACTIVE_FLAG) != 0)
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

		val &= ~HFPLL_VOTE_FSM_RESET;
		WR4(sc, sc->mode_reg, val);

		val &= ~(HFPLL_BIAS_COUNT_MASK << HFPLL_BIAS_COUNT_SHIFT);
		val |= bias_cnt << HFPLL_BIAS_COUNT_SHIFT;
		WR4(sc, sc->mode_reg, val);

		val &= ~(HFPLL_LOCK_COUNT_MASK << HFPLL_LOCK_COUNT_SHIFT);
		val |= lock_cnt << HFPLL_LOCK_COUNT_SHIFT;
		WR4(sc, sc->mode_reg, val);

		val |= HFPLL_VOTE_FSM_ENA;
		WR4(sc, sc->mode_reg, val);

	}
}

#endif

static clknode_method_t apg8064_hfpll_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		apg8064_hfpll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	apg8064_hfpll_recalc),
	CLKNODEMETHOD(clknode_set_gate,		apg8064_hfpll_set_gate),
//	CLKNODEMETHOD(clknode_set_freq,		apg8064_hfpll_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(apg8064_hfpll, apg8064_hfpll_class, apg8064_hfpll_methods,
   sizeof(struct hfpll_sc), clknode_class);


int
apg8064_hfpll_register(struct clkdom *clkdom, struct clk_hfpll_def *clkdef)
{
	struct clknode *clk;
	struct hfpll_sc *sc;

	clk = clknode_create(clkdom, &apg8064_hfpll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->l_reg = clkdef->l_reg;
	sc->m_reg = clkdef->m_reg;
	sc->n_reg = clkdef->n_reg;
	sc->status_reg = clkdef->status_reg;
	sc->mode_reg = clkdef->mode_reg;
	sc->droop_reg = clkdef->droop_reg;
	sc->cfg_reg = clkdef->cfg_reg;
	sc->cfg_val = 0x7845c665;
	sc->droop_val = 0x0108c000;

	clknode_register(clkdom, clk);
//	if (clkdef->cfg != NULL)
//		configure_hfpll(clk, clkdef->cfg, 1, 8);
	return (0);
}

