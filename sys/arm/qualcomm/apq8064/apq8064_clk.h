/*-
 * Copyright 1992-2014 The FreeBSD Project. All rights reserved.
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

#ifndef _APQ8064_CLK_
#define _APQ8064_CLK_

#include "clkdev_if.h"

static inline uint32_t
_clk_read(device_t dev, bus_addr_t off)
{
	uint32_t val;

	CLKDEV_READ(dev, off, &val);
	return(val);
}

#define	WR4(_sc, off, val)						\
	CLKDEV_WRITE(clknode_get_device((_sc)->clknode), off, val)
#define	MD4(_sc, off, clr, set )					\
	CLKDEV_MODIFY(clknode_get_device((_sc)->clknode), off, clr, set)
#define	RD4(_sc, off)						\
	_clk_read(clknode_get_device((_sc)->clknode), off)


struct pll_freq_tbl {
	uint64_t	freq;
	uint32_t	l;
	uint32_t	m;
	uint32_t	n;
	uint32_t	ibits;
};

struct clk_pll_cfg {
	uint32_t	l_val;
	uint32_t	m_val;
	uint32_t	n_val;
	uint32_t	vco_val;
	uint32_t	vco_mask;
	uint32_t	pre_div_val;
	uint32_t	pre_div_mask;
	uint32_t	post_div_val;
	uint32_t	post_div_mask;
	uint32_t	mn_ena_val;
	uint32_t	mn_ena_mask;
	uint32_t	main_output_val;
	uint32_t	main_output_mask;
	int		fsm_mode;
};

struct clk_pll_def {
	struct clknode_init_def clkdef;
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
	const struct clk_pll_cfg *cfg;
};

struct clk_hfpll_def {
	struct clknode_init_def clkdef;
	uint32_t	l_reg;
	uint32_t	m_reg;
	uint32_t	n_reg;
	uint32_t	cfg_reg;
	uint32_t	mode_reg;
	uint32_t	status_reg;
	uint32_t	droop_reg;
};

struct clk_rcg_def {
	struct clknode_init_def clkdef;
	uint32_t	md_reg;
	uint32_t	ns_reg;
	uint32_t	mask;
};

#define APQ8064_HALT_INVERTED 0x001
struct clk_br_def {
	struct clknode_init_def clkdef;
	uint32_t	ena_reg;
	uint32_t	ena_mask;
	uint32_t	halt_reg;
	uint32_t	halt_mask;
	uint32_t	hwcg_reg;
	uint32_t	hwcg_mask;
	uint32_t	flags;
};

int apg8064_pll_register(struct clkdom *clkdom, struct clk_pll_def *clkdef);
int apg8064_hfpll_register(struct clkdom *clkdom, struct clk_hfpll_def *clkdef);
int apg8064_rcg_register(struct clkdom *clkdom, struct clk_rcg_def *clkdef);
int apg8064_br_register(struct clkdom *clkdom, struct clk_br_def *clkdef);
#endif /*_APQ8064_CLK_*/