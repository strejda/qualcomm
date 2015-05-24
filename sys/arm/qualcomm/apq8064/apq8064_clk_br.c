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
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <dev/clk/clk.h>

#include "apq8064_clk.h"


/*
 * Clock branch (aka enable/disable)
 *
 */

struct br_sc {
	struct clknode	*clknode;
	uint32_t	ena_reg;
	uint32_t	ena_mask;
	uint32_t	halt_reg;
	uint32_t	halt_mask;
	uint32_t	hwcg_reg;
	uint32_t	hwcg_mask;

	uint32_t	flags;
};

/* Return true if branch is gated by hw */
static int
is_hw_gated(struct br_sc *sc)
{
	uint32_t val;

	if (sc->hwcg_reg == 0)
		return (0);

	val = RD4(sc, sc->hwcg_reg);
	return (val & sc->hwcg_mask ? 1: 0);
}

static int
is_halted(struct br_sc *sc)
{
	uint32_t val;

	val = RD4(sc, sc->halt_reg);
	val = (val & sc->halt_mask ? 1: 0);
	if (sc->flags & APQ8064_HALT_INVERTED)
		val ^= 1;
	return (val);
}

static int
wait_for_halt(struct br_sc *sc, int enable)
{
	int i;
	int halted;

	i = 100;
	do {
		halted = is_halted(sc);
		if (enable && !halted)
			return (0);
		if (!enable && halted)
			return (0);
		DELAY(1);
		i--;
	} while(i > 0);

	return (ETIMEDOUT);
}

static int
apg8064_br_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return(0);
}

static int
apg8064_br_set_gate(struct clknode *clk, int enable)
{
	struct br_sc *sc;
	uint32_t val;
	int rv;

	sc = clknode_get_softc(clk);
	val = RD4(sc, sc->ena_reg);
	if (enable)
		val |=  sc->ena_mask;
	else
		val &=  ~sc->ena_mask;
	WR4(sc, sc->ena_reg, val);

	if ((sc->halt_reg == 0) || is_hw_gated(sc))
		return (0);

	/* Wait for finish */
	rv = wait_for_halt(sc, enable);

	return (rv);
}

static clknode_method_t apg8064_br_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		apg8064_br_init),
	CLKNODEMETHOD(clknode_set_gate,		apg8064_br_set_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(apg8064_br, apg8064_br_class, apg8064_br_methods,
   sizeof(struct br_sc), clknode_class);

int
apg8064_br_register(struct clkdom *clkdom, struct clk_br_def *clkdef)
{
	struct clknode *clk;
	struct br_sc *sc;

	clk = clknode_create(clkdom, &apg8064_br_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->ena_reg = clkdef->ena_reg;
	sc->ena_mask = clkdef->ena_mask;
	sc->hwcg_reg = clkdef->hwcg_reg;
	sc->hwcg_mask = clkdef->hwcg_mask;
	sc->halt_reg = clkdef->halt_reg;
	sc->halt_mask = clkdef->halt_mask;
	sc->flags = clkdef->flags;
	clknode_register(clkdom, clk);
	return (0);
}
