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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");


#include <sys/param.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/clk/clk_div.h>

#include "clkdev_if.h"

#define	WR4(_sc, off, val)						\
	CLKDEV_WRITE(clknode_get_device(_sc), off, val)
#define	RD4(_sc, off, val)							\
	CLKDEV_READ(clknode_get_device(_sc), off, val)

static int clknode_div_init(struct clknode *clk, device_t dev);
static int clknode_div_recalc(struct clknode *clk, uint64_t *req);
static int clknode_div_set_freq(struct clknode *clknode, uint64_t fin,
    uint64_t *fout, uint64_t *prec, int *stop);

struct clknode_div_sc {
	bus_addr_t	offset;
	uint32_t	i_shift;
	uint32_t	i_mask;
	uint32_t	i_width;
	uint32_t	f_shift;
	uint32_t	f_mask;
	uint32_t	f_width;
	int		div_flags;
	uint32_t	divider;	/* in natural form */
};

static int
clknode_div_init(struct clknode *clk, device_t dev)
{
	uint32_t reg;
	struct clknode_div_sc *sc;
	uint32_t i_div, f_div;

	sc = clknode_get_softc(clk);

	RD4(clk, sc->offset, &reg);
	i_div = (reg >> sc->i_shift) & sc->i_mask;
	if (!(sc->div_flags & CLK_DIV_ZERO_BASED))
		i_div++;
	f_div = (reg >> sc->f_shift) & sc->f_mask;
	sc->divider = i_div << sc->f_width | f_div;
//printf("%s: %s (div: %u) - 0x%08X\n", __func__, clknode_get_name(clk), sc->divider, reg);
	clknode_init_parent_idx(clk, 0);
	return(0);
}

static int
clknode_div_recalc(struct clknode *clk, uint64_t *freq)
{
	struct clknode_div_sc *sc;

	sc = clknode_get_softc(clk);
	if (sc->divider == 0) {
		*freq = 0;
		return(EINVAL);
	}
	*freq = (*freq << sc->f_width) / sc->divider;
	return (0);
}

/* XXXX Not implemented yet !!! */
static int
clknode_div_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
   uint64_t *prec, int *stop)
{
	struct clknode_div_sc *sc;

	sc = clknode_get_softc(clk);
	/* Passthru for now */
	*fout = (*fout * sc->divider) >> sc->f_width;
	*prec = (*prec * sc->divider) >> sc->f_width;
	*stop = 0;
	return (0);
}

static clknode_method_t clknode_div_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		clknode_div_init),
	CLKNODEMETHOD(clknode_recalc_freq,	clknode_div_recalc),
	CLKNODEMETHOD(clknode_set_freq,		clknode_div_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(clknode_div, clknode_div_class, clknode_div_methods,
   sizeof(struct clknode_div_sc), clknode_class);

int
clknode_div_register(struct clkdom *clkdom, struct clk_div_def *clkdef)
{
	struct clknode *clk;
	struct clknode_div_sc *sc;

	clk = clknode_create(clkdom, &clknode_div_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->offset = clkdef->offset;
	sc->i_shift = clkdef->i_shift;
	sc->i_width = clkdef->i_width;
	sc->i_mask = (1 << clkdef->i_width) - 1;
	sc->f_shift = clkdef->f_shift;
	sc->f_width = clkdef->f_width;
	sc->f_mask = (1 << clkdef->f_width) - 1;
	sc->div_flags = clkdef->div_flags;

	clknode_register(clkdom, clk);
	return (0);
}
