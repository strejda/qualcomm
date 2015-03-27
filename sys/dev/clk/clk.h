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

#ifndef _DEV_CLK_H_
#define _DEV_CLK_H_
#include "opt_platform.h"

#include <sys/kobj.h>
#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#endif
#include "clknode_if.h"

#define CLKNODE_IDX_NONE	-1		/* not selected index */

/* Common clock flags. */
#define CLK_FLAGS_STATIC	0x0000001	/* Static strings. */
#define CLK_FLAGS_GLITCH_FREE 	0x0000002	/* Set menthod can be applied */
						/* to enabled clock. */
typedef struct clk *clk_t;

/* Parameters for clocknode crete */
struct clknode_init_def {
	char *name;
	intptr_t id;
	const char **parent_names;
	int parents_num;
	int flags;
};

/*
 * Shorthands for constructing method tables.
 */
#define	CLKNODEMETHOD		KOBJMETHOD
#define	CLKNODEMETHOD_END	KOBJMETHOD_END
#define clknode_method_t	kobj_method_t
#define clknode_class_t		kobj_class_t
DECLARE_CLASS(clknode_class);

/* Clock domain functions */
struct clkdom *clkdom_create(device_t dev);
int clkdom_finit(struct clkdom *clkdom);
struct clknode *clknode_create(struct clkdom *clkdom,
    clknode_class_t clknode_class, struct clknode_init_def *def);
struct clknode *clknode_register(struct clkdom *cldom, struct clknode *clk);
#ifdef FDT
typedef int clknode_ofw_map_t(struct clkdom *clkdom, phandle_t xref,
    uint32_t ncells, phandle_t *cells, struct clknode **clk);
void clkdom_set_ofw_mapper(struct clkdom *clkdom, clknode_ofw_map_t *cmp);
#endif

/* Clock providers interface */
struct clkdom *clkdom_get_by_dev(const device_t dev);

void clknode_init_parent_idx(struct clknode *clknode, int idx);
int clknode_set_parent(struct clknode *clk, int idx);
int clknode_set_parent_by_name(struct clknode *clk, char *name);
const char *clknode_get_name(struct clknode *clk);
const char **clknode_get_parent_names(struct clknode *clk);
int clknode_get_parents_num(struct clknode *clk);
int clknode_get_parent_idx(struct clknode *clk);
struct clknode *clknode_get_parent(struct clknode *clk);
int clknode_get_flags(struct clknode *clk);
void *clknode_get_softc(struct clknode *clk);
device_t clknode_get_device(struct clknode *clk);
struct clknode *clknode_find_by_name(struct clkdom *clkdom, const char *name);
struct clknode *clknode_find_by_id(struct clkdom *clkdom, intptr_t id);

int clknode_get_freq(struct clknode *clknode, uint64_t *freq);
int clknode_set_freq(struct clknode *clknode, uint64_t freq, uint64_t prec,
    int enablecnt);
int clknode_enable(struct clknode *clknode);
int clknode_disable(struct clknode *clknode);


/* Clock consumers interface */
int clk_get_by_name(struct clkdom *clkdom, const char *name, clk_t *clk);
int clk_get_by_id(struct clkdom *clkdom, intptr_t id, clk_t *clk);
int clk_release(clk_t clk);
int clk_get_freq(clk_t clk, uint64_t *freq);
int clk_set_freq(clk_t clk, uint64_t freq, uint64_t prec);
int clk_enable(clk_t clk);
int clk_disable(clk_t clk);

#ifdef FDT
int clk_get_by_ofw_index(phandle_t node, int idx, clk_t *clk);
int clk_get_by_ofw_name(phandle_t node, char *name, clk_t *clk);
#endif

#endif /* _DEV_CLK_H_ */
