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
#include <sys/queue.h>
#include <sys/kobj.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/lock.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/sx.h>

#ifdef FDT
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/clk/clk.h>


MALLOC_DEFINE(M_CLOCK, "clocks", "Clock framework");

/* Forward declarations */
struct clk;
struct clknodenode;
struct clkdom;

typedef TAILQ_HEAD(clknode_list, clknode) clknode_list_t;
typedef TAILQ_HEAD(clkdom_list, clkdom) clkdom_list_t;

/* default clock methods */
static int clknode_method_init(struct clknode *clk, device_t dev);
static int clknode_method_recalc_freq(struct clknode *clk, uint64_t *freq);
static int clknode_method_set_freq(struct clknode *clk, uint64_t fin,
    uint64_t *fout, uint64_t *prec, int *stop);
static int clknode_method_set_gate(struct clknode *clk, int enable);
static int clknode_method_set_mux(struct clknode *clk, int idx);

/*
 * Clock controller methods
 */
static clknode_method_t clknode_methods[] = {
	CLKNODEMETHOD(clknode_init, clknode_method_init),
	CLKNODEMETHOD(clknode_recalc_freq, clknode_method_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq, clknode_method_set_freq),
	CLKNODEMETHOD(clknode_set_gate, clknode_method_set_gate),
	CLKNODEMETHOD(clknode_set_mux, clknode_method_set_mux),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_0(clknode, clknode_class, clknode_methods, 0);

/*
 * Clock node - basic element for modelling SOC clock graph
 */
struct clknode {
	KOBJ_FIELDS;

	/*
	 * Clock nodes hierarchy.
	 */
	struct clkdom 	*clkdom;	/* owning clock domain */
	TAILQ_ENTRY(clknode) clkdom_e;	/* domain list entry */

	const char	**parent_names;	/* array of parent names */
	int		parents_num;	/* number of parents */

	int		parent_idx;	/* parent index or -1 */
	struct clknode	**parents;	/* array of a parents */
	struct clknode	*parent;	/* actual parent */

	clknode_list_t	children;	/* list of all childs */
	TAILQ_ENTRY(clknode) childern_e; /* children list entry */

	/*
	 * Details of this device.
	 */
	const char	*name;		/* clock name */
	intptr_t	id;		/* clock id */
	int		flags;		/* CLK_FLAGS_  */
	void		*softc;		/* instance variables */
	struct sx	node_lock;	/* lock for this clock */

	uint64_t	freq;		/* actual frequency */
	int		ref_cnt;	/* refereced counter */
	int		enable_cnt;	/* enabled counter */
};

/*
 *  Per consumer data. This structre is used as handle in consumer interface.
 */
struct clk {
	struct clknode	*clknode;
	int		enable_cnt;
};

/*
 * Clock domain - group of clock and clock controllers associated to one
 * clock device.
 */
struct clkdom {
	device_t 	device; 	/*link to device */
	TAILQ_ENTRY(clkdom) link;	/* global domain list membership */
	clknode_list_t	clknode_list;	/* all clocks list*/
	struct sx	dom_lock;		/* shared lock for domain */

#ifdef FDT
	clknode_ofw_map_t *ofw_mapper;
#endif
};

#define DOMAIN_SLOCK(_sc)	sx_slock(&((_sc)->dom_lock))
#define DOMAIN_XLOCK(_sc)	sx_xlock(&((_sc)->dom_lock))
#define DOMAIN_UNLOCK(_sc)	sx_unlock(&((_sc)->dom_lock))

#define CLKDOM_SLOCK(_sc)	sx_slock(&((_sc)->clkdom->dom_lock))
#define CLKDOM_XLOCK(_sc)	sx_xlock(&((_sc)->clkdom->dom_lock))
#define CLKDOM_UNLOCK(_sc)	sx_unlock(&((_sc)->clkdom->dom_lock))

#define CLKNODE_SLOCK(_sc)	sx_slock(&((_sc)->node_lock))
#define CLKNODE_XLOCK(_sc)	sx_xlock(&((_sc)->node_lock))
#define CLKNODE_UNLOCK(_sc)	sx_unlock(&((_sc)->node_lock))

/* List of all domains in kernel */
static clkdom_list_t clkdom_list =
   TAILQ_HEAD_INITIALIZER(clkdom_list);

static void clknode_adjust_parent(struct clknode *clknode, int idx);

#ifdef FDT
/*
 *  !!!! Move this to ofw_bus_subr.c !!!!!!
 */

/*
 * Parse property that contain list of xrefs and nodes
 *
 */
static int
ofw_parse_xref_list_alloc(phandle_t node, char *list_name, char *cells_name,
    int idx, phandle_t *parent, int *ncells, phandle_t **cells)
{
	phandle_t pnode;
	phandle_t *elems;
	uint32_t  pcells;
	int rv, i, j, nelems, cnt;

	elems = NULL;
	nelems = OF_getencprop_alloc(node, list_name,  sizeof(*elems),
	    (void **)&elems);
	if (nelems <= 0)
		return (ENOENT);
	rv = ENOENT;
	for (i = 0, cnt = 0; i < nelems; i += pcells, cnt++) {
		pnode = elems[i++];
		if (OF_searchencprop(OF_node_from_xref(pnode),
		    cells_name, &pcells, sizeof(pcells)) == -1) {
			printf("Missing %s property\n", cells_name);
			rv = ENOENT;
			break;
		}

		if (pcells < 1 || (i + pcells) > nelems) {
			printf("Invalid %s property value <%d>\n", cells_name,
			    pcells);
			rv = ERANGE;
			break;
		}
		if (cnt == idx) {
			*cells= malloc(pcells * sizeof(**cells), M_OFWPROP,
			    M_WAITOK);
			*parent = pnode;
			*ncells = pcells;
			for (j = 0; j < pcells; j++)
				(*cells)[j] = elems[i + j];
			rv = 0;
			break;
		}
	}
	if (elems != NULL)
		free(elems, M_OFWPROP);
	return (rv);
}

/*
 * Find index of string in string list property.
 */
static int
ofw_find_string_index(phandle_t node, char *list_name, char *name, int *idx)
{
	char *elems;
	int rv, i, cnt, nelems;

	elems = NULL;
	nelems = OF_getprop_alloc(node, list_name, 1, (void **)&elems);
	if (nelems <= 0)
		return (ENOENT);

	rv = ENOENT;
	for (i = 0, cnt = 0; i < nelems; cnt++) {
		if (strcmp(elems + i, name) == 0) {
			*idx = cnt;
			rv = 0;
			break;
		}
		i += strlen(elems + i) + 1;
	}

	if (elems != NULL)
		free(elems, M_OFWPROP);
	return (rv);
}
#endif

/* -------------------------------------------------------------------------- */
/*
 *  Support functions
 */

/* duplicate  array of names */
static const char **
strdup_list(const char **names, int num)
{
	size_t len, slen;
	const char **outptr, *ptr;
	int i;

	/* compute total memory size */
	len = sizeof(char *) * num;
	for (i = 0; i < num; i++) {
		if (names[i] == NULL)
			continue;
		slen = strlen(names[i]);
		if (slen == 0)
			panic("Clock parent names array have empty string");
		len += slen + 1;
	}
	outptr = malloc(len, M_CLOCK, M_WAITOK | M_ZERO);
	ptr = (char *)(outptr + num);
	for (i = 0; i < num; i++) {
		if (names[i] == NULL)
			continue;
		outptr[i] = ptr;
		slen = strlen(names[i]) + 1;
		bcopy(names[i], __DECONST(void *, outptr[i]), slen);
		ptr += slen;
	}
	return (outptr);
}

/* -------------------------------------------------------------------------- */

/* Default clock methods implementation. */
static int
clknode_method_init(struct clknode *clknode, device_t dev)
{
	return (0);
}

static int
clknode_method_recalc_freq(struct clknode *clknode, uint64_t *freq)
{
	return (0);
}

static int
clknode_method_set_freq(struct clknode *clknode, uint64_t fin,  uint64_t *fout,
   uint64_t *prec, int *stop)
{
	*stop = 0;
	return (0);
}

static int
clknode_method_set_gate(struct clknode *clk, int enable)
{
	return (0);
}

static int
clknode_method_set_mux(struct clknode *clk, int idx)
{
	return (0);
}

/* --------------------------------------------------------------------------
 *
 * Internal functions.
 *
 */

/* Refresh (recompute) frequency cache for this node and all its childerns. */
static int
clknode_refresh_cache(struct clknode *clknode, uint64_t freq)
{
	int rv;
	struct clknode *entry, *tmp;

	CLKNODE_XLOCK(clknode);
	/* Compoute generated  frequncy */
	rv = CLKNODE_RECALC_FREQ(clknode, &freq);
	if (rv != 0) {
		CLKNODE_UNLOCK(clknode);
		return (rv);
	}
	/* Refresh cache */
	clknode->freq = freq;
	CLKNODE_UNLOCK(clknode);

	/* Refresh cache for all childs */
	TAILQ_FOREACH_SAFE(entry, &(clknode->children), childern_e, tmp) {
		rv = clknode_refresh_cache(entry, freq);
		if (rv != 0)
			return (rv);
	}
	return (0);
}

struct clknode *
clknode_find_by_name(struct clkdom *clkdom, const char *name)
{
	struct clknode *entry, *tmp;

	TAILQ_FOREACH_SAFE(entry, &clkdom->clknode_list, clkdom_e, tmp) {
		if (strcmp(entry->name, name) == 0)
			return (entry);
	}
	return (NULL);
}

struct clknode *
clknode_find_by_id(struct clkdom *clkdom, intptr_t id)
{
	struct clknode *entry, *tmp;

	TAILQ_FOREACH_SAFE(entry, &clkdom->clknode_list, clkdom_e, tmp) {
		if (entry->id ==  id) {
			return (entry);
			break;
		}
	}

	return (NULL);
}

/* -------------------------------------------------------------------------- */
/*
 * Clock domain functions
 */

/* Find clock domain associated to device in global list. */
struct clkdom *
clkdom_get_by_dev(const device_t dev)
{
	struct clkdom *entry, *tmp;

	TAILQ_FOREACH_SAFE(entry, &clkdom_list, link, tmp) {
		if (entry->device == dev)
			return (entry);
	}
	return (NULL);
}


#ifdef FDT
/* Map FTD data to clknode. */
static int
clknode_default_ofw_map(struct clkdom *clkdom, phandle_t xref, uint32_t ncells,
    phandle_t *cells, struct clknode **clk)
{
	if (ncells == 0)
		*clk = clknode_find_by_id(clkdom, 1);
	else if (ncells == 1)
		*clk = clknode_find_by_id(clkdom, cells[0]);
	else
		return (ERANGE);
	if (*clk == NULL)
		return (ENXIO);
	return (0);
}

#endif

/* Create clock domain. */
struct clkdom *
clkdom_create(device_t dev)
{
	struct clkdom *clkdom;
#ifdef FDT
	phandle_t node;

	if ((node = ofw_bus_get_node(dev)) == -1)
		return (NULL);
#endif
	clkdom = malloc(sizeof(struct clkdom), M_CLOCK, M_WAITOK | M_ZERO);
	clkdom->device = dev;
	sx_init(&clkdom->dom_lock, "Clock domain lock");
	TAILQ_INIT(&clkdom->clknode_list);
#ifdef FDT
	clkdom->ofw_mapper = clknode_default_ofw_map;
#endif
	TAILQ_INSERT_TAIL(&clkdom_list, clkdom, link);
#ifdef FDT
	OF_device_register_xref(OF_xref_from_node(node), dev);
#endif
	return (clkdom);
}

/* Finalize initialization of clock domain. */
int
clkdom_finit(struct clkdom * clkdom)
{
	struct clknode *clknode, *tmp;
	int rv;
	int i;
	uint64_t freq;

	rv = 0;
	DOMAIN_SLOCK(clkdom);

	/*
	 * At this point all nodes must be defined and all
	 * parents must be valid
	 */
	TAILQ_FOREACH_SAFE(clknode, &clkdom->clknode_list, clkdom_e, tmp) {
		if (clknode->parents_num == 0)
			continue;
		for (i = 0; i < clknode->parents_num; i++) {
			if (clknode->parents[i] != NULL)
				continue;
			if (clknode->parent_names[i] == NULL)
				continue;
			clknode->parents[i] = clknode_find_by_name(clkdom,
			    clknode->parent_names[i]);
			if (clknode->parents[i] == NULL) {
				device_printf(clkdom->device,
				    "Clock %s have unknown parent: %s\n",
				    clknode->name, clknode->parent_names[i]);
				rv = 1;
			}
		}

		/* if parent index is not set yet */
		if (clknode->parent_idx == CLKNODE_IDX_NONE) {
			device_printf(clkdom->device,
			    "Clock %s have not set parent idx\n",
			    clknode->name);
			continue;
		}
		if (clknode->parents[clknode->parent_idx] == NULL) {
			device_printf(clkdom->device,
			    "Clock %s have unknown parent(idx %d): %s\n",
			    clknode->name, clknode->parent_idx,
			    clknode->parent_names[clknode->parent_idx]);
			rv = 1;
			continue;
		}
		clknode_adjust_parent(clknode, clknode->parent_idx);
	}
#if 1
	TAILQ_FOREACH_SAFE(clknode, &clkdom->clknode_list, clkdom_e, tmp) {
		rv = clknode_get_freq(clknode, &freq);
		printf("Clock: %s, parent: %s(%d), freq: %llu\n", clknode->name,
		    clknode->parent == NULL ? "(NULL)" : clknode->parent->name,
		    clknode->parent_idx,
		    ((rv == 0) ? freq: rv));
	}
#endif
	DOMAIN_UNLOCK(clkdom);
	return (rv);
}

/* Create and initialize clock object, but not register it. */
struct clknode *
clknode_create(struct clkdom * clkdom, clknode_class_t clknode_class,
    struct clknode_init_def *def)
{
	struct clknode *clknode;

	KASSERT(def->name != NULL, ("clock name is NULL"));
	KASSERT(def->name[0] != '\0', ("clock name is empty"));

	DOMAIN_SLOCK(clkdom);
	if (clknode_find_by_name(clkdom, def->name) != NULL)
		panic("Duplicated clock registration: %s\n", def->name);
	DOMAIN_UNLOCK(clkdom);

	/* create object and initialize it */
	clknode = malloc(sizeof(struct clknode), M_CLOCK, M_WAITOK | M_ZERO);
	kobj_init((kobj_t)clknode, (kobj_class_t)clknode_class);
	sx_init(&clknode->node_lock, "Clock lock");

	/* allocate softc if required */
	if (clknode_class->size > 0) {
		clknode->softc = malloc(clknode_class->size,
		    M_CLOCK, M_WAITOK | M_ZERO);
	}

	/* prepare array for ptrs to parent clocks */
	clknode->parents = malloc(sizeof(struct clknode *) * def->parents_num,
	    M_CLOCK, M_WAITOK | M_ZERO);

	/* copy all strings if requested */
	if (def->flags & CLK_FLAGS_STATIC) {
		clknode->name = def->name;
		clknode->parent_names = def->parent_names;
	} else {
		clknode->name = strdup(def->name, M_CLOCK);
		clknode->parent_names =
		    strdup_list(def->parent_names, def->parents_num);
	}

	/* rest of init */
	clknode->id = def->id;
	clknode->clkdom = clkdom;
	clknode->flags = def->flags;
	clknode->parents_num = def->parents_num;
	clknode->parent = NULL;
	clknode->parent_idx = CLKNODE_IDX_NONE;
	TAILQ_INIT(&clknode->children);

	return (clknode);

#if 0
	if (!(def->flags & CLK_FLAGS_STATIC) && (clknode->parent_names != NULL))
		free((void *)clknode->parent_names, M_CLOCK);
	if (!(def->flags & CLK_FLAGS_STATIC) && (clknode->name != NULL))
		free((void *)clknode->name, M_CLOCK);
	if (clk->parents != NULL)
		free(clknode->parents, M_CLOCK);
	if (clk->softc != NULL)
		free(clknode->softc, M_CLOCK);
	kobj_delete((kobj_t)clknode, M_CLOCK);
	return (NULL);
#endif
}

/* Register clock object into clock domain hierarchy. */
struct clknode *
clknode_register(struct clkdom * clkdom, struct clknode *clknode)
{
	int i, rv;

	/* Lazy initialization of parents references */
	for (i = 0; i < clknode->parents_num; i++) {
		if (clknode->parents[i] == NULL) continue;
		clknode->parents[i] =
		    clknode_find_by_name(clkdom, clknode->parent_names[i]);
	}

	rv = CLKNODE_INIT(clknode, clknode_get_device(clknode));
	if (rv != 0) {
		printf(" CLKNODE_INIT failed: %d\n", rv);
		return (NULL);
	}
	DOMAIN_XLOCK(clkdom);

	TAILQ_INSERT_TAIL(&clkdom->clknode_list, clknode, clkdom_e);
	DOMAIN_UNLOCK(clkdom);

	return (clknode);
}

/* --------------------------------------------------------------------------
 *
 * Clock providers interface
 *
 */
 /* Set parent index - init function */
void
clknode_init_parent_idx(struct clknode *clknode, int idx)
{

	if (clknode->parents_num == 0)
		return;
	if ((idx == CLKNODE_IDX_NONE) ||
	    (idx >= clknode->parents_num) ||
	    (clknode->parent_names[idx] == NULL))
		panic("%s: Invalid clock parent index: %d\n", __func__, idx);

	clknode->parent_idx = idx;
}

/* Set new parent for this clock */
static void
clknode_adjust_parent(struct clknode *clknode, int idx)
{

	if (clknode->parents_num == 0)
		return;
	if ((idx == CLKNODE_IDX_NONE) || (idx >= clknode->parents_num))
		panic("Invalid clock parent index\n");

	if (clknode->parents[idx] == NULL)
		panic("%s: Attempt to set invalid parent %d for clock %s",
		    __func__, idx, clknode->name);

	/* Remove me from old parent childern list */
	if (clknode->parent != NULL) {
		TAILQ_REMOVE(&clknode->parent->children, clknode, childern_e);
	}

	/* and setup new */
	clknode->parent_idx = idx;
	clknode->parent = clknode->parents[idx];
	TAILQ_INSERT_TAIL(&clknode->parent->children,
	    clknode, childern_e);
}

int
clknode_set_parent(struct clknode *clknode, int idx)
{
	int rv;
	uint64_t freq;
	int  oldidx;

	if (clknode->parents_num == 0)
		return (0);

	if (clknode->parents_num == 1) {
		clknode_set_parent(clknode->parent, idx);
		return (0);
	}

	if (clknode->parent_idx == idx)
		return (0);

	CLKNODE_XLOCK(clknode);
	oldidx = clknode->parent_idx;
	clknode_adjust_parent(clknode, idx);
	rv = CLKNODE_SET_MUX(clknode, idx);
	if (rv != 0) {
		clknode_adjust_parent(clknode, oldidx);
		CLKNODE_UNLOCK(clknode);
		return (rv);
	}
	rv = clknode_get_freq(clknode->parent, &freq);
	CLKNODE_UNLOCK(clknode);
	if (rv != 0)
		return (rv);
	rv = clknode_refresh_cache(clknode, freq);
	return (rv);
}


int
clknode_set_parent_by_name(struct clknode *clknode,  char *name)
{
	int rv;
	uint64_t freq;
	int  oldidx, idx;

	if (clknode->parents_num == 0)
		return (0);

	if (clknode->parents_num == 1) {
		clknode_set_parent_by_name(clknode->parent, name);
		return (0);
	}

	for (idx = 0; idx < clknode->parents_num; idx++) {

		if (clknode->parent_names[idx] == NULL)
			continue;
		if (strcmp(clknode->parent_names[idx], name) == 0)
			break;
	}
	if (idx >= clknode->parents_num) {
		return (ENXIO);
	}
	if (clknode->parent_idx == idx)
		return (0);

	CLKNODE_XLOCK(clknode);
	oldidx = clknode->parent_idx;
	clknode_adjust_parent(clknode, idx);
	rv = CLKNODE_SET_MUX(clknode, idx);
	if (rv != 0) {
		clknode_adjust_parent(clknode, oldidx);
		CLKNODE_UNLOCK(clknode);
		return (rv);
	}
	rv = clknode_get_freq(clknode->parent, &freq);
	CLKNODE_UNLOCK(clknode);
	if (rv != 0)
		return (rv);
	rv = clknode_refresh_cache(clknode, freq);
	return (rv);
}

struct clknode *
clknode_get_parent(struct clknode *clknode)
{
	return (clknode->parent);
}

const char *
clknode_get_name(struct clknode *clknode)
{
	return (clknode->name);
}

const char **
clknode_get_parent_names(struct clknode *clknode)
{
	return (clknode->parent_names);
}

int
clknode_get_parents_num(struct clknode *clknode)
{
	return (clknode->parents_num);
}

int
clknode_get_parent_idx(struct clknode *clknode)
{
	return (clknode->parent_idx);
}

int
clknode_get_flags(struct clknode *clknode)
{
	return (clknode->flags);
}


void *
clknode_get_softc(struct clknode *clknode)
{
	return (clknode->softc);
}

device_t
clknode_get_device(struct clknode *clknode)
{
	return (clknode->clkdom->device);
}

#ifdef FDT
void
clkdom_set_ofw_mapper(struct clkdom * clkdom,  clknode_ofw_map_t *map)
{
	clkdom->ofw_mapper = map;
}
#endif

/* --------------------------------------------------------------------------
 *
 * Real consumers executive
 *
 */
int
clknode_get_freq(struct clknode *clknode, uint64_t *freq)
{
	int rv;

	/* Use cached value, if exist */
	*freq  = clknode->freq;
	if (*freq != 0)
		return (0);

	/* Get frequency from parent, if exist */
	if (clknode->parents_num > 0) {
		rv = clknode_get_freq(clknode->parent, freq);
		if (rv != 0) {
			return (rv);
		}
	}

	/* And recalculate my output frequency */
	CLKNODE_XLOCK(clknode);
	rv = CLKNODE_RECALC_FREQ(clknode, freq);
	if (rv != 0) {
		CLKNODE_UNLOCK(clknode);
		printf("Cannot get grequency for clk: %s, error: %d\n",
		    clknode->name, rv);
		return (rv);
	}

	/* Save new frequency to cache */
	clknode->freq = *freq;
	CLKNODE_UNLOCK(clknode);
	return (0);
}

int
clknode_set_freq(struct clknode *clknode, uint64_t freq, uint64_t prec,
    int enablecnt)
{
	int rv, done;
	uint64_t parent_freq;

	parent_freq = 0;
	CLKNODE_XLOCK(clknode);
//printf("%s: for %s, freq: %lld\n", __func__, clknode->name, freq);
	/*
	 * We can set frequency only if
	 *   clock is disabled
	 * OR
	 *   clock is glitch free and is enbled by calling consumer only
	 */
	if ((clknode->enable_cnt > 1) &&
	    ((clknode->enable_cnt > enablecnt) ||
	    !(clknode->flags & CLK_FLAGS_GLITCH_FREE))) {
	    	CLKNODE_UNLOCK(clknode);
		return (EBUSY);
	}

	/* Get frequency from parent, if exist */
	if (clknode->parents_num > 0) {
		rv = clknode_get_freq(clknode->parent, &parent_freq);
		if (rv != 0) {
			CLKNODE_UNLOCK(clknode);
			return (rv);
		}
	}

	/* Set frequency for this clock */
	rv = CLKNODE_SET_FREQ(clknode, parent_freq, &freq,  &prec, &done);
	if (done) {
		clknode->freq = freq;
	}
	CLKNODE_UNLOCK(clknode);
	if (rv != 0) {
		printf("Cannot set frequency for clk: %s, error: %d\n",
		    clknode->name, rv);
		return (rv);
	}

	if (done) {
		/* Success - invalidate frequency cache for all childs */
		clknode_refresh_cache(clknode, parent_freq);
	} else if (clknode->parent != NULL) {
		/* Nothing changed, pass request to parent */
		rv = clknode_set_freq(clknode->parent, freq, prec, enablecnt);
	} else {
		/* End of chain without action */
		printf("Cannot set frequency for clk: %s, end of chain\n",
		    clknode->name);

		rv = ENXIO;
	}

	return (rv);
}

int
clknode_enable(struct clknode *clknode)
{
	int rv;

	/* Enable clock for each node in chain, starting from source */
	if (clknode->parents_num > 0) {
		rv = clknode_enable(clknode->parent);
		if (rv != 0) {
			return (rv);
		}
	}

	/* Handle this node */
	CLKNODE_XLOCK(clknode);
	if (clknode->enable_cnt == 0) {
		rv = CLKNODE_SET_GATE(clknode, 1);
		if (rv != 0) {
			CLKNODE_UNLOCK(clknode);
			return (rv);
		}
	}
	clknode->enable_cnt++;
	CLKNODE_UNLOCK(clknode);
	return (0);
}

int
clknode_disable(struct clknode *clknode)
{
	int rv;

	rv = 0;
	CLKNODE_XLOCK(clknode);
	/* Disable clock for each node in chain, starting from consumer */
	if (clknode->enable_cnt == 1) {
		rv = CLKNODE_SET_GATE(clknode, 0);
		if (rv != 0) {
			CLKNODE_UNLOCK(clknode);
			return (rv);
		}
	}
	clknode->enable_cnt--;
	CLKNODE_UNLOCK(clknode);

	if (clknode->parents_num > 0) {
		rv = clknode_disable(clknode->parent);
	}
	return (rv);
}

/* Helper function for clk_get*() */
static clk_t
clk_create(struct clknode *clknode)
{
	struct clk *clk;

	clk =  malloc(sizeof(struct clk), M_CLOCK, M_WAITOK);
	CLKNODE_XLOCK(clknode);
	clk->clknode = clknode;
	clk->enable_cnt = 0;
	clknode->ref_cnt++;
	CLKNODE_UNLOCK(clknode);

	return (clk);
}

/* --------------------------------------------------------------------------
 *
 * Clock consumers interface.
 *
 */
int
clk_get_freq(clk_t clk, uint64_t *freq)
{
	int rv;
	struct clknode *clknode;

	clknode = clk->clknode;
	KASSERT(clknode->ref_cnt > 0,
	   ("Attempt to access unreferenced clock: %s\n", clknode->name));

	CLKDOM_SLOCK(clknode);
	rv = clknode_get_freq(clknode, freq);
	CLKDOM_UNLOCK(clknode);
	return (rv);
}

int
clk_set_freq(clk_t clk, uint64_t freq, uint64_t prec)
{
	int rv;
	struct clknode *clknode;

	clknode = clk->clknode;
	KASSERT(clknode->ref_cnt > 0,
	   ("Attempt to access unreferenced clock: %s\n", clknode->name));
	CLKDOM_XLOCK(clknode);
	rv = clknode_set_freq(clknode, freq, prec, clk->enable_cnt);
	CLKDOM_UNLOCK(clknode);;
	return (rv);
}

int
clk_enable(clk_t clk)
{
	int rv;
	struct clknode *clknode;

	clknode = clk->clknode;
	KASSERT(clknode->ref_cnt > 0,
	   ("Attempt to access unreferenced clock: %s\n", clknode->name));
	CLKDOM_SLOCK(clknode);
	rv = clknode_enable(clknode);
	if (rv == 0)
		clk->enable_cnt++;
	CLKDOM_UNLOCK(clknode);
	return (rv);
}

int
clk_disable(clk_t clk)
{
	int rv;
	struct clknode *clknode;

	clknode = clk->clknode;
	KASSERT(clknode->ref_cnt > 0,
	   ("Attempt to access unreferenced clock: %s\n", clknode->name));
	KASSERT(clk->enable_cnt > 0,
	   ("Attempt to disable already disabled clock: %s\n", clknode->name));
	CLKDOM_SLOCK(clknode);
	rv = clknode_disable(clknode);
	if (rv == 0)
		clk->enable_cnt--;
	CLKDOM_UNLOCK(clknode);
	return (rv);
}

int
clk_release(clk_t clk)
{
	struct clknode *clknode;

	clknode = clk->clknode;
	KASSERT(clknode->ref_cnt > 0,
	   ("Attempt to access unreferenced clock: %s\n", clknode->name));
	CLKDOM_SLOCK(clknode);
	while (clk->enable_cnt > 0) {
		clknode_disable(clknode);
		clk->enable_cnt--;
	}
	CLKNODE_XLOCK(clknode);
	clknode->ref_cnt--;
	CLKNODE_UNLOCK(clknode);
	CLKDOM_UNLOCK(clknode);

	free(clk, M_CLOCK);
	return (0);
}

int
clk_get_by_name(struct clkdom * clkdom, const char *name, clk_t *clk)
{
	struct clknode *clknode;

	DOMAIN_SLOCK(clkdom);
	clknode = clknode_find_by_name(clkdom, name);
	if (clknode != NULL) {
		DOMAIN_UNLOCK(clkdom);
		return (ENODEV);
	}
	*clk = clk_create(clknode);
	DOMAIN_UNLOCK(clkdom);
	return (0);
}

int
clk_get_by_id(struct clkdom *clkdom, intptr_t id, clk_t *clk)
{
	struct clknode *clknode;

	DOMAIN_SLOCK(clkdom);
	clknode = clknode_find_by_id(clkdom, id);
	if (clknode != NULL) {
		DOMAIN_UNLOCK(clkdom);
		return (ENODEV);
	}
	*clk = clk_create(clknode);
	DOMAIN_UNLOCK(clkdom);
	return (0);
}

#ifdef FDT

int
clk_get_by_ofw_index(phandle_t cnode, int idx, clk_t *clk)
{
	phandle_t parent, *cells;
	device_t clockdev;
	int ncells, rv;
	struct clkdom *clkdom;
	struct clknode *clknode;

	*clk = NULL;
	rv = ofw_parse_xref_list_alloc(cnode, "clocks", "#clock-cells", idx,
	    &parent, &ncells, &cells);
	if (rv != 0) {
		printf("%s:, ofw_parse_xref_list_alloc: %d\n", __func__, rv);
		return (rv);
	}
	clockdev = OF_device_from_xref(parent);
	if (clockdev == NULL) {
		printf("%s:, OF_device_from_xref: NULL\n", __func__);
		rv = ENODEV;
		goto done;
	}

	clkdom = clkdom_get_by_dev(clockdev);
	if (clkdom == NULL){
		printf("%s:, clkdom_get_by_dev: NULL\n", __func__);
		rv = ENXIO;
		goto done;
	}

	DOMAIN_SLOCK(clkdom);
	rv = clkdom->ofw_mapper(clkdom, cnode, ncells, cells, &clknode);
	if (rv == 0) {
printf("%s:,  got clock %s(id: %d)\n", __func__, clknode->name, clknode->id);
		*clk = clk_create(clknode);
	}
	DOMAIN_UNLOCK(clkdom);

done:
	if (cells != NULL)
		free(cells, M_OFWPROP);
	return (rv);
}

int
clk_get_by_ofw_name(phandle_t cnode, char *name, clk_t *clk)
{
	int rv, idx;

	rv = ofw_find_string_index(cnode, "clock-names", name, &idx);
	if (rv != 0)
		return (rv);
	return (clk_get_by_ofw_index(cnode, idx, clk));
}
#endif