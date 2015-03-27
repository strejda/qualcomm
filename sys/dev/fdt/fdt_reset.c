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
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/malloc.h>
//#include <sys/lock.h>
//#include <sys/mutex.h>
//#include <sys/queue.h>
//#include <sys/systm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_reset.h>

struct reset_handle {
	device_t device;		/* provider device*/
	int 	rst_id;			/* reset id */
};

int fdt_reset_default_map(device_t provider, phandle_t xref, int ncells,
    pcell_t *cells, intptr_t *num)
{
	if (ncells != 1)
		return (ERANGE);
	*num = (int)cells[0];
	return (0);
}

int
fdt_reset_assert(device_t consumer, reset_t rst)
{

	return (FDT_RESET_SET(rst->device, rst->rst_id, 1));
}

int
fdt_reset_clear(device_t consumer, reset_t rst)
{

	return (FDT_RESET_SET(rst->device, rst->rst_id, 0));
}

int
fdt_reset_status(device_t consumer, reset_t rst, int *value)
{

	return (FDT_RESET_GET(rst->device, rst->rst_id, value));
}

reset_t
fdt_reset_get_by_idx(device_t consumer, int idx)
{
	struct reset_handle *rst;
	phandle_t cnode, xnode;
	pcell_t *cells;
	device_t rstdev;
	int ncells, rv;
	intptr_t id;

	/* Get data from FDT */
	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	rv = ofw_bus_parse_xref_list_alloc(cnode, "resets", "#reset-cells", idx,
	    &xnode, &ncells, &cells);
	if (rv != 0)
		return (NULL);

	/* Tranlate provider to device */
	rstdev = OF_device_from_xref(xnode);
	if (rstdev == NULL) {
		free(cells, M_OFWPROP);
		return (NULL);
	}
	/* Map reset to number */
	rv = FDT_RESET_MAP(rstdev, xnode, ncells, cells, &id);
	free(cells, M_OFWPROP);
	if (rv != 0)
		return (NULL);

	/* Create handle */
	rst = malloc(sizeof(struct reset_handle), M_OFWPROP, M_WAITOK);
	rst->device =  rstdev;
	rst->rst_id = id;
	return (rst);
}

reset_t
fdt_reset_get_by_name(device_t consumer, char *name)
{
	int rv, idx;
	phandle_t cnode;

	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	rv = ofw_bus_find_string_index(cnode, "reset-names", name, &idx);
	if (rv != 0)
		return (NULL);
	return (fdt_reset_get_by_idx(consumer, idx));
}

void
fdt_reset_release(reset_t rst)
{
	free(rst, M_OFWPROP);
}

int
fdt_reset_unreset_by_idx(device_t consumer, int idx)
{
	struct reset_handle *rst;
	int rv;

	rst = fdt_reset_get_by_idx(consumer, idx);
	if (rst == NULL)
		return (ENXIO);
	rv = fdt_reset_clear(consumer, rst);
	fdt_reset_release(rst);

	return (rv);
}

int
fdt_reset_unreset_by_name(device_t consumer, char *name)
{
	struct reset_handle *rst;
	int rv;

	rst = fdt_reset_get_by_name(consumer, name);
	if (rst == NULL)
		return (ENXIO);
	rv = fdt_reset_clear(consumer, rst);
	fdt_reset_release(rst);

	return (rv);
}

void
fdt_reset_register_provider(device_t provider)
{

	OF_device_register_xref(
	    OF_xref_from_node(ofw_bus_get_node(provider)), provider);
}

void
fdt_reset_unregister_provider(device_t provider)
{

	OF_device_register_xref(OF_xref_from_device(provider), NULL);
}
