/*-
 * Copyright (c) 2015 Michal Meloun
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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_regulator.h>

struct regulator_handle {
	device_t 	device;		/* provider device*/
	uintptr_t	regulator_id;		/* regulator id */
};

int fdt_regulator_default_map(device_t provider, phandle_t xref, int ncells,
    pcell_t *cells, int *num)
{
	if (ncells == 0) {
		*num = (uintptr_t)xref;
		return (0);
	}
	if (ncells == 1) {
		*num = (int)cells[0];
		return (0);
	}
	return (ERANGE);
}

int
fdt_regulator_enable(device_t consumer, regulator_t reg)
{

	return (FDT_REGULATOR_SET(reg->device, reg->regulator_id, 1));
}

int
fdt_regulator_disable(device_t consumer, regulator_t reg)
{

	return (FDT_REGULATOR_SET(reg->device, reg->regulator_id, 0));
}

int
fdt_regulator_status(device_t consumer, regulator_t reg, int *value)
{

	return (FDT_REGULATOR_GET(reg->device, reg->regulator_id, value));
}

regulator_t
fdt_regulator_get_by_name(device_t consumer, char *name)
{
	struct regulator_handle *reg;
	phandle_t cnode, xnode;
	pcell_t *cells;
	device_t regdev;
	int ncells, rv;
	intptr_t id;

	/* Get data from FDT */
	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	cells = NULL;
	ncells = OF_getencprop_alloc(cnode, name,  sizeof(*cells),
	    (void **)&cells);
	if (ncells <= 0)
		return (NULL);

	xnode = cells[0];
	/* Tranlate provider to device */
	regdev = OF_device_from_xref(xnode);
	if (regdev == NULL) {
		free(cells, M_OFWPROP);
		return (NULL);
	}
	/* Map regulator to number */
	rv = FDT_REGULATOR_MAP(regdev, xnode, ncells - 1, cells + 1, &id);
	free(cells, M_OFWPROP);
	if (rv != 0)
		return (NULL);

	/* Create handle */
	reg = malloc(sizeof(struct regulator_handle), M_OFWPROP, M_WAITOK);
	reg->device =  regdev;
	reg->regulator_id = id;
	return (reg);
}

void
fdt_regulator_release(regulator_t reg)
{
	free(reg, M_OFWPROP);
}

int
fdt_regulator_enable_by_name(device_t consumer, char *name)
{
	struct regulator_handle *reg;
	int rv;

	reg = fdt_regulator_get_by_name(consumer, name);
	if (reg == NULL)
		return (ENXIO);
	rv = fdt_regulator_enable(consumer, reg);
	fdt_regulator_release(reg);

	return (rv);
}

void
fdt_regulator_register_provider(device_t provider, phandle_t node)
{

	if (node == 0)
		node = ofw_bus_get_node(provider);
	node = OF_xref_from_node(node);
	OF_device_register_xref(node, provider);
}

void
fdt_regulator_unregister_provider(device_t provider, phandle_t node)
{

	if (node == 0)
		node = ofw_bus_get_node(provider);
	node = OF_xref_from_node(node);
	OF_device_register_xref(node, NULL);
}
