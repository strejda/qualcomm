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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_phy.h>

struct phy_handle {
	device_t 	device;		/* provider device*/
	uintptr_t	phy_id;		/* phy id */
};

int fdt_phy_default_map(device_t provider, phandle_t xref, int ncells,
    pcell_t *cells, int *num)
{
	if (ncells == 0) {
		*num = 1;
		return (0);
	}
	if (ncells == 1) {
		*num = (int)cells[0];
		return (0);
	}
	return (ERANGE);
}

int
fdt_phy_enable(device_t consumer, phy_t phy)
{

	return (FDT_PHY_SET(phy->device, phy->phy_id, 1));
}

int
fdt_phy_disable(device_t consumer, phy_t phy)
{

	return (FDT_PHY_SET(phy->device, phy->phy_id, 0));
}

int
fdt_phy_status(device_t consumer, phy_t phy, int *value)
{

	return (FDT_PHY_GET(phy->device, phy->phy_id, value));
}

phy_t
fdt_phy_get_by_property(device_t consumer, char *name)
{
	struct phy_handle *phy;
	phandle_t cnode, phynode;
	device_t phydev;
	int rv;
	intptr_t id;

	/* Get data from FDT */

	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	rv = OF_getencprop(cnode, name, &phynode, sizeof(phy));
	if (rv <= 0)
		return (NULL);

	/* Tranlate provider to device */
	phydev = OF_device_from_xref(phynode);
	if (phydev == NULL) {
		return (NULL);
	}
	/* Map phy to number */
	rv = FDT_PHY_MAP(phydev, phynode, 0, NULL, &id);
	if (rv != 0)
		return (NULL);

	/* Create handle */
	phy = malloc(sizeof(struct phy_handle), M_OFWPROP, M_WAITOK);
	phy->device =  phydev;
	phy->phy_id = id;
	return (phy);
}

phy_t
fdt_phy_get_by_idx(device_t consumer, int idx)
{
	struct phy_handle *phy;
	phandle_t cnode, xnode;
	pcell_t *cells;
	device_t phydev;
	int ncells, rv;
	intptr_t id;

	/* Get data from FDT */
	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	rv = ofw_bus_parse_xref_list_alloc(cnode, "phys", "#phy-cells", idx,
	    &xnode, &ncells, &cells);
	if (rv != 0)
		return (NULL);

	/* Tranlate provider to device */
	phydev = OF_device_from_xref(xnode);
	if (phydev == NULL) {
		free(cells, M_OFWPROP);
		return (NULL);
	}
	/* Map phy to number */
	rv = FDT_PHY_MAP(phydev, xnode, ncells, cells, &id);
	free(cells, M_OFWPROP);
	if (rv != 0)
		return (NULL);

	/* Create handle */
	phy = malloc(sizeof(struct phy_handle), M_OFWPROP, M_WAITOK);
	phy->device =  phydev;
	phy->phy_id = id;
	return (phy);
}

phy_t
fdt_phy_get_by_name(device_t consumer, char *name)
{
	int rv, idx;
	phandle_t cnode;

	cnode = ofw_bus_get_node(consumer);
	if (cnode <= 0)
		return (NULL);
	rv = ofw_bus_find_string_index(cnode, "phy-names", name, &idx);
	if (rv != 0)
		return (NULL);
	return (fdt_phy_get_by_idx(consumer, idx));
}

void
fdt_phy_release(phy_t phy)
{
	free(phy, M_OFWPROP);
}

int
fdt_phy_enable_by_idx(device_t consumer, int idx)
{
	struct phy_handle *phy;
	int rv;

	phy = fdt_phy_get_by_idx(consumer, idx);
	if (phy == NULL)
		return (ENXIO);
	rv = fdt_phy_enable(consumer, phy);
	fdt_phy_release(phy);

	return (rv);
}

int
fdt_phy_enable_by_name(device_t consumer, char *name)
{
	struct phy_handle *phy;
	int rv;

	phy = fdt_phy_get_by_name(consumer, name);
	if (phy == NULL)
		return (ENXIO);
	rv = fdt_phy_enable(consumer, phy);
	fdt_phy_release(phy);

	return (rv);
}

void
fdt_phy_register_provider(device_t provider)
{

	OF_device_register_xref(
	    OF_xref_from_node(ofw_bus_get_node(provider)), provider);
}

void
fdt_phy_unregister_provider(device_t provider)
{

	OF_device_register_xref(OF_xref_from_device(provider), NULL);
}
