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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef DEV_FDT_PHY_H
#define DEV_FDT_PHY_H

#include <dev/ofw/openfirm.h>
#include "fdt_phy_if.h"

typedef struct phy_handle *phy_t;

/*
 * Get/release phy handle from fdt data
 * name - name of phy in "phy-names" property
 * idx  - index in "phys" property
 * Returns handle on success or NULL.
 */
phy_t fdt_phy_get_by_name(device_t consumer, char *name);
phy_t fdt_phy_get_by_idx(device_t consumer, int idx);
void fdt_phy_release(phy_t phy);


/*
 * Enable/disable phy
 */
int fdt_phy_enable(device_t consumer, phy_t phy);
int fdt_phy_disable(device_t consumer, phy_t phy);
int fdt_phy_status(device_t consumer, phy_t phy, int *value);


/*
 *  Helper function for quick enable
 */
int fdt_phy_enable_by_name(device_t consumer, char *name);
int fdt_phy_enable_by_idx(device_t consumer, int idx);

/*
 * [Un]register the given device instance as a driver that implements the
 * fdt_phy interface.
 */
void fdt_phy_register_provider(device_t provider);
void fdt_phy_unregister_provider(device_t provider);

#endif /* DEV_FDT_PHY_H */
