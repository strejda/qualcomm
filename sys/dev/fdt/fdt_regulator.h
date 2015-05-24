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

#ifndef DEV_FDT_REGULATOR_H
#define DEV_FDT_REGULATOR_H

#include <dev/ofw/openfirm.h>
#include "fdt_regulator_if.h"

typedef struct regulator_handle *regulator_t;

/*
 * Get/release regulator handle from fdt data
 * name - name of supply property
 * Returns handle on success or NULL.
 */
regulator_t fdt_regulator_get_by_name(device_t consumer, char *name);
void fdt_regulator_release(regulator_t reg);


/*
 * Enable/disable regulator
 */
int fdt_regulator_enable(device_t consumer, regulator_t reg);
int fdt_regulator_disable(device_t consumer, regulator_t reg);
int fdt_regulator_status(device_t consumer, regulator_t reg, int *value);
/*
 * get/set_voltage
 * get/set_current
 * get_load
 */
/*
 *  Helper function for quick enable
 */
int fdt_regulator_enable_by_name(device_t consumer, char *name);

/*
 * [Un]register the given device instance as a driver that implements the
 * fdt_regulator interface.
 */
void fdt_regulator_register_provider(device_t provider, phandle_t node);
void fdt_regulator_unregister_provider(device_t provider, phandle_t node);

#endif /* DEV_FDT_REGULATOR_H */
