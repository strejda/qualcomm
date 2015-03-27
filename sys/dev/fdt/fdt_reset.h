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

#ifndef DEV_FDT_RESET_H
#define DEV_FDT_RESET_H

#include <dev/ofw/openfirm.h>
#include "fdt_reset_if.h"

typedef struct reset_handle *reset_t;

/*
 * Get/ release reset handle from fdt data
 * name - name of reset in "reset-names" property
 * idx  - index in "resets" property
 * Returns handle on success or NULL.
 */
reset_t fdt_reset_get_by_name(device_t consumer, char *name);
reset_t fdt_reset_get_by_idx(device_t consumer, int idx);
void fdt_reset_release(reset_t consumer);


/*
 * Assert/clear reset
 */
int fdt_reset_assert(device_t consumer, reset_t rst);
int fdt_reset_clear(device_t consumer, reset_t rst);
int fdt_reset_status(device_t consumer, reset_t rst, int *value);

/*
 *  Helper function for quick unreset
 */
int fdt_reset_unreset_by_name(device_t consumer, char *name);
int fdt_reset_unreset_by_idx(device_t consumer, int idx);

/*
 * [Un]register the given device instance as a driver that implements the
 * fdt_reset interface.
 */
void fdt_reset_register_provider(device_t provider);
void fdt_reset_unregister_provider(device_t provider);

#endif /* DEV_FDT_RESET_H */
