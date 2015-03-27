#-
# Copyright (c) 2015 Michal Meloun
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# $FreeBSD$
#

#include <sys/types.h>
#include <dev/ofw/ofw_bus.h>

INTERFACE fdt_reset;
HEADER {
int fdt_reset_default_map(device_t , phandle_t, int, pcell_t *, intptr_t *);
}
#
# Assert/release given reset
# Returns 0 on success or a standard errno value.
#
METHOD int set {
	device_t	provider;
	intptr_t	id;
	int		value;
};

#
# get actual status of given reset
# Returns 0 on success or a standard errno value.
#
METHOD int get {
	device_t	provider;
	intptr_t	id;
	int		*value;
};

#
# map fdt property cells to reset number
# Returns 0 on success or a standard errno value.
#
METHOD int map {
	device_t	provider;
	phandle_t	xref;
	int		ncells;
	pcell_t		*cells;
	int		*num;
} DEFAULT fdt_reset_default_map;
