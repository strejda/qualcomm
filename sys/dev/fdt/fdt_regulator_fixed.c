/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Redistribution and use in source and binullry forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binullry form must reproduce the above copyright
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/fdt/fdt_regulator.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>


struct regulator_fixed_softc {
	device_t		dev;
};

static struct ofw_compat_data compat_data[] = {
	{"regulator-fixed",		1},
	{NULL,				0},
};


static int
regulator_fixed_set(device_t provider, intptr_t id, int val)
{

	return (0);
}

static int
regulator_fixed_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "fixed regulator");
	return (BUS_PROBE_DEFAULT);
}

static int
regulator_fixed_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
regulator_fixed_attach(device_t dev)
{
	struct regulator_fixed_softc * sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	fdt_regulator_register_provider(dev, 0);

	return (0);
}

static device_method_t regulator_fixed_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		regulator_fixed_probe),
	DEVMETHOD(device_attach,	regulator_fixed_attach),
	DEVMETHOD(device_detach,	regulator_fixed_detach),

	DEVMETHOD(fdt_regulator_set,	regulator_fixed_set),

	DEVMETHOD_END
};

static driver_t regulator_fixed_driver = {
	"regulator_fixed",
	regulator_fixed_methods,
	sizeof(struct regulator_fixed_softc),
};

static devclass_t regulator_fixed_devclass;
EARLY_DRIVER_MODULE(regulator_fixed, simplebus, regulator_fixed_driver,
   regulator_fixed_devclass, 0, 0, 70);
