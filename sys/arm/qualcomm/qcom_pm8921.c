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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "qcom_ssbi_if.h"

/*
 * this driver is simple placeholder for future interrupt controller
 */

struct pm8921_softc {
	struct simplebus_softc	simplebus_sc;
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,pm8921",		1},
	{NULL,			0},
};


static int
pm8921_read(device_t dev, device_t consumer, uint32_t addr, uint8_t *buf, int len)
{
	int rv;

	rv = QCOM_SSBI_READ(device_get_parent(dev), consumer, addr, buf, len);
	return (rv);
}

static int
pm8921_write(device_t dev, device_t consumer, uint32_t addr, uint8_t *buf,
    int len)
{
	int rv;

	rv = QCOM_SSBI_WRITE(device_get_parent(dev), consumer, addr, buf, len);
	return (rv);
}

static int
pm8921_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm PM8921 core");
	return (BUS_PROBE_DEFAULT);
}

static int
pm8921_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
pm8921_attach(device_t dev)
{
	struct pm8921_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(sc->dev);

	simplebus_init(dev, 0);
	for (node = OF_child(node); node > 0; node = OF_peer(node))
		simplebus_add_device(dev, node, 0, NULL, -1, NULL);
	return (bus_generic_attach(dev));
}

static device_method_t qcom_pm8921_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		pm8921_probe),
	DEVMETHOD(device_attach,	pm8921_attach),
	DEVMETHOD(device_detach,	pm8921_detach),

	/* PM8921 interface */
	DEVMETHOD(qcom_ssbi_write,	pm8921_write),
	DEVMETHOD(qcom_ssbi_read,	pm8921_read),

	DEVMETHOD_END
};

static devclass_t qcom_pm8921_devclass;
DEFINE_CLASS_1(qcom_pm8921, qcom_pm8921_driver, qcom_pm8921_methods,
    sizeof(struct pm8921_softc), simplebus_driver);
DRIVER_MODULE(qcom_pm8921, qcom_ssbi, qcom_pm8921_driver, qcom_pm8921_devclass,
    0, 0);
