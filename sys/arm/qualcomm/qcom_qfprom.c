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

/*
 * This driver provide access to various unrelated bits packed to single
 * register space
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "qcom_qfprom_if.h"

#define QFPROM_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	QFPROM_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define QFPROM_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "qfprom", MTX_DEF)
#define QFPROM_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx);
#define QFPROMASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED);
#define QFPROM_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);

struct qfprom_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct mtx		mtx;
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,qfprom",	1},
	{NULL,		0}
};

static uint8_t
qfprom_read_1(device_t dev, device_t consumer, bus_size_t offset)
{
	struct qfprom_softc *sc;
	uint8_t val;

	sc = device_get_softc(dev);

	QFPROM_LOCK(sc);
	val = bus_read_1(sc->mem_res, offset);
	QFPROM_UNLOCK(sc);
	return (val);
}

static void
qfprom_write_1(device_t	dev, device_t consumer, bus_size_t offset, uint8_t val)
{
	struct qfprom_softc *sc;

	sc = device_get_softc(dev);

	QFPROM_LOCK(sc);
	bus_write_1(sc->mem_res, offset, val);
	QFPROM_UNLOCK(sc);
}

static void
qfprom_modify_1(device_t dev,  device_t consumer, bus_size_t offset,
    uint8_t clear_bits, uint8_t set_bits)
{
	struct qfprom_softc *sc;
	uint8_t val;

	sc = device_get_softc(dev);

	QFPROM_LOCK(sc);
	val = bus_read_1(sc->mem_res, offset);
	val &= ~clear_bits;
	val |= ~set_bits;
	bus_write_1(sc->mem_res, offset, val);
	QFPROM_UNLOCK(sc);
}

static int
qfprom_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "qfprom");
	return (BUS_PROBE_DEFAULT);
}

static int
qfprom_attach(device_t dev)
{
	struct qfprom_softc *sc;
	int rid;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(sc->dev);

	QFPROM_LOCK_INIT(sc);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource\n");
		return (ENXIO);
	}

	OF_device_register_xref(OF_xref_from_node(node), dev);
	return (0);
}

static int
qfprom_detach(device_t dev)
{
	struct qfprom_softc *sc;

	sc = device_get_softc(dev);

	OF_device_register_xref(OF_xref_from_device(dev), NULL);

	QFPROM_LOCK_DESTROY(sc);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	return (0);
}

static device_method_t qfprom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qfprom_probe),
	DEVMETHOD(device_attach,	qfprom_attach),
	DEVMETHOD(device_detach,	qfprom_detach),

	/* QFPROM interface */
	DEVMETHOD(qcom_qfprom_read_1,	qfprom_read_1),
	DEVMETHOD(qcom_qfprom_write_1,	qfprom_write_1),
	DEVMETHOD(qcom_qfprom_modify_1,	qfprom_modify_1),

	DEVMETHOD_END
};

DEFINE_CLASS_0(qfprom, qfprom_driver, qfprom_methods,
    sizeof(struct qfprom_softc));
static devclass_t qfprom_devclass;
EARLY_DRIVER_MODULE(qfprom, simplebus, qfprom_driver, qfprom_devclass, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(qfprom, 1);
