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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "clkdev_if.h"
#include "qcom_qfprom_if.h"
#include "qcom_tsens_if.h"

#define TSENS_LOCK(_sc)			mtx_lock(&(_sc)->mtx)
#define	TSENS_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define TSENS_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "tsens", MTX_DEF)
#define TSENS_LOCK_DESTROY(_sc)		mtx_destroy(&_sc->mtx);
#define TSENS_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED);
#define TSENS_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);

#define	WR4(_sc, off, val)						\
	CLKDEV_WRITE(device_get_parent((_sc)->dev), off, val)
#define	MD4(_sc, off, clr, set )					\
	CLKDEV_MODIFY(device_get_parent((_sc)->dev), off, clr, set)
#define	RD4(_sc, off)						\
	tsens_read(device_get_parent((_sc)->dev), off)

#define CAL_MDEGC			30000
#define DEF_SLOPE			1150

#define BASE_OFFSET			-5717
#define BASE_SLOPE			7623

#define TSENS_CNTL			0x3620
#define  CNTL_TSENS_SLP_CLK_ENA			(1 << 26)
#define  CNTL_MEASURE_PERIOD(x)			((x) << 18)
#define  CNTL_SENSOR0_EN(x)			((x) << 3)
#define  CNTL_TSENS_ADC_CLK_SEL			(1 << 2)
#define  CNTL_SENS_SW_RST			(1 << 1)
#define  CNTL_TSENS_EN				(1 << 0)

#define TSENS_CONFIG			0x3640
#define TSENS_STATUS_CNTL		0x3660
#define  STATUS_TSENS_MIN_STATUS_MASK		(1 << 0)
#define  STATUS_TSENS_LOWER_STATUS_CLR		(1 << 1)
#define  STATUS_TSENS_UPPER_STATUS_CLR		(1 << 2)
#define  STATUS_TSENS_MAX_STATUS_MASK		(1 << 0)

#define TSENS_THRESHOLD			0x3624
#define  THRESHOLD_MAX_LIMIT_TH(x)		((x) << 24)
#define  THRESHOLD_MIN_LIMIT_TH(x)		((x) << 16)
#define  THRESHOLD_UPPER_LIMIT_TH(x)		((x) <<  8)
#define  THRESHOLD_LOWER_LIMIT_TH(x)		((x) <<  0)

#define DEFAUT_THRESHOLD						\
	(THRESHOLD_MAX_LIMIT_TH(0xff)	|				\
	THRESHOLD_MIN_LIMIT_TH(0x00)	|				\
	THRESHOLD_UPPER_LIMIT_TH(0xdf)	|				\
	THRESHOLD_LOWER_LIMIT_TH(0x50))

#define TSENS_INT_STATUS		0x363c
#define  INT_STATUS_TRDY			(1 << 7)

/* Status register bits */
#define  STATUS_LAST_TEMP_MASK			0xFF


bus_addr_t status_regs[] =
{
	0x3628,
	0x362C,
	0x3630,
	0x3634,
	0x3638,
	0x3664,
	0x3668,
	0x366C,
	0x3670,
	0x3674,
	0x3678
};


struct tsensor {
	uintptr_t	id;
	int		hwid;
	int 		slope;
	int 		offset;
	int		have_first_sample;
	bus_addr_t 	status;
};

struct tsens_softc {
	device_t	dev;
	struct mtx	mtx;
	uintptr_t	type;

	device_t	qprom_dev;
	int		ntsensors;
	struct tsensor	*tsensors;
};

#define TYPE_APQ8064_TSENS	1

static struct ofw_compat_data compat_data[] = {
	{"qcom,apq8064-tsens",	TYPE_APQ8064_TSENS},
	{NULL,			0},
};

static inline uint32_t
tsens_read(device_t dev, bus_addr_t off)
{
	uint32_t val;

	CLKDEV_READ(dev, off, &val);
	return(val);
}

static int
tsens_convert_raw(struct tsensor *tsensor, uint32_t raw)
{
	int val;

	val = raw & STATUS_LAST_TEMP_MASK;
	val *= tsensor->slope;
	val += tsensor->offset;
	return (val);
}



static int
tsens_read_temp(struct tsens_softc *sc, uintptr_t id, int *temp)
{
	struct tsensor *tsensor;
	uint32_t tmp;

	if (id >= sc->ntsensors) {
		*temp = 0;
		return (ERANGE);
	}
	tsensor = sc->tsensors + id;

	/* wait for first sample */
	if (tsensor->have_first_sample == 0) {
		for (;;) {
			tmp = RD4(sc, TSENS_INT_STATUS);
			if ((tmp & INT_STATUS_TRDY) != 0)
				break;
			DELAY(1000);
		}
		tsensor->have_first_sample = 1;
	}

	tmp = RD4(sc, tsensor->status);
	*temp = tsens_convert_raw(tsensor, tmp);
	return 0;
}

static int
tsens_get_temperature(device_t dev, device_t cdev, uintptr_t id, int *val)
{
 	struct tsens_softc *sc;

	sc = device_get_softc(dev);
	if (id >= sc->ntsensors)
		return (ERANGE);
	return(tsens_read_temp(sc, id, val));
}


/* XXX Temporary hack */
static void
tsens_init_calibration(struct tsens_softc *sc)
{
	int rv;
	phandle_t node, pnode, eeproms[2];

	node = ofw_bus_get_node(sc->dev);
	rv = OF_getencprop(node, "eeproms", eeproms, sizeof(eeproms));
	if (rv != sizeof(eeproms))
		panic("Invalid \"eeproms\" property: %d\n", rv);
	pnode = OF_parent(OF_node_from_xref(eeproms[0]));
	sc->qprom_dev = OF_device_from_xref(OF_xref_from_node(pnode));
	if (sc->qprom_dev == NULL)
		panic("Cannot find qfuse device\n");
}

static void
tsens_init_sensors(struct tsens_softc *sc)
{
	uint32_t reg;

	/* Issue reset first */
	MD4(sc, TSENS_CNTL, 0, CNTL_SENS_SW_RST);

	/* setup magic to config */
	MD4(sc, TSENS_CONFIG, 0x0f, 0x9b);

	reg = CNTL_TSENS_SLP_CLK_ENA | CNTL_MEASURE_PERIOD(1) |
	   CNTL_SENSOR0_EN((1 << sc->ntsensors) - 1);
	WR4(sc, TSENS_CNTL, reg);

	/* Mask all interrups */
	MD4(sc, TSENS_STATUS_CNTL, 0, STATUS_TSENS_MIN_STATUS_MASK |
	    STATUS_TSENS_LOWER_STATUS_CLR |
	    STATUS_TSENS_UPPER_STATUS_CLR |
	    STATUS_TSENS_MAX_STATUS_MASK);

	reg |= CNTL_TSENS_EN;
	WR4(sc, TSENS_CNTL, reg);

	WR4(sc, TSENS_THRESHOLD, DEFAUT_THRESHOLD);
	WR4(sc, TSENS_CNTL, reg);
}

static int
tsens_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm temperature sensors");
	return (BUS_PROBE_DEFAULT);
}

static int
tsens_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
tsens_attach(device_t dev)
{
	struct tsens_softc *sc;
	phandle_t node;
	int nslopes, nhwids;
	int *slopes, *hwids;
	int i;
	uint8_t cal;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(sc->dev);
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	TSENS_LOCK_INIT(sc);
	nslopes = OF_getencprop_alloc(node, "qcom,tsens-slopes",
	    sizeof(*slopes), (void **)&slopes);
	nhwids = OF_getencprop_alloc(node, "qcom,sensor-id", sizeof(*hwids),
	    (void **)&hwids);

	if ((nslopes <= 0) && (nhwids <= 0)) {
		device_printf(sc->dev, "At least one of \"qcom,tsens-slopes\""
		" or \"qcom,sensor-id\" must be defined\n");
		goto fail;
	}

	if ((nslopes > 0) && (nhwids > 0) && (nslopes != nhwids) ) {
		device_printf(sc->dev, "Size of \"qcom,tsens-slopes\""
		" and \"qcom,sensor-id\" must be same\n");
		goto fail;
	}
	sc->ntsensors = nslopes;
	if (sc->ntsensors <= 0)
		sc->ntsensors = nhwids;
	sc->tsensors = malloc(sc->ntsensors * sizeof(struct tsensor), M_OFWPROP,
	    M_WAITOK | M_ZERO);
	tsens_init_calibration(sc);

	for (i = 0; i < sc->ntsensors; i++) {
		if (nslopes > 0)
			sc->tsensors[i].slope = slopes[i];
		else
			sc->tsensors[i].slope = BASE_SLOPE;
		sc->tsensors[i].id = i;
		if (nhwids > 0)
			sc->tsensors[i].hwid = hwids[i];
		else
		sc->tsensors[i].hwid = i;
		sc->tsensors[i].status =  status_regs[sc->tsensors[i].hwid];
		/* XXX Temporary hack until ve get NVRAM interface */
		cal = QCOM_QFPROM_READ_1(sc->qprom_dev, sc->dev, 0x404 + i);
		if (cal == 0)
			cal = QCOM_QFPROM_READ_1(sc->qprom_dev, sc->dev,
			    0x414 + i);
		if (cal == 0) {
			device_printf(sc->dev, "Cannot read calibration data "
			    " for sensor: %d\n", i);
			cal = 0x60;
		}
		sc->tsensors[i].offset = CAL_MDEGC -
		    sc->tsensors[i].slope * cal;
	}
	tsens_init_sensors(sc);

	OF_device_register_xref(OF_xref_from_node(node), dev);
//{
//int temp;
//for (i = 0; i < sc->ntsensors; i++)
// tsens_read_temp(sc, i, &temp);
//}
	if (nslopes > 0)
		free(slopes, M_OFWPROP);
	if (nhwids > 0)
		free(hwids, M_OFWPROP);

	return (0);

fail:
	if (nslopes > 0)
		free(slopes, M_OFWPROP);
	if (nhwids > 0)
		free(hwids, M_OFWPROP);
	if (sc->tsensors != NULL)
		free(sc->tsensors, M_OFWPROP);
	return (ENXIO);
}

static device_method_t qcom_tsens_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			tsens_probe),
	DEVMETHOD(device_attach,		tsens_attach),
	DEVMETHOD(device_detach,		tsens_detach),
	/* TSENS interface */
	DEVMETHOD(qcom_tsens_get_temperature,	tsens_get_temperature),

	DEVMETHOD_END
};

static devclass_t qcom_tsens_devclass;
DEFINE_CLASS_0(qcom_tsens, qcom_tsens_driver, qcom_tsens_methods,
    sizeof(struct tsens_softc));
EARLY_DRIVER_MODULE(qcom_tsens, apq8064_gcc, qcom_tsens_driver, qcom_tsens_devclass,
    0, 0, 79);
