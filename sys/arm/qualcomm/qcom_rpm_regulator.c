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

#include <dev/fdt/fdt_regulator.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <gnu/dts/include/dt-bindings/mfd/qcom-rpm.h>

#include "qcom_rpm_if.h"


#define MAX_REQUEST_LEN 2
#define FORCE_MODE_IS_2_BITS(sc) \
	(((sc)->req_def->fm.mask >> (sc)->req_def->fm.shift) == 3)

struct req_field {
	uint32_t	word;
	uint32_t	mask;
	int		shift;
};

struct req_def {
	struct req_field mV;		/* used if voltage is in mV */
	struct req_field uV;		/* used if voltage is in uV */
	struct req_field ip;		/* peak current in mA */
	struct req_field pd;		/* pull down enable */
	struct req_field ia;		/* average current in mA */
	struct req_field fm;		/* force mode */
	struct req_field pm;		/* power mode */
	struct req_field pc;		/* pin control */
	struct req_field pf;		/* pin function */
	struct req_field enable_state;	/* NCP and switch */
	struct req_field comp_mode;	/* NCP */
	struct req_field freq;		/* frequency: NCP and SMPS */
	struct req_field freq_clk_src;	/* clock source: SMPS */
	struct req_field hpm;		/* switch: control OCP and SS */
	int request_len;
};

struct qcom_rpm_part_def {
	char		*name;
	struct req_def	*req_def;
	int 		have_force_mode_auto;
	int		have_force_mode_bypass;
};

static struct req_def rpm8960_ldo_fields = {
	.request_len	= 2,
	.uV		= { 0, 0x007FFFFF,  0 },
	.pd		= { 0, 0x00800000, 23 },
	.pc		= { 0, 0x0F000000, 24 },
	.pf		= { 0, 0xF0000000, 28 },
	.ip		= { 1, 0x000003FF,  0 },
	.ia		= { 1, 0x000FFC00, 10 },
	.fm		= { 1, 0x00700000, 20 },
};

static struct req_def rpm8960_smps_fields	= {
	.request_len	= 2,
	.uV		= { 0, 0x007FFFFF,  0 },
	.pd		= { 0, 0x00800000, 23 },
	.pc		= { 0, 0x0F000000, 24 },
	.pf		= { 0, 0xF0000000, 28 },
	.ip		= { 1, 0x000003FF,  0 },
	.ia		= { 1, 0x000FFC00, 10 },
	.fm		= { 1, 0x00700000, 20 },
	.pm		= { 1, 0x00800000, 23 },
	.freq		= { 1, 0x1F000000, 24 },
	.freq_clk_src	= { 1, 0x60000000, 29 },
};

static struct req_def rpm8960_switch_fields = {
	.request_len	= 1,
	.enable_state	= { 0, 0x00000001,  0 },
	.pd		= { 0, 0x00000002,  1 },
	.pc		= { 0, 0x0000003C,  2 },
	.pf		= { 0, 0x000003C0,  6 },
	.hpm		= { 0, 0x00000C00, 10 },
};

static struct req_def rpm8960_ncp_fields = {
	.request_len	= 1,
	.uV		= { 0, 0x007FFFFF,  0 },
	.enable_state	= { 0, 0x00800000, 23 },
	.comp_mode	= { 0, 0x01000000, 24 },
	.freq		= { 0, 0x3E000000, 25 },
};


/*
 * PM8921 regulators
 */
static struct qcom_rpm_part_def pm8921_pldo = {
	.name = "PM8921 pldo regulator",
	.req_def = &rpm8960_ldo_fields,
	.have_force_mode_bypass = 1,
};

static struct qcom_rpm_part_def pm8921_nldo = {
	.name = "PM8921 nldo regulator",
	.req_def = &rpm8960_ldo_fields,
	.have_force_mode_bypass = 1,
};

static struct qcom_rpm_part_def pm8921_nldo1200 = {
	.name = "PM8921 nldo1200 regulator",
	.req_def = &rpm8960_ldo_fields,
	.have_force_mode_bypass = 1,
};

static struct qcom_rpm_part_def pm8921_smps = {
	.name = "PM8921 smps regulator",
	.req_def = &rpm8960_smps_fields,
	.have_force_mode_auto = 1,
};

static struct qcom_rpm_part_def pm8921_ftsmps = {
	.name = "PM8921 ftsmp regulator",
	.req_def = &rpm8960_smps_fields,
	.have_force_mode_auto = 1,
};

static struct qcom_rpm_part_def pm8921_ncp = {
	.name = "PM8921 ncp regulator",
	.req_def = &rpm8960_ncp_fields,
};

static struct qcom_rpm_part_def pm8921_switch = {
	.name = "PM8921 switch regulator",
	.req_def = &rpm8960_switch_fields,
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,rpm-pm8921-pldo",	(uintptr_t)&pm8921_pldo },
	{"qcom,rpm-pm8921-nldo",	(uintptr_t)&pm8921_nldo},
	{"qcom,rpm-pm8921-nldo1200",	(uintptr_t)&pm8921_nldo1200},
	{"qcom,rpm-pm8921-smps",	(uintptr_t)&pm8921_smps},
	{"qcom,rpm-pm8921-ftsmps",	(uintptr_t)&pm8921_ftsmps},
	{"qcom,rpm-pm8921-ncp",		(uintptr_t)&pm8921_ncp},
	{"qcom,rpm-pm8921-switch",	(uintptr_t)&pm8921_switch},

	{NULL,				(uintptr_t)NULL},
};

struct rpm_reg_softc {
	device_t		dev;
	struct mtx		mtx;

	int 			id;
	uint32_t 		req_buf[MAX_REQUEST_LEN];
	struct req_def		*req_def;
	int 			have_force_mode_auto;
	int			have_force_mode_bypass;
	int			min_uvolt;
	int			max_uvolt;
	int			cur_uvolt;

/*

	int uV;
	int is_enabled;

*/
};

/* Set single field in request buffer */
static void
rpm_reg_set_field(struct rpm_reg_softc *sc, struct req_field *req, int value)
{

	if (req->mask == 0 || (value << req->shift) & ~req->mask)
		panic("Bad request");

	sc->req_buf[req->word] &= ~req->mask;
	sc->req_buf[req->word] |= value << req->shift;
}

static int
rpm_reg_write(struct rpm_reg_softc *sc)
{
	int rv;

	rv = QCOM_RPM_WRITE_CMD(device_get_parent(sc->dev), sc->dev,
	    QCOM_RPM_ACTIVE_STATE, sc->id, sc->req_buf,
	    sc->req_def->request_len);

//	uint32_t buf[2];
//	rv = QCOM_RPM_READ_STATUS(device_get_parent(sc->dev), sc->dev,
//	    QCOM_RPM_ACTIVE_STATE, sc->id, buf,
//	    sc->req_def->request_len);
//printf("%s: id: %d, len: %d\n", __func__, sc->id, sc->req_def->request_len);
//printf(" reqbuf: 0x%08X, 0x%08X\n", sc->req_buf[0], sc->req_buf[1]);
//printf(" respbuf: 0x%08X, 0x%08X\n", buf[0], buf[1]);
//printf(" -------------------------------------\n");
	return (rv);
}

static int
rpm_reg_regulator_set(device_t dev, intptr_t id, int val)
{
	struct rpm_reg_softc *sc;
	struct req_field *fld;
	int regval;

	sc = device_get_softc(dev);

	val = val ? 1 : 0;
	fld = NULL;
 	if (sc->req_def->enable_state.mask != 0) {
 		/* Enable / disable pover switch */
		fld = &sc->req_def->enable_state;
		regval = val;
		if (bootverbose)
			device_printf(sc->dev, "%s power switch\n",
			val != 0 ? "Enabling": "Disabling");
	} else if (sc->req_def->uV.mask) {
		/* uV based regulator */
		fld = &sc->req_def->uV;
		regval = val != 0 ? sc->cur_uvolt : 0;
		if (bootverbose)
			device_printf(sc->dev, "%s regulator, voltage %d uV\n",
			val != 0 ? "Enabling": "Disabling", regval);
	} else if ( sc->req_def->mV.mask != 0) {
		fld = &sc->req_def->mV;
		regval = val != 0 ? sc->cur_uvolt / 1000 : 0;
		if (bootverbose)
			device_printf(sc->dev, "%s regulator, voltage %d mV\n",
			val != 0 ? "Enabling": "Disabling", regval);
	}

	if (fld  == NULL) {
		device_printf(sc->dev, "Cannot enable power\n");
		return (ENXIO);
	}
	rpm_reg_set_field(sc, fld, regval);
	return (rpm_reg_write(sc));
}

static int
rpm_reg_parse_freq(struct rpm_reg_softc *sc, phandle_t node)
{
	static int freq_table[] = {
		19200000, 9600000, 6400000, 4800000, 3840000, 3200000, 2740000,
		2400000, 2130000, 1920000, 1750000, 1600000, 1480000, 1370000,
		1280000, 1200000,

	};
	uint32_t val;
	int i, rv;

	rv = OF_getencprop(node, "qcom,switch-mode-frequency", &val,
	    sizeof(val));
	if (rv <= 0) {
		device_printf(sc->dev,
		    "Cannot read witch-mode-frequency property\n");
		return (ENXIO);
	}

	for (i = 0; i < nitems(freq_table); i++) {
		if (val == freq_table[i]) {
			rpm_reg_set_field(sc, &sc->req_def->freq, i + 1);
			return (0);
		}
	}

	device_printf(sc->dev, "Cannot parse witch-mode-frequency property\n");
	return (ENXIO);
}

static int
rpm_reg_probe(device_t dev)
{
	struct qcom_rpm_part_def *def;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	def = (struct qcom_rpm_part_def *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (def == NULL)
		return (ENXIO);

	device_set_desc(dev, def->name);
	return (BUS_PROBE_DEFAULT);
}

static int
rpm_reg_attach(device_t dev)
{
	struct rpm_reg_softc *sc;
	phandle_t node;
	int rv, val, mode;

struct qcom_rpm_part_def *def;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	def = (struct qcom_rpm_part_def *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	if (def == NULL)
		return (ENXIO);
	sc->req_def = def->req_def;
	sc->have_force_mode_auto = def->have_force_mode_auto;
	sc->have_force_mode_bypass = def->have_force_mode_bypass;

	rv = OF_getencprop(node, "regulator-min-microvolt", &sc->min_uvolt,
	    sizeof(sc->min_uvolt));
	if (rv <= 0)
		sc->min_uvolt = 0;

	rv = OF_getencprop(node, "regulator-max-microvolt", &sc->max_uvolt,
	    sizeof(sc->max_uvolt));
	if (rv <= 0)
		sc->max_uvolt = 0;

	rv = OF_getencprop(node, "reg", &sc->id, sizeof(sc->id));
	if (rv <= 0) {
		device_printf(sc->dev, "Cannot read register property\n");
		return (ENXIO);
	}

	if (OF_hasprop(node, "bias-pull-down"))
		rpm_reg_set_field(sc, &sc->req_def->pd, 1);

	if (sc->req_def->freq.mask != 0) {
		rv = rpm_reg_parse_freq(sc, node);
		if (rv < 0)
		return (rv);
	}

	if (sc->req_def->pm.mask != 0) {
		val = OF_hasprop(node, "qcom,power-mode-hysteretic") ? 0 : 1;
		rpm_reg_set_field(sc, &sc->req_def->pm, val);
	}

	if (sc->req_def->fm.mask != 0) {
		mode = -1;

		rv = OF_getencprop(node, "qcom,force-mode", &val, sizeof(val));
		if (rv <= 0)
			val = QCOM_RPM_FORCE_MODE_NONE;

		switch (val) {
		case QCOM_RPM_FORCE_MODE_NONE:
			mode = 0;
			break;
		case QCOM_RPM_FORCE_MODE_LPM:
			mode = 1;
			break;
		case QCOM_RPM_FORCE_MODE_HPM:
			if (FORCE_MODE_IS_2_BITS(sc))
				mode = 2;
			else
				mode = 3;
			break;
		case QCOM_RPM_FORCE_MODE_AUTO:
			if (sc->have_force_mode_auto)
				mode = 2;
			break;
		case QCOM_RPM_FORCE_MODE_BYPASS:
			if (sc->have_force_mode_bypass)
				mode = 4;
			break;
		}

		if (mode == -1) {
			device_printf(sc->dev, "Cannot parse force mode\n");
			return (ENXIO);
		}
		rpm_reg_set_field(sc, &sc->req_def->fm, mode);
	}

	sc->cur_uvolt = sc->max_uvolt;
	fdt_regulator_register_provider(dev, node);
	return (0);
}

static int
rpm_reg_detach(device_t dev)
{
	return (EBUSY);
}

static device_method_t rpm_reg_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rpm_reg_probe),
	DEVMETHOD(device_attach,	rpm_reg_attach),
	DEVMETHOD(device_detach,	rpm_reg_detach),

	/* Regulator interface */
	DEVMETHOD(fdt_regulator_set,	rpm_reg_regulator_set),

	DEVMETHOD_END
};

static devclass_t rpm_reg_devclass;

static driver_t rpm_reg_driver = {
	"rpm_reg",
	rpm_reg_methods,
	sizeof(struct rpm_reg_softc),
};

EARLY_DRIVER_MODULE(rpm_reg, qcom_rpm, rpm_reg_driver, rpm_reg_devclass, 0,
    0, 73);
