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

MALLOC_DEFINE(M_RPM_REG, "PRM regulator", "RM regulator");

#define MAX_REQUEST_LEN 2
#define FORCE_MODE_IS_2_BITS(sc) \
	(((sc)->fields_def->fm.mask >> (sc)->fields_def->fm.shift) == 3)

struct rpm_buf_field {
	uint32_t	word;
	uint32_t	mask;
	int		shift;
};

struct rmp_fields_def {
	struct rpm_buf_field	mV;		/* used if voltage is in mV */
	struct rpm_buf_field	uV;		/* used if voltage is in uV */
	struct rpm_buf_field	ip;		/* peak current in mA */
	struct rpm_buf_field	pd;		/* pull down enable */
	struct rpm_buf_field	ia;		/* average current in mA */
	struct rpm_buf_field	fm;		/* force mode */
	struct rpm_buf_field	pm;		/* power mode */
	struct rpm_buf_field	pc;		/* pin control */
	struct rpm_buf_field	pf;		/* pin function */
	struct rpm_buf_field	enable_state;	/* NCP and switch */
	struct rpm_buf_field	comp_mode;	/* NCP */
	struct rpm_buf_field	freq;		/* frequency: NCP and SMPS */
	struct rpm_buf_field	freq_clk_src;	/* clock source: SMPS */
	struct rpm_buf_field	hpm;		/* switch: control OCP and SS */
	int			buf_len;
};

struct rmp_type_def {
	struct rmp_fields_def	*fields_def;
	int 			force_mode_auto;
	int			force_mode_bypass;
};


struct qcom_rpm_regs_softc {
	device_t		dev;
	struct rmp_reg_list	*reg_list;
	int			reg_count;
	struct rmp_reg	**regs;
};

struct rmp_reg {
	struct qcom_rpm_regs_softc *sc;
	phandle_t		xref;

	int 			id;
	uint32_t 		req_buf[MAX_REQUEST_LEN];
	struct rmp_fields_def	*fields_def;
	int 			force_mode_auto;
	int			force_mode_bypass;
	int			min_uvolt;
	int			max_uvolt;
	int			cur_uvolt;
};

static struct rmp_fields_def rpm8960_ldo_fields = {
	.buf_len	= 2,
	.uV		= { 0, 0x007FFFFF,  0 },
	.pd		= { 0, 0x00800000, 23 },
	.pc		= { 0, 0x0F000000, 24 },
	.pf		= { 0, 0xF0000000, 28 },
	.ip		= { 1, 0x000003FF,  0 },
	.ia		= { 1, 0x000FFC00, 10 },
	.fm		= { 1, 0x00700000, 20 },
};

static struct rmp_fields_def rpm8960_smps_fields	= {
	.buf_len	= 2,
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

static struct rmp_fields_def rpm8960_switch_fields = {
	.buf_len	= 1,
	.enable_state	= { 0, 0x00000001,  0 },
	.pd		= { 0, 0x00000002,  1 },
	.pc		= { 0, 0x0000003C,  2 },
	.pf		= { 0, 0x000003C0,  6 },
	.hpm		= { 0, 0x00000C00, 10 },
};

static struct rmp_fields_def rpm8960_ncp_fields = {
	.buf_len	= 1,
	.uV		= { 0, 0x007FFFFF,  0 },
	.enable_state	= { 0, 0x00800000, 23 },
	.comp_mode	= { 0, 0x01000000, 24 },
	.freq		= { 0, 0x3E000000, 25 },
};


/*
 * PM8921 regulators
 */
static struct rmp_type_def pm8921_pldo = {
	.fields_def = &rpm8960_ldo_fields,
	.force_mode_bypass = 1,
};

static struct rmp_type_def pm8921_nldo = {
	.fields_def = &rpm8960_ldo_fields,
	.force_mode_bypass = 1,
};

static struct rmp_type_def pm8921_nldo1200 = {
	.fields_def = &rpm8960_ldo_fields,
	.force_mode_bypass = 1,
};

static struct rmp_type_def pm8921_smps = {
	.fields_def = &rpm8960_smps_fields,
	.force_mode_auto = 1,
};

static struct rmp_type_def pm8921_ncp = {
	.fields_def = &rpm8960_ncp_fields,
};

static struct rmp_type_def pm8921_switch = {
	.fields_def = &rpm8960_switch_fields,
};


/* Structure s for definition of regulator chip */
struct rmp_reg_list {
	char			*name;
	int			id;
	struct rmp_type_def 	*type_def;
	char			*supply;
};

struct rpm_chip_def {
	char			*chip_name;
	struct rmp_reg_list	*reg_list;
	int			reg_count;
};

static struct rmp_reg_list rpm_pm8921_list[] = {
	{ "s1",  QCOM_RPM_PM8921_SMPS1, &pm8921_smps, "vdd_s1" },
	{ "s2",  QCOM_RPM_PM8921_SMPS2, &pm8921_smps, "vdd_s2" },
	{ "s3",  QCOM_RPM_PM8921_SMPS3, &pm8921_smps },
	{ "s4",  QCOM_RPM_PM8921_SMPS4, &pm8921_smps, "vdd_s4" },
	{ "s7",  QCOM_RPM_PM8921_SMPS7, &pm8921_smps, "vdd_s7" },
	{ "s8",  QCOM_RPM_PM8921_SMPS8, &pm8921_smps, "vdd_s8"  },

	{ "l1",  QCOM_RPM_PM8921_LDO1, &pm8921_nldo, "vdd_l1_l2_l12_l18" },
	{ "l2",  QCOM_RPM_PM8921_LDO2, &pm8921_nldo, "vdd_l1_l2_l12_l18" },
	{ "l3",  QCOM_RPM_PM8921_LDO3, &pm8921_pldo, "vdd_l3_l15_l17" },
	{ "l4",  QCOM_RPM_PM8921_LDO4, &pm8921_pldo, "vdd_l4_l14" },
	{ "l5",  QCOM_RPM_PM8921_LDO5, &pm8921_pldo, "vdd_l5_l8_l16" },
	{ "l6",  QCOM_RPM_PM8921_LDO6, &pm8921_pldo, "vdd_l6_l7" },
	{ "l7",  QCOM_RPM_PM8921_LDO7, &pm8921_pldo, "vdd_l6_l7" },
	{ "l8",  QCOM_RPM_PM8921_LDO8, &pm8921_pldo, "vdd_l5_l8_l16" },
	{ "l9",  QCOM_RPM_PM8921_LDO9, &pm8921_pldo, "vdd_l9_l11" },
	{ "l10", QCOM_RPM_PM8921_LDO10, &pm8921_pldo, "vdd_l10_l22" },
	{ "l11", QCOM_RPM_PM8921_LDO11, &pm8921_pldo, "vdd_l9_l11" },
	{ "l12", QCOM_RPM_PM8921_LDO12, &pm8921_nldo, "vdd_l1_l2_l12_l18" },
	{ "l14", QCOM_RPM_PM8921_LDO14, &pm8921_pldo, "vdd_l4_l14" },
	{ "l15", QCOM_RPM_PM8921_LDO15, &pm8921_pldo, "vdd_l3_l15_l17" },
	{ "l16", QCOM_RPM_PM8921_LDO16, &pm8921_pldo, "vdd_l5_l8_l16" },
	{ "l17", QCOM_RPM_PM8921_LDO17, &pm8921_pldo, "vdd_l3_l15_l17" },
	{ "l18", QCOM_RPM_PM8921_LDO18, &pm8921_nldo, "vdd_l1_l2_l12_l18" },
	{ "l21", QCOM_RPM_PM8921_LDO21, &pm8921_pldo, "vdd_l21_l23_l29" },
	{ "l22", QCOM_RPM_PM8921_LDO22, &pm8921_pldo, "vdd_l10_l22" },
	{ "l23", QCOM_RPM_PM8921_LDO23, &pm8921_pldo, "vdd_l21_l23_l29" },
	{ "l24", QCOM_RPM_PM8921_LDO24, &pm8921_nldo1200, "vdd_l24" },
	{ "l25", QCOM_RPM_PM8921_LDO25, &pm8921_nldo1200, "vdd_l25" },
	{ "l26", QCOM_RPM_PM8921_LDO26, &pm8921_nldo1200, "vdd_l26" },
	{ "l27", QCOM_RPM_PM8921_LDO27, &pm8921_nldo1200, "vdd_l27" },
	{ "l28", QCOM_RPM_PM8921_LDO28, &pm8921_nldo1200, "vdd_l28" },
	{ "l29", QCOM_RPM_PM8921_LDO29, &pm8921_pldo, "vdd_l21_l23_l29" },

	{ "lvs1", QCOM_RPM_PM8921_LVS1, &pm8921_switch, "vin_lvs1_3_6" },
	{ "lvs2", QCOM_RPM_PM8921_LVS2, &pm8921_switch, "vin_lvs2" },
	{ "lvs3", QCOM_RPM_PM8921_LVS3, &pm8921_switch, "vin_lvs1_3_6" },
	{ "lvs4", QCOM_RPM_PM8921_LVS4, &pm8921_switch, "vin_lvs4_5_7" },
	{ "lvs5", QCOM_RPM_PM8921_LVS5, &pm8921_switch, "vin_lvs4_5_7" },
	{ "lvs6", QCOM_RPM_PM8921_LVS6, &pm8921_switch, "vin_lvs1_3_6" },
	{ "lvs7", QCOM_RPM_PM8921_LVS7, &pm8921_switch, "vin_lvs4_5_7" },

	{ "usb-switch", QCOM_RPM_USB_OTG_SWITCH, &pm8921_switch, "vin_5vs" },
	{ "hdmi-switch", QCOM_RPM_HDMI_SWITCH, &pm8921_switch, "vin_5vs" },
	{ "ncp", QCOM_RPM_PM8921_NCP, &pm8921_ncp, "vdd_ncp" },
};



static struct rpm_chip_def rpm_pm8921_def =
{
	.chip_name = "PM8921 regulator",
	.reg_list = rpm_pm8921_list,
	.reg_count = nitems(rpm_pm8921_list),
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,rpm-pm8921-regulators",	(uintptr_t)&rpm_pm8921_def},
	{NULL,				(uintptr_t)NULL},
};


/* Set single field in request buffer */
static void
rpm_reg_set_field(struct rmp_reg *reg, struct rpm_buf_field *req, int value)
{

	if (req->mask == 0 || (value << req->shift) & ~req->mask)
		panic("Bad request");

	reg->req_buf[req->word] &= ~req->mask;
	reg->req_buf[req->word] |= value << req->shift;
}

static int
rpm_reg_write(struct rmp_reg *reg)
{
	int rv;
	device_t dev;

	dev = reg->sc->dev;
	rv = QCOM_RPM_WRITE_CMD(device_get_parent(dev), dev,
	    QCOM_RPM_ACTIVE_STATE, reg->id, reg->req_buf,
	    reg->fields_def->buf_len);

//	uint32_t buf[2];
//	rv = QCOM_RPM_READ_STATUS(device_get_parent(sc->dev), sc->dev,
//	    QCOM_RPM_ACTIVE_STATE, sc->id, buf,
//	    sc->rmp_fields_def->buf_len);
//printf("%s: id: %d, len: %d\n", __func__, sc->id, sc->rmp_fields_def->buf_len);
//printf(" reqbuf: 0x%08X, 0x%08X\n", sc->req_buf[0], sc->req_buf[1]);
//printf(" respbuf: 0x%08X, 0x%08X\n", buf[0], buf[1]);
//printf(" -------------------------------------\n");
	return (rv);
}

static int
rpm_reg_regulator_set(struct rmp_reg *reg, int val)
{
	struct rpm_buf_field *fld;
	int regval;


	val = val ? 1 : 0;
	fld = NULL;
 	if (reg->fields_def->enable_state.mask != 0) {
 		/* Enable / disable pover switch */
		fld = &reg->fields_def->enable_state;
		regval = val;
		if (bootverbose)
			device_printf(reg->sc->dev, "%s power switch\n",
			val != 0 ? "Enabling": "Disabling");
	} else if (reg->fields_def->uV.mask) {
		/* uV based regulator */
		fld = &reg->fields_def->uV;
		regval = val != 0 ? reg->cur_uvolt : 0;
		if (bootverbose)
			device_printf(reg->sc->dev,
			    "%s regulator, voltage %d uV\n",
			    val != 0 ? "Enabling": "Disabling", regval);
	} else if (reg->fields_def->mV.mask != 0) {
		fld = &reg->fields_def->mV;
		regval = val != 0 ? reg->cur_uvolt / 1000 : 0;
		if (bootverbose)
			device_printf(reg->sc->dev,
			    "%s regulator, voltage %d mV\n",
			    val != 0 ? "Enabling": "Disabling", regval);
	}

	if (fld  == NULL) {
		device_printf(reg->sc->dev, "Cannot enable power\n");
		return (ENXIO);
	}
	rpm_reg_set_field(reg, fld, regval);
	return (rpm_reg_write(reg));
}

static int
rpm_reg_parse_freq(struct rmp_reg *reg, phandle_t node)
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
		device_printf(reg->sc->dev,
		    "Cannot read witch-mode-frequency property\n");
		return (ENXIO);
	}

	for (i = 0; i < nitems(freq_table); i++) {
		if (val == freq_table[i]) {
			rpm_reg_set_field(reg, &reg->fields_def->freq, i + 1);
			return (0);
		}
	}

	device_printf(reg->sc->dev,
	    "Cannot parse witch-mode-frequency property\n");
	return (ENXIO);
}

static struct rmp_reg *
qcom_rpm_attach_regulator(struct qcom_rpm_regs_softc *sc, phandle_t node,
    struct rmp_reg_list *def)
{
	struct rmp_reg *reg;
	int rv, val, mode;

	reg = malloc(sizeof(struct rmp_reg), M_RPM_REG,
	    M_WAITOK | M_ZERO);

	reg->sc = sc,
	reg->xref = OF_xref_from_node(node),
	reg->id = def->id;
	reg->fields_def = def->type_def->fields_def;
	reg->force_mode_auto = def->type_def->force_mode_auto;
	reg->force_mode_bypass = def->type_def->force_mode_bypass;

	rv = OF_getencprop(node, "regulator-min-microvolt", &reg->min_uvolt,
	    sizeof(reg->min_uvolt));
	if (rv <= 0)
		reg->min_uvolt = 0;

	rv = OF_getencprop(node, "regulator-max-microvolt", &reg->max_uvolt,
	    sizeof(reg->max_uvolt));
	if (rv <= 0)
		reg->max_uvolt = 0;

	if (OF_hasprop(node, "bias-pull-down"))
		rpm_reg_set_field(reg, &reg->fields_def->pd, 1);

	if (reg->fields_def->freq.mask != 0) {
		rv = rpm_reg_parse_freq(reg, node);
		if (rv < 0)
		return (NULL);
	}

	if (reg->fields_def->pm.mask != 0) {
		val = OF_hasprop(node, "qcom,power-mode-hysteretic") ? 0 : 1;
		rpm_reg_set_field(reg, &reg->fields_def->pm, val);
	}

	if (reg->fields_def->fm.mask != 0) {
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
			if (FORCE_MODE_IS_2_BITS(reg))
				mode = 2;
			else
				mode = 3;
			break;
		case QCOM_RPM_FORCE_MODE_AUTO:
			if (reg->force_mode_auto)
				mode = 2;
			break;
		case QCOM_RPM_FORCE_MODE_BYPASS:
			if (reg->force_mode_bypass)
				mode = 4;
			break;
		}

		if (mode == -1) {
			device_printf(sc->dev, "Cannot parse force mode\n");
			return (NULL);
		}
		rpm_reg_set_field(reg, &reg->fields_def->fm, mode);
	}

	reg->cur_uvolt = reg->max_uvolt;
	fdt_regulator_register_provider(sc->dev, node);
	return (reg);
}

static int
qcom_rpm_regs_regulator_set(device_t dev, intptr_t id, int val)
{
	struct qcom_rpm_regs_softc *sc;
	struct rmp_reg *reg;

	sc = device_get_softc(dev);
	if (id >= sc->reg_count)
		return(ENXIO);

	reg =sc->regs[id];
	if (reg == NULL)
		return(ENXIO);

	return (rpm_reg_regulator_set(reg, val));
}

static int
qcom_rpm_regs_regulator_map(device_t dev, phandle_t xref, int ncells,
    pcell_t *cells, int *num)
{
	struct qcom_rpm_regs_softc *sc;
	int i;

	sc = device_get_softc(dev);
	for (i = 0; i < sc->reg_count; i++) {
		if (sc->regs[i] == NULL)
			continue;
		if (sc->regs[i]->xref == xref) {
			*num = i;
			return (0);
		}
	}
	return (ENXIO);
}

static int
qcom_rpm_regs_probe(device_t dev)
{
	struct rpm_chip_def *def;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	def = (struct rpm_chip_def *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (def == NULL)
		return (ENXIO);

	device_set_desc(dev, def->chip_name);
	return (BUS_PROBE_DEFAULT);
}

static int
qcom_rpm_regs_attach(device_t dev)
{
	struct qcom_rpm_regs_softc *sc;
	struct rpm_chip_def *def;
	struct rmp_reg *reg;
	phandle_t node, child;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	def = (struct rpm_chip_def *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (def == NULL)
		return (ENXIO);

	sc->reg_list = def->reg_list;
	sc->reg_count = def->reg_count;
	sc->regs = malloc(sizeof(struct rmp_reg *) * sc->reg_count,
	    M_RPM_REG, M_WAITOK | M_ZERO);

	/* Attach all known regulators if exist in DT */
	for (i = 0; i < sc->reg_count; i++) {
		child = ofw_bus_find_child(node, sc->reg_list[i].name);
		if (child == 0)
			continue;
		reg = qcom_rpm_attach_regulator(sc, child, sc->reg_list + i);
		if (reg == NULL) {
			device_printf(sc->dev, "Cannot attach regulator: %s\n",
			    sc->reg_list[i].name);
			return (ENXIO);
		}
		sc->regs[i] = reg;
	}

	return (0);
}

static int
qcom_rpm_regs_detach(device_t dev)
{
	return (EBUSY);
}

static device_method_t qcom_rpm_regs_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_rpm_regs_probe),
	DEVMETHOD(device_attach,	qcom_rpm_regs_attach),
	DEVMETHOD(device_detach,	qcom_rpm_regs_detach),

	/* Regulator interface */
	DEVMETHOD(fdt_regulator_map,	qcom_rpm_regs_regulator_map),
	DEVMETHOD(fdt_regulator_set,	qcom_rpm_regs_regulator_set),
	DEVMETHOD_END
};

static devclass_t qcom_rpm_regs_devclass;

static driver_t qcom_rpm_regs_driver = {
	"qcom_rpm_regs",
	qcom_rpm_regs_methods,
	sizeof(struct qcom_rpm_regs_softc),
};

EARLY_DRIVER_MODULE(qcom_rpm_regs, qcom_rpm, qcom_rpm_regs_driver,
    qcom_rpm_regs_devclass, 0, 0, 73);
