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
#include <machine/fdt.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/pinctrl/qcom,pmic-gpio.h>

#include "qcom_ssbi_if.h"

#define SSBI_REG_ADDR_GPIO_BASE		0x150	/* XXX mo this to DT */

#define PINCTRL_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	PINCTRL_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define PINCTRL_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->dev), "pinctrl_pmic", MTX_DEF)
#define PINCTRL_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx);
#define PINCTRL_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED);
#define PINCTRL_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);

#define  RVAL(r, m, s, v) r = (r & ~((m) << (s))) | ((v) << (s))
#define  RBIT(r, m, v) r = v ? (r & ~(m)) : (r | (m))

#define NGPIO				44

#define PMIC_BYTES_PER_PIN		6


#define PMIC_CFG0_POWER_SOURCE_MASK	0x07
#define PMIC_CFG0_POWER_SOURCE_SHIFT	1

#define PMIC_CFG1_DIRECTION_MASK	0x03
#define PMIC_CFG1_DIRECTION_SHIFT	2
#define PMIC_CFG1_DIR_OUT		1
#define PMIC_CFG1_DIR_IN		2
#define PMIC_CFG1_OUTPUT_BUFFER		(1 << 1)
#define PMIC_CFG1_OUTPUT_VALUE		(1 << 0)

#define PMIC_CFG2_BIAS_MASK		0x07
#define PMIC_CFG2_BIAS_SHIFT		1
#define PMIC_CFG2_BIAS_PU_30		0
#define PMIC_CFG2_BIAS_PU_1P5		1
#define PMIC_CFG2_BIAS_PU_31P5		2
#define PMIC_CFG2_BIAS_PU_1P5_30	3
#define PMIC_CFG2_BIAS_PD		4
#define PMIC_CFG2_BIAS_NP		5

#define PMIC_CFG3_OUTPUT_STRENGTH_MASK	0x03
#define PMIC_CFG3_OUTPUT_STRENGTH_SHIFT	2
#define PMIC_CFG3_DISABLE		(1 << 0)

#define PMIC_CFG4_FUNCTION_MASK		0x07
#define PMIC_CFG4_FUNCTION_SHIFT	1


#define PMIC_CFG5_NOT_INVERTED		(1 << 3)

struct pinctrl_softc {
	device_t		dev;
	struct mtx		mtx;
	device_t		gpio_busdev;
	int			gpio_npins;
	struct gpio_pin		gpio_pins[NGPIO];
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,pm8921-gpio",	1},
	{NULL,			0},
};

/* ---------- Move to fdt_pinctrl  -------------- */

enum prop_id {
	PIN_ID_BIAS_DISABLE,
	PIN_ID_BIAS_HIGH_IMPEDANCE,
	PIN_ID_BIAS_BUS_HOLD,
	PIN_ID_BIAS_PULL_UP,
	PIN_ID_BIAS_PULL_DOWN,
	PIN_ID_BIAS_PULL_PIN_DEFAULT,
	PIN_ID_DRIVE_PUSH_PULL,
	PIN_ID_DRIVE_OPEN_DRAIN,
	PIN_ID_DRIVE_OPEN_SOURCE,
	PIN_ID_DRIVE_STRENGTH,

	PIN_ID_INPUT_ENABLE,
	PIN_ID_INPUT_SCHMITT_ENABLE,
	PIN_ID_INPUT_DEBOUNCE,
	PIN_ID_POWER_SOURCE,
	PIN_ID_SLEW_RATE,
	PIN_ID_LOW_POWER_MODE,
	PIN_ID_OUTPUT,
	PMIC_ID_PULL_UP,
	PMIC_ID_STRENGTH,
	PROP_ID_MAX_ID
};

/* Parameters */
static const struct prop_name {
	const char	*name;
	enum prop_id	id;
	int		have_value;
} prop_names[] = {
	{ "bias-disable", PIN_ID_BIAS_DISABLE, 0 },
	{ "bias-high-impedance", PIN_ID_BIAS_HIGH_IMPEDANCE, 0 },
	{ "bias-bus-hold", PIN_ID_BIAS_BUS_HOLD, 0 },
	{ "bias-pull-up", PIN_ID_BIAS_PULL_UP, 1 },
	{ "bias-pull-down", PIN_ID_BIAS_PULL_DOWN, 1 },
	{ "bias-pull-pin-default", PIN_ID_BIAS_PULL_PIN_DEFAULT, 1 },
	{ "drive-push-pull", PIN_ID_DRIVE_PUSH_PULL, 0 },
	{ "drive-open-drain", PIN_ID_DRIVE_OPEN_DRAIN, 0 },
	{ "drive-open-source", PIN_ID_DRIVE_OPEN_SOURCE, 0 },
	{ "drive-strength", PIN_ID_DRIVE_STRENGTH, 0 },
	{ "input-enable", PIN_ID_INPUT_ENABLE, 1 },
	{ "input-disable", PIN_ID_INPUT_ENABLE, 0 },
	{ "input-schmitt-enable", PIN_ID_INPUT_SCHMITT_ENABLE, 1 },
	{ "input-schmitt-disable", PIN_ID_INPUT_SCHMITT_ENABLE, 0 },

	{ "input-debounce", PIN_ID_INPUT_DEBOUNCE, 0 },
	{ "power-source", PIN_ID_POWER_SOURCE, 1 },
	{ "low-power-enable", PIN_ID_LOW_POWER_MODE, 1 },
	{ "low-power-disable", PIN_ID_LOW_POWER_MODE, 0 },
	{ "output-low", PIN_ID_OUTPUT, 0, },
	{ "output-high", PIN_ID_OUTPUT, 1, },
	{ "slew-rate", PIN_ID_SLEW_RATE, 0},

	{ "qcom,pull-up-strength", PMIC_ID_PULL_UP, 1},
	{ "qcom,drive-strength", PMIC_ID_STRENGTH, 1},
};

/*
 * configuration for one pin group.
 */
struct pinctrl_cfg {
	char	*function;
	int	params[PROP_ID_MAX_ID];
};

/* ------------------------------------------------------------------ */

struct gpio_mux {
	int id;
	char *name;
	char **functions;
};

#define GDEF(_id)							\
{									\
	.id = _id,							\
	.name = "gpio" #_id,						\
	.functions = fnc_tbl,						\
}

static char *fnc_tbl[] = {
	PMIC_GPIO_FUNC_NORMAL, PMIC_GPIO_FUNC_PAIRED,
	PMIC_GPIO_FUNC_FUNC1, PMIC_GPIO_FUNC_FUNC2,
	PMIC_GPIO_FUNC_DTEST1, PMIC_GPIO_FUNC_DTEST2,
	PMIC_GPIO_FUNC_DTEST3, PMIC_GPIO_FUNC_DTEST4,
};


static const struct gpio_mux gpio_muxes[] = {
 GDEF(1),  GDEF(2),  GDEF(3),  GDEF(4),  GDEF(5),  GDEF(6),  GDEF(7),  GDEF(8),
 GDEF(9),  GDEF(10), GDEF(11), GDEF(12), GDEF(13), GDEF(14), GDEF(15), GDEF(16),
 GDEF(17), GDEF(18), GDEF(19), GDEF(20), GDEF(21), GDEF(22), GDEF(23), GDEF(24),
 GDEF(25), GDEF(26), GDEF(27), GDEF(28), GDEF(29), GDEF(30), GDEF(31), GDEF(32),
 GDEF(33), GDEF(34), GDEF(35), GDEF(36), GDEF(37), GDEF(38), GDEF(39), GDEF(40),
 GDEF(41), GDEF(42), GDEF(43), GDEF(44)
};

static int
read_reg(struct pinctrl_softc *sc, int pin, int reg, uint8_t *val)
{
	int rv;
	uint8_t addr;

	addr = reg << 4;
	rv = QCOM_SSBI_WRITE(device_get_parent(sc->dev), sc->dev,
	    SSBI_REG_ADDR_GPIO_BASE + pin, &addr, 1);
	if (rv != 0) {
		device_printf(sc->dev, "Write to PMIC pin %d failed: %d\n",
		    pin, rv);
		return (rv);
	}

	rv = QCOM_SSBI_READ(device_get_parent(sc->dev), sc->dev,
	    SSBI_REG_ADDR_GPIO_BASE + pin, val, 1);
	if (rv) {
		device_printf(sc->dev, "Read from PMIC pin %d failed: %d\n",
		    pin, rv);
		return (rv);
	}

	return (0);
}

static uint8_t
write_reg(struct pinctrl_softc *sc, int pin, int reg, uint8_t val)

{
	int rv;

	val = reg << 4;
	rv = QCOM_SSBI_WRITE(device_get_parent(sc->dev), sc->dev,
	    SSBI_REG_ADDR_GPIO_BASE + pin, &val, 1);
	if (rv != 0) {
		device_printf(sc->dev, "Write to PMIC pin %d failed: %d\n",
		    pin, rv);
		return (rv);
	}

	return (0);
}


/*
 * 	Pinctrl functions.
 */

static const struct gpio_mux *
pinctrl_search_gmux(char *pin_name)
{
	int i;

	for (i = 0; i < nitems(gpio_muxes); i++) {
		if (strcmp(pin_name, gpio_muxes[i].name) == 0)
			return 	(&gpio_muxes[i]);
	}

	return (NULL);
}

static int
pinctrl_gmux_function(const struct gpio_mux *gmux, char *fnc_name)
{
	int i;

	for (i = 0; i < 16; i++) {
		if ((gmux->functions[i] != NULL) &&
		    (strcmp(fnc_name, gmux->functions[i]) == 0))
			return 	(i);
	}

	return (-1);
}

static int
pinctrl_config_gmux(struct pinctrl_softc *sc, char *pin_name,
    const struct gpio_mux *gmux, struct pinctrl_cfg *cfg)
{
	int tmp, i, rv;
	uint32_t val;
	uint8_t reg[PMIC_BYTES_PER_PIN];

	PINCTRL_LOCK(sc);
	for (i = 0; i < PMIC_BYTES_PER_PIN; i++) {
		rv = read_reg(sc, gmux->id, i, reg + i);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot configure pin %d: %d\n",
			    gmux->id, rv);
			return (rv);
		}

	}
	if (cfg->function != NULL) {
		tmp = pinctrl_gmux_function(gmux, cfg->function);
		if (tmp == -1) {
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev,
			    "Unknown function %s for pin %s\n", cfg->function,
			    pin_name);
			return (rv);
		}
		RVAL(reg[4], PMIC_CFG4_FUNCTION_MASK, PMIC_CFG4_FUNCTION_SHIFT,
		    tmp);
	}


	for (i = 0 ; i < PROP_ID_MAX_ID; i++) {
		if (cfg->params[i] == -1)
			continue;
		switch (i) {
		case PIN_ID_BIAS_DISABLE:
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_SHIFT,
			    PMIC_CFG2_BIAS_NP);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
			break;
		case PIN_ID_BIAS_PULL_DOWN:
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_SHIFT,
			    PMIC_CFG2_BIAS_PD);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
			break;
		case PMIC_ID_PULL_UP:
			val = cfg->params[i];
			if ((val < PMIC_CFG2_BIAS_PU_30) ||
			    (val > PMIC_CFG2_BIAS_PU_1P5_30)) {
				PINCTRL_UNLOCK(sc);
				device_printf(sc->dev,
				    "Pull-up property value is out of "
				    "range: %d\n", val);
				    return (EINVAL);
			}
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_SHIFT,
			    val);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
			break;
		case PIN_ID_BIAS_HIGH_IMPEDANCE:
			RBIT(reg[3], PMIC_CFG3_DISABLE, 1);
			break;
		case PIN_ID_INPUT_ENABLE:
			RVAL(reg[1], PMIC_CFG1_DIRECTION_MASK,
			    PMIC_CFG1_DIRECTION_SHIFT,
			    PMIC_CFG1_DIR_IN);
			break;
		case PIN_ID_OUTPUT:
			RVAL(reg[1], PMIC_CFG1_DIRECTION_MASK,
			    PMIC_CFG1_DIRECTION_SHIFT,
			    PMIC_CFG1_DIR_OUT);
			break;
		case PIN_ID_POWER_SOURCE:
			RVAL(reg[0], PMIC_CFG0_POWER_SOURCE_MASK,
			    PMIC_CFG0_POWER_SOURCE_SHIFT, cfg->params[i]);
			break;
		case PMIC_ID_STRENGTH:
			val = cfg->params[i];
			if (val > PMIC_GPIO_STRENGTH_LOW) {
				PINCTRL_UNLOCK(sc);
				device_printf(sc->dev,
				    "Strength property value is out of "
				    "range: %d\n", val);
				    return (EINVAL);
			}
			RVAL(reg[3], PMIC_CFG3_OUTPUT_STRENGTH_MASK,
			    PMIC_CFG3_OUTPUT_STRENGTH_SHIFT, cfg->params[i]);
			break;
		case PIN_ID_DRIVE_PUSH_PULL:
			RBIT(reg[1], PMIC_CFG1_OUTPUT_BUFFER, 0);
			break;
		case PIN_ID_DRIVE_OPEN_DRAIN:
			RBIT(reg[1], PMIC_CFG1_OUTPUT_BUFFER, 1);
			break;
		default:
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev,
			    "Unsupported parameter: %d(%d) \n", i, PIN_ID_POWER_SOURCE);
			return  (ENXIO);
		}
	}

	for (i = 0; i < PMIC_BYTES_PER_PIN; i++) {
		rv = write_reg(sc, gmux->id, i, reg[i]);
		if (rv != 0) {
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev, "Cannot configure pin %d: %d\n",
			    gmux->id, rv);
			return (rv);
		}

	}
	PINCTRL_UNLOCK(sc);
	return (0);
}

static int
pinctrl_config_node(struct pinctrl_softc *sc, char *pin_name,
    struct pinctrl_cfg *cfg)
{
	const struct gpio_mux *gmux;
	int rv;

	/* Handle GPIO pins */
	gmux = pinctrl_search_gmux(pin_name);

	if (gmux != NULL) {
		rv = pinctrl_config_gmux(sc, pin_name, gmux, cfg);
		return (rv);
	}
	device_printf(sc->dev, "Unknown pin: %s\n", pin_name);
	return (ENXIO);
}

static int
pinctrl_read_node(struct pinctrl_softc *sc, phandle_t node,
    struct pinctrl_cfg *cfg, char **pins, int *lpins)
{
	int rv, i, id;

	*lpins = OF_getprop_alloc(node, "pins", 1, (void **)pins);
	if (*lpins <= 0)
		return (ENOENT);

	/* Read function (mux) settings. */
	rv = OF_getprop_alloc(node, "function", 1,
	    (void **)&cfg->function);
	if (rv <= 0)
		cfg->function = NULL;

	/* Read rest of properties. */
	for (i = 0; i < nitems(prop_names); i++) {
		id = prop_names[i].id;
		rv = OF_getencprop(node, prop_names[i].name,
		    &cfg->params[id], sizeof(cfg->params[id]));
		if (prop_names[i].have_value) {
			if (rv == 0) {
				device_printf(sc->dev,
				"Missing value for propety \"%s\"\n",
				prop_names[i].name);
			}
			cfg->params[id] = 0;
		}
		if (rv < 0)
			cfg->params[id] = -1;
	}
	return (0);
}

static int
pinctrl_process_node(struct pinctrl_softc *sc, phandle_t node)
{
	struct pinctrl_cfg cfg;
	char *pins, *pname;
	int i, len, lpins, rv;

	bzero(&cfg, sizeof(cfg));
	rv = pinctrl_read_node(sc, node, &cfg, &pins, &lpins);
	if (rv != 0)
		return (rv);

	len = 0;
	pname = pins;
	do {
		i = strlen(pname) + 1;
		rv = pinctrl_config_node(sc, pname, &cfg);
		if (rv != 0)
			device_printf(sc->dev,
			    "Cannot configure pin: %s: %d\n", pname, rv);

		len += i;
		pname += i;
	} while (len < lpins);

	if (pins != NULL)
		free(pins, M_OFWPROP);
	if (cfg.function != NULL)
		free(cfg.function, M_OFWPROP);

	return (rv);
}

static int
pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct pinctrl_softc *sc;
	phandle_t node, cfgnode;
	int rv;

	sc = device_get_softc(dev);
	cfgnode = OF_node_from_xref(cfgxref);


	for (node = OF_child(cfgnode); node != 0; node = OF_peer(node)) {
		if (!fdt_is_enabled(node))
			continue;
		rv = pinctrl_process_node(sc, node);
		if (rv != 0)
 		 device_printf(dev, "Pin config failed: %d\n", rv);

	}

	return (0);
}


/*
 * 	GPIO functions.
 */
static uint32_t
read_flags(struct pinctrl_softc *sc, uint32_t pin)
{
	uint32_t ret;
	uint8_t reg[PMIC_BYTES_PER_PIN];
	uint8_t tmp;
	int i, rv;

	ret = 0;
	for (i = 0; i < PMIC_BYTES_PER_PIN; i++) {
		rv = read_reg(sc, pin, i, reg + i);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot read pin %d: %d\n",
			    pin, rv);
			return (ret);
		}

	}
#if 0
printf("%s: Pin: %u, [0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]\n", __func__, pin,
reg[0], reg[1], reg[2], reg[3], reg[4], reg[5]);
#endif
	tmp = (reg[1] >> PMIC_CFG1_DIRECTION_SHIFT) & PMIC_CFG1_DIRECTION_MASK;
	ret = tmp == PMIC_CFG1_DIR_OUT ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

	tmp = (reg[2] >> PMIC_CFG2_BIAS_SHIFT) & PMIC_CFG2_BIAS_MASK;
	if (tmp == PMIC_CFG2_BIAS_PD)
		ret |= GPIO_PIN_PULLDOWN;
	else if ((tmp >= PMIC_CFG2_BIAS_PU_30) &&
	    (tmp <= PMIC_CFG2_BIAS_PU_1P5_30))
		ret |= GPIO_PIN_PULLUP;
	return (ret);
}

static device_t
pinctrl_get_bus(device_t dev)
{
	struct pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->gpio_busdev);
}

static int
pinctrl_pin_max(device_t dev, int *maxpin)
{
	struct pinctrl_softc *sc;

	sc = device_get_softc(dev);
	*maxpin = sc->gpio_npins - 1;
	return (0);
}

static int
pinctrl_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	memcpy(name, sc->gpio_pins[pin].gp_name, GPIOMAXNAME);
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct pinctrl_softc *sc;
	uint8_t reg;
	int rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);
	rv = read_reg(sc, pin, 4, &reg);

	PINCTRL_LOCK(sc);
	rv = read_reg(sc, pin, 4, &reg);
	reg  = (reg >> PMIC_CFG4_FUNCTION_SHIFT) & PMIC_CFG4_FUNCTION_MASK;
	if (reg != 0) {
		PINCTRL_UNLOCK(sc);
		return (ENXIO);
	}
	*flags = sc->gpio_pins[pin].gp_flags;
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct pinctrl_softc *sc;
	uint8_t reg;
	int rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	rv = read_reg(sc, pin, 4, &reg);
	reg  = (reg >> PMIC_CFG4_FUNCTION_SHIFT) & PMIC_CFG4_FUNCTION_MASK;
	if (reg != 0) {
		PINCTRL_UNLOCK(sc);
		return (ENXIO);
	}
	*caps = sc->gpio_pins[pin].gp_caps;
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct pinctrl_softc *sc;
	uint8_t reg[PMIC_BYTES_PER_PIN];
	uint8_t tmp;
	int i, rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	for (i = 0; i < PMIC_BYTES_PER_PIN; i++) {
		rv = read_reg(sc, pin, i, reg + i);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot configure pin %d: %d\n",
			    pin, rv);
			PINCTRL_UNLOCK(sc);
			return (rv);
		}

	}
	tmp = (reg[4] >> PMIC_CFG4_FUNCTION_SHIFT) & PMIC_CFG4_FUNCTION_MASK;
	if (tmp != 0) {
		PINCTRL_UNLOCK(sc);
		return (ENXIO);
	}

	sc->gpio_pins[pin].gp_flags = flags;

	RVAL(reg[1], PMIC_CFG1_DIRECTION_MASK, PMIC_CFG1_DIRECTION_SHIFT,
	    (flags & GPIO_PIN_OUTPUT) ? PMIC_CFG1_DIR_OUT: PMIC_CFG1_DIR_IN);
	flags &= GPIO_PIN_PULLDOWN | GPIO_PIN_PULLUP;
	if (flags == GPIO_PIN_PULLDOWN) {
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_SHIFT,
			    PMIC_CFG2_BIAS_PD);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
	} else if (flags == GPIO_PIN_PULLUP) {
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_PU_30,
			    PMIC_CFG2_BIAS_PD);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
	} else if (flags == 0) {
			RVAL(reg[2], PMIC_CFG2_BIAS_MASK, PMIC_CFG2_BIAS_SHIFT,
			    PMIC_CFG2_BIAS_NP);
			RBIT(reg[3], PMIC_CFG3_DISABLE, 0);
	}
	for (i = 0; i < PMIC_BYTES_PER_PIN; i++) {
		rv = write_reg(sc, pin, i, reg[i]);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot configure pin %d: %d\n",
			    pin, rv);
			PINCTRL_UNLOCK(sc);
			return (rv);
		}

	};
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct pinctrl_softc *sc;
	uint8_t reg;
	int rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	rv = read_reg(sc, pin,  1, &reg);
	RBIT(reg, PMIC_CFG1_OUTPUT_VALUE, reg);
	rv = write_reg(sc, pin, 1, reg);

	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct pinctrl_softc *sc;
	uint8_t reg;
	int rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	rv = read_reg(sc, pin, 1, &reg);
	RBIT(reg, PMIC_CFG1_OUTPUT_VALUE, reg);
	if (reg &  PMIC_CFG1_OUTPUT_VALUE)
		*val = 1;
	else
		*val = 0;
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_toggle(device_t dev, uint32_t pin)
{
	struct pinctrl_softc *sc;
	uint8_t reg;
	int rv;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	rv = read_reg(sc, pin, 1, &reg);
	reg ^= PMIC_CFG1_OUTPUT_VALUE;
	rv = write_reg(sc, pin, 1, reg);
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_map_gpios(device_t dev, phandle_t pdev, phandle_t gparent,
    int gcells, pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{
	if (gcells != 2)
		return (ERANGE);
	*pin = gpios[0];
	*flags= gpios[1];
	return (0);
}

static int
pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "APQ8064 pin configuration and GPIO");
	return (BUS_PROBE_DEFAULT);
}

static int
pinctrl_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
pinctrl_attach(device_t dev)
{
	struct pinctrl_softc *sc;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	PINCTRL_LOCK_INIT(sc);
	sc->gpio_npins = NGPIO;
	for (i = 0; i < sc->gpio_npins; i++) {
		sc->gpio_pins[i].gp_pin = gpio_muxes[i].id;
		sc->gpio_pins[i].gp_caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
		    GPIO_PIN_TRISTATE | GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
		snprintf(sc->gpio_pins[i].gp_name, GPIOMAXNAME, "%s",
		    gpio_muxes[i].name);

		sc->gpio_pins[i].gp_flags = read_flags(sc, i);
	}


	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_by_name(dev, "default");

	sc->gpio_busdev = gpiobus_attach_bus(dev);
	if (sc->gpio_busdev == NULL) {
		goto fail;
	}

	return (0);
fail:
	return (ENXIO);
}


static phandle_t
pinctrl_get_node(device_t bus, device_t dev)
{

	/* We only have one child, the GPIO bus, which needs our own node. */
	return (ofw_bus_get_node(bus));
}

static device_method_t pm8921_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		pinctrl_probe),
	DEVMETHOD(device_attach,	pinctrl_attach),
	DEVMETHOD(device_detach,	pinctrl_detach),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure,pinctrl_configure),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		pinctrl_get_bus),
	DEVMETHOD(gpio_pin_max,		pinctrl_pin_max),
	DEVMETHOD(gpio_pin_getname,	pinctrl_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	pinctrl_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	pinctrl_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	pinctrl_pin_setflags),
	DEVMETHOD(gpio_pin_get,		pinctrl_pin_get),
	DEVMETHOD(gpio_pin_set,		pinctrl_pin_set),
	DEVMETHOD(gpio_pin_toggle,	pinctrl_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	pinctrl_map_gpios),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	pinctrl_get_node),

	DEVMETHOD_END
};

static driver_t pm8921_gpio_driver = {
	"gpio",
	pm8921_gpio_methods,
	sizeof(struct pinctrl_softc),
};

static devclass_t pm8921_gpio_devclass;

DRIVER_MODULE(pm8921_gpio, qcom_pm8921, pm8921_gpio_driver,
    pm8921_gpio_devclass, 0, 0);
