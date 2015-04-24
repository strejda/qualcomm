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

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define PINCTRL_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	PINCTRL_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define PINCTRL_LOCK_INIT(_sc)		mtx_init(&_sc->mtx, 		\
	    device_get_nameunit(_sc->sc_dev), "pinctrl", MTX_DEF)
#define PINCTRL_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx);
#define PINCTRL_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED);
#define PINCTRL_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED);

#define	WR4(_sc, _r, _v)						\
	    bus_write_4((_sc)->mem_res, (_r), (_v))
#define	RD4(_sc, _r)							\
	    bus_read_4((_sc)->mem_res, (_r))

#define NGPIO			90
#define GPIO_CFG(n)		(0x1000 + 0x10 * (n))
#define  GPIO_CFG_OE			(1 << 9)
#define  GPIO_CFG_DRV_STRENGTH_MASK	0x07
#define  GPIO_CFG_DRV_STRENGTH_SHIFT	6
#define  GPIO_CFG_FUNC_SEL_MASK		0x0F
#define  GPIO_CFG_FUNC_SEL_SHIFT	2
#define  GPIO_CFG_GPIO_PULL_MASK	0x03
#define  GPIO_CFG_GPIO_PULL_SHIFT	0
#define  GPIO_CFG_GPIO_PULL_NO		0
#define  GPIO_CFG_GPIO_PULL_DOWN	1
#define  GPIO_CFG_GPIO_PULL_KEEPER	2
#define  GPIO_CFG_GPIO_PULL_UP		3
#define GPIO_IN_OUT(n)		(0x1004 + 0x10 * (n))
#define  GPIO_IN_OUT_IN_MASK		(1 << 0)
#define  GPIO_IN_OUT_OUT_MASK		(1 << 1)

#define GPIO_INTR_CFG(n)	(0x1008 + 0x10 * (n))
#define GPIO_INTR_STATUS(n)	(0x100C + 0x10 * (n))
#define GPIO_INTR_STATUS_ACTIVE		(1 << 0)

struct pinctrl_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	struct mtx		mtx;
	void			*gpio_ih;
	device_t		gpio_busdev;
	int			gpio_npins;
	struct gpio_pin		gpio_pins[NGPIO];
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,apq8064-pinctrl",	1},
	{NULL,				0},
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
	PIN_ID_INPUT_SCHMITT,
	PIN_ID_INPUT_DEBOUNCE,
	PIN_ID_POWER_SOURCE,
	PIN_ID_SLEW_RATE,
	PIN_ID_LOW_POWER_MODE,
	PIN_ID_OUTPUT,
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
	{ "power-source", PIN_ID_POWER_SOURCE, 0 },
	{ "low-power-enable", PIN_ID_LOW_POWER_MODE, 1 },
	{ "low-power-disable", PIN_ID_LOW_POWER_MODE, 0 },
	{ "output-low", PIN_ID_OUTPUT, 0, },
	{ "output-high", PIN_ID_OUTPUT, 1, },
	{ "slew-rate", PIN_ID_SLEW_RATE, 0},
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
	char *functions[16];
};

#define GDEF(_id, ...)							\
{									\
	.id = _id,							\
	.name = "gpio" #_id,						\
	.functions = {"gpio", __VA_ARGS__}				\
}

static const struct gpio_mux gpio_muxes[] = {
	GDEF(0),
	GDEF(1),
	GDEF(2),
	GDEF(3),
	GDEF(4, NULL, NULL, "cam_mclk"),
	GDEF(5, NULL, "cam_mclk"),
	GDEF(6, "gsbi3"),
	GDEF(7, "gsbi3"),
	GDEF(8, "gsbi3"),
	GDEF(9, "gsbi3"),
	GDEF(10, "gsbi4", NULL, NULL, NULL, NULL, NULL, NULL, NULL, "gsbi4_cam_i2c"),
	GDEF(11, "gsbi4", NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, "gsbi4_cam_i2c"),
	GDEF(12, "gsbi4", NULL, NULL, NULL, NULL, "gsbi4_cam_i2c"),
	GDEF(13, "gsbi4", NULL, NULL, NULL, NULL, "gsbi4_cam_i2c"),
	GDEF(14, "riva_fm", "gsbi6"),
	GDEF(15, "riva_fm", "gsbi6"),
	GDEF(16, "riva_bt", "gsbi6"),
	GDEF(17, "riva_bt", "gsbi6"),
	GDEF(18, "gsbi1"),
	GDEF(19, "gsbi1"),
	GDEF(20, "gsbi1"),
	GDEF(21, "gsbi1"),
	GDEF(22, "gsbi2"),
	GDEF(23, "gsbi2"),
	GDEF(24, "gsbi2"),
	GDEF(25, "gsbi2"),
	GDEF(26),
	GDEF(27, "mi2s"),
	GDEF(28, "mi2s"),
	GDEF(29, "mi2s"),
	GDEF(30, "mi2s"),
	GDEF(31, "mi2s", NULL, "gsbi5_spi_cs2", "gsbi6_spi_cs2", "gsbi7_spi_cs2"),
	GDEF(32, "mi2s", NULL, NULL, NULL, NULL, "gsbi5_spi_cs3", "gsbi6_spi_cs3", "gsbi7_spi_cs3"),
	GDEF(33, "mi2s"),
	GDEF(34, "codec_mic_i2s"),
	GDEF(35, "codec_mic_i2s"),
	GDEF(36, "codec_mic_i2s"),
	GDEF(37, "codec_mic_i2s"),
	GDEF(38, "codec_mic_i2s"),
	GDEF(39, "codec_spkr_i2s"),
	GDEF(40, "slimbus", "codec_spkr_i2s"),
	GDEF(41, "slimbus", "codec_spkr_i2s"),
	GDEF(42, "codec_spkr_i2s"),
	GDEF(43),
	GDEF(44),
	GDEF(45),
	GDEF(46),
	GDEF(47, "spkr_i2s", "gsbi5_spi_cs1","gsbi6_spi_cs1", "gsbi7_spi_cs1"),
	GDEF(48, "spkr_i2s"),
	GDEF(49, "spkr_i2s"),
	GDEF(50, "spkr_i2s"),
	GDEF(51, NULL, "gsbi5"),
	GDEF(52, NULL, "gsbi5"),
	GDEF(53, NULL, "gsbi5"),
	GDEF(54, NULL, "gsbi5"),
	GDEF(55, "tsif1"),
	GDEF(56, "tsif1"),
	GDEF(57, "tsif1", "sdc2"),
	GDEF(58, "tsif2", "sdc2"),
	GDEF(59, "tsif2", "sdc2"),
	GDEF(60, "tsif2", "sdc2"),
	GDEF(61, NULL, "sdc2"),
	GDEF(62, NULL, "sdc2"),
	GDEF(63, NULL, "sdc4"),
	GDEF(64, "riva_wlan", "sdc4"),
	GDEF(65, "riva_wlan", "sdc4"),
	GDEF(66, "riva_wlan", "sdc4"),
	GDEF(67, "riva_wlan", "sdc4"),
	GDEF(68, "riva_wlan", "sdc4"),
	GDEF(69, "hdmi"),
	GDEF(70, "hdmi"),
	GDEF(71, "hdmi"),
	GDEF(72, "hdmi"),
	GDEF(73),
	GDEF(74),
	GDEF(75),
	GDEF(76),
	GDEF(77),
	GDEF(78, "ps_hold"),
	GDEF(79),
	GDEF(80),
	GDEF(81),
	GDEF(82, NULL, "gsbi7"),
	GDEF(83, "gsbi7"),
	GDEF(84, NULL, "gsbi7"),
	GDEF(85, NULL, NULL, "gsbi7"),
	GDEF(86),
	GDEF(87),
	GDEF(88, "usb2_hsic"),
	GDEF(89, "usb2_hsic"),
};

struct spec_pin {
	char *name;
	uint32_t reg;
	uint32_t pull_shift;
	uint32_t hdrv_shift;
};

#define SDEF(n, r, ps, hs...)						\
{									\
	.name = n,							\
	.reg = r,							\
	.pull_shift = ps,							\
	.hdrv_shift = hs,							\
}

static const struct spec_pin spec_pins[] = {
	SDEF("sdc1_clk", 0x20a0, 13, 6),
	SDEF("sdc1_cmd", 0x20a0, 11, 3),
	SDEF("sdc1_data", 0x20a0, 9, 0),

	SDEF("sdc3_clk", 0x20a4, 14, 6),
	SDEF("sdc3_cmd", 0x20a4, 11, 3),
	SDEF("sdc3_data", 0x20a4, 9, 0),
};


/*
 * 	Pinctrl functions.
 */
 static const struct spec_pin *
pinctrl_search_spin(char *pin_name)
{
	int i;

	for (i = 0; i < nitems(spec_pins); i++) {
		if (strcmp(pin_name, spec_pins[i].name) == 0)
			return 	(&spec_pins[i]);
	}

	return (NULL);
}

static int
pinctrl_config_spin(struct pinctrl_softc *sc, char *pin_name,
    const struct spec_pin *spin, struct pinctrl_cfg *cfg)
{
	int i;
	uint32_t reg, val;

	PINCTRL_LOCK(sc);
	reg = RD4(sc, spin->reg);


	for (i = 0 ; i < PROP_ID_MAX_ID; i++) {
		if (cfg->params[i] == -1)
			continue;
		switch (i) {
		case PIN_ID_BIAS_DISABLE:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				spin->pull_shift);
			reg |= GPIO_CFG_GPIO_PULL_NO <<
				spin->pull_shift;
			break;
		case PIN_ID_BIAS_PULL_DOWN:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				spin->pull_shift);
			reg |= GPIO_CFG_GPIO_PULL_DOWN <<
				spin->pull_shift;
			break;
		case PIN_ID_BIAS_BUS_HOLD:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				spin->pull_shift);
			reg |= GPIO_CFG_GPIO_PULL_KEEPER <<
				spin->pull_shift;
			break;
		case PIN_ID_BIAS_PULL_UP:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				spin->pull_shift);
			reg |= GPIO_CFG_GPIO_PULL_UP <<
				spin->pull_shift;
			break;
		case PIN_ID_DRIVE_STRENGTH:
			val = cfg->params[i];
			if (val > 16)
				val = 16;
			val = (val - 1) / 2;
			reg &= ~(GPIO_CFG_DRV_STRENGTH_MASK <<
				spin->hdrv_shift);
			 reg |= val << spin->hdrv_shift;
			break;
		default:
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev,
			    "Unsupported parameter (%d)\n", i);
			return  (ENXIO);
		}
	}

	WR4(sc, spin->reg, reg);
	PINCTRL_UNLOCK(sc);
	return (0);
}


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
		if (strcmp(fnc_name, gmux->functions[i]) == 0)
			return 	(i);
	}

	return (-1);
}

static int
pinctrl_config_gmux(struct pinctrl_softc *sc, char *pin_name,
    const struct gpio_mux *gmux, struct pinctrl_cfg *cfg)
{
	int tmp, i;
	uint32_t reg, val;

	PINCTRL_LOCK(sc);
	reg = RD4(sc, GPIO_CFG(gmux->id));

	if (cfg->function != NULL) {
		tmp = pinctrl_gmux_function(gmux, cfg->function);
		if (tmp == -1) {
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev,
			    "Unknown function %s for pin %s\n", cfg->function,
			    pin_name);
			return (ENXIO);
		}
		reg &= ~(GPIO_CFG_FUNC_SEL_MASK << GPIO_CFG_FUNC_SEL_SHIFT);
		reg |=  (tmp & GPIO_CFG_FUNC_SEL_MASK) <<
		    GPIO_CFG_FUNC_SEL_SHIFT;
	}

	for (i = 0 ; i < PROP_ID_MAX_ID; i++) {
		if (cfg->params[i] == -1)
			continue;
		switch (i) {
		case PIN_ID_BIAS_DISABLE:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				GPIO_CFG_GPIO_PULL_SHIFT);
			reg |= GPIO_CFG_GPIO_PULL_NO <<
				GPIO_CFG_GPIO_PULL_SHIFT;
			break;
		case PIN_ID_BIAS_PULL_DOWN:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				GPIO_CFG_GPIO_PULL_SHIFT);
			reg |= GPIO_CFG_GPIO_PULL_DOWN <<
				GPIO_CFG_GPIO_PULL_SHIFT;
			break;
		case PIN_ID_BIAS_BUS_HOLD:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				GPIO_CFG_GPIO_PULL_SHIFT);
			reg |= GPIO_CFG_GPIO_PULL_KEEPER <<
				GPIO_CFG_GPIO_PULL_SHIFT;
			break;
		case PIN_ID_BIAS_PULL_UP:
			reg &= ~(GPIO_CFG_GPIO_PULL_MASK <<
				GPIO_CFG_GPIO_PULL_SHIFT);
			reg |= GPIO_CFG_GPIO_PULL_UP <<
				GPIO_CFG_GPIO_PULL_SHIFT;
			break;
		case PIN_ID_DRIVE_STRENGTH:
			val = cfg->params[i];
			if (val > 16)
				val = 16;
			val = (val - 1) / 2;
			reg &= ~(GPIO_CFG_DRV_STRENGTH_MASK <<
				GPIO_CFG_DRV_STRENGTH_SHIFT);
			 reg |= val << GPIO_CFG_DRV_STRENGTH_SHIFT;
			break;
		case PIN_ID_OUTPUT:
			reg |= GPIO_CFG_OE;
			WR4(sc, GPIO_IN_OUT(gmux->id),
			    (cfg->params[i] & 1) << 1);
			break;
		default:
			PINCTRL_UNLOCK(sc);
			device_printf(sc->dev,
			    "Unsupported parameter (%d)\n", i);
			return  (ENXIO);
		}
	}

	WR4(sc, GPIO_CFG(gmux->id), reg);
	PINCTRL_UNLOCK(sc);
	return (0);
}

static int
pinctrl_config_node(struct pinctrl_softc *sc, char *pin_name,
    struct pinctrl_cfg *cfg)
{
	const struct gpio_mux *gmux;
	const struct spec_pin *spin;
	int rv;


	/* Handle GPIO pins */
	gmux = pinctrl_search_gmux(pin_name);

	if (gmux != NULL) {
		rv = pinctrl_config_gmux(sc, pin_name, gmux, cfg);
		return (rv);
	}
	/* Handle special pin groups */
	spin = pinctrl_search_spin(pin_name);
	if (spin != NULL) {
		rv = pinctrl_config_spin(sc, pin_name, spin, cfg);
		return (rv);
	}
	device_printf(sc->dev, "Unknown pin: %s\n", pin_name);
	return (ENXIO);
}

static int
pinctrl_read_node(struct pinctrl_softc *sc, phandle_t node,
    struct pinctrl_cfg *cfg, char **pins, int *lpins)
{
	int rv, i;

	*lpins = OF_getprop_alloc(node, "pins", 1, (void **)pins);
	if (*lpins <= 0)
		return (ENOENT);

	/* Read function (mux) settings. */
	rv = OF_getprop_alloc(node, "function", 1,
	    (void **)&cfg->function);
	if (rv <= 0)
		cfg->function = NULL;

	/* Read rest of properties. */
	for (i = 0; i < PROP_ID_MAX_ID; i++) {
		rv = OF_getencprop(node, prop_names[i].name, &cfg->params[i],
		    sizeof(cfg->params[i]));
		if (prop_names[i].have_value) {
			if (rv == 0) {
				device_printf(sc->dev,
				"Missing value for propety \"%s\"\n",
				prop_names[i].name);
			}
			cfg->params[i] = 0;
		}
		if (rv < 0)
			cfg->params[i] = -1;
	}
	return (0);
}

static int
pinctrl_process_node(struct pinctrl_softc *sc, phandle_t node)
{
	struct pinctrl_cfg cfg;
	char *pins, *pname;
	int i, len, lpins, rv;

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
	uint32_t cfg, pull, ret;

	cfg = RD4(sc, GPIO_CFG(pin));
	ret = (cfg & GPIO_CFG_OE) ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

	pull = (cfg >> GPIO_CFG_GPIO_PULL_SHIFT) & GPIO_CFG_GPIO_PULL_MASK;
	if (pull == GPIO_CFG_GPIO_PULL_DOWN)
		ret |= GPIO_PIN_PULLDOWN;
	else if (pull == GPIO_CFG_GPIO_PULL_UP)
		ret |= GPIO_PIN_PULLUP;
	else if (pull == GPIO_CFG_GPIO_PULL_KEEPER)
		ret |= GPIO_PIN_PULLDOWN | GPIO_PIN_PULLUP;
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
	uint32_t cfg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	cfg = RD4(sc, GPIO_CFG(pin));
	cfg = (cfg >> GPIO_CFG_FUNC_SEL_SHIFT) & GPIO_CFG_FUNC_SEL_MASK;
	if (cfg != 0) {
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
	uint32_t cfg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	cfg = RD4(sc, GPIO_CFG(pin));
	cfg = (cfg >> GPIO_CFG_FUNC_SEL_SHIFT) & GPIO_CFG_FUNC_SEL_MASK;
	if (cfg != 0) {
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
	uint32_t cfg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	cfg = RD4(sc, GPIO_CFG(pin));
	cfg = (cfg >> GPIO_CFG_FUNC_SEL_SHIFT) & GPIO_CFG_FUNC_SEL_MASK;
	if (cfg != 0) {
		PINCTRL_UNLOCK(sc);
		return (ENXIO);
	}
	cfg &= ~(GPIO_CFG_OE |
	    (GPIO_CFG_GPIO_PULL_MASK << GPIO_CFG_GPIO_PULL_SHIFT));
	if (flags & GPIO_PIN_OUTPUT)
		cfg |= GPIO_CFG_OE;

	flags &= GPIO_PIN_PULLDOWN | GPIO_PIN_PULLUP;
	if (flags == GPIO_PIN_PULLDOWN)
		cfg |= GPIO_CFG_GPIO_PULL_DOWN << GPIO_CFG_GPIO_PULL_SHIFT;
	else if (flags == GPIO_PIN_PULLUP)
		cfg |= GPIO_CFG_GPIO_PULL_UP << GPIO_CFG_GPIO_PULL_SHIFT;
	else if (flags == (GPIO_PIN_PULLDOWN | GPIO_PIN_PULLUP))
		cfg |= (GPIO_CFG_GPIO_PULL_DOWN | GPIO_CFG_GPIO_PULL_UP)
		    << GPIO_CFG_GPIO_PULL_SHIFT;

	WR4(sc, GPIO_CFG(pin), cfg);
	sc->gpio_pins[pin].gp_flags = flags;
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct pinctrl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	reg = RD4(sc, GPIO_IN_OUT(pin));
	if (value)
		reg |= GPIO_IN_OUT_OUT_MASK;
	else
		reg &= ~GPIO_IN_OUT_OUT_MASK;
	WR4(sc, GPIO_IN_OUT(pin), reg);
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct pinctrl_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	reg = RD4(sc, GPIO_IN_OUT(pin));
	if (reg &  GPIO_IN_OUT_IN_MASK)
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
	uint32_t reg;

	sc = device_get_softc(dev);
	if (pin >= sc->gpio_npins)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	reg = RD4(sc, GPIO_IN_OUT(pin));
	if (reg &  GPIO_IN_OUT_IN_MASK)
		reg &= ~GPIO_IN_OUT_OUT_MASK;
	else
		reg |= GPIO_IN_OUT_OUT_MASK;
	WR4(sc, GPIO_IN_OUT(pin), reg);
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
pinctrl_gpio_intr(void *arg)
{
	struct pinctrl_softc *sc;
	uint32_t reg;
	int i;

	sc = arg;
	for (i = 0; i < NGPIO; i += sc->gpio_npins) {
		reg = RD4(sc, GPIO_INTR_STATUS(i));
		if (reg & GPIO_INTR_STATUS_ACTIVE) {
			/* XXX Handle interrupt */
			WR4(sc, GPIO_INTR_STATUS(i), GPIO_INTR_STATUS_ACTIVE);
		}
	}
	return (FILTER_HANDLED);
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
	struct pinctrl_softc * sc;
	int i, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->gpio_npins = NGPIO;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource\n");
		goto fail;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource\n");
		goto fail;
	}
	if ((bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC,
	    pinctrl_gpio_intr, NULL, sc, &sc->gpio_ih))) {
		device_printf(dev,
		    "WARNING: unnable to register interrupt handler\n");
		goto fail;
	}
	for (i = 0; i < sc->gpio_npins; i++) {
		sc->gpio_pins[i].gp_pin = i;
		sc->gpio_pins[i].gp_caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
		    GPIO_PIN_TRISTATE | GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
		snprintf(sc->gpio_pins[i].gp_name, GPIOMAXNAME, "gpio%d", i);

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

static device_method_t apq8064_pinctrl_methods[] = {
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

static driver_t apq8064_pinctrl_driver = {
	"gpio",
	apq8064_pinctrl_methods,
	sizeof(struct pinctrl_softc),
};

static devclass_t apq8064_pinctrl_devclass;

EARLY_DRIVER_MODULE(apq8064_pinctrl, simplebus, apq8064_pinctrl_driver,
    apq8064_pinctrl_devclass, 0, 0, 70);
