/*-
 * Copyright (c) 2014 Michal Meloun <meloun@miracle.cz>
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
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "cpufreq_if.h"

#define DEBUG
#ifdef DEBUG
#define DPRINTF(fmt, ...) do {			\
	printf("%s:%u: ", __func__, __LINE__);	\
	printf(fmt, ##__VA_ARGS__);		\
} while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define	HZ2MHZ(freq) ((freq) / (1000 * 1000))
#define	MHZ2HZ(freq) ((freq) * (1000 * 1000))

#define MHZSTEP 27
#define HZSTEP (MHZSTEP * 1000000)
#define TRANSITION_LATENCY 100

struct bus_level {
	int			bw;
};

struct l2_level {
	int			freq;
	int			vdd_dig_uvolt;
	int			vdd_mem_uvolt;
	struct bus_level	*bus_level;
};

struct cpu_level {
	int			freq;
	int			vdd_cpu_uvolt;
	struct l2_level		*l2_level;
};

#define BW_MBPS(x) {(x)}
#define BW(x) (bus_level_tbl + (x))
#define L2(x) (l2_level_tbl + (x))

/* Requested bus bandwidth in bits per second */
static struct bus_level bus_level_tbl[]  = {
 [0] =  BW_MBPS(640), /*  80 MHz */
 [1] = BW_MBPS(1064), /* 133 MHz */
 [2] = BW_MBPS(1600), /* 200 MHz */
 [3] = BW_MBPS(2128), /* 266 MHz */
 [4] = BW_MBPS(3200), /* 400 MHz */
 [5] = BW_MBPS(4264), /* 533 MHz */
};

/* L2 performance levels */
static struct l2_level l2_level_tbl[]  = {
 [0]  = { 384000,  950000, 1050000, BW(1)},
 [1]  = { 432000, 1050000, 1050000, BW(2)},
 [2]  = { 486000, 1050000, 1050000, BW(2)},
 [3]  = { 540000, 1050000, 1050000, BW(2)},
 [4]  = { 594000, 1050000, 1050000, BW(2)},
 [5]  = { 648000, 1050000, 1050000, BW(4)},
 [6]  = { 702000, 1150000, 1150000, BW(4)},
 [7]  = { 756000, 1150000, 1150000, BW(4)},
 [8]  = { 810000, 1150000, 1150000, BW(4)},
 [9]  = { 864000, 1150000, 1150000, BW(4)},
 [10] = { 918000, 1150000, 1150000, BW(5)},
 [11] = { 972000, 1150000, 1150000, BW(5)},
 [12] = {1026000, 1150000, 1150000, BW(5)},
 [13] = {1080000, 1150000, 1150000, BW(5)},
 [14] = {1134000, 1150000, 1150000, BW(5)},
};

/* CPU performance levels */
static struct cpu_level cpu_level_tbl[] = {
 { 384000,  950000,  L2(0)},
 { 432000,  975000,  L2(5)},
 { 486000,  975000,  L2(5)},
 { 540000, 1000000,  L2(5)},
 { 594000, 1000000,  L2(5)},
 { 648000, 1025000,  L2(5)},
 { 702000, 1025000,  L2(5)},
 { 756000, 1075000,  L2(5)},
 { 810000, 1075000,  L2(5)},
 { 864000, 1100000,  L2(5)},
 { 918000, 1100000,  L2(5)},
 { 972000, 1125000,  L2(5)},
 {1026000, 1125000,  L2(5)},
 {1080000, 1175000, L2(14)},
 {1134000, 1175000, L2(14)},
 {1188000, 1200000, L2(14)},
 {1242000, 1200000, L2(14)},
 {1296000, 1225000, L2(14)},
 {1350000, 1225000, L2(14)},
 {1404000, 1237500, L2(14)},
 {1458000, 1237500, L2(14)},
 {1512000, 1250000, L2(14)},
};

struct apq8064_cpufreq_softc {
	device_t		dev;
	phandle_t		node;

	int			latency;
	clk_t			cpu_clk;
	struct cpu_level	*cpu_lvl;

};

static int cpufreq_lowest_freq = 1;
TUNABLE_INT("hw.apq8064.cpufreq.lowest_freq", &cpufreq_lowest_freq);

static struct cpu_level *
apq8064_cpufreq_get_level(struct apq8064_cpufreq_softc *sc, int freq)
{
	int i;

	if (cpu_level_tbl[0].freq >= freq)
		return (cpu_level_tbl + 0);

	for (i = 0; i < (nitems(cpu_level_tbl) - 1); i++) {
		if (cpu_level_tbl[i + 1].freq > freq)
			return (cpu_level_tbl + i);
	}

	return (cpu_level_tbl + nitems(cpu_level_tbl) - 1);
}


static int
apq8064_cpufreq_set(device_t dev, const struct cf_setting *cf)
{
	struct apq8064_cpufreq_softc *sc;
	struct cpu_level *lvl;
	int rv;

	if (cf == NULL || cf->freq < 0)
		return (EINVAL);

	sc = device_get_softc(dev);
	lvl = apq8064_cpufreq_get_level(sc, cf->freq * 1000);

	rv = clk_set_freq(sc->cpu_clk, lvl->freq * 1000, 1);
	if (rv != 0) {
		device_printf(dev, "Can't set CPU clock frequency\n");
		return (rv);
	}

	sc->cpu_lvl = lvl;
	return (0);
}


static int
apq8064_cpufreq_get(device_t dev, struct cf_setting *cf)
{
	struct apq8064_cpufreq_softc *sc;

	if (cf == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	memset(cf, CPUFREQ_VAL_UNKNOWN, sizeof(*cf));
	cf->dev = NULL;

	cf->freq = sc->cpu_lvl->freq / 1000;
	cf->volts = sc->cpu_lvl->vdd_cpu_uvolt / 1000;
	/* Transition latency in us. */
	cf->lat = sc->latency;
	/* Driver providing this setting. */
	cf->dev = dev;

	return (0);
}

static int
apq8064_cpufreq_settings(device_t dev, struct cf_setting *sets, int *count)
{
	struct apq8064_cpufreq_softc *sc;
	int i, j, max_cnt;

	if (sets == NULL || count == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	memset(sets, CPUFREQ_VAL_UNKNOWN, sizeof(*sets) * (*count));

	max_cnt = min(nitems(cpu_level_tbl), *count);
	for (i = 0; i < max_cnt; i++) {
		j = nitems(cpu_level_tbl) - 1 - i;
		sets[i].freq = cpu_level_tbl[j].freq / 1000;
		sets[i].volts = cpu_level_tbl[j].vdd_cpu_uvolt / 1000;
		sets[i].lat = sc->latency;
		sets[i].dev = dev;
	}
	*count = max_cnt;

	return (0);
}

static int
apq8064_cpufreq_type(device_t dev, int *type)
{

	if (type == NULL)
		return (EINVAL);
	*type = CPUFREQ_TYPE_ABSOLUTE;

	return (0);
}

static int
apq8064_cpufreq_ofw_parse(struct apq8064_cpufreq_softc *sc)
{
	int rv;


	rv = OF_getencprop(sc->node, "clock-latency", &sc->latency,
	    sizeof(sc->latency));
	if (rv <=  0)
		sc->latency = 100000;
	rv = clk_get_by_ofw_name(sc->node, "cpu", &sc->cpu_clk);
	if (rv !=  0) {
		device_printf(sc->dev, "Cannot get CPU clock: %d\n", rv);
		return (rv);
	}
	return (0);
}

static void
apq8064_cpufreq_identify(driver_t *driver, device_t parent)
{

	DPRINTF("driver=%p, parent=%p\n", driver, parent);
	if (device_find_child(parent, "apq8064_cpufreq", -1) != NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "apq8064_cpufreq", -1) == NULL)
		device_printf(parent, "add child failed\n");
}

static int
apq8064_cpufreq_probe(device_t dev)
{

	device_set_desc(dev, "CPU Frequency Control");

	return (0);
}


static int
apq8064_cpufreq_attach(device_t dev)
{
	struct apq8064_cpufreq_softc *sc;
	uint64_t freq;
	int rv;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(device_get_parent(dev));

	rv = apq8064_cpufreq_ofw_parse(sc);
	if (rv != 0)
		return (rv);

	rv = clk_get_freq(sc->cpu_clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Can't get CPU clock frequency\n");
		return (rv);
	}
	sc->cpu_lvl = apq8064_cpufreq_get_level(sc, (int)(freq / 1000));

	/* this device is controlled by cpufreq(4) */
	cpufreq_register(dev);

	return (0);
}

static int
apq8064_cpufreq_detach(device_t dev)
{
	struct apq8064_cpufreq_softc *sc;
	int rv;

	sc = device_get_softc(dev);
	rv = cpufreq_unregister(dev);
	if (rv != 0)
		return (rv);

	if (sc->cpu_clk != NULL)
		clk_release(sc->cpu_clk);
	return (0);
}


static device_method_t apq8064_cpufreq_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	apq8064_cpufreq_identify),
	DEVMETHOD(device_probe,		apq8064_cpufreq_probe),
	DEVMETHOD(device_attach,	apq8064_cpufreq_attach),
	DEVMETHOD(device_detach,	apq8064_cpufreq_detach),

	/* cpufreq interface */
	DEVMETHOD(cpufreq_drv_set,	apq8064_cpufreq_set),
	DEVMETHOD(cpufreq_drv_get,	apq8064_cpufreq_get),
	DEVMETHOD(cpufreq_drv_settings,	apq8064_cpufreq_settings),
	DEVMETHOD(cpufreq_drv_type,	apq8064_cpufreq_type),

	DEVMETHOD_END
};

static devclass_t apq8064_cpufreq_devclass;
static driver_t apq8064_cpufreq_driver = {
	"apq8064_cpufreq",
	apq8064_cpufreq_methods,
	sizeof(struct apq8064_cpufreq_softc),
};

DRIVER_MODULE(apq8064_cpufreq, cpu, apq8064_cpufreq_driver,
    apq8064_cpufreq_devclass, 0, 0);
