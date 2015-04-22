/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <sys/kdb.h>
#ifdef SMP
#include "apq8064_machdep.h"
#endif

/*
 * The Krait have 3 per CPU timers. Each timer is accessible using per CPU
 * banked registers at 0x0200A000. Other CPUs can access foregin tiners using
 * direct map (0x0208A000) for CPU0.
 *
 * GPT0_BASE runs at 32768 HZ clock and we stole its CPU0 copy as global
 * timecounter.
 *
 * GPT1 is not accessible from not-secure world.
 *
 * per CPU DGT timer is used as event timer and is clocked at 27000000 / 4,
 * so minimum AHB bus frequency is 27 MHz.
 *
 * Beware!!! - initial configuration of PPI GIC interrupts is invalid for
 * secondary cores
 */

#define GPT0_BASE				0x04
#define GPT1_BASE				0x14
#define DGT_BASE				0x24

#define	TMR_MATCH_VAL				0x00
#define	TMR_COUNT_VAL				0x04
#define	TMR_ENABLE				0x08
#define	 TMR_ENABLE_CLR_ON_MATCH_EN			(1 << 1)
#define	 TMR_ENABLE_EN					(1 << 0)
#define	TMR_CLEAR				0x0C

#define	DGT_CLK_CTL				0x34

#define	SPSS_TIMER_STATUS			0x88
#define	 SPSS_TIMER_STATUS_EN				(1 << 0)
#define	 SPSS_TIMER_STATUS_CLR_ON_MTCH			(1 << 1)
#define	 SPSS_TIMER_STATUS_CLR_PEND			(1 << 2)
#define	 SPSS_TIMER_STATUS_WR_PEND			(1 << 3)

#define	 SPSS_TIMER_STATUS_DGT_EN			(1 << 0)
#define	 SPSS_TIMER_STATUS_DGT_CLR_ON_MTCH		(1 << 1)
#define	 SPSS_TIMER_STATUS_DGT_CLR_PEND			(1 << 2)
#define	 SPSS_TIMER_STATUS_DGT_WR_PEND			(1 << 3)

#define	 SPSS_TIMER_STATUS_GPT_EN			(1 << 8)
#define	 SPSS_TIMER_STATUS_GPT_CLR_ON_MTCH		(1 << 9)
#define	 SPSS_TIMER_STATUS_GPT_CLR_PEND			(1 << 10)
#define	 SPSS_TIMER_STATUS_GPT_WR_PEND			(1 << 11)

enum {
	DGT_CLK_CTL_DIV_1 = 0,
	DGT_CLK_CTL_DIV_2 = 1,
	DGT_CLK_CTL_DIV_3 = 2,
	DGT_CLK_CTL_DIV_4 = 3,
};

struct krait_timer_softc;

struct et_softc {
	struct krait_timer_softc *sc;
	struct resource		*et_res;
	bus_addr_t		et_base;
	int			et_status_shift;
	struct eventtimer	et;
};

struct tc_softc {
	struct krait_timer_softc *sc;
	struct resource		*tc_res;
	bus_addr_t		tc_base;
	int			tc_status_shift;
	struct timecounter	tc;
};


struct krait_timer_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource		*tc_res;
	struct resource		*dgt_irq_res;
	struct resource		*gpt0_irq_res;
/*	struct resource		*gpt1_irq_res; */
	void			*dgt_ih;
	void			*gpt0_ih;
/*	void			*gpt1_ih; */
	void			*sc_ih;
	struct et_softc		dgt_sc;
	struct tc_softc		gpt0_sc;
/*	struct tc_softc		gpt1_sc; */
};

#define	RD_4(sc, reg)							\
	bus_read_4((sc)->mem_res, reg)
#define	WR_4(sc, reg, val)						\
	bus_write_4((sc)->mem_res, reg, val)


#define	ET_RD_4(et_sc, reg)						\
	bus_read_4((et_sc)->et_res, (et_sc)->et_base + (reg))
#define	ET_WR_4(et_sc, reg, val)					\
	bus_write_4((et_sc)->et_res, (et_sc)->et_base + (reg), val)

#define	TC_RD_4(tc_sc, reg)						\
	bus_read_4((tc_sc)->tc_res, (tc_sc)->tc_base + (reg))
#define	TC_WR_4(eh_sc, reg, val)					\
	bus_write_4((tc_sc)->tc_res, (tc_sc)->tc_base + (reg), val)


static int krait_timer_initialized = 0;

static struct krait_timer_softc *krait_timer_sc = NULL;

static __inline void
wait_for_clear(struct krait_timer_softc *sc, int status_shift,
    uint32_t status_bit)
{

	status_bit <<= status_shift;
	while ((RD_4(sc, SPSS_TIMER_STATUS)  & status_bit) == status_bit)
		;
}

static int
krait_et_start(struct eventtimer *et, sbintime_t first,
    sbintime_t period)
{
	struct et_softc *et_sc;
	uint32_t count, match;
	uint32_t val;
	et_sc = (struct et_softc *)et->et_priv;

	/* Stop timer first and wait until its disabled */
	val = ET_RD_4(et_sc, TMR_ENABLE);
	val &= ~TMR_ENABLE_EN;
	ET_WR_4(et_sc, TMR_ENABLE, val);
	wait_for_clear(et_sc->sc, et_sc->et_status_shift, SPSS_TIMER_STATUS_EN);

	/* Compute next load and count value */
	match = ((uint32_t)et->et_frequency * period) >> 32;
	count = (uint32_t)((et->et_frequency * first) >> 32);

	if (period != 0)
		val |= TMR_ENABLE_CLR_ON_MATCH_EN;
	else
		val &= ~TMR_ENABLE_CLR_ON_MATCH_EN;


	if (first != 0) {
		if (period != 0)  {
			/* First and period */
			if (count > match) {
				/*
				 * First time is bigger that period -> we
				 * cannot handle this case. Short first time
				 * to period time
				 */
				count = 0;
			} else {
				count = match - count;
			}

		} else  {
			/* Single shot */
			match  = count;
			count = 0;
		}
	} else {
		if (period != 0)  {
			/* Period only */
			count = 0;
		} else {
			/* Notnig - impossible case */
			panic("%s: both period and first are zero", __func__);
		}
	}
	/* Order of this 2 writes must be preserved */
	ET_WR_4(et_sc, TMR_COUNT_VAL, count);
	ET_WR_4(et_sc, TMR_MATCH_VAL, match);

	/* wait until writes are completed */
	wait_for_clear(et_sc->sc, et_sc->et_status_shift,
	    SPSS_TIMER_STATUS_DGT_WR_PEND);

	val |= TMR_ENABLE_EN;
	ET_WR_4(et_sc, TMR_ENABLE, val);

	return (0);
}

static int
krait_et_stop(struct eventtimer *et)
{
	struct et_softc *et_sc;
	uint32_t val;

	et_sc = (struct et_softc *)et->et_priv;

	val = ET_RD_4(et_sc, TMR_ENABLE);
	val &= ~TMR_ENABLE_EN;
	ET_WR_4(et_sc, TMR_ENABLE, val);
	wait_for_clear(et_sc->sc, et_sc->et_status_shift, SPSS_TIMER_STATUS_EN);

	return (0);
}

static int
krait_et_intr(void *arg)
{
	struct et_softc *et_sc;
	uint32_t val;


	et_sc = (struct et_softc *)arg;
	val = ET_RD_4(et_sc, TMR_ENABLE);
	if((val & TMR_ENABLE_CLR_ON_MATCH_EN) == 0) {
		val &= ~TMR_ENABLE_EN;
		ET_WR_4(et_sc, TMR_ENABLE, val);
		wait_for_clear(et_sc->sc, et_sc->et_status_shift,
		    SPSS_TIMER_STATUS_EN);
	}

	if (et_sc->et.et_active)
		et_sc->et.et_event_cb(&et_sc->et, et_sc->et.et_arg);

	return (FILTER_HANDLED);
}
static int
krait_tc_intr(void *arg)
{
	/*
 	* Nothing to do for now
 	*
	struct tc_softc *tc_sc;

	tc_sc = (struct tc_softc *)arg;
	printf("  -- hardlock for gpt0 -------- count: 0x%08X, match: 0x%08X\n", TC_RD_4(tc_sc, TMR_COUNT_VAL), TC_RD_4(tc_sc, TMR_MATCH_VAL));
 	*/
	return (FILTER_HANDLED);
}


static u_int
krait_tc_get_timecount(struct timecounter *tc)
{
	struct tc_softc *tc_sc;

	tc_sc = (struct tc_softc *)tc->tc_priv;

	u_int timecount;

	if (krait_timer_sc == NULL)
		return (0);

	timecount = TC_RD_4(tc_sc, TMR_COUNT_VAL);
	return (timecount);
}

#ifdef SMP
void
arm_init_secondary_timer(void)
{
	uint32_t val;
	int err;


	/* Set clock divider for DGT timer -> se max frequency restrion in TRM */
	WR_4(krait_timer_sc, DGT_CLK_CTL, DGT_CLK_CTL_DIV_4);

	/* Disable event timer */
	val = ET_RD_4(&(krait_timer_sc->dgt_sc), TMR_ENABLE);
	val &= ~TMR_ENABLE_EN;
	ET_WR_4(&(krait_timer_sc->dgt_sc), TMR_ENABLE, val);
	wait_for_clear(krait_timer_sc, krait_timer_sc->dgt_sc.et_status_shift,
	    SPSS_TIMER_STATUS_EN);

	/* Reconfigure interrupts */
	err = BUS_CONFIG_INTR(krait_timer_sc->dev,
	    rman_get_start(krait_timer_sc->dgt_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0)
		panic("Unable to configure the clock irq: %d ", err);
	arm_unmask_irq(rman_get_start(krait_timer_sc->dgt_irq_res));
/*
	err = BUS_CONFIG_INTR(krait_timer_sc->dev,
	    rman_get_start(krait_timer_sc->gpt0_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0) {
		panic("Unable to configure the clock irq: %d ", err);
	}
*/
/*
	err = BUS_CONFIG_INTR(krait_timer_sc->dev,
	    rman_get_start(krait_timer_sc->gpt1_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0) {
		panic("Unable to configure the clock irq: %d ", err);
	}
*/
}
#endif

static void
krait_timer_init_et(struct et_softc *et_sc, struct krait_timer_softc *sc,
    struct resource *res, bus_addr_t base, int status_shift, int clk_freq,
    int quality)
{
	uint32_t val;

	et_sc->sc = sc;
	et_sc->et_res = res;
	et_sc->et_base = base;
	et_sc->et_status_shift = status_shift;

	et_sc->et.et_frequency = clk_freq;
	et_sc->et.et_name = "Krait Subsystem Eventtimer";
	et_sc->et.et_flags = ET_FLAGS_PERIODIC | ET_FLAGS_ONESHOT |
	    ET_FLAGS_PERCPU;
	et_sc->et.et_quality = quality;
	et_sc->et.et_min_period = (0x00000002LLU << 32) /
	    et_sc->et.et_frequency;
	et_sc->et.et_max_period = (0xfffffffeLLU << 32) /
	    et_sc->et.et_frequency;
	et_sc->et.et_start = krait_et_start;
	et_sc->et.et_stop = krait_et_stop;
	et_sc->et.et_priv = et_sc;

	val = ET_RD_4(et_sc, TMR_ENABLE);
	val &= ~TMR_ENABLE_EN;
	ET_WR_4(et_sc, TMR_ENABLE, val);
	wait_for_clear(et_sc->sc, et_sc->et_status_shift, SPSS_TIMER_STATUS_EN);
}

static void
krait_timer_init_tc(struct tc_softc *tc_sc, struct krait_timer_softc *sc,
    struct resource *res, bus_addr_t base, int status_shift, int clk_freq,
    int quality)
{
	uint32_t val;

	tc_sc->sc = sc;
	tc_sc->tc_res = res;
	tc_sc->tc_base = base;
	tc_sc->tc_status_shift = status_shift;

	tc_sc->tc.tc_name = "Krait Subsystem Timecounter";
	tc_sc->tc.tc_get_timecount = krait_tc_get_timecount;
	tc_sc->tc.tc_counter_mask = ~0u;
	tc_sc->tc.tc_frequency = clk_freq;
	tc_sc->tc.tc_quality = quality;
	tc_sc->tc.tc_priv = tc_sc;

	/* Initialize counter  */
	val = TC_RD_4(tc_sc, TMR_ENABLE);
	val &= ~TMR_ENABLE_EN;
	TC_WR_4(tc_sc, TMR_ENABLE, val);
	wait_for_clear(tc_sc->sc, tc_sc->tc_status_shift, SPSS_TIMER_STATUS_EN);
	TC_WR_4(tc_sc, TMR_COUNT_VAL, 0);
	TC_WR_4(tc_sc, TMR_MATCH_VAL, ~0);
	wait_for_clear(tc_sc->sc, tc_sc->tc_status_shift,
	    SPSS_TIMER_STATUS_WR_PEND);
	val |= TMR_ENABLE_EN | TMR_ENABLE_CLR_ON_MATCH_EN;
	TC_WR_4(tc_sc, TMR_ENABLE, val);
}

static int
krait_timer_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "qcom,msm-timer"))
		return (ENXIO);

	device_set_desc(dev, "Qualcomm KRAIT GPT/DGTtimer");
	return (BUS_PROBE_DEFAULT);
}

static int
krait_timer_attach(device_t dev)
{
	struct krait_timer_softc *sc;
	int err, rid;
	uint32_t offset;
	uint32_t clks[2];
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/* We use CPU0's DGT for the clocksource */
	err = OF_getencprop(node, "cpu-offset", &offset, sizeof(offset));
	if (err <= 0) {
		device_printf(dev, "Cannot get cpu-offset property.\n");
		return (ENXIO);
	}

	err = OF_getencprop(node, "clock-frequency", (phandle_t *)&clks, sizeof(clks));
	if (err <= 0) {
		device_printf(dev, "Cannot get clock-frequency property.\n");
		return (ENXIO);
	}
	err = ENXIO;

	/* Allocate memory  resources. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource.\n");
		goto fail;
	}
	rid = 1;
	sc->tc_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    rman_get_start(sc->mem_res) + offset,
	    rman_get_end(sc->mem_res) + offset,
	    rman_get_size(sc->mem_res), RF_ACTIVE);
	if (sc->tc_res == NULL) {
		device_printf(dev, "Cannot allocate memory resource.\n");
		goto fail;
	}

	/* Allocate interrupts resources. */
	rid = 0;
	sc->dgt_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->dgt_irq_res == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource.\n");
		goto fail;
	}
	rid = 1;
	sc->gpt0_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->dgt_irq_res == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource.\n");
		goto fail;
	}
/*
	rid = 2;
	sc->gpt1_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->dgt_irq_res == NULL) {
		device_printf(dev, "Cannot allocate interrupt resource.\n");
		goto fail;
	}
*/
	/* Set divider for DGT timer */
	WR_4(sc, DGT_CLK_CTL, DGT_CLK_CTL_DIV_4);

	/* Init event counters */
	krait_timer_init_et(&sc->dgt_sc, sc, sc->mem_res, DGT_BASE, 0,
	    clks[0] / 4, 1000);
	/* Init time couter */
	krait_timer_init_tc(&sc->gpt0_sc, sc, sc->tc_res, GPT0_BASE, 8,
	    clks[1], 100);
/*
	krait_timer_init_tc(&sc->gpt1_sc, sc, sc->tc_res, GPT1_BASE, 16,
	    clks[1], 100);
*/
	/* Setup interrupts */
	err = bus_setup_intr(dev, sc->dgt_irq_res, INTR_TYPE_CLK,
	    krait_et_intr, NULL, &sc->dgt_sc, &sc->dgt_ih);
	if (err != 0) {
		device_printf(dev, "Unable to setup the clock irq handler");
		goto fail;
	}
	err = BUS_CONFIG_INTR(dev, rman_get_start(sc->dgt_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0) {
		device_printf(dev, "Unable to configure the clock irq ");
		goto fail;
	}

	err = bus_setup_intr(dev, sc->gpt0_irq_res, INTR_TYPE_CLK,
	    krait_tc_intr, NULL, &sc->gpt0_sc, &sc->gpt0_ih);
	if (err != 0) {
		device_printf(dev, "Unable to setup the clock irq handler");
		goto fail;
	}
	err = BUS_CONFIG_INTR(dev, rman_get_start(sc->gpt0_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0) {
		device_printf(dev, "Unable to configure the clock irq ");
		goto fail;
	}

/*
	err = bus_setup_intr(dev, sc->gpt1_irq_res, INTR_TYPE_CLK,
	    krait_tc_intr, NULL, &sc->gpt1_sc, &sc->gpt1_ih);
	if (err != 0) {
		device_printf(dev, "Unable to setup the clock irq handler");
		goto fail;
	}

	err = BUS_CONFIG_INTR(dev, rman_get_start(sc->gpt1_irq_res),
	    INTR_TRIGGER_EDGE, INTR_POLARITY_LOW);
	if (err != 0) {
		device_printf(dev, "Unable to configure the clock irq ");
		goto fail;
	}
*/
	if (device_get_unit(dev) == 0)
		krait_timer_sc = sc;

	/* Register the nmachinery */
	et_register(&sc->dgt_sc.et);
	tc_init(&sc->gpt0_sc.tc);
/*	tc_init(&sc->gpt1_sc.tc); */


	if (bootverbose) {
		device_printf(sc->dev, "clock: hz=%d stathz = %d\n",
		    hz, stathz);

		device_printf(sc->dev, "event timer1 clock frequency %lld\n",
		    sc->dgt_sc.et.et_frequency );
		device_printf(sc->dev, "timecounter clock frequency %lld\n",
		     sc->gpt0_sc.tc.tc_frequency);
	}

	krait_timer_initialized = 1;
	return (0);
fail:
/*
	if (sc->gpt1_ih != NULL)
		bus_teardown_intr(dev, sc->gpt1_irq_res, sc->gpt1_ih);
*/
	if (sc->gpt0_ih != NULL)
		bus_teardown_intr(dev, sc->gpt0_irq_res, sc->gpt0_ih);
	if (sc->dgt_ih != NULL)
		bus_teardown_intr(dev, sc->dgt_irq_res, sc->dgt_ih);

/*
	if (sc->gpt1_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->gpt1_irq_res);
*/
	if (sc->gpt0_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->gpt0_irq_res);
	if (sc->dgt_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->dgt_irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (err);
}

static void __used
krait_timer_DELAY(int usec)
{
	int32_t counts, s;
	uint32_t first, last, match;
	struct tc_softc *tc_sc;

	/*
	 * Check the timers are setup, if not just
	 * use a for loop for the meantime
	 */
	if (!krait_timer_initialized) {
		for (; usec > 0; usec--)
			for (counts = 200; counts > 0; counts--)
				cpufunc_nullop();
		return;
	}

	tc_sc = &(krait_timer_sc->gpt0_sc);
	s = counts = (usec * 33 + 1000 - 33) / 1000;
	match = TC_RD_4(tc_sc, TMR_MATCH_VAL);
	first = TC_RD_4(tc_sc, TMR_COUNT_VAL);
	while (counts > 0) {
		last = TC_RD_4(tc_sc, TMR_COUNT_VAL);
		if (last >= first) {
			counts -= (int32_t)(last - first);
		} else {
			counts -= (int32_t)((match - first) + last);
		}
		first = last;
	}
}
__weak_reference(krait_timer_DELAY, DELAY);

static device_method_t krait_timer_methods[] = {
	DEVMETHOD(device_probe,		krait_timer_probe),
	DEVMETHOD(device_attach,	krait_timer_attach),

	DEVMETHOD_END
};

static driver_t krait_timer_driver = {
	"krait_timer",
	krait_timer_methods,
	sizeof(struct krait_timer_softc),
};

static devclass_t krait_timer_devclass;
EARLY_DRIVER_MODULE(krait_timer, simplebus, krait_timer_driver,
    krait_timer_devclass, 0, 0, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);