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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpu-v6.h>
#include <machine/cpuinfo.h>
#include <machine/smp.h>
#include <machine/fdt.h>
#include <machine/intr.h>

#include <arm/qualcomm/qcom_scm.h>

#include "apq8064_machdep.h"

/* Krait core ACC and SAW2 not banked registers */
#define	KRAIT_CPU_ACC(n)		(0x02088000 + 0x00010000 * (n))
#define  APCS_CPU_PWR_CTL		0x04
#define  APCS_CPU_PWR_CTL_PLL_CLAMP		(1 << 8)
#define  APCS_CPU_PWR_CTL_CORE_PWRD_UP		(1 << 7)
#define  APCS_CPU_PWR_CTL_COREPOR_RST		(1 << 5)
#define  APCS_CPU_PWR_CTL_CORE_RST		(1 << 4)
#define  APCS_CPU_PWR_CTL_L2DT_SLP		(1 << 3)
#define  APCS_CPU_PWR_CTL_CLAMP			(1 << 0)

#define	KRAIT_CPU_SAW(n)		(0x02089000 + 0x00010000 * (n))
#define APCS_SAW2_VCTL			0x14

void
platform_mp_init_secondary(void)
{
	arm_pic_init_secondary();
	arm_init_secondary_timer();
}

static int
get_cores(void)
{
	if ((cpuinfo.mpidr >> 30) != 2) {
		/*
		 * Not multiprocessor extensions or
		 * single core with multiprocessor extensions
		 */
		return (1);
	}
	/* Parse number of cores from MIDR */
	return (((cpuinfo.midr >> 4) & 3) + 1);
}

void
platform_mp_setmaxid(void)
{
	int ncpu;

	/* If we've already set the global vars don't bother to do it again. */
	if (mp_ncpus != 0)
		return;

	ncpu = get_cores();
	mp_ncpus = ncpu;
	mp_maxid = ncpu - 1;
}

int
platform_mp_probe(void)
{

	/* I think platform_mp_setmaxid must get called first, but be safe. */
	if (mp_ncpus == 0)
		platform_mp_setmaxid();

	return (mp_ncpus > 1);
}

static void
wakeup_cpu(int cpu)
{
	int rv;
	bus_space_handle_t acc_reg;
	bus_space_handle_t saw_reg;
	uint32_t val;

	rv = scm_cold_boot_addr(pmap_kextract((vm_offset_t)mpentry), cpu);
	if (rv != 0) {
		panic("Cannot set boot address for AP CPUs\n");
		return;
	}

	rv = bus_space_map(fdtbus_bs_tag, KRAIT_CPU_ACC(cpu), 0x1000, 0,
	    &acc_reg);
	if (rv != 0)
		panic("Couldn't map ACC registers\n");
	rv = bus_space_map(fdtbus_bs_tag, KRAIT_CPU_SAW(cpu), 0x1000, 0,
	    &saw_reg);
	if (rv != 0)
		panic("Couldn't map SAW registers\n");

	/* Enable core power - readback for sync */
	bus_space_write_4(fdtbus_bs_tag, saw_reg , APCS_SAW2_VCTL, 0xA4);
	bus_space_read_4(fdtbus_bs_tag, saw_reg , APCS_SAW2_VCTL);
	DELAY(512);

	/* Release core from reset */
	val = APCS_CPU_PWR_CTL_PLL_CLAMP | APCS_CPU_PWR_CTL_L2DT_SLP |
	    APCS_CPU_PWR_CTL_CLAMP;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);

	val &= ~APCS_CPU_PWR_CTL_L2DT_SLP;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);
	DELAY(300);

	val |= APCS_CPU_PWR_CTL_COREPOR_RST;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);
	DELAY(2);

	val &= ~APCS_CPU_PWR_CTL_CLAMP;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);
	DELAY(2);

	val &= ~APCS_CPU_PWR_CTL_COREPOR_RST;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);
	DELAY(100);

	val |= APCS_CPU_PWR_CTL_CORE_PWRD_UP;
	bus_space_write_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL, val);
	bus_space_read_4(fdtbus_bs_tag, acc_reg , APCS_CPU_PWR_CTL);

	bus_space_unmap(fdtbus_bs_tag, saw_reg, 0x1000);
	bus_space_unmap(fdtbus_bs_tag, acc_reg, 0x1000);

	armv7_sev();
}

void
platform_mp_start_ap(void)
{
	int i;

	for(i = 1; i < mp_ncpus; i++)
		wakeup_cpu(i);

}

void
platform_ipi_send(cpuset_t cpus, u_int ipi)
{

	pic_ipi_send(cpus, ipi);
}
