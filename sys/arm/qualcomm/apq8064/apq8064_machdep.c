/*-
 * Copyright (c) 2013 Ian Lepore <ian@freebsd.org>
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

#define _ARM32_BUS_DMA_PRIVATE
#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/reboot.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/devmap.h>
#include <machine/fdt.h>
#include <machine/intr.h>
#include <machine/machdep.h>
#include <machine/platformvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include "platform_if.h"

struct fdt_fixup_entry fdt_fixup_table[] = {
	{ NULL, NULL }
};


fdt_pic_decode_t fdt_pic_table[] = {
	&gic_decode_fdt,
	NULL
};

struct arm32_dma_range *
bus_dma_get_range(void)
{

	return (NULL);
}

int
bus_dma_get_range_nb(void)
{

	return (0);
}

static vm_offset_t
apq8064_lastaddr(platform_t plat)
{

	return (arm_devmap_lastaddr());
}

static int
apq8064_attach(platform_t plat)
{


	return (0);
}

static void
apq8064_late_init(platform_t plat)
{

}

/*
 * Set up static device mappings.
 *
 */
static int
apq8064_devmap_init(platform_t plat)
{

	arm_devmap_add_entry(0x16600000, 0x00100000);

	return (0);
}

#define PMC_PHYSBASE		0x7000e400
#define PMC_SIZE		0x400
#define PMC_CONTROL_REG		0x0
#define PMC_SCRATCH0		0x50

void
cpu_reset(void)
{
//	bus_space_handle_t pmc;
//	uint32_t reg;

	printf("Resetting...\n");
//	bus_space_map(fdtbus_bs_tag, PMC_PHYSBASE, PMC_SIZE, 0, &pmc);

//	reg = bus_space_read_4(fdtbus_bs_tag, pmc, PMC_SCRATCH0);
//	bus_space_write_4(fdtbus_bs_tag, pmc, PMC_SCRATCH0,
//	   (reg & ~0xc0000002) | 0x40000000); /* boot to bootloader */
//	 bus_space_write_4(fdtbus_bs_tag, pmc, PMC_SCRATCH0,
//	    (reg & ~0xc0000002) | 0x00000002); /* boot to rcm */

//	reg = bus_space_read_4(fdtbus_bs_tag, pmc, PMC_CONTROL_REG);
//	bus_space_write_4(fdtbus_bs_tag, pmc, PMC_CONTROL_REG, reg | 0x10);

	while(1)
		;
	
}
/*
void
DELAY(int usec)
{
	int i;
	
	for (i = 0; i < (usec * 1000); i++)
	    __asm __volatile ("nop");
}
*/

/*
 * Early putc routine for EARLY_PRINTF support.  To use, add to kernel config:
 *   option SOCDEV_PA=0x16600000
 *   option SOCDEV_VA=0x16600000
 *   option EARLY_PRINTF
 */
#if 1
static void
apq8064_early_putc(int c)
{
	volatile uint32_t * UART_STAT_REG = (uint32_t *)0x16640008;
	volatile uint32_t * UART_TX_REG   = (uint32_t *)0x16640070;
	volatile uint32_t * UART_TX_NCHAR_REG   = (uint32_t *)0x16640040;
	const uint32_t      UART_TXRDY    = (1 << 2);

	while ((*UART_STAT_REG & UART_TXRDY) == 0)
		continue;
	*UART_TX_NCHAR_REG = 1;
	*UART_TX_REG = c;
}
early_putc_t *early_putc = apq8064_early_putc;

#endif

static platform_method_t apq8064_methods[] = {
	PLATFORMMETHOD(platform_attach,		apq8064_attach),
	PLATFORMMETHOD(platform_lastaddr,	apq8064_lastaddr),
	PLATFORMMETHOD(platform_devmap_init,	apq8064_devmap_init),
	PLATFORMMETHOD(platform_late_init,	apq8064_late_init),

	PLATFORMMETHOD_END,
};

FDT_PLATFORM_DEF(apq8064, "InForce IFC6410", 0, "qcom,apq8064");
