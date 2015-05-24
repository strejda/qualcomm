/*-
 * Copyright (c) 2015 Michal Meloun <meloun@miracle.cz>
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
#include <sys/kobj.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_mux.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <gnu/dts/include/dt-bindings/clock/qcom,gcc-msm8960.h>
#include <gnu/dts/include/dt-bindings/reset/qcom,gcc-msm8960.h>
#include "apq8064_clk.h"



/*
 * PLL freqency VCO = REF * [L+(M/N)] / (2 * P)
 *
 *
 * PLL	  DS Name  Type   Source   Freq (MHz)	VDD)	Notes		 Subsystem
 * PLL0	  GPLL0	   SR	 PXO	   800.000	0.945	FABRIC/DDR PLL	 RPM
 * PLL1	  MMPLL0   SR	 PXO	  1332.000	1.050	MM FABRIC	 Multimedia
 * PLL2	  MMPLL1   SR	 PXO	   800.000	0.945	Display		 Multimedia
 * PLL3	  GPLL01   SR2	 PXO	  1200.000	1.050	QDS		 Apps
 * PLL4	  PLL4     SR	 PXO	   393.216	0.945	LPA		 LPA
 	 			   491.520
 * PLL5	  MPLL0	   SR	 CXO	   288.000	0.945	GPS PLL		 Modem
 * PLL8	  PLL8     SR	 PXO	   384.000	0.945	Peripherals,
 *							RPM, and modem	 Shared
 * PLL9	  SC_PLL0  HF	 PXO	Up to 2000	1.050	Apps Core0	 Apps
 * PLL10  SC_PLL1  HF	 PXO	Up to 2000	1.050	Apps Core1	 Apps
 * PLL11  EBI1PLL  SR2	 PXO	  1066.000	1.050	Turbo FABRIC/DDR RPM
 * PLL12  SCL2PLL  HF	 PXO	Up to 1700	1.050	Apps L2		 Apps
 * PLL13  WCNPLL   SR2	 WCNXO	   960.000	0.945	WCNPLL		 WCNSS
   		  	 CXO - used if WCNXO absent
 * PLL14  PLL14    SR	 PXO	   480.000	0.945	HSIC PLL	 Apps
 * PLL15  MMPLL3   SR	 PXO	   975.000	0.945	Graphics PLL	 Apps
 * PLL16  SC_PLL2  HF	 PXO	Up to 2000	1.050	Apps Core2	 Apps
 * PLL17  SC_PLL3  HF	 PXO 	Up to 2000	1.050	Apps Core3	 Apps
 *
 */

static struct ofw_compat_data compat_data[] = {
	{"qcom,gcc-apq8064",	1},
	{NULL,		 	0},
};

struct reset {
	uint32_t 	reg;
	uint32_t	bit;
};

struct apq8064_gcc_softc {
	device_t		dev;
	struct resource *	mem_res;
	struct mtx		mtx;
	struct clkdom 		*clkdom;
	int			type;
};


#define PLIST(x) static const char *x[]
#define FRATE(_id, cname, _freq)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parents_num = 0,					\
	.clkdef.flags = CLK_FLAGS_STATIC,				\
	.freq = _freq,							\
}

static struct clk_fixed_def fixed_tbl[] = {
	FRATE(0, "cxo", 19200000),
	FRATE(0, "pxo", 27000000),
	FRATE(0, "mxo", 27000000),
};

PLIST(p_pxo_N_cxo) = {"pxo", NULL, "cxo"};
PLIST(p_pxo_cxo) = {"pxo", "cxo"};
PLIST(p_pxo) = {"pxo"};

static struct clk_pll_cfg pll14_cfg = {
	.l_val = 0x11 | (0x620 << 7),
	.m_val = 0x7,
	.n_val = 0x9,
	.vco_val = 0x0,
	.vco_mask = 3 << 16,
	.pre_div_val = 0x0,
	.pre_div_mask = 1 <<19,
	.post_div_val = 0x0,
	.post_div_mask = 3 << 20,
	.mn_ena_val = 1 << 22,
	.mn_ena_mask = 1 << 22,
	.main_output_val = 1 << 23,
	.main_output_mask = 1 << 23,
	.fsm_mode = 1
};

#define PLL(_id, cn, plist, mr, l, m, n, cr, sr, sm, cf)		\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cn,						\
	.clkdef.parent_names = plist,					\
	.clkdef.parents_num = nitems(plist),				\
	.mode_reg = mr,							\
	.l_reg = l,							\
	.m_reg = m,							\
	.n_reg = n,							\
	.cfg_reg = cr,							\
	.status_reg = sr,						\
	.status_mask = (1 << sm),					\
	.cfg = cf,							\
}

static struct clk_pll_def pll_tbl[] = {
 PLL(PLL0,  "pll0",  p_pxo,       0x30C0, 0x30C4, 0x30C8, 0x30C8, 0x30D4, 0x30D8, 16, NULL),
 PLL(PLL5,  "pll5",  p_pxo_N_cxo, 0x30E0, 0x30E4, 0x30E8, 0x30EC, 0x30F4, 0x30F8, 16, NULL),
 PLL(PLL8,  "pll8",  p_pxo,       0x3140, 0x3144, 0x3148, 0x314C, 0x3154, 0x3158, 16, NULL),
 PLL(PLL3,  "pll3",  p_pxo,       0x3160, 0x3164, 0x3168, 0x316C, 0x3174, 0x3178, 16, NULL),  /* GPLL1 in TRM */
 PLL(PLL14, "pll14", p_pxo_cxo,   0x31C0, 0x31C4, 0x31C8, 0x31CC, 0x31D4, 0x31D8, 16, &pll14_cfg),
};

#define HFPLL(_id, cn, plist, mr, l, m, n, cr, sr, dr)			\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cn,						\
	.clkdef.parent_names = plist,					\
	.clkdef.parents_num = nitems(plist),				\
	.mode_reg = mr,							\
	.l_reg = l,							\
	.m_reg = m,							\
	.n_reg = n,							\
	.cfg_reg = cr,							\
	.status_reg = sr,						\
	.droop_reg = dr,						\
}

static struct clk_hfpll_def hfpll_tbl[] = {
 HFPLL(PLL9,  "pll9",  p_pxo, 0x3200, 0x3208, 0x320c, 0x3210, 0x3204, 0x3204, 0x3214), /* SC_PLL0 in TRM */
 HFPLL(PLL10, "pll10", p_pxo, 0x3240, 0x3248, 0x324c, 0x3250, 0x3244, 0x325c, 0x3254), /* SC_PLL1 in TRM */
 HFPLL(PLL16, "pll16", p_pxo, 0x3280, 0x3288, 0x328c, 0x3290, 0x3284, 0x329c, 0x3294), /* SC_PLL2 in TRM*/
 HFPLL(PLL17, "pll17", p_pxo, 0x32c0, 0x32c8, 0x32cc, 0x32d0, 0x32c4, 0x32dc, 0x32d4), /* SC_PLL3 in TRM*/
 HFPLL(PLL12, "pll12", p_pxo, 0x3300, 0x3308, 0x330c, 0x3310, 0x3304, 0x331c, 0x3314), /* SC_L2_PLL3 in TRM */
};


#define GATE(_id, cname, plist, o, s)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = (const char *[]){plist},			\
	.clkdef.parents_num = 1,					\
	.clkdef.flags = CLK_FLAGS_STATIC,				\
	.offset = o,							\
	.shift = s,							\
	.mask = 1,							\
	.on_value = 1,							\
	.off_value = 0,							\
}

static struct clk_gate_def gate_tbl[] = {
 GATE( PLL0_VOTE,  "pll0_vote",  "pll0", 0x34c0,  0),
 GATE( PLL3_VOTE,  "pll3_vote",  "pll3", 0x34c0,  3),
 GATE( PLL5_VOTE,  "pll5_vote",  "pll5", 0x34c0,  5),
 GATE( PLL8_VOTE,  "pll8_vote",  "pll8", 0x34c0,  8),
 GATE(PLL14_VOTE, "pll14_vote", "pll14", 0x34c0, 14),
};

#define RCG(_id, cname, plist, ns, md, ms)				\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = plist,					\
	.clkdef.parents_num = nitems(plist),				\
	.clkdef.flags = CLK_FLAGS_STATIC,				\
	.ns_reg = ns,							\
	.md_reg = md,							\
	.mask = ((1 << ms) - 1),					\
}

PLIST(gcc_pxo_pll8_map) = {"pxo", "mxo", "pll0_vote", "pll8_vote"};
PLIST(gcc_pxo_pll8_cxo_map) = {"pxo", NULL, "pll0_vote", "pll8_vote", NULL, "cxo"};
PLIST(gcc_pxo_pll8_pll3_map) = {"pxo", NULL, "pll0_vote", "pll8_vote", NULL, NULL, "pll3_vote"};

static struct clk_rcg_def rcg_tbl[] = {
 RCG(GSBI1_UART_SRC, "gsbi1_uart_src", gcc_pxo_pll8_map, 0x29d4, 0x29d0, 16),
 RCG(GSBI2_UART_SRC, "gsbi2_uart_src", gcc_pxo_pll8_map, 0x29f4, 0x29f0, 16),
 RCG(GSBI3_UART_SRC, "gsbi3_uart_src", gcc_pxo_pll8_map, 0x2a14, 0x2a10, 16),
 RCG(GSBI4_UART_SRC, "gsbi4_uart_src", gcc_pxo_pll8_map, 0x2a34, 0x2a30, 16),
 RCG(GSBI5_UART_SRC, "gsbi5_uart_src", gcc_pxo_pll8_map, 0x2a54, 0x2a50, 16),
 RCG(GSBI6_UART_SRC, "gsbi6_uart_src", gcc_pxo_pll8_map, 0x2a74, 0x2a70, 16),
 RCG(GSBI7_UART_SRC, "gsbi7_uart_src", gcc_pxo_pll8_map, 0x2a94, 0x2a90, 16),

 RCG(GSBI1_QUP_SRC, "gsbi1_qup_src", gcc_pxo_pll8_map, 0x29cc, 0x29c8, 8),
 RCG(GSBI2_QUP_SRC, "gsbi2_qup_src", gcc_pxo_pll8_map, 0x29ec, 0x29e8, 8),
 RCG(GSBI3_QUP_SRC, "gsbi3_qup_src", gcc_pxo_pll8_map, 0x2a0c, 0x2a08, 8),
 RCG(GSBI4_QUP_SRC, "gsbi4_qup_src", gcc_pxo_pll8_map, 0x2a2c, 0x2a28, 8),
 RCG(GSBI5_QUP_SRC, "gsbi5_qup_src", gcc_pxo_pll8_map, 0x2a4c, 0x2a48, 8),
 RCG(GSBI6_QUP_SRC, "gsbi6_qup_src", gcc_pxo_pll8_map, 0x2a6c, 0x2a68, 8),
 RCG(GSBI7_QUP_SRC, "gsbi7_qup_src", gcc_pxo_pll8_map, 0x2a8c, 0x2a88, 8),

 RCG(GP0_SRC, "gp0_src", gcc_pxo_pll8_cxo_map, 0x2d24, 0x2d00, 8),
 RCG(GP1_SRC, "gp1_src", gcc_pxo_pll8_cxo_map, 0x2d44, 0x2d40, 8),
 RCG(GP2_SRC, "gp2_src", gcc_pxo_pll8_cxo_map, 0x2d64, 0x2d60, 8),

 RCG(PRNG_SRC, "prng_src", gcc_pxo_pll8_map, 0x2e80, 0, 0), // BADBADBAD

 RCG(SDC1_SRC, "sdc1_src", gcc_pxo_pll8_map, 0x282c, 0x2828, 8),
 RCG(SDC2_SRC, "sdc2_src", gcc_pxo_pll8_map, 0x284c, 0x2848, 8),
 RCG(SDC3_SRC, "sdc3_src", gcc_pxo_pll8_map, 0x286c, 0x2868, 8),
 RCG(SDC4_SRC, "sdc4_src", gcc_pxo_pll8_map, 0x288c, 0x2888, 8),
 RCG(SDC5_SRC, "sdc5_src", gcc_pxo_pll8_map, 0x28ac, 0x28a8, 8),

 RCG(TSIF_REF_SRC, "tsif_ref_src", gcc_pxo_pll8_map, 0x2710, 0x270c, 16),

 RCG(USB_HS1_XCVR_SRC, "usb_hs1_xcvr_src", gcc_pxo_pll8_map, 0x290c, 0x2908, 8),
 RCG(USB_HS3_XCVR_SRC, "usb_hs3_xcvr_src", gcc_pxo_pll8_map, 0x370c, 0x3708, 8),
 RCG(USB_HS4_XCVR_SRC, "usb_hs4_xcvr_src", gcc_pxo_pll8_map, 0x372c, 0x3728, 8),
 RCG(USB_HSIC_XCVR_FS_SRC, "usb_hsic_xcvr_fs_src", gcc_pxo_pll8_map, 0x2928, 0x2924, 8),
 RCG(USB_FS1_XCVR_FS_CLK, "usb_fs1_xcvr_fs_src", gcc_pxo_pll8_map, 0x2968, 0x2964, 8),

 RCG(CE3_SRC, "ce3_src", gcc_pxo_pll8_pll3_map, 0x36c0, 0, 0), // BADBADBAD
 RCG(SATA_CLK_SRC, "sata_clk_src", gcc_pxo_pll8_pll3_map, 0x2c08, 0, 0), // BADBADBAD
};

#define BR(_id, cname, pname, er, em, hr, hm, hwr, hwm)			\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = (const char *[]){pname},			\
	.clkdef.parents_num = 1,					\
	.clkdef.flags = CLK_FLAGS_STATIC,				\
	.ena_reg = er,							\
	.ena_mask = (1 << em),						\
	.halt_reg = hr,							\
	.halt_mask = (1 << hm),						\
	.hwcg_reg = hwr,						\
	.hwcg_mask = (1 << hwm),					\
}

static struct clk_br_def br_tbl[] = {
 BR(GSBI1_UART_CLK, "gsbi1_uart_clk", "gsbi1_uart_src", 0x29d4, 9, 0x2fcc, 10, 0, 0),
 BR(GSBI2_UART_CLK, "gsbi2_uart_clk", "gsbi2_uart_src", 0x29f4, 9, 0x2fcc,  6, 0, 0),
 BR(GSBI3_UART_CLK, "gsbi3_uart_clk", "gsbi3_uart_src", 0x2a14, 9, 0x2fcc,  2, 0, 0),
 BR(GSBI4_UART_CLK, "gsbi4_uart_clk", "gsbi4_uart_src", 0x2a34, 9, 0x2fd0, 26, 0, 0),
 BR(GSBI5_UART_CLK, "gsbi5_uart_clk", "gsbi5_uart_src", 0x2a54, 9, 0x2fd0, 22, 0, 0),
 BR(GSBI6_UART_CLK, "gsbi6_uart_clk", "gsbi6_uart_src", 0x2a74, 9, 0x2fd0, 18, 0, 0),
 BR(GSBI7_UART_CLK, "gsbi7_uart_clk", "gsbi7_uart_src", 0x2a94, 9, 0x2fd0, 14, 0, 0),

 BR(GSBI1_QUP_CLK, "gsbi1_qup_clk", "gsbi1_qup_src", 0x29cc, 9, 0x2fcc,  9, 0, 0),
 BR(GSBI2_QUP_CLK, "gsbi2_qup_clk", "gsbi2_qup_src", 0x29ec, 9, 0x2fcc,  4, 0, 0),
 BR(GSBI3_QUP_CLK, "gsbi3_qup_clk", "gsbi3_qup_src", 0x2a0c, 9, 0x2fcc,  0, 0, 0),
 BR(GSBI4_QUP_CLK, "gsbi4_qup_clk", "gsbi4_qup_src", 0x2a2c, 9, 0x2fd0, 24, 0, 0),
 BR(GSBI5_QUP_CLK, "gsbi5_qup_clk", "gsbi5_qup_src", 0x2a4c, 9, 0x2fd0, 20, 0, 0),
 BR(GSBI6_QUP_CLK, "gsbi6_qup_clk", "gsbi6_qup_src", 0x2a6c, 9, 0x2fd0, 16, 0, 0),
 BR(GSBI7_QUP_CLK, "gsbi7_qup_clk", "gsbi7_qup_src", 0x2a8c, 9, 0x2fd0, 12, 0, 0),

 BR(GP0_CLK, "gp0_clk", "gp0_src", 0x2d24, 9, 0x2fd8, 7, 0, 0),
 BR(GP1_CLK, "gp1_clk", "gp1_src", 0x2d44, 9, 0x2fd8, 6, 0, 0),
 BR(GP2_CLK, "gp2_clk", "gp2_src", 0x2d64, 9, 0x2fd8, 5, 0, 0),

 BR(PMEM_A_CLK, "pmem_clk", "cxo", 0x25a0, 4, 0x2fc8, 20, 0x25a0, 6),

 BR(PRNG_CLK, "prng_clk", "prng_src", 0x3080, 10, 0x2fd8, 10, 0, 0),

 BR(SDC1_CLK, "sdc1_clk", "sdc1_src", 0x282c, 9, 0x2fc8, 6, 0, 0),
 BR(SDC2_CLK, "sdc2_clk", "sdc2_src", 0x284c, 9, 0x2fc8, 5, 0, 0),
 BR(SDC3_CLK, "sdc3_clk", "sdc3_src", 0x286c, 9, 0x2fc8, 4, 0, 0),
 BR(SDC4_CLK, "sdc4_clk", "sdc4_src", 0x288c, 9, 0x2fc8, 3, 0, 0),

 BR(TSIF_REF_CLK, "tsif_ref_clk", "tsif_ref_src", 0x2710, 9, 0x2fd4, 5, 0, 0),

 BR(USB_HS1_XCVR_CLK, "usb_hs1_xcvr_clk", "usb_hs1_xcvr_src", 0x290c, 9, 0x2fc8,  0, 0, 0),
 BR(USB_HS3_XCVR_CLK, "usb_hs3_xcvr_clk", "usb_hs3_xcvr_src", 0x370c, 9, 0x2fc8,  30, 0, 0),
 BR(USB_HS4_XCVR_CLK, "usb_hs4_xcvr_clk", "usb_hs4_xcvr_src", 0x372c, 9, 0x2fc8,  2, 0, 0),

 BR(USB_HSIC_XCVR_FS_CLK, "usb_hsic_xcvr_fs_clk", "usb_hsic_xcvr_fs_src", 0x2928, 9, 0x2fc8,  2, 0, 0),
 BR(USB_HSIC_SYSTEM_CLK, "usb_hsic_system_clk", "usb_hsic_xcvr_fs_src", 0x292c, 4, 0x2fcc, 24, 0, 0),
 BR(USB_HSIC_HSIC_CLK, "usb_hsic_hsic_clk", "pll14_vote", 0x2b44, 0, 0x2fcc, 19, 0, 0),
 BR(USB_HSIC_HSIO_CAL_CLK, "usb_hsic_hsio_cal_clk", "pxo", 0x2b48, 0, 0x2fcc, 23, 0, 0),
 BR(USB_FS1_XCVR_FS_CLK, "usb_fs1_xcvr_fs_clk", "usb_fs1_xcvr_fs_src", 0x2968, 9, 0x2fcc, 15, 0, 0),
 BR(USB_FS1_SYSTEM_CLK, "usb_fs1_system_clk", "usb_fs1_xcvr_fs_src", 0x296c, 4, 0x2fcc, 16, 0, 0),

 BR(SATA_H_CLK, "sata_h_clk", "cxo", 0x2c00, 4, 0x2fdc, 27, 0, 0),
 BR(SATA_RXOOB_CLK, "sata_rxoob_clk", "sata_clk_src", 0x2c0c, 4, 0x2fdc, 26, 0, 0),
 BR(SATA_PMALIVE_CLK, "sata_pmalive_clk", "sata_clk_src", 0x2c10, 4, 0x2fdc, 25, 0, 0),
 BR(SATA_PHY_REF_CLK, "sata_phy_ref_clk", "pxo", 0x2c14, 4, 0x2fdc, 24, 0, 0),
 BR(SATA_PHY_CFG_CLK, "sata_phy_cfg_clk", "cxo", 0x2c40, 4, 0x2fcc, 12, 0, 0),
 BR(SATA_A_CLK, "sata_a_clk", "cxo", 0x2c20, 4, 0x2fc0, 12, 0, 0),
 BR(SFAB_SATA_S_H_CLK, "sfab_sata_s_h_clk", "cxo", 0x2480, 4, 0x2fc4, 14, 0, 0),

 BR(CE3_CORE_CLK, "ce3_core_clk", "ce3_src", 0x36c4, 4, 0x2fdc, 5, 0, 0),
 BR(CE3_H_CLK, "ce3_h_clk", "ce3_src", 0x36c4, 4, 0x2fc4, 16, 0, 0),

 BR(GSBI1_H_CLK, "gsbi1_h_clk", "cxo", 0x29c0, 4, 0x2fcc, 11, 0x29c0, 6),
 BR(GSBI2_H_CLK, "gsbi2_h_clk", "cxo", 0x29e0, 4, 0x2fcc,  7, 0x29e0, 6),
 BR(GSBI3_H_CLK, "gsbi3_h_clk", "cxo", 0x2a00, 4, 0x2fcc,  3, 0x2a00, 6),
 BR(GSBI4_H_CLK, "gsbi4_h_clk", "cxo", 0x2a20, 4, 0x2fd0, 27, 0x2a20, 6),
 BR(GSBI5_H_CLK, "gsbi5_h_clk", "cxo", 0x2a40, 4, 0x2fd0, 23, 0x2a40, 6),
 BR(GSBI6_H_CLK, "gsbi6_h_clk", "cxo", 0x2a60, 4, 0x2fd0, 19, 0x2a60, 6),
 BR(GSBI7_H_CLK, "gsbi7_h_clk", "cxo", 0x2a80, 4, 0x2fd0, 15, 0x2a80, 6),

 BR(TSIF_H_CLK, "tsif_h_clk", "cxo", 0x2700, 4, 0x2fd4, 7, 0x2700, 6),
 BR(USB_FS1_H_CLK, "usb_fs1_h_clk", "cxo", 0x2960, 4, 0x2fcc, 17, 0, 0),
 BR(USB_HS1_H_CLK, "usb_hs1_h_clk", "cxo", 0x2900, 4, 0x2fc8, 1, 0, 0),
 BR(USB_HSIC_H_CLK, "usb_hsic_h_clk", "cxo", 0x2920, 4, 0x2fcc, 28, 0, 0),
 BR(USB_HS3_H_CLK, "usb_hs3_h_clk", "cxo", 0x3700, 4, 0x2fc8, 31, 0, 0),
 BR(USB_HS4_H_CLK, "usb_hs4_h_clk", "cxo", 0x3720, 4, 0x2fc8, 7, 0, 0),

 BR(SDC1_H_CLK, "sdc1_h_clk", "cxo", 0x2820, 4, 0x2fc8, 11, 0x2820, 6),
 BR(SDC2_H_CLK, "sdc2_h_clk", "cxo", 0x2840, 4, 0x2fc8, 10, 0x2840, 6),
 BR(SDC3_H_CLK, "sdc3_h_clk", "cxo", 0x2860, 4, 0x2fc8,  9, 0x2860, 6),
 BR(SDC4_H_CLK, "sdc4_h_clk", "cxo", 0x2880, 4, 0x2fc8,  8, 0x2880, 6),

 BR(ADM0_CLK, "adm0_clk", "cxo", 0x3080, 2, 0x2fdc, 14, 0, 0),
 BR(ADM0_PBUS_CLK, "adm0_pbus_clk", "cxo", 0x3080, 3, 0x2fdc, 13, 0x2208, 6),

 BR(PCIE_A_CLK, "pcie_a_clk", "cxo", 0x22c0, 4, 0x2fc0, 13, 0, 0),
 BR(PCIE_PHY_REF_CLK, "pcie_phy_ref_clk", "cxo", 0x22d0, 4, 0x2fdc, 29, 0, 0),
 BR(PCIE_H_CLK, "pcie_h_clk", "cxo", 0x22cc, 4, 0x2fd4, 8, 0, 0),
 BR(PMIC_ARB0_H_CLK, "pmic_arb0_h_clk", "cxo", 0x3080, 8, 0x2fd8, 22, 0, 0),
 BR(PMIC_ARB1_H_CLK, "pmic_arb1_h_clk", "cxo", 0x3080, 9, 0x2fd8, 21, 0, 0),
 BR(PMIC_SSBI2_CLK, "pmic_ssbi2_clk", "cxo", 0x3080, 7, 0x2fd8, 23, 0, 0),
 BR(RPM_MSG_RAM_H_CLK, "rpm_msg_ram_h_clk", "cxo", 0x3080, 6, 0x2fd8, 12, 0x27e0, 6),
};

static const struct reset resets[] = {
	[QDSS_STM_RESET] = { 0x2060, 6 },
	[AFAB_SMPSS_S_RESET] = { 0x20b8, 2 },
	[AFAB_SMPSS_M1_RESET] = { 0x20b8, 1 },
	[AFAB_SMPSS_M0_RESET] = { 0x20b8, 0 },
	[AFAB_EBI1_CH0_RESET] = { 0x20c0, 7 },
	[AFAB_EBI1_CH1_RESET] = { 0x20c4, 7},
	[SFAB_ADM0_M0_RESET] = { 0x21e0, 7 },
	[SFAB_ADM0_M1_RESET] = { 0x21e4, 7 },
	[SFAB_ADM0_M2_RESET] = { 0x21e8, 7 },
	[ADM0_C2_RESET] = { 0x220c, 4},
	[ADM0_C1_RESET] = { 0x220c, 3},
	[ADM0_C0_RESET] = { 0x220c, 2},
	[ADM0_PBUS_RESET] = { 0x220c, 1 },
	[ADM0_RESET] = { 0x220c, 0 },
	[QDSS_CLKS_SW_RESET] = { 0x2260, 5 },
	[QDSS_POR_RESET] = { 0x2260, 4 },
	[QDSS_TSCTR_RESET] = { 0x2260, 3 },
	[QDSS_HRESET_RESET] = { 0x2260, 2 },
	[QDSS_AXI_RESET] = { 0x2260, 1 },
	[QDSS_DBG_RESET] = { 0x2260, 0 },
	[SFAB_PCIE_M_RESET] = { 0x22d8, 1 },
	[SFAB_PCIE_S_RESET] = { 0x22d8, 0 },
	[PCIE_EXT_PCI_RESET] = { 0x22dc, 6 },
	[PCIE_PHY_RESET] = { 0x22dc, 5 },
	[PCIE_PCI_RESET] = { 0x22dc, 4 },
	[PCIE_POR_RESET] = { 0x22dc, 3 },
	[PCIE_HCLK_RESET] = { 0x22dc, 2 },
	[PCIE_ACLK_RESET] = { 0x22dc, 0 },
	[SFAB_USB3_M_RESET] = { 0x2360, 7 },
	[SFAB_RIVA_M_RESET] = { 0x2380, 7 },
	[SFAB_LPASS_RESET] = { 0x23a0, 7 },
	[SFAB_AFAB_M_RESET] = { 0x23e0, 7 },
	[AFAB_SFAB_M0_RESET] = { 0x2420, 7 },
	[AFAB_SFAB_M1_RESET] = { 0x2424, 7 },
	[SFAB_SATA_S_RESET] = { 0x2480, 7 },
	[SFAB_DFAB_M_RESET] = { 0x2500, 7 },
	[DFAB_SFAB_M_RESET] = { 0x2520, 7 },
	[DFAB_SWAY0_RESET] = { 0x2540, 7 },
	[DFAB_SWAY1_RESET] = { 0x2544, 7 },
	[DFAB_ARB0_RESET] = { 0x2560, 7 },
	[DFAB_ARB1_RESET] = { 0x2564, 7 },
	[PPSS_PROC_RESET] = { 0x2594, 1 },
	[PPSS_RESET] = { 0x2594, 0},
	[DMA_BAM_RESET] = { 0x25c0, 7 },
	[SPS_TIC_H_RESET] = { 0x2600, 7 },
	[SFAB_CFPB_M_RESET] = { 0x2680, 7 },
	[SFAB_CFPB_S_RESET] = { 0x26c0, 7 },
	[TSIF_H_RESET] = { 0x2700, 7 },
	[CE1_H_RESET] = { 0x2720, 7 },
	[CE1_CORE_RESET] = { 0x2724, 7 },
	[CE1_SLEEP_RESET] = { 0x2728, 7 },
	[CE2_H_RESET] = { 0x2740, 7 },
	[CE2_CORE_RESET] = { 0x2744, 7 },
	[SFAB_SFPB_M_RESET] = { 0x2780, 7 },
	[SFAB_SFPB_S_RESET] = { 0x27a0, 7 },
	[RPM_PROC_RESET] = { 0x27c0, 7 },
	[PMIC_SSBI2_RESET] = { 0x280c, 12 },
	[SDC1_RESET] = { 0x2830, 0 },
	[SDC2_RESET] = { 0x2850, 0 },
	[SDC3_RESET] = { 0x2870, 0 },
	[SDC4_RESET] = { 0x2890, 0 },
	[USB_HS1_RESET] = { 0x2910, 0 },
	[USB_HSIC_RESET] = { 0x2934, 0 },
	[USB_FS1_XCVR_RESET] = { 0x2974, 1 },
	[USB_FS1_RESET] = { 0x2974, 0 },
	[GSBI1_RESET] = { 0x29dc, 0 },
	[GSBI2_RESET] = { 0x29fc, 0 },
	[GSBI3_RESET] = { 0x2a1c, 0 },
	[GSBI4_RESET] = { 0x2a3c, 0 },
	[GSBI5_RESET] = { 0x2a5c, 0 },
	[GSBI6_RESET] = { 0x2a7c, 0 },
	[GSBI7_RESET] = { 0x2a9c, 0 },
	[SPDM_RESET] = { 0x2b6c, 0 },
	[TLMM_H_RESET] = { 0x2ba0, 7 },
	[SATA_SFAB_M_RESET] = { 0x2c18, 0 },
	[SATA_RESET] = { 0x2c1c, 0 },
	[GSS_SLP_RESET] = { 0x2c60, 7 },
	[GSS_RESET] = { 0x2c64, 0 },
	[TSSC_RESET] = { 0x2ca0, 7 },
	[PDM_RESET] = { 0x2cc0, 12 },
	[MPM_H_RESET] = { 0x2da0, 7 },
	[MPM_RESET] = { 0x2da4, 0 },
	[SFAB_SMPSS_S_RESET] = { 0x2e00, 7 },
	[PRNG_RESET] = { 0x2e80, 12 },
	[RIVA_RESET] = { 0x35e0, 0 },
	[CE3_H_RESET] = { 0x36c4, 7 },
	[SFAB_CE3_M_RESET] = { 0x36c8, 1 },
	[SFAB_CE3_S_RESET] = { 0x36c8, 0 },
	[CE3_RESET] = { 0x36cc, 7 },
	[CE3_SLEEP_RESET] = { 0x36d0, 7 },
	[USB_HS3_RESET] = { 0x3710, 0 },
	[USB_HS4_RESET] = { 0x3730, 0 },
};

static void
init_fixed(struct apq8064_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(fixed_tbl); i++) {
		rv = clknode_fixed_register(sc->clkdom, fixed_tbl + i);
		if (rv != 0)
			panic("clknode_fixed_register failed");
	}
}


static void
init_gates(struct apq8064_gcc_softc *sc)
{
	int i, rv;


	for (i = 0; i < nitems(gate_tbl); i++) {
		rv = clknode_gate_register(sc->clkdom, gate_tbl + i);
		if (rv != 0)
			panic("clk_gate_register failed");
	}
}


static void
init_plls(struct apq8064_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(pll_tbl); i++) {
		rv = apg8064_pll_register(sc->clkdom, pll_tbl + i);
		if (rv != 0)
			panic("apg8064_pll_register failed");
	}
}

static void
init_hfplls(struct apq8064_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(hfpll_tbl); i++) {
		rv = apg8064_hfpll_register(sc->clkdom, hfpll_tbl + i);
		if (rv != 0)
			panic("apg8064_hfpll_register failed");
	}
}

static void
init_rcgs(struct apq8064_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(rcg_tbl); i++) {
		rv = apg8064_rcg_register(sc->clkdom, rcg_tbl + i);
		if (rv != 0)
			panic("apg8064_rcg_register failed");
	}
}
static void
init_branches(struct apq8064_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(br_tbl); i++) {
		rv = apg8064_br_register(sc->clkdom, br_tbl + i);
		if (rv != 0)
			panic("apg8064_br_register failed");
	}
}

static void
register_clocks(device_t dev)
{
	struct apq8064_gcc_softc *sc;
	struct clknode *clknode;

	sc = device_get_softc(dev);
	sc->clkdom = clkdom_create(dev);
	init_fixed(sc);
	init_plls(sc);
	init_hfplls(sc);
	init_gates(sc);
	init_rcgs(sc);
	init_branches(sc);
	clkdom_finit(sc->clkdom);

	clknode = clknode_find_by_name(sc->clkdom, "gsbi6_uart_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
clknode_set_freq(clknode, 1843200, 1, 1);

	clknode = clknode_find_by_name(sc->clkdom, "sdc1_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode = clknode_find_by_name(sc->clkdom, "sdc3_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode = clknode_find_by_name(sc->clkdom, "sdc4_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode = clknode_find_by_name(sc->clkdom, "sata_clk_src");
	clknode_set_parent_by_name(clknode, "pll3_vote");
clknode_set_freq(clknode, 100000000, 1, 1);


	clknode = clknode_find_by_name(sc->clkdom, "usb_hs1_xcvr_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode_set_freq(clknode, 60000000, 1, 1);
	clknode = clknode_find_by_name(sc->clkdom, "usb_hs3_xcvr_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode_set_freq(clknode, 60000000, 1, 1);
	clknode = clknode_find_by_name(sc->clkdom, "usb_hs4_xcvr_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode_set_freq(clknode, 60000000, 1, 1);
	clknode = clknode_find_by_name(sc->clkdom, "usb_fs1_xcvr_fs_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode_set_freq(clknode, 60000000, 1, 1);
	clknode = clknode_find_by_name(sc->clkdom, "usb_hsic_xcvr_fs_src");
	clknode_set_parent_by_name(clknode, "pll8_vote");
	clknode_set_freq(clknode, 60000000, 1, 1);

	clknode = clknode_find_by_name(sc->clkdom, "usb_hsic_hsic_clk");
	clknode_set_freq(clknode, 480000000, 1, 1);
	clknode = clknode_find_by_name(sc->clkdom, "usb_hsic_system_clk");
	clknode_set_freq(clknode, 60000000, 1, 1);

	clkdom_finit(sc->clkdom);
//	postinit_clock(sc);
}

static int
apq8064_gcc_clkdev_read(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct apq8064_gcc_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	*val = bus_read_4(sc->mem_res, addr);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apq8064_gcc_clkdev_write(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct apq8064_gcc_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	bus_write_4(sc->mem_res, addr, val);
	mtx_unlock(&sc->mtx);
	return (0);
}


static int
apq8064_gcc_clkdev_modify(device_t dev, bus_addr_t addr, uint32_t clear_mask,
    uint32_t set_mask)
{
	struct apq8064_gcc_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
	reg = bus_read_4(sc->mem_res, addr);
	reg &= clear_mask;
	reg |= set_mask;
	bus_write_4(sc->mem_res, addr, reg);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apq8064_gcc_reset_set(device_t dev, int id, int value)
{
	struct apq8064_gcc_softc *sc = device_get_softc(dev);
	uint32_t reg;

	if (id >= nitems(resets))
		return (ENXIO);

	if (resets[id].reg == 0)
		return (ENXIO);
	mtx_lock(&sc->mtx);
	reg = bus_read_4(sc->mem_res, resets[id].reg);
	if (value)
		reg |= 1 << resets[id].bit;
	else
		reg &= ~(1 << resets[id].bit);
	bus_write_4(sc->mem_res, resets[id].reg, reg);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apq8064_gcc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Qualcomm GCC Clock Driver");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}


static int
apq8064_gcc_attach(device_t dev)
{
	struct apq8064_gcc_softc *sc = device_get_softc(dev);
	int rid, rv;

	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	/* Resource setup. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "cannot allocate memory resource\n");
		rv = ENXIO;
		goto fail;
	}
	register_clocks(dev);
	fdt_reset_register_provider(dev);

	return (0);

fail:
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (rv);
}

static int
apq8064_gcc_detach(device_t dev)
{

	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t apq8064_gcc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apq8064_gcc_probe),
	DEVMETHOD(device_attach,	apq8064_gcc_attach),
	DEVMETHOD(device_detach,	apq8064_gcc_detach),
	/* fdt reset interface */
	DEVMETHOD(fdt_reset_set,	apq8064_gcc_reset_set),
	/* clkdev  interface*/
	DEVMETHOD(clkdev_read,		apq8064_gcc_clkdev_read),
	DEVMETHOD(clkdev_write,		apq8064_gcc_clkdev_write),
	DEVMETHOD(clkdev_modify,	apq8064_gcc_clkdev_modify),

	DEVMETHOD_END
};

static devclass_t apq8064_gcc_devclass;

static driver_t apq8064_gcc_driver = {
	"apq8064_gcc",
	apq8064_gcc_methods,
	sizeof(struct apq8064_gcc_softc),
};

EARLY_DRIVER_MODULE(apq8064_gcc, simplebus, apq8064_gcc_driver,
    apq8064_gcc_devclass, 0, 0, BUS_PASS_TIMER);

