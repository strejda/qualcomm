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
 *
 * $FreeBSD$
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/fdt_phy.h>
#include <dev/fdt/fdt_reset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define UNIPHY_PLL_REFCLK_CFG		0x000
#define UNIPHY_PLL_POSTDIV1_CFG		0x004
#define UNIPHY_PLL_CHGPUMP_CFG		0x008
#define UNIPHY_PLL_VCOLPF_CFG		0x00C
#define UNIPHY_PLL_VREG_CFG		0x010
#define UNIPHY_PLL_PWRGEN_CFG		0x014
#define UNIPHY_PLL_DMUX_CFG		0x018
#define UNIPHY_PLL_AMUX_CFG		0x01C
#define UNIPHY_PLL_GLB_CFG		0x020
#define UNIPHY_PLL_POSTDIV2_CFG		0x024
#define UNIPHY_PLL_POSTDIV3_CFG		0x028
#define UNIPHY_PLL_LPFR_CFG		0x02C
#define UNIPHY_PLL_LPFC1_CFG		0x030
#define UNIPHY_PLL_LPFC2_CFG		0x034
#define UNIPHY_PLL_SDM_CFG0		0x038
#define UNIPHY_PLL_SDM_CFG1		0x03C
#define UNIPHY_PLL_SDM_CFG2		0x040
#define UNIPHY_PLL_SDM_CFG3		0x044
#define UNIPHY_PLL_SDM_CFG4		0x048
#define UNIPHY_PLL_SSC_CFG0		0x04C
#define UNIPHY_PLL_SSC_CFG1		0x050
#define UNIPHY_PLL_SSC_CFG2		0x054
#define UNIPHY_PLL_SSC_CFG3		0x058
#define UNIPHY_PLL_LKDET_CFG0		0x05C
#define UNIPHY_PLL_LKDET_CFG1		0x060
#define UNIPHY_PLL_LKDET_CFG2		0x064
#define UNIPHY_PLL_TEST_CFG		0x068
#define UNIPHY_PLL_CAL_CFG0		0x06C
#define UNIPHY_PLL_CAL_CFG1		0x070
#define UNIPHY_PLL_CAL_CFG2		0x074
#define UNIPHY_PLL_CAL_CFG3		0x078
#define UNIPHY_PLL_CAL_CFG4		0x07C
#define UNIPHY_PLL_CAL_CFG5		0x080
#define UNIPHY_PLL_CAL_CFG6		0x084
#define UNIPHY_PLL_CAL_CFG7		0x088
#define UNIPHY_PLL_CAL_CFG8		0x08C
#define UNIPHY_PLL_CAL_CFG9		0x090
#define UNIPHY_PLL_CAL_CFG10		0x094
#define UNIPHY_PLL_CAL_CFG11		0x098
#define UNIPHY_PLL_EFUSE_CFG		0x09C
#define UNIPHY_PLL_DEBUG_BUS_SEL	0x0A0
#define UNIPHY_PLL_CTRL_42		0x0A4
#define UNIPHY_PLL_CTRL_43		0x0A8
#define UNIPHY_PLL_CTRL_44		0x0AC
#define UNIPHY_PLL_CTRL_45		0x0B0
#define UNIPHY_PLL_CTRL_46		0x0B4
#define UNIPHY_PLL_CTRL_47		0x0B8
#define UNIPHY_PLL_CTRL_48		0x0BC
#define UNIPHY_PLL_STATUS		0x0C0
#define UNIPHY_PLL_DEBUG_BUS0		0x0C4
#define UNIPHY_PLL_DEBUG_BUS1		0x0C8
#define UNIPHY_PLL_DEBUG_BUS2		0x0CC
#define UNIPHY_PLL_DEBUG_BUS3		0x0D0
#define UNIPHY_PLL_CTRL_54		0x0D4

#define SATA_PHY_SER_CTRL		0x100
#define SATA_PHY_TX_DRIV_CTRL0		0x104
#define SATA_PHY_TX_DRIV_CTRL1		0x108
#define SATA_PHY_TX_DRIV_CTRL2		0x10C
#define SATA_PHY_TX_DRIV_CTRL3		0x110
#define SATA_PHY_TX_RESV0		0x114
#define SATA_PHY_TX_RESV1		0x118
#define SATA_PHY_TX_IMCAL0		0x11C
#define SATA_PHY_TX_IMCAL1		0x120
#define SATA_PHY_TX_IMCAL2		0x124
#define SATA_PHY_RX_IMCAL0		0x128
#define SATA_PHY_RX_IMCAL1		0x12C
#define SATA_PHY_RX_IMCAL2		0x130
#define SATA_PHY_RX_TERM		0x134
#define SATA_PHY_RX_TERM_RESV		0x138
#define SATA_PHY_EQUAL			0x13C
#define SATA_PHY_EQUAL_RESV		0x140
#define SATA_PHY_OOB_TERM		0x144
#define SATA_PHY_CDR_CTRL0		0x148
#define SATA_PHY_CDR_CTRL1		0x14C
#define SATA_PHY_CDR_CTRL2		0x150
#define SATA_PHY_CDR_CTRL3		0x154
#define SATA_PHY_CDR_CTRL4		0x158
#define SATA_PHY_FA_LOAD0		0x15C
#define SATA_PHY_FA_LOAD1		0x160
#define SATA_PHY_CDR_CTRL_RESV		0x164
#define SATA_PHY_PI_CTRL0		0x168
#define SATA_PHY_PI_CTRL1		0x16C
#define SATA_PHY_DESER_RESV		0x170
#define SATA_PHY_RX_RESV0		0x174
#define SATA_PHY_AD_TPA_CTRL		0x178
#define SATA_PHY_REFCLK_CTRL		0x17C
#define SATA_PHY_POW_DWN_CTRL0		0x180
#define SATA_PHY_POW_DWN_CTRL1		0x184
#define SATA_PHY_TX_DATA_CTRL		0x188
#define SATA_PHY_BIST_GEN0		0x18C
#define SATA_PHY_BIST_GEN1		0x190
#define SATA_PHY_BIST_GEN2		0x194
#define SATA_PHY_BIST_GEN3		0x198
#define SATA_PHY_LBK_CTRL		0x19C
#define SATA_PHY_TEST_DEBUG_CTRL	0x1A0
#define SATA_PHY_ALIGNP			0x1A4
#define SATA_PHY_PRBS_CFG0		0x1A8
#define SATA_PHY_PRBS_CFG1		0x1AC
#define SATA_PHY_PRBS_CFG2		0x1B0
#define SATA_PHY_PRBS_CFG3		0x1B4
#define SATA_PHY_CHAN_COMP_CHK_CNT	0x1B8
#define SATA_PHY_RESET_CTRL		0x1BC
#define SATA_PHY_RX_CLR			0x1C0
#define SATA_PHY_RX_EBUF_CTRL		0x1C4
#define SATA_PHY_ID0			0x1C8
#define SATA_PHY_ID1			0x1CC
#define SATA_PHY_ID2			0x1D0
#define SATA_PHY_ID3			0x1D4
#define SATA_PHY_RX_CHK_ERR_CNT0	0x1D8
#define SATA_PHY_RX_CHK_ERR_CNT1	0x1DC
#define SATA_PHY_RX_CHK_STAT		0x1E0
#define SATA_PHY_TX_IMCAL_STAT		0x1E4
#define SATA_PHY_RX_IMCAL_STAT		0x1E8
#define SATA_PHY_RX_EBUF_STAT		0x1EC
#define SATA_PHY_DEBUG_BUS_STAT0	0x1F0
#define SATA_PHY_DEBUG_BUS_STAT1	0x1F4
#define SATA_PHY_DEBUG_BUS_STAT2	0x1F8
#define SATA_PHY_DEBUG_BUS_STAT3	0x1FC

#define UNIPHY_PLL_LOCK			(1 <<  0)
#define SATA_PHY_TX_CAL			(1 <<  0)
#define SATA_PHY_RX_CAL			(1 <<  0)

#define TIMEOUT_MS		10000

struct sataphy_softc {
	device_t		dev;
	struct resource		*mem_res;
	clk_t			clk_cfg;
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,apq8064-sata-phy",	1},
	{NULL,				0},
};

#define RD4(sc, offs)		bus_read_4(sc->mem_res, offs)
#define WR4(sc, offs, val)	bus_write_4(sc->mem_res, offs, val)


static int
wait_for_reg(struct sataphy_softc *sc, uint32_t reg, uint32_t mask)
{
	int i;

	for (i = 0; i < (TIMEOUT_MS * 100); i++) {
		if ((RD4(sc, reg) & mask) != 0)
			break;
		DELAY(10);
	}
	if (i >= (TIMEOUT_MS * 100))
		return (ETIMEDOUT);
	return (0);
}

static int
sataphy_enable(struct sataphy_softc *sc)
{
	int rv;

	/* Take phy to defined state */
//	WR4(sc, SATA_PHY_RESET_CTRL, 0x01);
//	DELAY(10);
//	WR4(sc, SATA_PHY_RESET_CTRL, 0x00);
	WR4(sc, SATA_PHY_SER_CTRL, 0x01);
	WR4(sc, SATA_PHY_POW_DWN_CTRL0, 0xB1);
	RD4(sc, SATA_PHY_POW_DWN_CTRL0);
	DELAY(10);

	WR4(sc, SATA_PHY_POW_DWN_CTRL0, 0x01);
	WR4(sc, SATA_PHY_POW_DWN_CTRL1, 0x3E);

	WR4(sc, SATA_PHY_RX_IMCAL0, 0x01);
	WR4(sc, SATA_PHY_TX_IMCAL0, 0x01);
	WR4(sc, SATA_PHY_TX_IMCAL2, 0x02);
	RD4(sc, SATA_PHY_TX_IMCAL2);

	/* Configure UNIPHY PLL */
	WR4(sc, UNIPHY_PLL_REFCLK_CFG, 0x04);
	WR4(sc, UNIPHY_PLL_PWRGEN_CFG, 0x00);
	WR4(sc, UNIPHY_PLL_CAL_CFG0, 0x0A);
	WR4(sc, UNIPHY_PLL_CAL_CFG8, 0xF3);
	WR4(sc, UNIPHY_PLL_CAL_CFG9, 0x01);
	WR4(sc, UNIPHY_PLL_CAL_CFG10, 0xED);
	WR4(sc, UNIPHY_PLL_CAL_CFG11, 0x02);
	WR4(sc, UNIPHY_PLL_SDM_CFG0, 0x36);
	WR4(sc, UNIPHY_PLL_SDM_CFG1, 0x0D);
	WR4(sc, UNIPHY_PLL_SDM_CFG2, 0xA3);
	WR4(sc, UNIPHY_PLL_SDM_CFG3, 0xF0);
	WR4(sc, UNIPHY_PLL_SDM_CFG4, 0x00);
	WR4(sc, UNIPHY_PLL_SSC_CFG0, 0x19);
	WR4(sc, UNIPHY_PLL_SSC_CFG1, 0xE1);
	WR4(sc, UNIPHY_PLL_SSC_CFG2, 0x00);
	WR4(sc, UNIPHY_PLL_SSC_CFG3, 0x11);
	WR4(sc, UNIPHY_PLL_LKDET_CFG0, 0x04);
	WR4(sc, UNIPHY_PLL_LKDET_CFG1, 0xFF);
	WR4(sc, UNIPHY_PLL_GLB_CFG, 0x02);
	RD4(sc, UNIPHY_PLL_GLB_CFG);
	DELAY(10);
	WR4(sc, UNIPHY_PLL_GLB_CFG, 0x03);
	WR4(sc, UNIPHY_PLL_LKDET_CFG2, 0x05);

	/* Wait for PLL lock */
	rv = wait_for_reg(sc, UNIPHY_PLL_STATUS, UNIPHY_PLL_LOCK);
	if (rv != 0) {
		device_printf(sc->dev, "Uniphy PLL lock timeouted.\n");
		return (rv);
	}

	/* Wait for TX calibration  */
	rv = wait_for_reg(sc, SATA_PHY_TX_IMCAL_STAT, SATA_PHY_TX_CAL);
	if (rv != 0) {
		device_printf(sc->dev, "TX calibration timeouted\n");
		return (rv);
	}

	/* Wait for RX calibration  */
	rv = wait_for_reg(sc, SATA_PHY_RX_IMCAL_STAT, SATA_PHY_RX_CAL);
	if (rv != 0) {
		device_printf(sc->dev, "RX calibration timeouted\n");
		return (rv);
	}

	/* PLL is locked, calibrations done, set phy to operational state */
	WR4(sc, SATA_PHY_POW_DWN_CTRL1, 0x3E);
	WR4(sc, SATA_PHY_RX_IMCAL0, 0x01);
	WR4(sc, SATA_PHY_TX_IMCAL0, 0x01);

	WR4(sc, SATA_PHY_POW_DWN_CTRL1, 0x00);
	WR4(sc, SATA_PHY_CDR_CTRL0, 0x59);
	WR4(sc, SATA_PHY_CDR_CTRL1, 0x04);
	WR4(sc, SATA_PHY_CDR_CTRL2, 0x00);
	WR4(sc, SATA_PHY_PI_CTRL0, 0x00);
	WR4(sc, SATA_PHY_CDR_CTRL3, 0x00);
	WR4(sc, SATA_PHY_POW_DWN_CTRL0, 0x01);

	WR4(sc, SATA_PHY_TX_DATA_CTRL, 0x11);
	WR4(sc, SATA_PHY_ALIGNP, 0x43);
	WR4(sc, SATA_PHY_OOB_TERM, 0x04);

	WR4(sc, SATA_PHY_EQUAL, 0x01);
	WR4(sc, SATA_PHY_TX_DRIV_CTRL0, 0x09);
	WR4(sc, SATA_PHY_TX_DRIV_CTRL1, 0x09);

	return 0;
}

static int
sataphy_disable(struct sataphy_softc *sc)
{

	/* Powerdown PLL and PHY */
	WR4(sc, SATA_PHY_POW_DWN_CTRL0, 0xF8);
	WR4(sc, SATA_PHY_POW_DWN_CTRL1, 0xFE);
	WR4(sc, UNIPHY_PLL_GLB_CFG, 0x00);

	return 0;
}

static int
sataphy_phy_cfg(device_t dev, int id, int val)
{
	struct sataphy_softc *sc;
	int rv = 0;

	sc = device_get_softc(dev);
	if (val != 0)
		rv = sataphy_enable(sc);
	else
		rv = sataphy_disable(sc);
	return (rv);
}

static int
sataphy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm SATA phy");
	return (BUS_PROBE_DEFAULT);
}

static int
sataphy_detach(device_t dev)
{

	/* This device is always present. */
	return (EBUSY);
}

static int
sataphy_attach(device_t dev)
{
	struct sataphy_softc * sc;
	int rid, rv;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate memory resources\n");
		return (ENXIO);
	}

	node = ofw_bus_get_node(dev);
	rv = clk_get_by_ofw_name(node, "cfg", &sc->clk_cfg);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot get clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->clk_cfg);
	if (rv != 0) {
		device_printf(sc->dev, "Cannot enable clock: %d\n", rv);
		return (ENXIO);
	}

	fdt_phy_register_provider(dev);

	return (0);
}


static device_method_t qcom_sataphy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		sataphy_probe),
	DEVMETHOD(device_attach,	sataphy_attach),
	DEVMETHOD(device_detach,	sataphy_detach),

	/* fdt_phy interface */
	DEVMETHOD(fdt_phy_set,		sataphy_phy_cfg),

	DEVMETHOD_END
};

static driver_t qcom_sataphy_driver = {
	"qcom_sataphy",
	qcom_sataphy_methods,
	sizeof(struct sataphy_softc),
};

static devclass_t qcom_sataphy_devclass;

EARLY_DRIVER_MODULE(qcom_sataphy, simplebus, qcom_sataphy_driver,
    qcom_sataphy_devclass, 0, 0, 74);
