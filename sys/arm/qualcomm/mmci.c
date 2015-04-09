/*-
 * Copyright (c) 2014 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>

#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include <arm/qualcomm/mmci.h>

#define	DEBUG	1

#ifdef DEBUG
#define debugf(fmt, args...) do { printf("%s(): ", __func__);   \
    printf(fmt,##args); } while (0)
#else
#define debugf(fmt, args...)
#endif

struct mmci_softc {
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
	struct resource 	*irq_res;
	void 			*intrhand;
	clk_t			mclk;
	clk_t			pclk;
	int			clk_freq;
	
	struct mmc_host		host;
	struct mmc_request 	*req;
	uint32_t		fdt_caps;
	uint32_t		pwrup_cmd;
	int			use_pio;
	int			bus_busy;
};

#define	MCI_MAX_BLOCKSIZE	4096

static int mmci_probe(device_t);
static int mmci_attach(device_t);
static int mmci_detach(device_t);
static void mmci_intr(void *);

static void mmci_setup_xfer(struct mmci_softc *, struct mmc_data *);

static int mmci_update_ios(device_t, device_t);
static int mmci_request(device_t, device_t, struct mmc_request *);
static int mmci_get_ro(device_t, device_t);
static int mmci_acquire_host(device_t, device_t);
static int mmci_release_host(device_t, device_t);

#define	mmci_lock(_sc)						\
    mtx_lock(&_sc->mtx);
#define	mmci_unlock(_sc)						\
    mtx_unlock(&_sc->mtx);

#define	RD4(_sc, _reg)					\
    bus_read_4(_sc->mem_res, _reg)

#define	WR4(_sc, _reg, _value)				\
    bus_write_4(_sc->mem_res, _reg, _value)

static int abc = 27000;

static void
mmci_pio_read(struct mmci_softc *sc, struct mmc_command *cmd)
{
	struct mmc_data *data;
	uint32_t *p, status;

	if (cmd == NULL || cmd->data == NULL)
		return;

	data = cmd->data;
	if ((data->flags & MMC_DATA_READ) == 0)
		return;

	KASSERT((data->xfer_len & 3) == 0, ("xfer_len not aligned"));
	p = (uint32_t *)data->data + (data->xfer_len >> 2);

	while (data->xfer_len < data->len) {
		status = RD4(sc, MCI_STATUS);
		if ((status & (MCI_SCM_RXDATAAVLBL | MCI_SCM_RXFIFOHALFFULL | MCI_SCM_RXFIFOFULL)) == 0)
			break;
		*p++ = RD4(sc, MCI_FIFO);
		data->xfer_len += 4;
	}
	if (data->xfer_len >= data->len)
		WR4(sc, MCI_MASK0, MCI_IRQENABLE | MCI_SCM_DATAEND);

//debugf("pio read: len: %d, xfer_len: %d\n", data->len, data->xfer_len  );
}

static void
mmci_pio_write(struct mmci_softc *sc, struct mmc_command *cmd)
{
	struct mmc_data *data;
	uint32_t *p, status;

	if (cmd == NULL || cmd->data == NULL)
		return;

	data = cmd->data;
	if ((data->flags & MMC_DATA_WRITE) == 0)
		return;

	KASSERT((data->xfer_len & 3) == 0, ("xfer_len not aligned"));
	p = (uint32_t *)data->data + (data->xfer_len >> 2);
if (abc == 0)
debugf("pio write1: len: %d, xfer_len: %d, %d\n", data->len, data->xfer_len,  RD4(sc, MCI_FIFOCNT));

	while (data->xfer_len < data->len) {
		status = RD4(sc, MCI_STATUS);
//		if ((status & MCI_SCM_TXFIFOHALFEMPTY) == 0)
		if ((status & MCI_SCM_TXFIFOEMPTY) == 0)
			break;
		WR4(sc, MCI_FIFO, *p++);
		data->xfer_len += 4;
	}
	if (data->xfer_len >= data->len)
		WR4(sc, MCI_MASK0, MCI_IRQENABLE | MCI_SCM_DATAEND);
if (abc == 0)
debugf("pio write2: len: %d, xfer_len: %d, %d\n", data->len, data->xfer_len,  RD4(sc, MCI_FIFOCNT));
}

static void
mmci_setup_xfer(struct mmci_softc *sc, struct mmc_data *data)
{
	uint32_t datactrl, mask;

//if (!(data->flags & MMC_DATA_READ))
//abc = 0;
	
if (abc == 0)
debugf("data: %p, len: %d, %s\n", data,
	    data->len, (data->flags & MMC_DATA_READ) ? "read" : "write");

	if (data->flags & MMC_DATA_READ) {
		datactrl = MCI_DATA_CTL_READ;
	}

	if (data->flags & MMC_DATA_WRITE) {
		datactrl = MCI_DATA_CTL_WRITE;
	}


//	datactrl |= MCI_DATACTRL_DMAENABLE | MCI_DATACTRL_ENABLE;
	datactrl |= MCI_DATA_CTL_ENABLE;
//	datactrl |= (ffs(data->len) - 1) << 4;
	datactrl |= data->len << 4;

	if (data->flags & MMC_DATA_READ) {
		mask = MCI_SCM_RXFIFOHALFFULL;

		/*
		 * If we have less than the fifo 'half-full' threshold to
		 * transfer, trigger a PIO interrupt as soon as any data
		 * is available.
		 */
		//if (data->len < 16)
			mask |= MCI_SCM_RXDATAAVLBL;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		mask = 0 ;// MCI_SCM_TXFIFOHALFEMPTY;
	}

	WR4(sc, MCI_DATATIMER, 0xFFFF0000);
	WR4(sc, MCI_DATALENGTH, data->len);
	WR4(sc, MCI_DATA_CTL, datactrl);

	WR4(sc, MCI_MASK0, (RD4(sc, MCI_MASK0) | mask));

if (abc == 0)	
	debugf("datactrl: 0x%08x 0x%08x 0x%08x\n", RD4(sc, MCI_DATA_CTL),  RD4(sc, MCI_DATALENGTH),  RD4(sc, MCI_MASK0));
//debugf("mask: 0x%08x 0x%08x\n", RD4(sc, MCI_MASK0), mask);
}

static void
mmci_xfer_done(struct mmci_softc *sc)
{
if (abc == 0)	
 debugf("mmci_xfer_done\n");
	WR4(sc, MCI_MASK0, MCI_IRQENABLE);
	if (sc->req) {
		sc->req->done(sc->req);
		sc->req = NULL;
	}
}


static void
mmci_data_irq(struct mmci_softc *sc, uint32_t status)
{
	struct mmc_command *cmd;
	

	/* Check for stray interrupts first */
	if (sc->req == NULL) {
		device_printf(sc->dev, "Stray data interrupt: 0x%08X\n",
		    status);
		return;
	}
	if (sc->req->cmd->data == NULL)
		return;
	
	cmd = sc->req->cmd;
	/* First check for data errors */
	if (status & (MCI_SCM_DATACRCFAIL | MCI_SCM_DATATIMEOUT |
		      MCI_SCM_STARTBITERR | MCI_SCM_TXUNDERRUN |
		      MCI_SCM_RXOVERRUN)) {
		      	
		debugf("Data error: 0x%08X\n", status);
		if (sc->use_pio == 0) {
			/* XXXX Abort  DMA if active*/
		}
		

		if (status & MCI_SCM_DATATIMEOUT) {
			cmd->error = MMC_ERR_TIMEOUT;
		} else if (status & MCI_SCM_DATACRCFAIL) {
			cmd->error = MMC_ERR_BADCRC;
		} else if (status & (MCI_SCM_TXUNDERRUN | MCI_SCM_RXOVERRUN)) {
			cmd->error = MMC_ERR_FIFO;
		} else {
			cmd->error = MMC_ERR_FAILED;
		}
		mmci_xfer_done(sc);
		return;
	}

//	if (status & MCI_SCM_DATABLOCKEND)
//		device_printf(sc->dev, "Stray data block end interrupt\n");


	if (status & MCI_SCM_DATAEND || cmd->error) {
		if (sc->use_pio == 0) {
			/* XXXX Finish  DMA if active*/
		}		
		mmci_xfer_done(sc);
	}
}


static void
mmci_intr(void *arg)
{
	struct mmci_softc *sc = (struct mmci_softc *)arg;
	struct mmc_command *cmd;
	uint32_t status, status1, mask;

	status1 = RD4(sc, MCI_STATUS);
	mask = RD4(sc, MCI_MASK0);
	status = status1 & mask;
	WR4(sc, MCI_CLEAR, status);
//debugf("interrupt: 0x%08x 0x%08x\n", status, status1);
if (abc == 0)
  debugf("interrupt: 0x%08x 0x%08x\n", status, status1);
	if (sc->req == NULL) {
		debugf("Stray interrupt: 0x%08x\n", status);
		WR4(sc, MCI_CLEAR, status);
		return;
	}
	
	if (status & (MCI_SCM_TXFIFOHALFEMPTY | MCI_SCM_RXDATAAVLBL | MCI_SCM_RXFIFOFULL)) {
if (abc == 0)
debugf(" data interrupt: 0x%08x  0x%08x  0x%08x\n", status1, status1 & MCI_SCM_RXACTIVE, status1 & MCI_SCM_TXACTIVE);
		cmd = sc->req->cmd;
		if (status1 & MCI_SCM_RXACTIVE)
			mmci_pio_read(sc, cmd);
		if (status1 & MCI_SCM_TXACTIVE)
			mmci_pio_write(sc, cmd);	
	}

	mmci_data_irq(sc, status);
	
	if (sc->req == NULL)
		return;
	
	if (status & MCI_SCM_CMDCRCFAIL) {
		debugf("command crc fail\n");
		cmd = sc->req->cmd;
		cmd->error = cmd->flags & MMC_RSP_CRC
		    ? MMC_ERR_BADCRC : MMC_ERR_NONE;
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_xfer_done(sc);

	} else if (status & MCI_SCM_CMDTIMEOUT) {
		debugf("command timeout\n");
		cmd = sc->req->cmd;
		cmd->error = MMC_ERR_TIMEOUT;
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_xfer_done(sc);
	} else  if (status & MCI_SCM_CMDRESPEND) {
		cmd = sc->req->cmd;
if (abc == 0)
		debugf("command response: %p\n", cmd->data);

		
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = RD4(sc, MCI_RESP3);
			cmd->resp[2] = RD4(sc, MCI_RESP2);
			cmd->resp[1] = RD4(sc, MCI_RESP1);
		}

		cmd->resp[0] = RD4(sc, MCI_RESP0);
		cmd->error = MMC_ERR_NONE;
	
		if (cmd->data && (cmd->data->flags & MMC_DATA_WRITE))
		    WR4(sc, MCI_MASK0, (RD4(sc, MCI_MASK0) | MCI_SCM_TXFIFOHALFEMPTY));
	

		if (!cmd->data)
			mmci_xfer_done(sc);

	} else if (status & MCI_SCM_CMDSENT) {
if (abc == 0)
		debugf("command sent\n");
		cmd = sc->req->cmd;
		cmd->error = MMC_ERR_NONE;
		if (!cmd->data) 
			mmci_xfer_done(sc);
	}

WR4(sc, MCI_CLEAR, status);
		
if (abc == 0)
	debugf("done: 0x%08x 0x%08x)\n",  RD4(sc, MCI_STATUS), RD4(sc, MCI_MASK0));

}


static void
mmci_cmd(struct mmci_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmdreg = 0;

	if (RD4(sc, MCI_COMMAND) & MCI_COMMAND_ENABLE) {
		WR4(sc, MCI_COMMAND, 0);
		DELAY(1000);
	}


	cmdreg |= (cmd->opcode | MCI_COMMAND_ENABLE);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			cmdreg |= MCI_COMMAND_LONGRSP;
		cmdreg |= MCI_COMMAND_RESPONSE;
	}


	if (cmd->flags & MMC_CMD_ADTC)
		/* XXXX - implementation specific */
		cmdreg |= MCI_COMMAND_QCOM_DAT_CMD;
if (abc == 0)	
	debugf("cmd: %d, flag: 0x%08X,  arg: 0x%08x, cmdreg: 0x%08x, \n", cmd->opcode, cmd->flags, cmd->arg, cmdreg);

	WR4(sc, MCI_ARGUMENT, cmd->arg);
	WR4(sc, MCI_COMMAND, cmdreg);
}



static int
mmci_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct mmci_softc *sc = device_get_softc(bus);

//	debugf("request: %p\n", req);
	mmci_lock(sc);
	if (sc->req) {
		mmci_unlock(sc);
		return (EBUSY);
	}
	sc->req = req;

	if (req->cmd->data)
		mmci_setup_xfer(sc, req->cmd->data);

	mmci_cmd(sc, req->cmd);
	
	mmci_unlock(sc);

	return (0);
}






static int
mmci_read_ivar(device_t bus, device_t child, int which, 
    uintptr_t *result)
{
	struct mmci_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->host.ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = 1;
		break;
	}

	return (0);
}

static int
mmci_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct mmci_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->host.ios.vdd = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}
	return (0);
}

static int
mmci_update_ios(device_t bus, device_t child)
{
	struct mmci_softc *sc = device_get_softc(bus);
	struct mmc_ios *ios = &sc->host.ios;
	uint32_t clkdiv, pwr = 0;

	if (ios->clock != 0) {
		/* Calculate clock divider */
		clkdiv = (sc->clk_freq  / (2 * ios->clock)) - 1;

		/* Clock rate should not exceed rate requested in ios */
		if ((sc->clk_freq / (2 * (clkdiv + 1))) > ios->clock)
			clkdiv++;
		if (clkdiv > 255)
			clkdiv = 255;
		
		if (ios->bus_width == bus_width_4) {
			debugf("using wide bus mode\n");
			clkdiv |= MCI_CLOCK_WIDEBUS;
		}		
		clkdiv |= MCI_CLOCK_QCOM_FLOW_ENA; 
		clkdiv |= MCI_CLOCK_QCOM_SELECT_FBCLK;
		clkdiv |= MCI_CLOCK_ENABLE;
	} else {
		clkdiv = 0;
	}
	
	
	debugf("clock: %dHz, clkdiv: %d 0x%08X\n", ios->clock, clkdiv, clkdiv);

	WR4(sc, MCI_CLOCK, clkdiv);

	switch (ios->power_mode) {
	case power_off:
		pwr |= MCI_POWER_CTRL_OFF;
		break;
	case power_up:
		pwr |= MCI_POWER_CTRL_UP;
		break;
	case power_on:
		pwr |= sc->pwrup_cmd;
		break;
	}

	if (ios->bus_mode == opendrain)
		pwr |= MCI_POWER_OPENDRAIN;

	WR4(sc, MCI_POWER, pwr);

	return (0);
}

static int
mmci_get_ro(device_t bus, device_t child)
{
printf("%s: enter\n", __func__);
	return (0);
}

static int
mmci_acquire_host(device_t bus, device_t child)
{
	struct mmci_softc *sc = device_get_softc(bus);
	int error = 0;

	mmci_lock(sc);
	while (sc->bus_busy)
		error = mtx_sleep(sc, &sc->mtx, PZERO, "mmci", 0);

	sc->bus_busy++;
	mmci_unlock(sc);
	return (error);
}

static int
mmci_release_host(device_t bus, device_t child)
{
	struct mmci_softc *sc = device_get_softc(bus);

	mmci_lock(sc);
	sc->bus_busy--;
	wakeup(sc);
	mmci_unlock(sc);
	return (0);
}

static int
mmci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "arm,pl18x"))
		return (ENXIO);

	device_set_desc(dev, "ARM PL18x MMC/SD controller");
	return (BUS_PROBE_DEFAULT);
}


static int
parse_fdt(struct mmci_softc *sc)
{
	phandle_t node;

	if ((node = ofw_bus_get_node(sc->dev)) == -1)
		return (ENXIO);

#if 0
	if (OF_hasprop(node, "st,sig-dir-dat0"))
		sc->pwr_reg_add |= MCI_ST_DATA0DIREN;
	if (OF_hasprop(node, "st,sig-dir-dat2"))
		sc->pwr_reg_add |= MCI_ST_DATA2DIREN;
	if (OF_hasprop(node, "st,sig-dir-dat31"))
		sc->pwr_reg_add |= MCI_ST_DATA31DIREN;
	if (OF_hasprop(node, "st,sig-dir-dat74"))
		sc->pwr_reg_add |= MCI_ST_DATA74DIREN;
	if (OF_hasprop(node, "st,sig-dir-cmd"))
		sc->pwr_reg_add |= MCI_ST_CMDDIREN;
	if (OF_hasprop(node, "st,sig-pin-fbclk"))
		sc->pwr_reg_add |= MCI_ST_FBCLKEN;
#endif
	if (OF_hasprop(node, "mmc-cap-mmc-highspeed"))
		sc->fdt_caps |= MMC_CAP_HSPEED;
	if (OF_hasprop(node, "mmc-cap-sd-highspeed"))
		sc->fdt_caps |= MMC_CAP_HSPEED;

	return 0;
}

static int
mmci_attach(device_t dev)
{
	struct mmci_softc *sc = device_get_softc(dev);
	device_t child;
	int rid, rv;
	phandle_t node;
	uint64_t freq;

	sc->dev = dev;
	sc->req = NULL;
	node = ofw_bus_get_node(dev);

	rv = parse_fdt(sc);
	if (rv != 0) {
		device_printf(dev, "Can't get FDT property: %d.\n", rv);
		return (rv);
	}
	mtx_init(&sc->mtx, "mmci", "mmc", MTX_DEF);
	
	/* Enable interface clock */
	rv = clk_get_by_ofw_name(node, "apb_pclk", &sc->pclk);
	if (rv != 0) {
		device_printf(dev, "Cannot get interface clock: %d\n", rv);
		goto fail;
	}
	rv = clk_enable(sc->pclk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable interface clock: %d\n", rv);
		goto fail;
	}

	/* Enable core clock */
	rv = clk_get_by_ofw_name(node, "mclk", &sc->mclk);
	if (rv != 0) {
		device_printf(dev, "Cannot get core clock: %d\n", rv);
		goto fail;
	}
	rv = clk_enable(sc->mclk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable core clock: %d\n", rv);
		goto fail;;
	}
	rv = clk_get_freq(sc->mclk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get clock frequency: %d\n", rv);
		goto fail;
	}
	rv = clk_set_freq(sc->mclk, 51000000, 1);
	if (rv != 0) {
		device_printf(dev, "Cannot set clock frequency: %d\n", rv);
//		goto fail;
	}
	rv = clk_get_freq(sc->mclk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get clock frequency: %d\n", rv);
		goto fail;
	}
device_printf(dev, "got clock: %lld\n", freq);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		goto fail;
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		goto fail;
	}
	sc->use_pio = 1;
	sc->pwrup_cmd = MCI_POWER_CTRL_ON;
	

	WR4(sc, MCI_MASK0, 0);
	WR4(sc, MCI_MASK1, 0);
	WR4(sc, MCI_CLEAR, 0xFFF);

	sc->clk_freq = (int)freq;
	sc->host.f_max = sc->clk_freq;
	sc->host.f_min = sc->clk_freq   / 254;
	sc->host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->host.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_HSPEED;

	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, mmci_intr, sc, &sc->intrhand)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		goto fail;
	}

	WR4(sc, MCI_MASK0, MCI_IRQENABLE);

	child = device_add_child(dev, "mmc", -1);
	if (!child) {
		device_printf(dev, "attaching MMC bus failed!\n");
		goto fail;
	}
	return (bus_generic_attach(dev));

fail:
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	return (ENXIO);	
}

static int
mmci_detach(device_t dev)
{
printf("%s: enter\n", __func__);
	return (EBUSY);
}

static device_method_t mmci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mmci_probe),
	DEVMETHOD(device_attach,	mmci_attach),
	DEVMETHOD(device_detach,	mmci_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	mmci_read_ivar),
	DEVMETHOD(bus_write_ivar,	mmci_write_ivar),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	mmci_update_ios),
	DEVMETHOD(mmcbr_request,	mmci_request),
	DEVMETHOD(mmcbr_get_ro,		mmci_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	mmci_acquire_host),
	DEVMETHOD(mmcbr_release_host,	mmci_release_host),

	DEVMETHOD_END
};
	
static devclass_t mmci_devclass;

static driver_t mmci_driver = {
	"mmci",
	mmci_methods,
	sizeof(struct mmci_softc),
};

DRIVER_MODULE(mmci, simplebus, mmci_driver, mmci_devclass, 0, 0);
