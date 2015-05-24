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
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/fdt_regulator.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include <arm/qualcomm/mmci.h>

#define	LOCK(_sc)	mtx_lock(&_sc->mtx);
#define	UNLOCK(_sc)	mtx_unlock(&_sc->mtx);

#define	RD4(_sc, _r)	bus_read_4(_sc->mem_res, _r)
#define	WR4(_sc, _r, _v) bus_write_4(_sc->mem_res, _r, _v)

#define	PENDING_CMD	0x01
#define	PENDING_STOP	0x02
#define	CARD_INIT_DONE	0x04


#define	DEBUG	1
#ifdef DEBUG
static int dbg_lvl = 0;
#define debugf(d, fmt, args...) do { 					\
  if (dbg_lvl >= (d)) printf("%s(): " # fmt, __func__, ##args); 	\
} while (0)
#else
#define debugf(d, fmt, args...)
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
	regulator_t		supply_vmmc;
	struct gpiobus_pin	*gpio_cd;
	int			use_pio;
	uint32_t		max_freq;
	int			not_removable;

	struct mmc_host		host;
	struct mmc_request 	*req;
	struct mmc_command	*cmd;
	int			flags;
	int			bus_busy;

	/* Flags for handling R1b type respose */
	int			prog_enabled;
	int			got_progdone;	/* PROG_DONE received */
	int			got_respend;	/* CMD_RESP_END received */

	/* Flags for handling data transfer done */
	int			got_data_done;	/* data transfer finished */
	int			got_dataend;	/* DATA_DONE received */
};

/*
 * Wait until cwrite to register passes multiple clock domains. Worst fly
 * time is 3 MCLK and 3 HCLK
 */
static void
mmci_reg_wait(struct mmci_softc *sc)
{
	while((RD4(sc, MCI_STATUS2) & MCI_STATUS2_MCLK_REG_WR_ACTIVE) != 0)
		;
}

/*
 * PIO transfers
 */
static void
mmci_pio_read(struct mmci_softc *sc, struct mmc_command *cmd)
{
	struct mmc_data *data;
	uint32_t status;
	uintptr_t p;
	int len;

	data = cmd->data;
	p = (uintptr_t)data->data + data->xfer_len;
	KASSERT((data->flags & MMC_DATA_READ) != 0,
	    ("read on not read requrest"));
	KASSERT((data->xfer_len & 3) == 0, ("xfer_len not aligned"));

	while (data->xfer_len < data->len) {
		status = RD4(sc, MCI_STATUS);
		if (status & MCI_SCM_RXFIFO_FULL) {
			len = min(data->len - data->xfer_len,
			    MCI_FIFO_SIZE);
			bus_read_multi_4(sc->mem_res, MCI_FIFO, (uint32_t *)p,
			    len / 4);
			p += len;
			data->xfer_len += len;
		} else if (status & MCI_SCM_RXFIFO_HALF_FULL) {
			len = min(data->len - data->xfer_len,
			    MCI_FIFO_SIZE /2);
			bus_read_multi_4(sc->mem_res, MCI_FIFO, (uint32_t *)p,
			    len / 4);
			p += len;
			data->xfer_len += len;
		} else if (status & MCI_SCM_RXDATA_AVLBL) {
			*(uint32_t *)p = RD4(sc, MCI_FIFO);
			p += 4;
			data->xfer_len += 4;
		} else {
			break;
		}
	}

	if (data->xfer_len >= data->len)
		WR4(sc, MCI_MASK0, RD4(sc, MCI_MASK0) & ~MCI_IRQ_FIFO);
	else if ((data->len - data->xfer_len) < 16)
		WR4(sc, MCI_MASK0, (RD4(sc, MCI_MASK0) | MCI_SCM_RXDATA_AVLBL));
}

static void
mmci_pio_write(struct mmci_softc *sc, struct mmc_command *cmd)
{
	struct mmc_data *data;
	uint32_t status;
	uintptr_t p;
	int len, maxlen;

	data = cmd->data;
	p = (uintptr_t)data->data + data->xfer_len;
	KASSERT((data->flags & MMC_DATA_WRITE) != 0,
	    ("write on not write requrest"));
	KASSERT((data->xfer_len & 3) == 0, ("xfer_len not aligned"));

	while (data->xfer_len < data->len) {
		status = RD4(sc, MCI_STATUS);

		if (status & MCI_SCM_TXFIFO_EMPTY)
			maxlen = MCI_FIFO_SIZE;
		else if (status & MCI_SCM_TXFIFO_HALF_EMPTY)
			maxlen = MCI_FIFO_SIZE / 2;
		else
			break;

		len = min(data->len - data->xfer_len, maxlen);
		bus_write_multi_4(sc->mem_res, MCI_FIFO, (uint32_t *)p,
			    len / 4);
		p += len;
		data->xfer_len += len;
	}

	if (data->xfer_len >= data->len)
		WR4(sc, MCI_MASK0, RD4(sc, MCI_MASK0) & ~MCI_IRQ_FIFO);
}

/* Handle single MMC command */
static void
mmci_start_cmd(struct mmci_softc *sc, struct mmc_command *cmd)
{
	struct mmc_data *data;
	uint32_t blksz;
	uint32_t cmdreg;
	uint32_t datactrl, mask;

	sc->prog_enabled =0;
	sc->got_progdone = 0;
	sc->got_respend = 0;
	sc->got_data_done = 0;
	sc->got_dataend = 0;

	sc->cmd = cmd;
	mask = MCI_IRQENABLE;
	cmdreg = cmd->opcode | MCI_CMD_ENABLE;
	data = cmd->data;
	datactrl = 0;

	if (((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) &&
	    ((cmd->data == NULL) || (cmd->data->data == NULL)))
		panic("Data transfer requested without buffer");

	/* Stop controller */
	WR4(sc, MCI_CMD, 0);
	mmci_reg_wait(sc);
	WR4(sc, MCI_DATA_CTL, 0);
	mmci_reg_wait(sc);
	WR4(sc, MCI_CLEAR, 0xFFFFFFFF);

	/* Build cmdreg value */
	if (MMC_RSP(cmd->flags) != MMC_RSP_NONE) {
		cmdreg |= MCI_CMD_RESPONSE;
		if (cmd->flags & MMC_RSP_136)
			cmdreg |= MCI_CMD_LONGRSP;
	}
	if (cmd->flags & MMC_CMD_ADTC)
		cmdreg |= MCI_CMD_QCOM_DAT_CMD;
	if (MMC_RSP(cmd->flags) == MMC_RSP_R1B)
	{
		sc->prog_enabled = 1;
		cmdreg |= MCI_CMD_QCOM_PROG_ENA;
		mask |= MCI_SCM_PROG_DONE;
	}

	/* Setup data registers and interrupt mask */
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
		datactrl = MCI_DATA_CTL_ENABLE;

		if (data->flags & MMC_DATA_READ)
			datactrl |= MCI_DATA_CTL_READ;
		else /* For write, start DPSM after command is send */
			datactrl |= MCI_DATA_CTL_QCOM_DATA_PEND;

		if (data->flags & MMC_DATA_STREAM)
			datactrl |= MCI_DATA_CTL_MODE;

		blksz = (data->len < MMC_SECTOR_SIZE) ? \
			 data->len : MMC_SECTOR_SIZE;
		datactrl |= blksz << 4;

		if (data->flags & MMC_DATA_READ) {
			mask |= MCI_SCM_RXFIFO_HALF_FULL;
			if (data->len < 16)
				mask |= MCI_SCM_RXDATA_AVLBL;
		} else {
			mask |= MCI_SCM_TXFIFO_HALF_EMPTY;
		}

		mask |= MCI_SCM_DATAEND;

		/* XXXX FIXME Compute valid value ? */
		WR4(sc, MCI_DATATIMER, 0xFFFF0000);
		WR4(sc, MCI_DATALENGTH, data->len);
		WR4(sc, MCI_DATA_CTL, datactrl);
		mmci_reg_wait(sc);
	} else   {
		mask |= MCI_SCM_CMD_SENT | MCI_SCM_CMD_RESP_END;
	}

	debugf(2, "cmd: %d, flag: 0x%08X,  arg: 0x%08x, cmdreg: 0x%08x,"
	    "datactrl: 0x%08x, mask: 0x%08x, data: %p\n", cmd->opcode,
	    cmd->flags, cmd->arg, cmdreg, datactrl, mask, data);

	/* Fire up command */
	WR4(sc, MCI_ARGUMENT, cmd->arg);
	WR4(sc, MCI_CMD, cmdreg);
	WR4(sc, MCI_MASK0, (RD4(sc, MCI_MASK0) | mask));
	mmci_reg_wait(sc);
}

/* Handle single MMC request */
static void
mmci_next_cmd(struct mmci_softc *sc)
{
	struct mmc_request *req;

	WR4(sc, MCI_MASK0, MCI_IRQENABLE);
	WR4(sc, MCI_STATUS, MCI_IRQENABLE);
	WR4(sc, MCI_CMD, 0);
	mmci_reg_wait(sc);

	sc->cmd = NULL;
	req = sc->req;
	if (req == NULL)
		return;

	if (sc->flags & PENDING_CMD) {
		sc->flags &= ~PENDING_CMD;
		mmci_start_cmd(sc, req->cmd);
		return;
	}
	if (sc->flags & PENDING_STOP) {
		sc->flags &= ~PENDING_STOP;
		mmci_start_cmd(sc, req->stop);
		return;
	}

	/* Request finished */
	sc->req = NULL;
	req->done(req);
}

static int
mmci_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct mmci_softc *sc = device_get_softc(bus);

	LOCK(sc);
	if (sc->req) {
		UNLOCK(sc);
		return (EBUSY);
	}
	sc->req = req;
	sc->flags |= PENDING_CMD;
	if (sc->req->stop)
		sc->flags |= PENDING_STOP;

	mmci_next_cmd(sc);

	UNLOCK(sc);

	return (0);
}

static void
mmci_intr(void *arg)
{
	struct mmci_softc *sc = (struct mmci_softc *)arg;
	struct mmc_command *cmd;
	uint32_t status, raw_status, mask;

	cmd = sc->cmd;

	raw_status = RD4(sc, MCI_STATUS);
	mask = RD4(sc, MCI_MASK0);
	status = raw_status & mask;
	WR4(sc, MCI_CLEAR, status);

	debugf(3, "interrupt: 0x%08x raw: 0x%08x (0x%08x)\n", status,
	    raw_status, raw_status & (MCI_SCM_CMD_ACTIVE | MCI_SCM_TXACTIVE |
	    MCI_SCM_RXACTIVE));

	if (sc->req == NULL) {
		device_printf(sc->dev, "Stray interrupt: 0x%08x\n", status);
		WR4(sc, MCI_CLEAR, status);
		return;
	}

	/* Handle PIO mode data transfers */
	if (sc->use_pio && (cmd->data != NULL)) {
		if (MCI_HAVE_RX_DATA(raw_status)) {
			mmci_pio_read(sc, cmd);
		} else if (MCI_HAVE_TX_DATA(raw_status)) {
			mmci_pio_write(sc, cmd);
		}
	}

	/* Remember flags for multienvent actions */
	if (status & MCI_SCM_CMD_RESP_END)
		sc->got_respend = 1;
	if (status & MCI_SCM_PROG_DONE)
		sc->got_progdone = 1;

	if (sc->got_respend && (!sc->prog_enabled || sc->got_progdone)) {
		/* Command with response finished */
		debugf(2, "command response: 0x%08X\n", RD4(sc, MCI_RESP0));

		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = RD4(sc, MCI_RESP3);
			cmd->resp[2] = RD4(sc, MCI_RESP2);
			cmd->resp[1] = RD4(sc, MCI_RESP1);
		}
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		cmd->error = MMC_ERR_NONE;
		mmci_next_cmd(sc);
	} else if (status & MCI_SCM_CMD_SENT) {
		/* Command without response finished */
		debugf(2, "command sent: 0x%08X\n", RD4(sc, MCI_RESP0));
		cmd->error = MMC_ERR_NONE;
		mmci_next_cmd(sc);
	} else if (status & MCI_SCM_DATAEND) {
		/* Data transfer finished */
		debugf(3, "data end\n");
		cmd->error = MMC_ERR_NONE;
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_next_cmd(sc);
	}  else if (status & MCI_SCM_CMD_CRC_FAIL) {
		/* Command finished with error */
		debugf(1, "command crc fail: 0x%08X\n", RD4(sc, MCI_RESP0));
		cmd->error = cmd->flags & MMC_RSP_CRC
		    ? MMC_ERR_BADCRC : MMC_ERR_NONE;
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_next_cmd(sc);

	} else if (status & MCI_SCM_CMD_TIMEOUT) {
		/* Command finished with timeout */
		debugf(1, "command timeout: 0x%08X\n", RD4(sc, MCI_RESP0));
		cmd->error = MMC_ERR_TIMEOUT;
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_next_cmd(sc);
	} else if (status & (MCI_SCM_DATA_CRC_FAIL | MCI_SCM_DATA_TIMEOUT |
	     MCI_SCM_START_BIT_ERR | MCI_SCM_TX_UNDERRUN |
	     MCI_SCM_RX_OVERRUN)) {
		/* Data error */
		debugf(1, "Data error: 0x%08X\n", status);
		if (status & MCI_SCM_DATA_TIMEOUT) {
			cmd->error = MMC_ERR_TIMEOUT;
		} else if (status & MCI_SCM_DATA_CRC_FAIL) {
			cmd->error = MMC_ERR_BADCRC;
		} else if (status & (MCI_SCM_TX_UNDERRUN | MCI_SCM_RX_OVERRUN)) {
			cmd->error = MMC_ERR_FIFO;
		} else {
			cmd->error = MMC_ERR_FAILED;
		}
		cmd->resp[0] = RD4(sc, MCI_RESP0);
		mmci_next_cmd(sc);
	}

	WR4(sc, MCI_CLEAR, status);
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
		*(int *)result = 1024;
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

// https://searchcode.com/codesearch/view/41176635/
static int
mmci_update_ios(device_t bus, device_t child)
{
	struct mmci_softc *sc = device_get_softc(bus);
	struct mmc_ios *ios = &sc->host.ios;
	uint32_t clkreg, pwr;
	int rv;
	uint64_t freq;

	clkreg = 0;
	if (ios->bus_width == bus_width_8) {
		debugf(1, "using 8 bits wide bus mode\n");
		clkreg |= MCI_CLOCK_QCOM_BUS_8;
	} else if (ios->bus_width == bus_width_4) {
		debugf(1, "using 4 bits wide bus mode\n");
		clkreg |= MCI_CLOCK_QCOM_BUS_4;
	} else {
		clkreg |= MCI_CLOCK_QCOM_BUS_1;
	}

	clkreg |= MCI_CLOCK_QCOM_FLOW_ENA;

	clkreg |= MCI_CLOCK_QCOM_SELECT_FBCLK;
/*
	if (ios->timing == MMC_TIMING_UHS_SDR104) {
		clk |= MCI_CLOCK_QCOM_SELECT_UHS;
		host->tuning_needed = 1;
	} else if (ios->timing == MMC_TIMING_UHS_DDR50) {
		clk |= MCI_CLOCK_QCOM_SELECT_DDR;
	} else {
		clk |= MCI_CLOCK_QCOM_SELECT_FBCLK;
	}
*/
	/* Select free running MCLK as input clock of cm_dll_sdc4 */
	clkreg |= MCI_CLOCK_QCOM_MCLK_SEL_FREE;

	if (ios->clock != 0) {
		rv = clk_set_freq(sc->mclk, ios->clock, 1);
		if (rv != 0) {
			device_printf(sc->dev,
			    "Cannot set clock frequency: %d\n", rv);
			return (rv);
		}
		rv = clk_get_freq(sc->mclk, &freq);
		if (rv != 0) {
			device_printf(sc->dev,
			    "Cannot get clock frequency: %d\n", rv);
			return (rv);
		}
		ios->clock = freq;
		clkreg |= MCI_CLOCK_ENABLE;
	}

	debugf(3, "clock: %dHz,  bus width: %d, clkreg: 0x%08X\n",
	     ios->clock, ios->bus_width, clkreg);

	WR4(sc, MCI_CLOCK, clkreg);
	mmci_reg_wait(sc);

	pwr = RD4(sc, MCI_POWER);
	pwr &= ~MCI_POWER_CTRL_MASK;

	switch (ios->power_mode) {
	case power_off:
		pwr |= MCI_POWER_CTRL_OFF;
		break;
	case power_up:
		pwr |= MCI_POWER_CTRL_UP;
		break;
	case power_on:
		pwr |= MCI_POWER_CTRL_ON;
		break;
	}

	if (ios->bus_mode == opendrain)
		pwr |= MCI_POWER_OPENDRAIN;
	else
		pwr &= ~MCI_POWER_OPENDRAIN;

	WR4(sc, MCI_POWER, pwr);
	mmci_reg_wait(sc);

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

	LOCK(sc);
	while (sc->bus_busy)
		error = mtx_sleep(sc, &sc->mtx, PZERO, "mmci", 0);

	sc->bus_busy++;
	UNLOCK(sc);
	return (error);
}

static int
mmci_release_host(device_t bus, device_t child)
{
	struct mmci_softc *sc = device_get_softc(bus);

	LOCK(sc);
	sc->bus_busy--;
	wakeup(sc);
	UNLOCK(sc);
	return (0);
}

static int
mmci_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "arm,pl18x"))
		return (ENXIO);

	device_set_desc(dev, "ARM PL18x MMC/SD/SDIO controller");
	return (BUS_PROBE_DEFAULT);
}


static int
parse_fdt(struct mmci_softc *sc)
{
	phandle_t node;
	int bus_width;
	int err;

	if ((node = ofw_bus_get_node(sc->dev)) == -1)
		return (ENXIO);
	if (OF_hasprop(node, "mmc-cap-mmc-highspeed"))
		sc->host.caps |= MMC_CAP_HSPEED;
	if (OF_hasprop(node, "mmc-cap-sd-highspeed"))
		sc->host.caps |= MMC_CAP_HSPEED;
	if (OF_hasprop(node, "non-removable"))
		sc->not_removable = 1;
#if 0
	if (OF_hasprop(node, "no-1-8-v"))
		sc->host.host_ocr &= ~();
#endif
	if (OF_getencprop(node, "max-frequency", &sc->max_freq,
	    sizeof(sc->max_freq)) <= 0)
		sc->max_freq = 0;

	if (OF_getencprop(node, "bus-width", &bus_width,
	    sizeof(bus_width)) <= 0)
	    bus_width = 1;
	sc->host.caps &= ~(MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA);
	switch (bus_width) {
	case 8:
		sc->host.caps |= MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA |
		    MMC_CAP_FORCE_8_BIT_DATA;
		break;
	case 4:
		sc->host.caps |= MMC_CAP_4_BIT_DATA;
		break;
	}
	sc->supply_vmmc = fdt_regulator_get_by_name(sc->dev, "vmcc-supply");
	err = ofw_gpiobus_parse_gpios(sc->dev, "cd-gpios", &sc->gpio_cd);
	if (err != 1)
		sc->gpio_cd = NULL;
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

	sc->host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->host.caps = MMC_CAP_HSPEED | MMC_CAP_4_BIT_DATA;
	rv = parse_fdt(sc);
	if (rv != 0) {
		device_printf(dev, "Can't get FDT property: %d\n", rv);
		return (rv);
	}
	rv = fdt_pinctrl_configure_by_name(dev, "default");
	if ((rv != 0) && (rv != ENOENT)) {
		device_printf(dev, "Can't configure FDT pinctrl: %d\n", rv);
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

//	rv = clk_set_freq(sc->mclk, 51000000, 1);
	rv = clk_set_freq(sc->mclk,   400000, 1);
	if (rv != 0) {
		device_printf(dev, "Cannot set clock frequency: %d\n", rv);
		goto fail;
	}

	rv = clk_enable(sc->mclk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable core clock: %d\n", rv);
		goto fail;
	}

	rv = clk_get_freq(sc->mclk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get clock frequency: %d\n", rv);
		goto fail;
	}
device_printf(sc->dev, "Got frequency: %lld\n",  freq);
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

	WR4(sc, MCI_MASK0, 0);
	WR4(sc, MCI_MASK1, 0);
	WR4(sc, MCI_CLEAR, 0xFFF);

	sc->clk_freq = (int)freq;
	sc->host.f_max = sc->max_freq == 0 ? sc->clk_freq : sc->max_freq;
	sc->host.f_min = sc->host.f_max / 254;


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
