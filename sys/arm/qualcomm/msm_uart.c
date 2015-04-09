/*-
 * Copyright (c) 2014 Ganbold Tsagaankhuu <ganbold@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kdb.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/clk/clk.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>

#include <arm/qualcomm/msm_uart.h>

#include "uart_if.h"

#define	RD4(bas, reg)	\
    bus_space_read_4((bas)->bst, (bas)->bsh, (reg))
#define WR4(bas, reg, value)	\
    bus_space_write_4((bas)->bst, (bas)->bsh, (reg), (value))

struct msm_uart_softc {
	struct uart_softc 	base;
	clk_t			core_clk;
	clk_t			iface_clk;
	uint32_t		ier;
	uint32_t		readbuf;
	uint32_t		readcnt;
};

/*
 * UART class interface.
 */
static int
msm_uart_uart_param(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	int ulcon;

	ulcon = 0;

	switch (databits) {
	case 5:
		ulcon |= (UART_DM_5_BPS << 4);
		break;
	case 6:
		ulcon |= (UART_DM_6_BPS << 4);
		break;
	case 7:
		ulcon |= (UART_DM_7_BPS << 4);
		break;
	case 8:
		ulcon |= (UART_DM_8_BPS << 4);
		break;
	default:
		return (EINVAL);
	}

	switch (parity) {
	case UART_PARITY_NONE:
		ulcon |= UART_DM_NO_PARITY;
		break;
	case UART_PARITY_ODD:
		ulcon |= UART_DM_ODD_PARITY;
		break;
	case UART_PARITY_EVEN:
		ulcon |= UART_DM_EVEN_PARITY;
		break;
	case UART_PARITY_SPACE:
		ulcon |= UART_DM_SPACE_PARITY;
		break;
	case UART_PARITY_MARK:
	default:
		return (EINVAL);
	}

	switch (stopbits) {
	case 1:
		ulcon |= (UART_DM_SBL_1 << 2);
		break;
	case 2:
		ulcon |= (UART_DM_SBL_2 << 2);
		break;
	default:
		return (EINVAL);
	}
#if 0
	WR4(bas, UART_DM_MR2, ulcon);
#endif
	/* For now set 8-N-1 config: 8 data bits - No parity - 1 stop bit */
	WR4(bas, UART_DM_MR2, UART_DM_8_N_1_MODE);

	/* Set 115200 for both TX and RX. */;
	WR4(bas, UART_DM_CSR, UART_DM_CSR_115200);
	uart_barrier(bas);

	return (0);
}


static int
msm_uart_probe(struct uart_bas *bas)
{

	return (0);
}

static void
msm_uart_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
	struct msm_uart_softc *sc = (struct msm_uart_softc *)bas;
	uint32_t tmp, rxstale;

	sc->readbuf = 0;
	sc->readcnt = 0;



	/* Configure UART mode registers MR1 and MR2 */
	/* Hardware flow control isn't supported */
	WR4(bas, UART_DM_MR1, 0x0);

	/* Reset interrupt mask register. */
	WR4(bas, UART_DM_IMR, 0);

	/* Configure Tx and Rx watermarks configuration registers */
	/*
	 * TX watermark value is set to 0 - interrupt is generated when
	 * FIFO level is less than or equal to 0
	 */
	WR4(bas, UART_DM_TFWR, UART_DM_TFW_VALUE);

	/* Set RX watermark value */
	WR4(bas, UART_DM_RFWR, UART_DM_RFW_VALUE);

	rxstale = 31;
	tmp = UART_DM_IPR_STALE_TIMEOUT_LSB_MASK & rxstale;
	tmp |= UART_DM_IPR_STALE_LAST;
	tmp |= UART_DM_IPR_STALE_TIMEOUT_MSB_MASK & (rxstale << 2);
	WR4(bas, UART_DM_IPR, tmp);


	/* Configure Interrupt Programming Register */
	/* Set initial Stale timeout value */
	WR4(bas, UART_DM_IPR, UART_DM_STALE_TIMEOUT_LSB);

	/* Disable IRDA mode */
	WR4(bas, UART_DM_IRDA, 0x0);

	/* Configure and enable sim interface if required */
	/* Configure hunt character value in HCR register */
	/* Keep it in reset state */
	WR4(bas, UART_DM_HCR, 0x0);

	/* Issue soft reset command */
	WR4(bas, UART_DM_CR, UART_DM_CR_PROTECTION_EN);

	WR4(bas, UART_DM_CR, UART_DM_CR_TX_DISABLE);
	WR4(bas, UART_DM_CR, UART_DM_CR_RX_DISABLE);
	WR4(bas, UART_DM_CR, UART_DM_RESET_TX);
	WR4(bas, UART_DM_CR, UART_DM_RESET_RX);
	WR4(bas, UART_DM_CR, UART_DM_RESET_ERROR_STATUS);
	WR4(bas, UART_DM_CR, UART_DM_RESET_BREAK_INT);
	WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);

	/* Enable/Disable Rx/Tx DM interfaces */
	/* Disable Data Mover for now. */
	WR4(bas, UART_DM_DMEN, 0x0);

	/* Set default parameters */
	msm_uart_uart_param(bas, baudrate, databits, stopbits, parity);

	/* Enable transmitter and receiver */
	WR4(bas, UART_DM_CR, UART_DM_CR_RX_ENABLE);
	WR4(bas, UART_DM_CR, UART_DM_CR_TX_ENABLE);

	/* Clear stale status */
	WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);
	WR4(bas, UART_DM_DMRX, 0xFFFFFF);
	WR4(bas, UART_DM_CR, UART_DM_STALE_EVENT_ENABLE);
}

static void
msm_uart_term(struct uart_bas *bas)
{

	/* XXX */
}

static void
msm_uart_putc(struct uart_bas *bas, int c)
{

	/*
	 * BEWARE !!! TX expect UART_DM_NO_CHARS_FOR_TX chars to be proccesed,
	 * and same number of writes must be made to UART_DM_TF - address
	 * (index) have ho meaning.
	 * But UART_DM_NO_CHARS_FOR_TX can be changed only if TX FIFO is empty
	 * Moreover, the UART_DM_NO_CHARS_FOR_TX must be set before each write
	 * burst.
	 */
	while ((RD4(bas, UART_DM_SR) & UART_DM_SR_TXEMT) == 0)
		DELAY(4);
	WR4(bas, UART_DM_NO_CHARS_FOR_TX, 1);
	WR4(bas, UART_DM_TF(0), (c & 0xff));
}

static int
msm_uart_rxready(struct uart_bas *bas)
{
	struct msm_uart_softc *sc = (struct msm_uart_softc *)bas;

	return ((sc->readcnt > 0) ||
	    (RD4(bas, UART_DM_SR) & UART_DM_SR_RXRDY) ||
	    (UART_DM_RXFS_BUF_STATE(RD4(bas, UART_DM_RXFS)) > 0));
}

static int
msm_uart_getc(struct uart_bas *bas, struct mtx *mtx)
{
	int c;
	struct msm_uart_softc *sc = (struct msm_uart_softc *)bas;

	uart_lock(mtx);

	/* Get buffered character */
	if (sc->readcnt > 0) {
		c = (sc->readbuf & 0xFF );
		sc->readbuf >>= 8;
		sc->readcnt--;

		uart_unlock(mtx);
		return (c);
	}

	/* Whole word is in FIFO */
	if (RD4(bas, UART_DM_SR) & UART_DM_SR_RXRDY) {
		/* Read buffer */
		sc->readbuf = RD4(bas, UART_DM_RF(0));
		c = (sc->readbuf & 0xFF);
		sc->readbuf >>= 8;
		sc->readcnt--;

		uart_unlock(mtx);
		return (c);
		sc->readbuf = RD4(bas, UART_DM_RF(0));
		c = (sc->readbuf & 0xFF );
		sc->readcnt = 4;
		sc->readbuf >>= 8;
		sc->readcnt--;

		uart_unlock(mtx);
		return (c);
	}
	/* Check packing buffer last */
	do {
		sc->readcnt = UART_DM_RXFS_BUF_STATE(RD4(bas, UART_DM_RXFS));
	} while (sc->readcnt == 0);

	/* Force stale event */
	WR4(bas, UART_DM_CR, UART_DM_FORCE_STALE_EVENT);
	/* Read buffer */
	sc->readbuf = RD4(bas, UART_DM_RF(0));
	c = (sc->readbuf & 0xFF);
	sc->readbuf >>= 8;
	sc->readcnt--;
	WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);
	WR4(bas, UART_DM_DMRX, 0xFFFFFF);
	WR4(bas, UART_DM_CR, UART_DM_STALE_EVENT_ENABLE);

	uart_unlock(mtx);
	return (c);
};

struct uart_ops uart_msm_uart_ops = {
	.probe = msm_uart_probe,
	.init = msm_uart_init,
	.term = msm_uart_term,
	.putc = msm_uart_putc,
	.rxready = msm_uart_rxready,
	.getc = msm_uart_getc,
};

/*
 * UART bus interface.
 */
static int
msm_uart_bus_probe(struct uart_softc *sc)
{

	sc->sc_txfifosz = 64;
	sc->sc_rxfifosz = 64;

	device_set_desc(sc->sc_dev, "Qualcomm HSUART");

	return (0);
}

static int
msm_uart_bus_attach(struct uart_softc *sc)
{
	struct msm_uart_softc *u = (struct msm_uart_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;

	sc->sc_hwiflow = 0;
	sc->sc_hwoflow = 0;

	/* Set TX_READY, TXLEV, RXLEV, RXSTALE */
	u->ier = UART_DM_IMR_ENABLED;

	/* Configure Interrupt Mask register IMR */
	WR4(bas, UART_DM_IMR, u->ier);
	return (0);
}

/*
 * Write the current transmit buffer to the TX FIFO.
 */
static int
msm_uart_bus_transmit(struct uart_softc *sc)
{
	struct msm_uart_softc *u = (struct msm_uart_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;
	int i;

	uart_lock(sc->sc_hwmtx);

	/* Write some data */
	for (i = 0; i < sc->sc_txdatasz; i++) {
		/* Write TX data */
		msm_uart_putc(bas, sc->sc_txbuf[i]);
		uart_barrier(bas);
	}

	/* TX FIFO is empty now, enable TX_READY interrupt */
	u->ier |= UART_DM_TX_READY;
	WR4(bas, UART_DM_IMR, u->ier);
	uart_barrier(bas);

	/*
	 * Inform upper layer that it is transmitting data to hardware,
	 * this will be cleared when TXIDLE interrupt occurs.
	 */
	sc->sc_txbusy = 1;
	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
msm_uart_bus_setsig(struct uart_softc *sc, int sig)
{

	return (0);
}

/* XXX This not works  now.... */
static int
msm_uart_bus_receive(struct uart_softc *sc)
{
	struct msm_uart_softc *u = (struct msm_uart_softc *)sc;
	struct uart_bas *bas;
	int c;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);

	/* Initialize Receive Path and interrupt */
	WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);
	WR4(bas, UART_DM_CR, UART_DM_STALE_EVENT_ENABLE);
	u->ier |= UART_DM_RXLEV;
	WR4(bas, UART_DM_IMR, u->ier);

	/* Loop over until we are full, or no data is available */
	while (RD4(bas, UART_DM_SR) & UART_DM_SR_RXRDY) {
		if (uart_rx_full(sc)) {
			/* No space left in input buffer */
			sc->sc_rxbuf[sc->sc_rxput] = UART_STAT_OVERRUN;
			break;
		}

		/* Read RX FIFO */
		c = RD4(bas, UART_DM_RF(0));
		uart_rx_put(sc, c);
	}

	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
msm_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	int error;

	uart_lock(sc->sc_hwmtx);
	error = msm_uart_uart_param(&sc->sc_bas, baudrate, databits, stopbits,
	    parity);
	uart_unlock(sc->sc_hwmtx);

	return (error);
}

static int
msm_uart_bus_ipend(struct uart_softc *sc)
{
	struct msm_uart_softc *u = (struct msm_uart_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t isr;
	int ipend;

	uart_lock(sc->sc_hwmtx);

	/* Get ISR status */
	isr = RD4(bas, UART_DM_MISR);

	ipend = 0;

	/* Uart RX starting, notify upper layer */
	if (isr & UART_DM_RXLEV) {
		u->ier &= ~UART_DM_RXLEV;
		WR4(bas, UART_DM_IMR, u->ier);
		uart_barrier(bas);
		ipend |= SER_INT_RXREADY;
	}

	/* Stale RX interrupt */
	if (isr & UART_DM_RXSTALE) {
		/* Disable and reset it */
		WR4(bas, UART_DM_CR, UART_DM_STALE_EVENT_DISABLE);
		WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);
		uart_barrier(bas);
		ipend |= SER_INT_RXREADY;
	}

	/* TX READY interrupt */
	if (isr & UART_DM_TX_READY) {
		/* Clear  TX Ready */
		WR4(bas, UART_DM_CR, UART_DM_CLEAR_TX_READY);

		/* Disable TX_READY */
		u->ier &= ~UART_DM_TX_READY;
		WR4(bas, UART_DM_IMR, u->ier);
		uart_barrier(bas);

		if (sc->sc_txbusy != 0)
			ipend |= SER_INT_TXIDLE;
	}

	if (isr & UART_DM_TXLEV) {
		/* TX FIFO is empty */
		u->ier &= ~UART_DM_TXLEV;
		WR4(bas, UART_DM_IMR, u->ier);
		uart_barrier(bas);

		if (sc->sc_txbusy != 0)
			ipend |= SER_INT_TXIDLE;
	}

	uart_unlock(sc->sc_hwmtx);
	return (ipend);
}

static int
msm_uart_bus_flush(struct uart_softc *sc, int what)
{

	return (0);
}

static int
msm_uart_bus_getsig(struct uart_softc *sc)
{

	return (0);
}

static int
msm_uart_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{

	return (EINVAL);
}

static void
msm_uart_bus_grab(struct uart_softc *sc)
{
	struct uart_bas *bas = &sc->sc_bas;

	/* XXX: fix needed */
	/*
	 * Turn off all interrupts to enter polling mode. Leave the
	 * saved mask alone. We'll restore whatever it was in ungrab.
	 */
	uart_lock(sc->sc_hwmtx);
	WR4(bas, UART_DM_CR, UART_DM_RESET_STALE_INT);
	WR4(bas, UART_DM_IMR, 0);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

static void
msm_uart_bus_ungrab(struct uart_softc *sc)
{
	struct msm_uart_softc *u = (struct msm_uart_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;

	/*
	 * Restore previous interrupt mask
	 */
	uart_lock(sc->sc_hwmtx);
	WR4(bas, UART_DM_IMR, u->ier);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

static kobj_method_t msm_uart_methods[] = {
	KOBJMETHOD(uart_probe,		msm_uart_bus_probe),
	KOBJMETHOD(uart_attach, 	msm_uart_bus_attach),
	KOBJMETHOD(uart_flush,		msm_uart_bus_flush),
	KOBJMETHOD(uart_getsig,		msm_uart_bus_getsig),
	KOBJMETHOD(uart_ioctl,		msm_uart_bus_ioctl),
	KOBJMETHOD(uart_ipend,		msm_uart_bus_ipend),
	KOBJMETHOD(uart_param,		msm_uart_bus_param),
	KOBJMETHOD(uart_receive,	msm_uart_bus_receive),
	KOBJMETHOD(uart_setsig,		msm_uart_bus_setsig),
	KOBJMETHOD(uart_transmit,	msm_uart_bus_transmit),
	KOBJMETHOD(uart_grab,		msm_uart_bus_grab),
	KOBJMETHOD(uart_ungrab,		msm_uart_bus_ungrab),
	{0, 0 }
};

struct uart_class uart_msm_uart_class = {
	"msm_uart",
	msm_uart_methods,
	sizeof(struct msm_uart_softc),
	.uc_ops = &uart_msm_uart_ops,
	.uc_range = 8,
	.uc_rclk = 0,
};

static struct ofw_compat_data compat_data[] = {
	{"qcom,uart-dm",		(uintptr_t)&uart_msm_uart_class},
	{"qcom,msm-uartdm",		(uintptr_t)&uart_msm_uart_class},
	{"qcom,msm-uartdm-v1.3",	(uintptr_t)&uart_msm_uart_class},
	{NULL,				(uintptr_t)NULL},
};
UART_FDT_CLASS(compat_data);

/*
 * Device interface.
 */
static int
msm_uart_dev_detach(device_t dev)
{
	struct msm_uart_softc *sc;

	sc = device_get_softc(dev);
	if (sc->core_clk != NULL)
		clk_release(sc->core_clk);
	if (sc->iface_clk != NULL)
		clk_release(sc->iface_clk);
	return (uart_bus_detach(dev));
}

static int
msm_uart_dev_probe(device_t dev)
{
	struct msm_uart_softc *sc;
	const struct ofw_compat_data *cd;

	sc = device_get_softc(dev);
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	cd = ofw_bus_search_compatible(dev, compat_data);
	if (cd->ocd_data == 0)
		return (ENXIO);
	sc->base.sc_class = (struct uart_class *)cd->ocd_data;
	return (uart_bus_probe(dev, 0, 0, 0, 0));

}

static int
msm_uart_dev_attach(device_t dev)
{
	struct msm_uart_softc *sc;
	phandle_t node;
	uint64_t freq;
	int rv;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	/* Enable interface clock */
	rv = clk_get_by_ofw_name(node, "iface", &sc->iface_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get iface clock: %d\n", rv);
		goto fail;
	}
	rv = clk_enable(sc->iface_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable iface clock: %d\n", rv);
		goto fail;
	}

	/* Enable core clock */
	rv = clk_get_by_ofw_name(node, "core", &sc->core_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get core clock: %d\n", rv);
		goto fail;
	}
	rv = clk_enable(sc->core_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable core clock: %d\n", rv);
		goto fail;;
	}
	rv = clk_get_freq(sc->core_clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get clock frequency: %d\n", rv);
		goto fail;
	}
device_printf(dev, "got UART clock: %lld\n", freq);

	return (uart_bus_attach(dev));

fail:
	if (sc->core_clk != NULL)
		clk_release(sc->core_clk);
	if (sc->iface_clk != NULL)
		clk_release(sc->iface_clk);
	return (ENXIO);
}

static device_method_t msm_uart_dev_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		msm_uart_dev_probe),
	DEVMETHOD(device_attach,	msm_uart_dev_attach),
	DEVMETHOD(device_detach,	msm_uart_dev_detach),
	{ 0, 0 }
};

static driver_t msm_uart_driver = {
	uart_driver_name,
	msm_uart_dev_methods,
	sizeof(struct msm_uart_softc),
};

DRIVER_MODULE(msm_uart, simplebus,  msm_uart_driver, uart_devclass,
    0, 0);

