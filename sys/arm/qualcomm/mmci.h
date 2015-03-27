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
 *
 * $FreeBSD$
 */

#ifndef _MMCI_H_
#define	_MMCI_H_

#define	PL18XCC_CRCI_SDC1			6
#define	PL18XCC_CRCI_SDC2			7
#define	PL18XCC_CRCI_SDC3			12
#define	PL18XCC_CRCI_SDC4			13

#define	MCI_CLK				400000 /* 400KHz */

#define	MCI_POWER			0x00
#define	 MCI_POWER_OPENDRAIN			(1 << 6)
#define	 MCI_POWER_CTRL_OFF			0x00
#define	 MCI_POWER_CTRL_UP			0x02
#define	 MCI_POWER_CTRL_ON			0x03

#define	MCI_CLOCK			0x04

/* Qualcomm specific */
#define	 MCI_CLOCK_QCOM_SELECT_FBCLK		(1 << 15)
#define	 MCI_CLOCK_QCOM_SELECT_DDR		((1 << 14) | (1 << 15))
#define	 MCI_CLOCK_QCOM_INVERT_OUT		(1 << 13)
#define	 MCI_CLOCK_QCOM_FLOW_ENA		(1 << 12)
#define	 MCI_CLOCK_QCOM_BUS_8			((1 << 10) | (1 << 11))
#define	 MCI_CLOCK_QCOM_BUS_4			(1 << 11)
#define	 MCI_CLOCK_QCOM_BUS_1			(0)

#define	 MCI_CLOCK_WIDEBUS			(1 << 11)
#define	 MCI_CLOCK_BYPASS			(1 << 10)
#define	 MCI_CLOCK_PWRSAVE			(1 << 9)
#define	 MCI_CLOCK_ENABLE			(1 << 8)
#define	 MCI_CLOCK_CLKDIVMASK			0xff

#define	MCI_ARGUMENT			0x08

#define	MCI_COMMAND			0x0c
#define	 MCI_COMMAND_QCOM_AUTO_CMD19		(1 << 16)
#define	 MCI_COMMAND_QCOM_CCS_DISABLE		(1 << 15)
#define	 MCI_COMMAND_QCOM_CCS_ENABLE		(1 << 14)
#define	 MCI_COMMAND_QCOM_MCIABORT		(1 << 13)
#define	 MCI_COMMAND_QCOM_DAT_CMD		(1 << 12)
#define	 MCI_COMMAND_QCOM_PROG_ENA		(1 << 11)

#define	 MCI_COMMAND_ENABLE			(1 << 10)
#define	 MCI_COMMAND_PENDING			(1 << 9)
#define	 MCI_COMMAND_INTERRUPT		(1 << 8)
#define	 MCI_COMMAND_LONGRSP			(1 << 7)
#define	 MCI_COMMAND_RESPONSE			(1 << 6)
#define	 MCI_COMMAND_CMD_INDEX_MASK		0x3f

#define	MCI_RESP_CMD			0x10
#define	MCI_RESP0			0x14
#define	MCI_RESP1			0x18
#define	MCI_RESP2			0x1c
#define	MCI_RESP3			0x20
#define	MCI_DATATIMER			0x24
#define	MCI_DATALENGTH			0x28

#define	MCI_DATA_CTL			0x2c
#define	 MCI_DATA_CTL_MODE			(1 << 2)
#define	 MCI_DATA_CTL_BLOCKSIZE_SHIFT		4
#define	 MCI_DATA_CTL_BLOCKSIZE_MASK		0xf
#define	 MCI_DATA_CTL_DM_ENABLE			(1 << 3)
#define	 MCI_DATA_CTL_MODE			(1 << 2)
#define	 MCI_DATA_CTL_WRITE			(0 << 1)
#define	 MCI_DATA_CTL_READ			(1 << 1)
#define	 MCI_DATA_CTL_ENABLE			(1 << 0)

#define	MCI_DATACNT			0x30

#define	MCI_STATUS			0x34
#define	MCI_CLEAR			0x38
#define	MCI_MASK0			0x3c
#define	MCI_MASK1			0x40
#define	 MCI_SCM_CCSTIMEOUT			(1 << 26)
#define	 MCI_SCM_SDIOINTOPER			(1 << 25)
#define	 MCI_SCM_ATACMDCOMPL			(1 << 24)
#define	 MCI_SCM_PROGDONE			(1 << 23)
#define	 MCI_SCM_SDIOINTR			(1 << 22)
#define	 MCI_SCM_RXDATAAVLBL			(1 << 21)
#define	 MCI_SCM_TXDATAAVLBL			(1 << 20)
#define	 MCI_SCM_RXFIFOEMPTY			(1 << 19)
#define	 MCI_SCM_TXFIFOEMPTY			(1 << 18)
#define	 MCI_SCM_RXFIFOFULL			(1 << 17)
#define	 MCI_SCM_TXFIFOFULL			(1 << 16)
#define	 MCI_SCM_RXFIFOHALFFULL			(1 << 15)
#define	 MCI_SCM_TXFIFOHALFEMPTY		(1 << 14)
#define	 MCI_SCM_RXACTIVE			(1 << 13)
#define	 MCI_SCM_TXACTIVE			(1 << 12)
#define	 MCI_SCM_CMDACTIVE			(1 << 11)
#define	 MCI_SCM_DATABLOCKEND			(1 << 10)
#define	 MCI_SCM_STARTBITERR			(1 << 9)
#define	 MCI_SCM_DATAEND			(1 << 8)
#define	 MCI_SCM_CMDSENT			(1 << 7)
#define	 MCI_SCM_CMDRESPEND			(1 << 6)
#define	 MCI_SCM_RXOVERRUN			(1 << 5)
#define	 MCI_SCM_TXUNDERRUN			(1 << 4)
#define	 MCI_SCM_DATATIMEOUT			(1 << 3)
#define	 MCI_SCM_CMDTIMEOUT			(1 << 2)
#define	 MCI_SCM_DATACRCFAIL			(1 << 1)
#define	 MCI_SCM_CMDCRCFAIL			(1 << 0)


#define	MCI_FIFOCNT			0x44
#define	MCI_CCSTIMER			0x58
#define	MCI_FIFO			0x80 /* to 0xbc */

#define	MCI_FIFO_SIZE			64

#define	MCI_IRQENABLE	\
	(MCI_SCM_CMDCRCFAIL | MCI_SCM_DATACRCFAIL |			\
	MCI_SCM_CMDTIMEOUT | MCI_SCM_DATATIMEOUT |			\
	MCI_SCM_TXUNDERRUN | MCI_SCM_RXOVERRUN |			\
	MCI_SCM_CMDRESPEND | MCI_SCM_CMDSENT |				\
	MCI_SCM_STARTBITERR)

#define	MCI_IRQ_PIO \
	(MCI_RXDATAAVLBLMASK | MCI_TXDATAAVLBLMASK |			\
	MCI_RXFIFOEMPTYMASK | MCI_TXFIFOEMPTYMASK | 			\
	MCI_RXFIFOFULLMASK | MCI_TXFIFOFULLMASK | 			\
	MCI_RXFIFOHALFFULLMASK | MCI_TXFIFOHALFEMPTYMASK |		\
	MCI_RXACTIVEMASK | MCI_TXACTIVEMASK)

#define	MCI_FIFOHALFSIZE			(MCI_FIFOSIZE / 2)

#endif	/* _MMCI_H_ */