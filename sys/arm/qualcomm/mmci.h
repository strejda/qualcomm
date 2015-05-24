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


#define	MCI_POWER			0x00
#define	 MCI_POWER_OPENDRAIN			(1 << 6)
#define	 MCI_POWER_CTRL_OFF			0x00
#define	 MCI_POWER_CTRL_UP			0x02
#define	 MCI_POWER_CTRL_ON			0x03
#define	 MCI_POWER_CTRL_MASK			0x03

#define	MCI_CLOCK			0x04

/* Qualcomm specific */

#define	 MCI_CLOCK_QCOM_SDCC_CLK_EXT_EN		(1 << 26)
#define	 MCI_CLOCK_QCOM_RX_FLOW_TIMING		(1 << 25)
#define	 MCI_CLOCK_QCOM_MCLK_SEL_GATED		(0 << 23)
#define	 MCI_CLOCK_QCOM_MCLK_SEL_FBPAD		(1 << 23)
#define	 MCI_CLOCK_QCOM_MCLK_SEL_FREE		(2 << 23)
#define	 MCI_CLOCK_QCOM_CLK_INV			(1 << 22)
#define	 MCI_CLOCK_QCOM_IO_PAD_PWR_SWITCH	(1 << 21)
#define	 MCI_CLOCK_QCOM_CLK_FB_DLY_SEL		(1 << 20)
#define	 MCI_CLOCK_QCOM_SD_DEV_SEL(x)		((x) << 18)
#define	 MCI_CLOCK_QCOM_HCLKON_SW_EN		(1 << 17)
#define	 MCI_CLOCK_QCOM_SELECT_FALLING		(0 << 14)
#define	 MCI_CLOCK_QCOM_SELECT_RISING		(1 << 14)
#define	 MCI_CLOCK_QCOM_SELECT_FBCLK		(2 << 14)
#define	 MCI_CLOCK_QCOM_SELECT_DDR		(3 << 14)
#define	 MCI_CLOCK_QCOM_SELECT_UHS		(4 << 14)
#define	 MCI_CLOCK_QCOM_SELECT_UHSDIV2		(5 << 14)
#define	 MCI_CLOCK_QCOM_INVERT_OUT		(1 << 13)
#define	 MCI_CLOCK_QCOM_FLOW_ENA		(1 << 12)

#define	 MCI_CLOCK_QCOM_BUS_MASK		((1 << 10) | (1 << 11))
#define	 MCI_CLOCK_QCOM_BUS_8			(1 << 10)
#define	 MCI_CLOCK_QCOM_BUS_4			(2 << 10)
#define	 MCI_CLOCK_QCOM_BUS_1			(0 << 10)

#define	 MCI_CLOCK_WIDEBUS			(1 << 11)
#define	 MCI_CLOCK_BYPASS			(1 << 10)
#define	 MCI_CLOCK_PWRSAVE			(1 << 9)
#define	 MCI_CLOCK_ENABLE			(1 << 8)
#define	 MCI_CLOCK_CLKDIVMASK			0xff

#define	MCI_ARGUMENT			0x08

#define	MCI_CMD				0x0c
#define	 MCI_CMD_QCOM_AUTO_CMD19		(1 << 16)
#define	 MCI_CMD_QCOM_CCS_DISABLE		(1 << 15)
#define	 MCI_CMD_QCOM_CCS_ENABLE		(1 << 14)
#define	 MCI_CMD_QCOM_MCIABORT			(1 << 13)
#define	 MCI_CMD_QCOM_DAT_CMD			(1 << 12)
#define	 MCI_CMD_QCOM_PROG_ENA			(1 << 11)

#define	 MCI_CMD_ENABLE				(1 << 10)
#define	 MCI_CMD_PENDING			(1 << 9)
#define	 MCI_CMD_INTERRUPT			(1 << 8)
#define	 MCI_CMD_LONGRSP			(1 << 7)
#define	 MCI_CMD_RESPONSE			(1 << 6)
#define	 MCI_CMD_CMD_INDEX_MASK		0x3f

#define	MCI_RESP_CMD			0x10
#define	MCI_RESP0			0x14
#define	MCI_RESP1			0x18
#define	MCI_RESP2			0x1c
#define	MCI_RESP3			0x20
#define	MCI_DATATIMER			0x24
#define	MCI_DATALENGTH			0x28

#define	MCI_DATA_CTL			0x2c
#define	 MCI_DATA_CTL_QCOM_SW_CMD19		(1 << 21)
#define	 MCI_DATA_CTL_QCOM_AUTO_PROG_DONE	(1 << 19)
#define	 MCI_DATA_CTL_QCOM_INFINITE_TRANSFER	(1 << 18)
#define	 MCI_DATA_CTL_QCOM_DATA_PEND		(1 << 17)
#define	 MCI_DATA_CTL_BLOCKSIZE_SHIFT		4
#define	 MCI_DATA_CTL_BLOCKSIZE_MASK		0xf
#define	 MCI_DATA_CTL_DM_ENABLE			(1 << 3)
#define	 MCI_DATA_CTL_MODE			(1 << 2)
#define	 MCI_DATA_CTL_READ			(1 << 1)
#define	 MCI_DATA_CTL_ENABLE			(1 << 0)

#define	MCI_DATACNT			0x30

#define	MCI_STATUS			0x34
#define	MCI_CLEAR			0x38
#define	MCI_MASK0			0x3c
#define	MCI_MASK1			0x40
/* Status, Clear and Mask BITS */
#define	 MCI_SCM_AUTO_CMD19_TIMEOUT		(1 << 30)
#define	 MCI_SCM_CCS_TIMEOUT			(1 << 26)
#define	 MCI_SCM_SDIO_INTR_OPER			(1 << 25)
#define	 MCI_SCM_ATA_CMD_COMPL			(1 << 24)
#define	 MCI_SCM_PROG_DONE			(1 << 23)
#define	 MCI_SCM_SDIO_INTR			(1 << 22)
#define	 MCI_SCM_RXDATA_AVLBL			(1 << 21)
#define	 MCI_SCM_TXDATA_AVLBL			(1 << 20)
#define	 MCI_SCM_RXFIFO_EMPTY			(1 << 19)
#define	 MCI_SCM_TXFIFO_EMPTY			(1 << 18)
#define	 MCI_SCM_RXFIFO_FULL			(1 << 17)
#define	 MCI_SCM_TXFIFO_FULL			(1 << 16)
#define	 MCI_SCM_RXFIFO_HALF_FULL		(1 << 15)
#define	 MCI_SCM_TXFIFO_HALF_EMPTY		(1 << 14)
#define	 MCI_SCM_RXACTIVE			(1 << 13)
#define	 MCI_SCM_TXACTIVE			(1 << 12)
#define	 MCI_SCM_CMD_ACTIVE			(1 << 11)
#define	 MCI_SCM_DATA_BLK_END			(1 << 10)
#define	 MCI_SCM_START_BIT_ERR			(1 <<  9)
#define	 MCI_SCM_DATAEND			(1 <<  8)
#define	 MCI_SCM_CMD_SENT			(1 <<  7)
#define	 MCI_SCM_CMD_RESP_END			(1 <<  6)
#define	 MCI_SCM_RX_OVERRUN			(1 <<  5)
#define	 MCI_SCM_TX_UNDERRUN			(1 <<  4)
#define	 MCI_SCM_DATA_TIMEOUT			(1 <<  3)
#define	 MCI_SCM_CMD_TIMEOUT			(1 <<  2)
#define	 MCI_SCM_DATA_CRC_FAIL			(1 <<  1)
#define	 MCI_SCM_CMD_CRC_FAIL			(1 <<  0)


#define	MCI_FIFOCNT			0x44
#define	MCI_CCSTIMER			0x58
#define	MCI_STATUS2			0x6C
#define	 MCI_STATUS2_MCLK_REG_WR_ACTIVE		(1 << 0)

#define	MCI_FIFO			0x80 /* to 0xbc */

#define	MCI_FIFO_SIZE			16

#define	MCI_HAVE_RX_DATA(s)						\
	(((s) & (MCI_SCM_RXACTIVE | MCI_SCM_RXDATA_AVLBL)) ==		\
	    (MCI_SCM_RXACTIVE | MCI_SCM_RXDATA_AVLBL))

#define	MCI_HAVE_TX_DATA(s)						\
	(((s) & (MCI_SCM_TXACTIVE | MCI_SCM_TXFIFO_HALF_EMPTY)) ==	\
	    (MCI_SCM_TXACTIVE | MCI_SCM_TXFIFO_HALF_EMPTY))


#define	MCI_IRQ_ERRORS							\
	(MCI_SCM_CMD_CRC_FAIL | MCI_SCM_DATA_CRC_FAIL |			\
	 MCI_SCM_CMD_TIMEOUT | MCI_SCM_DATA_TIMEOUT |			\
	 MCI_SCM_TX_UNDERRUN | MCI_SCM_RX_OVERRUN |			\
	 MCI_SCM_START_BIT_ERR)

#define MCI_IRQ_FIFO							\
	(MCI_SCM_RXDATA_AVLBL | MCI_SCM_TXDATA_AVLBL |			\
	 MCI_SCM_RXFIFO_EMPTY | MCI_SCM_TXFIFO_EMPTY |			\
	 MCI_SCM_RXFIFO_FULL | MCI_SCM_TXFIFO_FULL |			\
	 MCI_SCM_RXFIFO_HALF_FULL | MCI_SCM_TXFIFO_HALF_EMPTY)

#define	MCI_IRQENABLE	 (MCI_IRQ_ERRORS)



#endif	/* _MMCI_H_ */