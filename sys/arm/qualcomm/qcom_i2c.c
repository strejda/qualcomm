/*-
 * Copyright (c) 2014 Michal Meloun <meloun@miracle.cz>
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
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/resource.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "iicbus_if.h"

/* frequency definitions for high speed and max speed */
#define QCOM_I2C_MAX_FAST_FREQ		1000000

#define QUP_CONFIG			0x000
/* I2C mini core related values */
#define QUP_CONFIG_CORE_CLK_ON_EN		(1 << 13)
#define QUP_CONFIG_APP_CLK_ON_EN		(1 << 12)
#define  QUP_CONFIG_MINI_CORE_I2C_MASTER	(2 << 8)
#define  QUP_CONFIG_MINI_CORE_I2C_SLAVE		(3 << 8)
#define  QUP_CONFIG_N				15

#define QUP_STATE			0x004
#define  QUP_STATE_I2C_MAST_GEN			(1 << 4)
#define  QUP_STATE_VALID			(1 << 2)
#define  QUP_STATE_RESET			0
#define  QUP_STATE_RUN				1
#define  QUP_STATE_PAUSE			3
#define  QUP_STATE_MASK				3

#define QUP_IO_MODES			0x008
#define  QUP_OUTPUT_BLK_MODE			(1 << 10)
#define  QUP_OUTPUT_BAM_MODE			(3 << 10)
#define  QUP_INPUT_BLK_MODE			(1 << 12)
#define  QUP_INPUT_BAM_MODE			(3 << 12)
#define  QUP_UNPACK_EN				(1 << 14)
#define  QUP_PACK_EN				(1 << 15)

#define QUP_SW_RESET			0x00c
#define QUP_TIME_OUT			0x010
#define QUP_TIME_OUT_CURRENT		0x014
#define QUP_OPERATIONAL			0x018
#define  QUP_OUTPUT_FIFO_NOT_EMPTY		(1 <<  4)
#define  QUP_INPUT_FIFO_NOT_EMPTY		(1 <<  5)
#define  QUP_OUTPUT_FIFO_FULL			(1 <<  6)
#define  QUP_INPUT_FIFO_FULL			(1 <<  7)
#define  QUP_OUTPUT_SERVICE_FLAG		(1 <<  8)
#define  QUP_INPUT_SERVICE_FLAG			(1 <<  9)
#define  QUP_MAX_OUTPUT_DONE_FLAG		(1 << 10)
#define  QUP_MAX_INPUT_DONE_FLAG		(1 << 11)
#define  QUP_OUT_BLOCK_WRITE_REQ		(1 << 12)
#define  QUP_IN_BLOCK_READ_REQ			(1 << 13)

#define QUP_ERROR_FLAGS			0x01c
#define QUP_ERROR_FLAGS_EN		0x020
#define QUP_TEST_CTRL			0x024

#define QUP_MX_OUTPUT_COUNT		0x100
#define QUP_MX_OUTPUT_COUNT_CURRENT	0x104
#define QUP_MX_OUTPUT_DEBUG		0x108
#define QUP_OUTPUT_FIFO_WORD_CNT	0x10C
#define QUP_OUT_FIFO_BASE		0x110
#define QUP_MX_WRITE_COUNT		0x150
#define QUP_MX_WRITE_COUNT_CURRENT	0x154
#define QUP_MX_INPUT_COUNT		0x200
#define QUP_MX_INPUT_COUNT_CURRENT	0x204
#define QUP_MX_READ_COUNT		0x208
#define QUP_MX_READ_COUNT_CURRENT	0x20C
#define QUP_INPUT_DEBUG			0x210
#define QUP_INPUT_FIFO_WORD_CNT		0x210
#define QUP_IN_FIFO_BASE		0x218
#define I2C_MASTER_CLK_CTL		0x400
#define I2C_MASTER_STATUS		0x404



#define QUP_OPERATIONAL_RESET	0x000ff0
#define QUP_I2C_STATUS_RESET	0xfffffc

/* QUP OPERATIONAL FLAGS */


/* Most significant word offset in FIFO port */
#define QUP_MSW_SHIFT		(QUP_CONFIG_N + 1)


#define QUP_OUTPUT_BLOCK_SIZE(x)(((x) >> 0) & 0x03)
#define QUP_OUTPUT_FIFO_SIZE(x)	(((x) >> 2) & 0x07)
#define QUP_INPUT_BLOCK_SIZE(x)	(((x) >> 5) & 0x03)
#define QUP_INPUT_FIFO_SIZE(x)	(((x) >> 7) & 0x07)

/* QUP V1 tags */
#define QUP_TAG_NOP		(0 << 8)
#define QUP_TAG_START		(1 << 8)
#define QUP_TAG_DATA		(2 << 8)
#define QUP_TAG_STOP		(3 << 8)
#define QUP_TAG_REC		(4 << 8)
#define QUP_TAG_IN_DATA		(5 << 8)
#define QUP_TAG_IN_STOP		(6 << 8)
#define QUP_TAG_IN_NACK		(7 << 8)



/* Status, Error flags */
#define I2C_STATUS_ERROR_MASK		0x38000fc
#define QUP_STATUS_ERROR_FLAGS		0x7c

#define QUP_READ_LIMIT			256


#define	WR4(_sc, _r, _v)		bus_write_4((_sc)->mem_res, (_r), (_v))
#define	RD4(_sc, _r)			bus_read_4((_sc)->mem_res, (_r))

#define	LOCK(_sc)			mtx_lock(&(_sc)->mtx)
#define	UNLOCK(_sc)			mtx_unlock(&(_sc)->mtx)
#define	SLEEP(_sc, timeout)						\
	mtx_sleep(sc, &sc->mtx, 0, "i2cbuswait", timeout);
#define	LOCK_INIT(_sc)							\
	mtx_init(&_sc->mtx, device_get_nameunit(_sc->dev), "qcom_i2c", MTX_DEF)
#define	LOCK_DESTROY(_sc)		mtx_destroy(&_sc->mtx)
#define	ASSERT_LOCKED(_sc)		mtx_assert(&_sc->mtx, MA_OWNED)
#define	ASSERT_UNLOCKED(_sc)		mtx_assert(&_sc->mtx, MA_NOTOWNED)

static struct ofw_compat_data compat_data[] = {
	{"qcom,i2c-qup-v1.1.1",	1},
	{NULL,			0}
};

struct qcom_i2c_softc {
	device_t		dev;
	struct mtx		mtx;

	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_h;

	device_t		iicbus;
	clk_t			core_clk;
	clk_t			iface_clk;
	uint32_t		core_freq;
	uint32_t		clk_reg;

	int			bus_inuse;

	int			rx_fifo_size;
	int			rx_block_size;
	int			tx_fifo_size;
	int			tx_block_size;

	struct iic_msg		*msg;
	int			msg_idx;
	uint32_t		qup_err;
	uint32_t		bus_err;
	int			timeout;
};

static void
dbg_dump(struct qcom_i2c_softc *sc, const char *proc, char *arg)
{
uint32_t st;

st = RD4(sc, I2C_MASTER_STATUS);

printf("%s: %s\n", proc, arg);
printf(" cfg: 0x%08X state: 0x%08X, io_modes: 0x%08X, oper: 0x%08X, err: 0x%08X\n", RD4(sc, QUP_CONFIG), RD4(sc, QUP_STATE), RD4(sc, QUP_IO_MODES), RD4(sc, QUP_OPERATIONAL), RD4(sc, QUP_ERROR_FLAGS));
st = RD4(sc, I2C_MASTER_STATUS);
printf(" i2c status 0x%08x, in 0x%02X, out 0x%02X, clk 0x%02X, data 0x%02X\n", st, (st >> 20) & 0x07, (st >> 16) & 0x0F, (st >> 13) & 0x07, (st >> 10) & 0x07);
printf(" output: out_cnt: %d out_cnt_cur: %d, write_cnt: %d, write_cnt_cur: %d, fifo_word_cnt: %d\n",
  RD4(sc, QUP_MX_OUTPUT_COUNT), RD4(sc, QUP_MX_OUTPUT_COUNT_CURRENT), RD4(sc, QUP_MX_WRITE_COUNT), RD4(sc, QUP_MX_WRITE_COUNT_CURRENT), RD4(sc, QUP_OUTPUT_FIFO_WORD_CNT) & 0xFF);
printf(" input: in_cnt: %d, in_cur: %d, read_cnt: %d, read_cnt_cur: %d, fifo_word_cnt: %d\n",
  RD4(sc, QUP_MX_INPUT_COUNT), RD4(sc, QUP_MX_INPUT_COUNT_CURRENT), RD4(sc, QUP_MX_READ_COUNT), RD4(sc, QUP_MX_READ_COUNT_CURRENT), RD4(sc, QUP_INPUT_FIFO_WORD_CNT) & 0xFF);

st = RD4(sc, QUP_OPERATIONAL);
printf("oper: \n");
if (st & QUP_OUTPUT_FIFO_NOT_EMPTY) printf("  OUTPUT_FIFO_NOT_EMPTY\n");
if (st & QUP_INPUT_FIFO_NOT_EMPTY) printf("  INPUT_FIFO_NOT_EMPTY\n");
if (st & QUP_OUTPUT_FIFO_FULL) printf("  OUTPUT_FIFO_FULL\n");
if (st & QUP_INPUT_FIFO_FULL) printf("  INPUT_FIFO_FULL\n");
if (st & QUP_OUTPUT_SERVICE_FLAG) printf("  OUTPUT_SERVICE_FLAG\n");
if (st & QUP_INPUT_SERVICE_FLAG) printf("  INPUT_SERVICE_FLAG\n");
if (st & QUP_MAX_OUTPUT_DONE_FLAG) printf("  MAX_OUTPUT_DONE_FLAG\n");
if (st & QUP_MAX_INPUT_DONE_FLAG) printf("  MAX_INPUT_DONE_FLAG\n");
if (st & QUP_OUT_BLOCK_WRITE_REQ) printf("  OUT_BLOCK_WRITE_REQ\n");
if (st & QUP_IN_BLOCK_READ_REQ) printf("  IN_BLOCK_READ_REQ\n");


}

/* Wait for valid state */
static uint32_t
qcom_i2c_wait_for_valid(struct qcom_i2c_softc *sc)
{
	int i;
	uint32_t reg;

	for (i = 0; i < 100; i++) {
		reg = RD4(sc, QUP_STATE);
		if (reg & QUP_STATE_VALID)
			return (0);

		DELAY(1);
	}
	device_printf(sc->dev, "I2C status timedout (1): 0x%08X\n", reg);
	return (ETIMEDOUT);
}


/* Wait for given state */
static int
qcom_i2c_wait_for_state(struct qcom_i2c_softc *sc, uint32_t state)
{
	int i;
	uint32_t reg;

	for (i = 0; i < 100; i++) {
		reg = RD4(sc, QUP_STATE);
		if ((reg & QUP_STATE_VALID) &&
		    ((reg & QUP_STATE_MASK) == state))
			return (0);

		DELAY(1);
	}
	device_printf(sc->dev, "I2C set state timedout for state %d: 0x%08X, config: 0x%08X\n", state, reg,  RD4(sc, QUP_CONFIG));
	return (ETIMEDOUT);
}

/* Set new state */
static int
qcom_i2c_set_state(struct qcom_i2c_softc *sc, uint32_t state)
{
	int rv;

	rv  = qcom_i2c_wait_for_valid(sc);
	if (rv != 0)
		return (rv);
	WR4(sc, QUP_STATE, state);
	return (qcom_i2c_wait_for_state(sc, state));
}

/* Wait for I2C master bit */
static  int
qcom_i2c_wait_for_master(struct qcom_i2c_softc *sc)
{
	int i;
	uint32_t reg;

	for (i = 0; i < 100; i++) {
		reg = RD4(sc, QUP_STATE);
		if ((reg & QUP_STATE_VALID) &&
		    ((reg & QUP_STATE_I2C_MAST_GEN) == QUP_STATE_I2C_MAST_GEN))
			return (0);

		DELAY(1);
	}
	device_printf(sc->dev, "I2C status timeouted\n");
	return (ETIMEDOUT);
}

static void
qcom_i2c_issue_read(struct qcom_i2c_softc *sc)
{
	uint32_t len;
	uint32_t val;

	len = (sc->msg->len >= QUP_READ_LIMIT) ? 0 : sc->msg->len;
	val = (QUP_TAG_REC | len) << QUP_MSW_SHIFT;
	val |= QUP_TAG_START | (sc->msg->slave << 1) | 1;
	WR4(sc, QUP_OUT_FIFO_BASE,  val);
}

static void
qcom_i2c_read_fifo(struct qcom_i2c_softc *sc)
{
	uint32_t oper, val;

	for (;;) {
		oper = RD4(sc, QUP_OPERATIONAL);
		if ((oper & QUP_INPUT_FIFO_NOT_EMPTY) == 0)
			break;

		val =  RD4(sc, QUP_IN_FIFO_BASE);
		if (sc->msg_idx < sc->msg->len) {
			sc->msg->buf[sc->msg_idx] = val & 0xFF;
			sc->msg_idx++;
		}
	}
}

static int
qcom_i2c_read_one(struct qcom_i2c_softc *sc, int run, int last)
{
	int rv;

	/* Setup transaction size*/
	WR4(sc, QUP_SW_RESET, 1);
	rv = qcom_i2c_wait_for_state(sc, QUP_STATE_RESET);
	if (rv != 0)
		return (rv);
	WR4(sc, QUP_OPERATIONAL, QUP_OPERATIONAL_RESET);
	WR4(sc, QUP_ERROR_FLAGS_EN, QUP_STATUS_ERROR_FLAGS);
	WR4(sc, QUP_CONFIG, QUP_CONFIG_MINI_CORE_I2C_MASTER | QUP_CONFIG_N); // | QUP_CONFIG_CORE_CLK_ON_EN | QUP_CONFIG_APP_CLK_ON_EN);


//dbg_dump(sc, __func__, "1");

	if (sc->msg->len < (sc->rx_fifo_size / 4)) {
		/* FIFO mode */
		WR4(sc, QUP_IO_MODES, QUP_UNPACK_EN);
		WR4(sc, QUP_MX_READ_COUNT, sc->msg->len);
	} else {
		/* BLOCK mode (transfer data on chunks) */
		WR4(sc, QUP_IO_MODES, QUP_INPUT_BLK_MODE | QUP_UNPACK_EN);
		WR4(sc, QUP_MX_INPUT_COUNT, sc->msg->len);
	}

//dbg_dump(sc, __func__, "2");

//	if (!run) {
		rv = qcom_i2c_set_state(sc, QUP_STATE_RUN);
		if (rv != 0)
			goto err;
	WR4(sc, I2C_MASTER_CLK_CTL, sc->clk_reg);
//	}
//dbg_dump(sc, __func__, "3");



	rv = qcom_i2c_set_state(sc, QUP_STATE_PAUSE);
	if (rv != 0)
		goto err;

//dbg_dump(sc, __func__, "4");
	qcom_i2c_issue_read(sc);

//dbg_dump(sc, __func__, "5");
	rv = qcom_i2c_set_state(sc, QUP_STATE_RUN);
	if (rv != 0)
		goto err;
	/* Repeat until all all data are send */
	for (;;) {
//dbg_dump(sc, __func__, "c1");
		/* Wait for interrupt */
		rv = mtx_sleep(sc, &sc->mtx, 0, "i2ciowait", sc->timeout);
//printf("%s: mtx_sleep: %d\n", __func__, rv);
//dbg_dump(sc, __func__, "c4");
		if (rv != 0) {
			rv = ETIMEDOUT;
			goto err;
		}

		if ((sc->qup_err != 0) || (sc->bus_err != 0)) {
			rv = EIO;
			goto err;
		}

		qcom_i2c_read_fifo(sc);
//printf("%s: len: %d, idx: %d\n", __func__, sc->msg->len, sc->msg_idx);
//dbg_dump(sc, __func__, "c5");
		if (sc->msg_idx >= sc->msg->len)
			break;
	}
 	rv = qcom_i2c_set_state(sc, QUP_STATE_RESET);
err:
//printf(" 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
// sc->msg->buf[0], sc->msg->buf[1], sc->msg->buf[2], sc->msg->buf[3], sc->msg->buf[4]);
//printf("%s: XXXXXXXXX Done: %d  XXXXXXXXX\n", __func__, rv);
//breakpoint();
	return (rv);
}

static void
qcom_i2c_issue_write(struct qcom_i2c_softc *sc)
{

	uint32_t tag;
	uint32_t oper;
	int i;
	uint32_t val;

	if (sc->msg_idx == 0) {
		val = QUP_TAG_START | (sc->msg->slave << 1);
		i = 1;
	} else {
		val = 0;
		i = 0;
	}

	while (sc->msg_idx < sc->msg->len) {
		oper = RD4(sc, QUP_OPERATIONAL);
		if ((oper &
		    (QUP_OUTPUT_FIFO_FULL | QUP_OUTPUT_FIFO_NOT_EMPTY)) ==
		    (QUP_OUTPUT_FIFO_FULL | QUP_OUTPUT_FIFO_NOT_EMPTY))
			break;

		if (sc->msg_idx == (sc->msg->len - 1))
			tag = QUP_TAG_STOP;
		else
			tag = QUP_TAG_DATA;

		if (i & 1)
			val |= (tag | sc->msg->buf[sc->msg_idx]) << QUP_MSW_SHIFT;
		else
			val = tag | sc->msg->buf[sc->msg_idx];

		if (i & 1 || sc->msg_idx== sc->msg->len- 1)
			WR4(sc, QUP_OUT_FIFO_BASE, val);

		sc->msg_idx++;
		i++;
	}
}

static int
qcom_i2c_write_one(struct qcom_i2c_softc *sc, int run, int last)
{
	int rv;

	/* Setup transaction size*/
	WR4(sc, QUP_SW_RESET, 1);
	rv = qcom_i2c_wait_for_state(sc, QUP_STATE_RESET);
	if (rv != 0)
		return (rv);
	WR4(sc, QUP_OPERATIONAL, QUP_OPERATIONAL_RESET);
	WR4(sc, QUP_ERROR_FLAGS_EN, QUP_STATUS_ERROR_FLAGS);
	WR4(sc, QUP_CONFIG, QUP_CONFIG_MINI_CORE_I2C_MASTER | QUP_CONFIG_N); // | QUP_CONFIG_CORE_CLK_ON_EN | QUP_CONFIG_APP_CLK_ON_EN);

//dbg_dump(sc, __func__, "1");
	if ((sc->msg->len + 1) < (sc->rx_fifo_size / 4)) {
		/* FIFO mode */
		WR4(sc, QUP_IO_MODES, QUP_UNPACK_EN); //0);
		WR4(sc, QUP_MX_WRITE_COUNT, sc->msg->len + 1);
	} else {
		/* BLOCK mode (transfer data on chunks) */
		WR4(sc, QUP_IO_MODES, QUP_OUTPUT_BLK_MODE | QUP_UNPACK_EN);
		WR4(sc, QUP_MX_OUTPUT_COUNT, sc->msg->len + 1);
	}
//dbg_dump(sc, __func__, "2");
//	if (!run) {
 		rv = qcom_i2c_set_state(sc, QUP_STATE_RUN);
 		if (rv != 0)
 			goto err;
	WR4(sc, I2C_MASTER_CLK_CTL, sc->clk_reg);
// 	}

	/* Repeat until all all data are send */
	for (;;) {
 		rv = qcom_i2c_set_state(sc, QUP_STATE_PAUSE);
 		if (rv != 0)
 			goto err;

		qcom_i2c_issue_write(sc);

 		rv = qcom_i2c_set_state(sc, QUP_STATE_RUN);
 		if (rv != 0)
 			goto err;
		/* Wait for interrupt */
		rv = mtx_sleep(sc, &sc->mtx, 0, "i2ciowait", sc->timeout);

		if (rv != 0) {
			rv = ETIMEDOUT;
			goto err;
		}

		if ((sc->qup_err != 0) || (sc->bus_err != 0)) {
			rv = EIO;
			goto err;
		}
		if (sc->msg_idx >= sc->msg->len)
			break;
	}
//	rv = 0;
 	rv = qcom_i2c_set_state(sc, QUP_STATE_RESET);

//	if (last)
//		rv = qcom_i2c_wait_ready(qup, QUP_OUT_NOT_EMPTY, RESET_BIT,
//					 ONE_BYTE);
err:
//printf("%s: XXXXXXXXX Done: %d  XXXXXXXXX\n", __func__, rv);
//printf("%s: 0x%02X 0x%02X\n", __func__,
// sc->msg->buf[0], sc->msg->buf[1]);
//dbg_dump(sc, __func__, "end");
//printf("%s: XXXXXXXXX Done: %d  XXXXXXXXX\n", __func__, rv);
//breakpoint();

//breakpoint();
	return (rv);
}

static int
qcom_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	int rv, i, last;
	struct qcom_i2c_softc *sc;
//printf("\n\n\n\n%s: enter - nmsgs: %d \n", __func__, nmsgs);

 	sc = device_get_softc(dev);
	LOCK(sc);

	/* Get the bus */
	while (sc->bus_inuse == 1)
		SLEEP(sc,  0);
	sc->bus_inuse = 1;

	WR4(sc, QUP_SW_RESET, 1);
	rv = qcom_i2c_wait_for_state(sc, QUP_STATE_RESET);
	if (rv != 0)
		return (rv);

	WR4(sc, QUP_OPERATIONAL, QUP_OPERATIONAL_RESET);
	WR4(sc, QUP_ERROR_FLAGS_EN, QUP_STATUS_ERROR_FLAGS);
	WR4(sc, QUP_CONFIG, QUP_CONFIG_MINI_CORE_I2C_MASTER | QUP_CONFIG_N);
	WR4(sc, I2C_MASTER_CLK_CTL, sc->clk_reg);

	rv = 0;
	for (i = 0; i < nmsgs; i++) {
//printf("\n\n\n\n\n%s: msg cycle - flags = 0x%08X\n", __func__, msgs[i].flags);
		sc->msg = &msgs[i];
		sc->msg_idx = 0;
		sc->qup_err = 0;
		sc->bus_err = 0;

		/* Zero byte transfers aren't allowed. */
		if (sc->msg == NULL || sc->msg->buf == NULL ||
		    sc->msg->len == 0) {
			rv = EINVAL;
			break;
		}
		rv = qcom_i2c_wait_for_master(sc);
//printf("%s: qcom_i2c_wait_for_master: %d\n", __func__, rv);
		if (rv != 0) {
			rv = EIO;
			break;
		}
		last = (i == (sc->msg_idx - 1));
		if ((sc->msg->flags & IIC_M_RD) != 0)
			rv = qcom_i2c_read_one(sc, i, last);
		else
			rv = qcom_i2c_write_one(sc, i, last);
//printf("%s: oper: %d\n", __func__, rv);
		if (rv != 0)
			break;
	}

	if (rv != 0) {
		rv = qcom_i2c_set_state(sc, QUP_STATE_RESET);
	}

	sc->msg = NULL;
	sc->bus_inuse = 0;

	/* Wake up the processes that are waiting for the bus. */
	wakeup(sc);

	UNLOCK(sc);

	return (rv);
}

static int
qcom_i2c_setup_clk(struct qcom_i2c_softc *sc, int clk_freq)
{
	uint32_t hs_div, fs_div;

	if (clk_freq > QCOM_I2C_MAX_FAST_FREQ) {
		fs_div =((QCOM_I2C_MAX_FAST_FREQ / clk_freq) / 2) - 3;
		hs_div = (sc->core_freq / clk_freq) / 3;
	} else {
		fs_div = ((sc->core_freq  / clk_freq) / 2) - 3;
		hs_div = 3;
	}

	hs_div = min(hs_div, 0x7);
	sc->clk_reg = (hs_div << 8) | (fs_div & 0xff);
	return (0);
}

static int
qcom_i2c_iicbus_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct qcom_i2c_softc *sc;
	int rv;
	int busfreq;


	sc = device_get_softc(dev);
	busfreq = IICBUS_GET_FREQUENCY(sc->iicbus, speed);
//printf("%s: enter, speed: %d, busfreq: %d \n", __func__, speed, busfreq);
	sc = device_get_softc(dev);
	LOCK(sc);
	rv = qcom_i2c_setup_clk(sc, speed);
	UNLOCK(sc);
	return (rv);
}



static void
qcom_i2c_intr(void *arg)
{
	struct qcom_i2c_softc *sc;
	uint32_t bus_err, qup_err, oper_reg;

 	sc = (struct qcom_i2c_softc *)arg;

	LOCK(sc);
	bus_err = RD4(sc, I2C_MASTER_STATUS);
	qup_err = RD4(sc, QUP_ERROR_FLAGS);
	oper_reg = RD4(sc, QUP_OPERATIONAL);
	sc->bus_err  = bus_err & I2C_STATUS_ERROR_MASK;
	sc->qup_err = qup_err & QUP_STATUS_ERROR_FLAGS;
//dbg_dump(sc, __func__, "1");
//printf("%s: bus: 0x%08x(0x%08x)  error: 0x%08x(0x%08x) oper: 0x%08x\n", __func__, sc->bus_err, bus_err, sc->qup_err, qup_err, oper_reg);
	if (sc->msg == NULL) {
		/* Unexpected interrupt - clear core */
		WR4(sc, QUP_STATE, QUP_STATE_RESET);
		UNLOCK(sc);
		return;
	}


	/* Handle errors first */
	if (sc->qup_err != 0) {
		WR4(sc, QUP_ERROR_FLAGS, qup_err);
		goto done;
	}

	if (sc->bus_err != 0) {
		WR4(sc, QUP_STATE, QUP_STATE_RESET);
		goto done;
	}

	/* Read service request */
	if (oper_reg & QUP_INPUT_SERVICE_FLAG) {
		WR4(sc, QUP_OPERATIONAL, QUP_INPUT_SERVICE_FLAG);
	}

	/* Write service request */
	if (oper_reg & QUP_OUTPUT_SERVICE_FLAG) {
		WR4(sc, QUP_OPERATIONAL, QUP_OUTPUT_SERVICE_FLAG);
	}
done:
	wakeup(sc);
	UNLOCK(sc);
}


static int
qcom_i2c_hw_init(struct qcom_i2c_softc *sc)
{
	uint32_t iomode;
	int size;
	static const int fifo_blk_tbl[] = {4, 16, 32};

	iomode = RD4(sc, QUP_IO_MODES);

	/* Get FIFO sizes */
	size = QUP_OUTPUT_BLOCK_SIZE(iomode);
	if (size >= nitems(fifo_blk_tbl))
		return(ERANGE);
	sc->tx_block_size = fifo_blk_tbl[size];

	size = QUP_INPUT_BLOCK_SIZE(iomode);
	if (size >= nitems(fifo_blk_tbl))
		return(ERANGE);
	sc->rx_block_size = fifo_blk_tbl[size];

	size = QUP_OUTPUT_FIFO_SIZE(iomode);
	sc->tx_fifo_size = sc->tx_block_size * (2 << size);

	size = QUP_INPUT_FIFO_SIZE(iomode);
	sc->rx_fifo_size = sc->rx_block_size * (2 << size);


	WR4(sc, QUP_SW_RESET, 1);
	qcom_i2c_wait_for_state(sc, QUP_STATE_RESET);

	qcom_i2c_setup_clk(sc, 100000);

	WR4(sc, QUP_IO_MODES, 0);
	WR4(sc, QUP_OPERATIONAL, QUP_OPERATIONAL_RESET);
	WR4(sc, QUP_ERROR_FLAGS_EN, QUP_STATUS_ERROR_FLAGS);
	WR4(sc, QUP_CONFIG, QUP_CONFIG_MINI_CORE_I2C_MASTER | QUP_CONFIG_N);
	WR4(sc, I2C_MASTER_CLK_CTL, sc->clk_reg);

//dbg_dump(sc, __func__, "");
	return (0);
}

static int
qcom_i2c_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

static int
qcom_i2c_attach(device_t dev)
{
	int rv, rid;
	phandle_t node;
	struct qcom_i2c_softc *sc;
	uint64_t freq;

 	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->timeout = 5 * hz;

	LOCK_INIT(sc);

	/* Get the memory resource for the register mapping. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot map registers.\n");
		rv = ENXIO;
		goto fail;
	}

	/* Allocate our IRQ resource. */
	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Cannot allocate interrupt.\n");
		rv = ENXIO;
		goto fail;
	}

	/* Setup pins */
	rv = fdt_pinctrl_configure_by_name(dev, "default");
	if (rv != 0) {

		device_printf(dev, "Cannot configure pins: %d\n", rv);
		goto fail;
	}
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
	sc->core_freq = freq;
//device_printf(sc->dev, "XXXXXXXXXXXX Got clock: %lld\n", freq);

	/* Init hardware */
	rv = qcom_i2c_hw_init(sc);
	if (rv) {
		device_printf(dev, "qcom_i2c_activate failed\n");
		goto fail;
	}

	/* Setup  interrupt */
	rv = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, qcom_i2c_intr, sc, &sc->irq_h);
	if (rv) {
		device_printf(dev, "Cannot setup interrupt.\n");
		goto fail;
	}

	/* Attach the iicbus */
	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "Could not allocate iicbus instance.\n");
		rv = ENXIO;
		goto fail;
	}
	if (bootverbose)
		device_printf(dev, "Tx FIFO size: %d bytes (%d blocks), "
		    "Rx FIFO size: %d bytes (%d blocks)\n",
		    sc->tx_fifo_size, sc->tx_fifo_size / sc->tx_block_size,
		    sc->rx_fifo_size, sc->rx_fifo_size/ sc->rx_block_size);

	/* Probe and attach the iicbus */
	return (bus_generic_attach(dev));

fail:
	if (sc->irq_h != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_h);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	LOCK_DESTROY(sc);

	return (rv);
}

static int
qcom_i2c_detach(device_t dev)
{
	struct qcom_i2c_softc *sc;
	int rv;

 	sc = device_get_softc(dev);
	qcom_i2c_hw_init(sc);
	LOCK_DESTROY(sc);
	if (sc->iicbus)
	    rv = device_delete_child(dev, sc->iicbus);
	return (bus_generic_detach(dev));
}


static phandle_t
qcom_i2c_get_node(device_t bus, device_t dev)
{

	/* Share controller node with iibus device. */
	return (ofw_bus_get_node(bus));
}

static device_method_t qcom_i2c_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_i2c_probe),
	DEVMETHOD(device_attach,	qcom_i2c_attach),
	DEVMETHOD(device_detach,	qcom_i2c_detach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_alloc_resource,	bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),

	/* OFW methods */
	DEVMETHOD(ofw_bus_get_node,	qcom_i2c_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		qcom_i2c_iicbus_reset),
	DEVMETHOD(iicbus_transfer,	qcom_i2c_transfer),

	DEVMETHOD_END
};

DEFINE_CLASS_0(iichb, qcom_i2c_driver, qcom_i2c_methods,
    sizeof(struct qcom_i2c_softc));
static devclass_t  qcom_i2c_devclass;
DRIVER_MODULE(iichb, qcom_gsbi, qcom_i2c_driver, qcom_i2c_devclass, 0, 0);
