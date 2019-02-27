/*
 *  Copyright Intel Corporation (C) 2017.
 * Modified by Richard Crewe <richard.crewe@nanoporetech.com> 2019
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * A modification of the i2c-altera.c driver, itself based on the
 * i2c-axxia.c driver.
 *
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include "ont_minit1c.h"
#include "ont_minit1c_reg.h"

#define ALTR_I2C_TFR_CMD	0x00	/* Transfer Command register */
#define     ALTR_I2C_TFR_CMD_STA	BIT(9)	/* send START before byte */
#define     ALTR_I2C_TFR_CMD_STO	BIT(8)	/* send STOP after byte */
#define     ALTR_I2C_TFR_CMD_RW_D	BIT(0)	/* Direction of transfer */
#define ALTR_I2C_RX_DATA	0x04	/* RX data FIFO register */
#define ALTR_I2C_CTRL		0x08	/* Control register */
#define     ALTR_I2C_CTRL_RXT_SHFT	4	/* RX FIFO Threshold */
#define     ALTR_I2C_CTRL_TCT_SHFT	2	/* TFER CMD FIFO Threshold */
#define     ALTR_I2C_CTRL_BSPEED	BIT(1)	/* Bus Speed (1=Fast) */
#define     ALTR_I2C_CTRL_EN	BIT(0)	/* Enable Core (1=Enable) */
#define ALTR_I2C_ISER		0x0C	/* Interrupt Status Enable register */
#define     ALTR_I2C_ISER_RXOF_EN	BIT(4)	/* Enable RX OVERFLOW IRQ */
#define     ALTR_I2C_ISER_ARB_EN	BIT(3)	/* Enable ARB LOST IRQ */
#define     ALTR_I2C_ISER_NACK_EN	BIT(2)	/* Enable NACK DET IRQ */
#define     ALTR_I2C_ISER_RXRDY_EN	BIT(1)	/* Enable RX Ready IRQ */
#define     ALTR_I2C_ISER_TXRDY_EN	BIT(0)	/* Enable TX Ready IRQ */
#define ALTR_I2C_ISR		0x10	/* Interrupt Status register */
#define     ALTR_I2C_ISR_RXOF		BIT(4)	/* RX OVERFLOW IRQ */
#define     ALTR_I2C_ISR_ARB		BIT(3)	/* ARB LOST IRQ */
#define     ALTR_I2C_ISR_NACK		BIT(2)	/* NACK DET IRQ */
#define     ALTR_I2C_ISR_RXRDY		BIT(1)	/* RX Ready IRQ */
#define     ALTR_I2C_ISR_TXRDY		BIT(0)	/* TX Ready IRQ */
#define ALTR_I2C_STATUS		0x14	/* Status register */
#define     ALTR_I2C_STAT_CORE		BIT(0)	/* Core Status (0=idle) */
#define ALTR_I2C_TC_FIFO_LVL	0x18	/* Transfer FIFO LVL register */
#define ALTR_I2C_RX_FIFO_LVL	0x1C	/* Receive FIFO LVL register */
#define ALTR_I2C_SCL_LOW	0x20	/* SCL low count register */
#define ALTR_I2C_SCL_HIGH	0x24	/* SCL high count register */
#define ALTR_I2C_SDA_HOLD	0x28	/* SDA hold count register */

#define ALTR_I2C_ALL_IRQ	(ALTR_I2C_ISR_RXOF | ALTR_I2C_ISR_ARB | \
				 ALTR_I2C_ISR_NACK | ALTR_I2C_ISR_RXRDY | \
				 ALTR_I2C_ISR_TXRDY)

#define ALTR_I2C_THRESHOLD	0	/* IRQ Threshold at 1 element */
#define ALTR_I2C_DFLT_FIFO_SZ	4
#define ALTR_I2C_TIMEOUT	100000	/* 100ms */

#ifdef ONT_VERBOSE_DEBUG
/* increased to 2500ms from 250ms due to all the debugging output */
#define ALTR_I2C_XFER_TIMEOUT	(msecs_to_jiffies(2500))
#else
#define ALTR_I2C_XFER_TIMEOUT	(msecs_to_jiffies(250))
#endif

/**
 * altr_i2c_dev - I2C device context
 * @base: pointer to register struct
 * @msg: pointer to current message
 * @msg_len: number of bytes transferred in msg
 * @msg_err: error code for completed message
 * @msg_complete: xfer completion object
 * @dev: device reference
 * @adapter: core i2c abstraction
 * @i2c_clk: clock reference for i2c input clock
 * @bus_clk_rate: current i2c bus clock rate
 * @buf: ptr to msg buffer for easier use.
 * @fifo_size: size of the FIFO passed in.
 * @isr_mask: cached copy of local ISR enables.
 * @isr_status: cached copy of local ISR status.
 * @lock: spinlock for IRQ synchronization.
 */
struct altr_i2c_dev {
	void __iomem *base;
	struct i2c_msg *msg;
	size_t msg_len;
    bool stop;
	int msg_err;
	struct completion msg_complete;
	struct device *dev;
	struct i2c_adapter adapter;
	u32 bus_clk_rate;
	u8 *buf;
	u32 fifo_size;
	u32 isr_mask;
	u32 isr_status;
	spinlock_t lock;	/* IRQ synchronization */
};

static void
altr_i2c_int_enable(struct altr_i2c_dev *idev, u32 mask, bool enable)
{
	unsigned long flags;
	u32 int_en;

	spin_lock_irqsave(&idev->lock, flags);

    int_en = READL(idev->base + ALTR_I2C_ISER);
    if (enable) {
		idev->isr_mask = int_en | mask;
    } else {
		idev->isr_mask = int_en & ~mask;
    }
    WRITEL(idev->isr_mask, idev->base + ALTR_I2C_ISER);

	spin_unlock_irqrestore(&idev->lock, flags);
}

static void altr_i2c_int_clear(struct altr_i2c_dev *idev, u32 mask)
{
    u32 int_en = READL(idev->base + ALTR_I2C_ISR);

    WRITEL(int_en | mask, idev->base + ALTR_I2C_ISR);
}

static void altr_i2c_core_disable(struct altr_i2c_dev *idev)
{
    u32 tmp = READL(idev->base + ALTR_I2C_CTRL);

    WRITEL(tmp & ~ALTR_I2C_CTRL_EN, idev->base + ALTR_I2C_CTRL);
}

static void altr_i2c_core_enable(struct altr_i2c_dev *idev)
{
    u32 tmp = READL(idev->base + ALTR_I2C_CTRL);

    WRITEL(tmp | ALTR_I2C_CTRL_EN, idev->base + ALTR_I2C_CTRL);
}

static void altr_i2c_reset(struct altr_i2c_dev *idev)
{
	altr_i2c_core_disable(idev);
	altr_i2c_core_enable(idev);
}

static inline void altr_i2c_stop(struct altr_i2c_dev *idev)
{
    WRITEL(ALTR_I2C_TFR_CMD_STO, idev->base + ALTR_I2C_TFR_CMD);
}

static void altr_i2c_init(struct altr_i2c_dev *idev)
{
    u32 divisor = I2C_PARENT_CLOCK_RATE / idev->bus_clk_rate;
    u32 clk_mhz = I2C_PARENT_CLOCK_RATE / 1000000;
	u32 tmp = (ALTR_I2C_THRESHOLD << ALTR_I2C_CTRL_RXT_SHFT) |
		  (ALTR_I2C_THRESHOLD << ALTR_I2C_CTRL_TCT_SHFT);
	u32 t_high, t_low;

	if (idev->bus_clk_rate <= 100000) {
		tmp &= ~ALTR_I2C_CTRL_BSPEED;
		/* Standard mode SCL 50/50 */
		t_high = divisor * 1 / 2;
		t_low = divisor * 1 / 2;
	} else {
		tmp |= ALTR_I2C_CTRL_BSPEED;
		/* Fast mode SCL 33/66 */
		t_high = divisor * 1 / 3;
		t_low = divisor * 2 / 3;
	}
    WRITEL(tmp, idev->base + ALTR_I2C_CTRL);

    dev_dbg(idev->dev, "rate=%uHz per_clk=%uMHz -> ratio=1:%u\n",
		idev->bus_clk_rate, clk_mhz, divisor);

	/* Reset controller */
	altr_i2c_reset(idev);

	/* SCL High Time */
    WRITEL(t_high, idev->base + ALTR_I2C_SCL_HIGH);
	/* SCL Low Time */
    WRITEL(t_low, idev->base + ALTR_I2C_SCL_LOW);
	/* SDA Hold Time, 300ns */
    WRITEL(div_u64(300 * clk_mhz, 1000), idev->base + ALTR_I2C_SDA_HOLD);

	/* Mask all master interrupt bits */
	altr_i2c_int_enable(idev, ALTR_I2C_ALL_IRQ, false);
}

/**
 * altr_i2c_transfer - On the last byte to be transmitted, send
 * a Stop bit on the last byte.
 */
static void altr_i2c_transfer(struct altr_i2c_dev *idev, u32 data)
{
	/* On the last byte to be transmitted, send STOP */
    if (idev->msg_len == 1 && idev->stop) {
        VPRINTK("stop bit set\n");
		data |= ALTR_I2C_TFR_CMD_STO;
    }
    if (idev->msg_len > 0) {
        WRITEL(data, idev->base + ALTR_I2C_TFR_CMD);
    }
}

/**
 * altr_i2c_empty_rx_fifo - Fetch data from RX FIFO until end of
 * transfer. Send a Stop bit on the last byte.
 */
static void altr_i2c_empty_rx_fifo(struct altr_i2c_dev *idev)
{
    size_t rx_fifo_avail = READL(idev->base + ALTR_I2C_RX_FIFO_LVL);
	int bytes_to_transfer = min(rx_fifo_avail, idev->msg_len);

	while (bytes_to_transfer-- > 0) {
        *idev->buf++ = READL(idev->base + ALTR_I2C_RX_DATA);
		idev->msg_len--;
		altr_i2c_transfer(idev, 0);
	}
}

/**
 * altr_i2c_fill_tx_fifo - Fill TX FIFO from current message buffer.
 * @return: Number of bytes left to transfer.
 */
static int altr_i2c_fill_tx_fifo(struct altr_i2c_dev *idev)
{
    size_t tx_fifo_avail = idev->fifo_size - READL(idev->base +
						       ALTR_I2C_TC_FIFO_LVL);
	int bytes_to_transfer = min(tx_fifo_avail, idev->msg_len);
	int ret = idev->msg_len - bytes_to_transfer;

	while (bytes_to_transfer-- > 0) {
		altr_i2c_transfer(idev, *idev->buf++);
		idev->msg_len--;
	}

	return ret;
}

static irqreturn_t altr_i2c_isr_quick(int irq, void *_dev)
{
	struct altr_i2c_dev *idev = _dev;
	irqreturn_t ret = IRQ_HANDLED;

	/* Read IRQ status but only interested in Enabled IRQs. */
    idev->isr_status = READL(idev->base + ALTR_I2C_ISR) & idev->isr_mask;
    if (idev->isr_status) {
		ret = IRQ_WAKE_THREAD;
    }

	return ret;
}

static irqreturn_t altr_i2c_isr(int irq, void *_dev)
{
	int ret;
	bool read, finish = false;
	struct altr_i2c_dev *idev = _dev;
	u32 status = idev->isr_status;

    VPRINTK("altr_i2c_isr 0x%08x\n",status);

	if (!idev->msg) {
		dev_warn(idev->dev, "unexpected interrupt\n");
		altr_i2c_int_clear(idev, ALTR_I2C_ALL_IRQ);
		return IRQ_HANDLED;
	}
	read = (idev->msg->flags & I2C_M_RD) != 0;

	/* handle Lost Arbitration */
	if (unlikely(status & ALTR_I2C_ISR_ARB)) {
		altr_i2c_int_clear(idev, ALTR_I2C_ISR_ARB);
		idev->msg_err = -EAGAIN;
		finish = true;
	} else if (unlikely(status & ALTR_I2C_ISR_NACK)) {
        dev_dbg(idev->dev, "Could not get ACK\n");
		idev->msg_err = -ENXIO;
		altr_i2c_int_clear(idev, ALTR_I2C_ISR_NACK);
		altr_i2c_stop(idev);
		finish = true;
	} else if (read && unlikely(status & ALTR_I2C_ISR_RXOF)) {
        /* handle RX FIFO Overflow */
		altr_i2c_empty_rx_fifo(idev);
		altr_i2c_int_clear(idev, ALTR_I2C_ISR_RXRDY);
		altr_i2c_stop(idev);
		dev_err(idev->dev, "RX FIFO Overflow\n");
		finish = true;
	} else if (read && (status & ALTR_I2C_ISR_RXRDY)) {
		/* RX FIFO needs service? */
		altr_i2c_empty_rx_fifo(idev);
		altr_i2c_int_clear(idev, ALTR_I2C_ISR_RXRDY);
        if (!idev->msg_len) {
            VPRINTK("completed rx message\n");
			finish = true;
        }
	} else if (!read && (status & ALTR_I2C_ISR_TXRDY)) {
		/* TX FIFO needs service? */
		altr_i2c_int_clear(idev, ALTR_I2C_ISR_TXRDY);
        if (idev->msg_len > 0) {
			altr_i2c_fill_tx_fifo(idev);
        } else {
            VPRINTK("completed tx message\n");
			finish = true;
        }
	} else {
		dev_warn(idev->dev, "Unexpected interrupt: 0x%x\n", status);
		altr_i2c_int_clear(idev, ALTR_I2C_ALL_IRQ);
	}

	if (finish) {
		/* Wait for the Core to finish */
        if (idev->stop) {
            VPRINTK("polling status\n");
            ret = readl_poll_timeout_atomic(idev->base + ALTR_I2C_STATUS,
                            status,
                            !(status & ALTR_I2C_STAT_CORE),
                            1, ALTR_I2C_TIMEOUT);
            if (ret) {
                dev_err(idev->dev, "message timeout (isr)\n");
            }
        } else {
            VPRINTK("not waiting for idle\n");
        }
		altr_i2c_int_enable(idev, ALTR_I2C_ALL_IRQ, false);
		altr_i2c_int_clear(idev, ALTR_I2C_ALL_IRQ);
		complete(&idev->msg_complete);
        dev_dbg(idev->dev, "Message Complete\n");
	}

    VPRINTK("exiting bh\n");
    return IRQ_HANDLED;
}

static int altr_i2c_xfer_msg(struct altr_i2c_dev *idev, struct i2c_msg *msg, bool last_message)
{
	u32 imask = ALTR_I2C_ISR_RXOF | ALTR_I2C_ISR_ARB | ALTR_I2C_ISR_NACK;
	unsigned long time_left;
	u32 value;
    //u8 addr = i2c_8bit_addr_from_msg(msg);
    u8 addr = (msg->addr << 1) | (msg->flags & I2C_M_RD ? 1 : 0);

	idev->msg = msg;
	idev->msg_len = msg->len;
    VPRINTK(" last-message %s, stop flag %s\n",
           last_message ? "set" : "clear",
           msg->flags & I2C_M_STOP ? "set" : "clear");
    idev->stop = last_message | (msg->flags & I2C_M_STOP ? true : false);
	idev->buf = msg->buf;
	idev->msg_err = 0;
	reinit_completion(&idev->msg_complete);
	altr_i2c_core_enable(idev);

	/* Make sure RX FIFO is empty */
	do {
        READL(idev->base + ALTR_I2C_RX_DATA);
    } while (READL(idev->base + ALTR_I2C_RX_FIFO_LVL));

    WRITEL(ALTR_I2C_TFR_CMD_STA | addr, idev->base + ALTR_I2C_TFR_CMD);

	if ((msg->flags & I2C_M_RD) != 0) {
        /* write the first byte to start the RX */
        altr_i2c_transfer(idev, 0);
        imask |= ALTR_I2C_ISER_RXOF_EN | ALTR_I2C_ISER_RXRDY_EN;
		altr_i2c_int_enable(idev, imask, true);
	} else {
        altr_i2c_fill_tx_fifo(idev);
        imask |= ALTR_I2C_ISR_TXRDY;
		altr_i2c_int_enable(idev, imask, true);
	}

	time_left = wait_for_completion_timeout(&idev->msg_complete,
						ALTR_I2C_XFER_TIMEOUT);
	altr_i2c_int_enable(idev, imask, false);

    value = READL(idev->base + ALTR_I2C_STATUS) & ALTR_I2C_STAT_CORE;

    // if we didn't send a stop bit, don't expect the core to be idle
    if (!idev->stop) {
        VPRINTK("ignoring core busy as we didn't send a stop bit\n");
        value &= ~ALTR_I2C_STAT_CORE;
    }
    if (value) {
		dev_err(idev->dev, "Core Status not IDLE...\n");
    }

	if (time_left == 0) {
		idev->msg_err = -ETIMEDOUT;
        dev_dbg(idev->dev, "Transaction timed out.\n");
	}

    if (idev->msg_err) {
        altr_i2c_stop(idev);
    }
    if (last_message || idev->msg_err) {
        altr_i2c_core_disable(idev);
    }

	return idev->msg_err;
}

static int
altr_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct altr_i2c_dev *idev = i2c_get_adapdata(adap);
    int ret;

    while (num) {
        num--;
        VPRINTK("message %d\n",num);
        ret = altr_i2c_xfer_msg(idev, msgs++, num == 0);
        if (ret) {
            return ret;
        }
    }
	return num;
}

static u32 altr_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm altr_i2c_algo = {
	.master_xfer = altr_i2c_xfer,
	.functionality = altr_i2c_func,
};

int borrowed_altr_i2c_probe(struct minit_device_s* m_dev)
{
	struct altr_i2c_dev *idev = NULL;
    int ret;
    struct device* dev = &m_dev->pci_device->dev;

    idev = devm_kzalloc(dev, sizeof(*idev), GFP_KERNEL);
    if (!idev) {
		return -ENOMEM;
    }

    idev->base = m_dev->ctrl_bar + I2C_BASE;
    if (IS_ERR(idev->base)) {
        return PTR_ERR(idev->base);
    }

    idev->dev = dev;
	init_completion(&idev->msg_complete);
	spin_lock_init(&idev->lock);

    dev_info(dev, "FIFO size set to default of %d\n", ALTR_I2C_DFLT_FIFO_SZ);
    idev->fifo_size = ALTR_I2C_DFLT_FIFO_SZ;

    dev_info(dev, "bus clock set to %dHz\n", EEPROM_CLOCK_RATE);
    idev->bus_clk_rate = EEPROM_CLOCK_RATE;

	if (idev->bus_clk_rate > 400000) {
        dev_err(dev, "invalid clock-frequency %d\n",
			idev->bus_clk_rate);
		return -EINVAL;
	}

	altr_i2c_init(idev);

	i2c_set_adapdata(&idev->adapter, idev);
    strlcpy(idev->adapter.name, "minit-i2c", sizeof(idev->adapter.name));
	idev->adapter.owner = THIS_MODULE;
	idev->adapter.algo = &altr_i2c_algo;
    idev->adapter.dev.parent = dev;

	ret = i2c_add_adapter(&idev->adapter);
	if (ret) {
		return ret;
	}

    m_dev->i2c_adapter = &idev->adapter;
    m_dev->i2c_dev = idev;
    m_dev->i2c_isr_quick = altr_i2c_isr_quick;
    m_dev->i2c_isr = altr_i2c_isr;

    dev_info(dev, "Altera SoftIP I2C Probe Complete\n");

	return 0;
}

void borrowed_altr_i2c_remove(struct minit_device_s* m_dev)
{
    struct i2c_adapter* adapter = m_dev->i2c_adapter;
    if (adapter) {
        i2c_del_adapter(adapter);
    }
}
