/* NOTES:
 * 1) The comments that do not respect the code width ( 80 caracters per line )
 * are going to be deleted soon.
 * 2) There are still few functions commented with " // " . They represents the
 * functions I am willing to uncomment/delete while making consistent progress
 * in the code. */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
//#include <linux/spi/at86rf230.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/ieee802154.h>
#include <linux/debugfs.h>
#include <net/mac802154.h>
#include <net/cfg802154.h>
#include "at86rf215.h"

/* The following structure is used by function "at86rf215_get_pdata"
 * and was declared in <linux/spi/at86rf230.h> for AT86RF230 driver. */
/* TODO: Probably, the following structure will be deleted later and "rstn" will be used directly */
struct at86rf215_platform_data {
	int	rstn;
};

struct at86rf215_local;

/* TODO: I couldnt't find the value of t_sleep_to_off in datasheet */
/* TODO: The following transition times will be edited. */
/* All timings are in us. */
struct at86rf215_chip_data {
	u16	t_sleep_cycle;
	u16	t_channel_switch;
	u16	t_reset_to_off;
	u16	t_off_to_aack;
	u16	t_off_to_tx_on;
	u16	t_off_to_sleep;
	u16	t_sleep_to_off;
	u16	t_frame;
	u16	t_p_ack;
	int	rssi_base_val;

	int	(*set_channel)(struct at86rf215_local *, u8, u8);
	int	(*set_txpower)(struct at86rf215_local *, s32);
};

/* TODO The buffer size might change. */
#define AT86RF215_MAX_BUF               (2047)
/* tx retries to access the TX_ON state */
#define AT86RF215_MAX_TX_RETRIES        7
/* We use the recommended 5 minutes timeout to recalibrate */
#define AT86RF215_CAL_LOOP_TIMEOUT      (5 * 60 * HZ)

struct at86rf215_state_change {
	struct at86rf215_local *lp;
	int			irq;

	struct hrtimer		timer;
	struct spi_message	msg;
	struct spi_transfer	trx;
	u8			buf[AT86RF215_MAX_BUF];

	void			(*complete)(void *context);
	u8			from_state;
	u8			to_state;

	bool			free;
};

struct at86rf215_local {
	struct spi_device *		spi;

	struct ieee802154_hw *		hw;
	struct at86rf215_chip_data *	data;
	struct regmap *			regmap;

	struct completion		state_complete;
	struct at86rf215_state_change	state;

	unsigned long			cal_timeout;
	bool				is_tx;
	bool				is_tx_from_off;
	u8				tx_retry;
	struct sk_buff *		tx_skb;
	struct at86rf215_state_change	tx;
};

/* TODO: For both following functions, should we check if we're in state SLEEP
 * and if so should we transit to state TRXOFF ?
 * Maybe they will be deleted and replaced directly by regmpap functions. */
static inline int __at86rf215_write(struct at86rf215_local *lp,
				    unsigned int addr, unsigned int data)
{
	return regmap_write(lp->regmap, addr, data);
}

static inline int __at86rf215_read(struct at86rf215_local *lp,
				   unsigned int addr, unsigned int *data)
{
	return regmap_read(lp->regmap, addr, data);
}

static inline int at86rf215_read_subreg(struct at86rf215_local *lp,
					unsigned int addr, unsigned int mask,
					unsigned int shift, unsigned int *data)
{
	int rc;

	rc = __at86rf215_read(lp, addr, data);
	if (!rc)
		*data = (*data & mask) >> shift;

	return rc;
}

/* TODO: For the following functions, should we check if we're in state SLEEP
 * and if so should we transit to state TRXOFF ?
 */
static inline int at86rf215_write_subreg(struct at86rf215_local *lp,
					 unsigned int addr, unsigned int mask,
					 unsigned int shift, unsigned int data)
{
	return regmap_update_bits(lp->regmap, addr, mask, data << shift);
}

/* TODO: The following functions should be updated all along the module implemetation.*/
static bool at86rf215_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RG_BBC0_OFDMPHRTX:
	case RG_EXAMPLE:
	case RG_EXAMPLE2:
	case RG_RF24_IRQM:
	case RG_RF09_CMD:
	case RG_RF09_STATE:
	case RG_BBC0_AMEDT:
	case RG_BBC0_AMCS:
	case RG_BBC0_AFC0:
	case RG_RF_CFG:
	case RG_BBC0_IRQM:
	case RG_RF09_IRQM:
	case RG_BBC0_TXFLL:
	case RG_BBC0_TXFLH:
	case RG_BBC0_PC:
	case RG_RF09_CCF0L:
	case RG_RF09_CCF0H:
	case RG_RF09_CS:
	case RG_RF09_CNL:
	case RG_RF09_RXBWC:
	case RG_RF09_RXDFE:
	case RG_RF09_EDD:
	case RG_RF09_TXCUTC:
	case RG_RF09_TXDFE:
	case RG_RF09_PAC:
	case RG_IQIFC1:
		return true;
	default:
		return false;
	}
}

static bool at86rf215_reg_readable(struct device *dev, unsigned int reg)
{
	bool rc;

	/* all writeable are also readable */
	rc = at86rf215_reg_writeable(dev, reg);
	if (rc)
		return rc;

	/* readonly regs */
	switch (reg) {
	case RG_RF_PN:
	case RG_RF_VN:
	case RG_RF09_CMD:
	case RG_RF09_IRQS:
		return true;
	default:
		return false;
	}
}

static bool at86rf215_reg_volatile(struct device *dev, unsigned int reg)
{
	/* can be changed during runtime */
	switch (reg) {
	case RG_RF24_IRQM:
	case RG_RF09_CMD:
	case RG_RF09_STATE:
	case RG_RF09_IRQS:
	case RG_BBC0_IRQS:
		return true;
	default:
		return false;
	}
}

static bool at86rf215_reg_precious(struct device *dev, unsigned int reg)
{
	/* don't clear irq line on read */
	switch (reg) {
	case RG_RF09_IRQS:
	case RG_BBC0_IRQS:
	case RG_BBC0_IRQM:
	case RG_RF09_IRQM:
		return true;
	default:
		return false;
	}
}

//#define AT86RF215_NUMREGS 0x12E
#define AT86RF215_NUMREGS 0xFFFFFFFF
static const struct regmap_config at86rf215_regmap_spi_config = {
	.reg_bits		= 16,
	.val_bits		= 8,
	.write_flag_mask	= CMD_WRITE,
	.read_flag_mask		= CMD_READ,
	.cache_type		= REGCACHE_RBTREE, /* Red Black tree algorithm */
	.max_register		= AT86RF215_NUMREGS,
	.writeable_reg		= at86rf215_reg_writeable,
	.readable_reg		= at86rf215_reg_readable,
	.volatile_reg		= at86rf215_reg_volatile,
	.precious_reg		= at86rf215_reg_precious,
};

static void at86rf215_async_error_recover_complete(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;

	if (ctx->free)
		kfree(ctx);

	ieee802154_wake_queue(lp->hw);
}

/* TODO: Check to which state we should switch ? */
/* static void at86rf215_async_error_recover(void *context)
 * {
 *      struct at86rf215_state_change *ctx = context;
 *      struct at86rf215_local *lp = ctx->lp;
 *
 *      lp->is_tx = 0;
 *      at86rf215_async_state_change(lp, ctx, STATE_RX_AACK_ON, at86rf215_async_error_recover_complete);
 * }
 */

static inline void at86rf215_async_error(struct at86rf215_local *	lp,
					 struct at86rf215_state_change *ctx,
					 int				rc)
{
	dev_err(&lp->spi->dev, "spi_async error %d\n", rc);

	//at86rf215_async_state_change(lp, ctx, STATE_FORCE_TRX_OFF, at86rf215_async_error_recover);
	/* TODO: What state should we reach whenever an error happens ?Ã*/
	at86rf215_async_error_recover_complete(ctx);
}

static void at86rf215_async_write_reg(struct at86rf215_local *lp, u16 reg,
			u8 val, struct at86rf215_state_change *ctx, void (*complete)(void *context))
{
	int rc;
	ctx->buf[0] = ((reg & CMD_REG_MSB) >> 8) | CMD_WRITE;
	ctx->buf[1] = reg & CMD_REG_LSB;
	ctx->buf[2] = val;
	//printk(KERN_DEBUG "buf[0]=%x buf[1]=%x buf[2]=%x",ctx->buf[0],ctx->buf[1],ctx->buf[2]);
	ctx->msg.complete = complete;
	rc = spi_async(lp->spi, &ctx->msg);
	if (rc){
		printk(KERN_DEBUG "spi_async failed in write_reg.");
		at86rf215_async_error(lp, ctx, rc);
	}
}

static void at86rf215_async_read_reg(struct at86rf215_local *lp, u16 reg,
				     struct at86rf215_state_change *ctx, void (*complete)(
					     void *context))
{
	int rc;
	u8 *tx_buf = ctx->buf;

	tx_buf[0] = ((reg & CMD_REG_MSB) >> 8);
	tx_buf[1] = reg & CMD_REG_LSB;
	//printk(KERN_DEBUG "buf[0]=%x buf[1]=%x",ctx->buf[0],ctx->buf[1]);
	ctx->msg.complete = complete;

	rc = spi_async(lp->spi, &ctx->msg);
	if (rc){
		printk(KERN_DEBUG "spi_async failed in read_reg.");
		at86rf215_async_error(lp, ctx, rc);
	}
}

/* TODO: Implement this IMPORTANT function */
static enum hrtimer_restart at86rf215_async_state_timer(struct hrtimer *timer){ return 0; }

static void at86rf215_tx_complete(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;

	ieee802154_xmit_complete(lp->hw, lp->tx_skb, false);
	kfree(ctx);
}

static void at86rf215_tx_on(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;

	//at86rf215_async_state_change(lp, ctx, STATE_RX_AACK_ON, at86rf215_tx_complete);
	/* The following will be edited once the state machine is implemented. */
	at86rf215_tx_complete(ctx);
}

static void at86rf215_tx_trac_check(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;

	/* The following will be edited once the state machine is implemented. */
	//at86rf215_async_state_change(lp, ctx, STATE_TX_ON, at86rf230_tx_on);
}

/* TODO: To be implemented */
static void at86rf215_rx_read_frame_complete(void *context){}
/* TODO: To be implemented */
static void at86rf215_rx_trac_check(void *context){}

static void at86rf215_irq_trx_end(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;

	if (lp->is_tx) {
		lp->is_tx = 0;
		/* TODO: Check why do we write the RG_RF09_STATE adress on the SPI bus
		 * and then we don't read the register value ????
		 */
		at86rf215_async_read_reg(lp, RG_RF09_STATE, ctx,
					 at86rf215_tx_trac_check);
	} else {
		at86rf215_async_read_reg(lp, RG_RF09_STATE, ctx,
					 at86rf215_rx_trac_check);
	}
}

static void at86rf215_reset_check(void *context)
{
	struct at86rf215_state_change *ctx = context;
        struct at86rf215_local *lp = ctx->lp;
        const u8 *buf = ctx->buf;
        int rc;
        unsigned int state;
        u8 val = buf[2];

        if (val != STATE_RF_TRXOFF)
	        printk(KERN_DEBUG "Impossible to move to TRXOFF state");
	else
		printk(KERN_DEBUG "We're in state TRXOFF !");
}

static void at86rf215_irq_reset_end( void *context)
{
	struct at86rf215_state_change *ctx = context;
        struct at86rf215_local *lp = ctx->lp;
	int rc;

        /* Determine the current state*/
        ctx->buf[0] = (RG_RF09_STATE & CMD_REG_MSB) >> 8;
        ctx->buf[1] = RG_RF09_STATE & CMD_REG_LSB;
        ctx->msg.complete = at86rf215_reset_check;
        rc = spi_async(lp->spi, &ctx->msg);
	if (rc){
		printk (KERN_DEBUG "error occured while reading the current state async");
		kfree(ctx);
	}
}

static void at86rf215_irq_status(void *context)
{
	struct at86rf215_state_change *ctx = context;
	struct at86rf215_local *lp = ctx->lp;
	const u8 *buf = ctx->buf;
	int rc;
	unsigned int state;
	u8 val = buf[2];

	enable_irq(lp->spi->irq);

	/* Read the bits RXFE and TXFE to check if reception is completed. */
	if ((val & IRQS_1_RXFE) || (val & IRQS_4_TXFE)) {
		printk(KERN_DEBUG "Reception/Transmition completed !");
		at86rf215_irq_trx_end(ctx);
	} else if (val & IRQS_0_WAKEUP){
		printk(KERN_DEBUG "WAKEUP INTERRUPTION");
		at86rf215_irq_reset_end(ctx);
	} else {
		//printk(KERN_DEBUG "No irq received: %x ", val);
		kfree(ctx);
	}
}

static void at86rf215_setup_spi_messages(struct at86rf215_local *	lp,
					 struct at86rf215_state_change *state)
{
	state->lp = lp;
	state->irq = lp->spi->irq;
	spi_message_init(&state->msg);  /* Initialize spi_message */
	state->msg.context = state;
	state->trx.len = 3;             /* 2 bytes(address) + 1 byte(value read/written) */
	state->trx.tx_buf = state->buf;
	state->trx.rx_buf = state->buf;
	spi_message_add_tail(&state->trx, &state->msg); /* Add the message */
	hrtimer_init(&state->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	state->timer.function = at86rf215_async_state_timer;
}

/* request the IRQ and associate an interrupt handler with it */
static irqreturn_t at86rf215_isr(int irq, void *data)
{
	struct at86rf215_local *lp = data;
	struct at86rf215_state_change *ctx;
	int rc;

	/* Disables the interrupt associated with "irq" without waiting for any
	 * currently executing instances of the interrupt handler to return*/
	disable_irq_nosync(irq);

	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		enable_irq(irq);
		return IRQ_NONE;
	}

	at86rf215_setup_spi_messages(lp, ctx);

	/* tell on error handling to free ctx*/
	ctx->free = true;
	/* Determine which IRQ has occurred : read the IRQ Status*/
	ctx->buf[0] = (RG_RF09_IRQS & CMD_REG_MSB) >> 8;
	ctx->buf[1] = RG_RF09_IRQS & CMD_REG_LSB;
	ctx->msg.complete = at86rf215_irq_status;

	rc = spi_async(lp->spi, &ctx->msg);
	if (rc) {
		printk(KERN_DEBUG "Failed to request IRQ.");
		at86rf215_async_error(lp, ctx, rc);
		enable_irq(irq);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/* TODO : The following function will be implemented. */
static int at86rf215_xmit(struct ieee802154_hw *hw, struct sk_buff *skb){return 0;}
static int at86rf215_tx(struct ieee802154_hw *hw)
{
	struct at86rf215_local *lp = hw->priv;
	unsigned int val, state, len_h, len_l,len = 0x7ff;
	int rc, k, i=0;
	uint16_t offset =0;
	uint8_t data;

	/* **** BABY STEPS **** */
	/* Check if were in state TRXOFF */
	rc = __at86rf215_read(lp, RG_RF09_STATE, &state);
	if (rc) {
		printk(KERN_DEBUG "Error while reading RG_RF09_CMD");
		return rc;
	}

	if (state != RF_TRXOFF_STATUS){
		printk (KERN_DEBUG "Switching from %x to TRXOFF_STATUS", state);
		rc = __at86rf215_write(lp, RG_RF09_CMD, RF_TRXOFF_STATUS);
	        if (rc) {
        	        printk(KERN_DEBUG "Error while reading RG_RF09_CMD");
                	return rc;
	        }
	}

	/* Write frame length */
	len_l = len & 0x00ff;
	len_h = ((len & 0x0f00) >> 8);
	rc = ( __at86rf215_write(lp, RG_BBC0_TXFLL, 0xff)  && at86rf215_write_subreg(lp, SR_BBC0_TXFLH, 0x7));
	if (rc){
		printk(KERN_ALERT "Error while writing frame length");
		return rc;
	}

	/* Enable FCS */
	/* SR_BBC0_PC_TXAFCS already enabled via at86rf215_config */
	/* Download the frame (Not from MAC, jsut random caracters) */
	for (k=0; k<len; k++){
		get_random_bytes(&data, sizeof(data));
		__at86rf215_write(lp, RG_BBC0_FBTXS + offset, data);
		offset ++;
	}

	/* Move to TXPREP state, transmission is possible ONLY from this state.*/
/*	rc = __at86rf215_read(lp, RG_RF09_CMD, &state);
        if (rc)
                return rc;
	if (state == RF_TRXOFF_STATUS){
	        rc = __at86rf215_write(lp, RG_RF09_CMD, RF_TXPREP_STATUS);
		        if (rc) {
                	printk(KERN_DEBUG "Error while reading RG_RF09_CMD");
                	return rc;
                	}
	}
	else
		return rc;
*/
	/* Wait until an interruption TRXRDY is triggered : read SR_IRQS_1_TRXRDY. (BC, boucle infinie) */
/*        while (!(IRQS_1_TRXRDY) & (i<100)) { i++; }
	if (i==100){
		printk(KERN_DEBUG " NO IRS TRXRDY TRIGGERED !");
		return rc;
	}
*/	/* Move to TX state, and write the frame */
/*        if (state == RF_TXPREP_STATUS){
                rc = __at86rf215_write(lp, RG_RF09_CMD, RF_TX_STATUS);
                        if (rc) {
                        printk(KERN_DEBUG "Error while reading RG_RF09_CMD");
                        return rc;
                        }
        }
        else
                return rc;
*/
	/* NB: When the frame transmission is completed, an interrupt TXFE is
	 * trigered and state TXPREP is reached automatically. The at86rf215_isr
	 * handler will be called and functions tx_complete, tx_on and
	 * tx_trac_check will be modified. */

/*õ	i =0;
	while (!(IRQS_4_TXFE) & (i<10000)) { i++; }
        if (i==10000){
		printk(KERN_DEBUG " NO IRQ TXFE TRIGGERED !");
                return rc;
	}
*/
	return 0;
}

/* TODO : What this does ? */
static int at86rf215_ed(struct ieee802154_hw *hw, u8 *level)
{
	WARN_ON(!level);
	*level = 0xbe;
	return 0;
}

/* TODO : Implement the reception handler */
/* This is the handler that 802.15.4 module calls for the hardware device
 * initialization ( to be ready to receive data ). */
static int at86rf215_start(struct ieee802154_hw *hw){ return 0;}
/* This is the handler that 802.15.4 module calls for the hardware device
 * cleanup (after reception). */
static void at86rf215_stop(struct ieee802154_hw *hw) {}

/* TODO:
 * 1. Before setting the channel, check if the state is on TRXOFF
 * 2. See with Madani if we should add equations here to determine the "Channel
 * Center Frequency" and "The channel Spacing" or not. */
/* The register CNM ( Both Channel Mode and Channel Number High Bit Settings
 * must be written "last". */
/* The register SR_RF09_CNM_CNH is only needed for frequency range (2400-2483.5)MHz,
 * MR-OFDM Option 4 and MR-FSK mode #1 for: TotalNumChan=416. So in our case, it's
 * gonna be set at 0. */
static int at86rf215_set_channel(struct at86rf215_local *lp, u8 page,
				 u8 channel)
{	int rc;
	unsigned int cf_low=0, cf_high=0, cs=0;

        /* Channel Center Frequency */
        /* cf_low = channel_freq & 0x00ff; */
        /* cf_high = ( channel_freq & 0xff00 ) >> 8; */
        rc = (__at86rf215_write( lp, RG_RF09_CCF0L, cf_low) && __at86rf215_write( lp, RG_RF09_CCF0H, cf_high));
	if (rc)
		return rc;
        /* Channel Spacing */
        rc = __at86rf215_write(lp, RG_RF09_CS, cs);
        if (rc)
                return rc;
	/* Channel Number */
	rc = (__at86rf215_write(lp, RG_RF09_CNL, channel) && at86rf215_write_subreg(lp, SR_RF09_CNM_CNH, 0x0));
        if (rc)
                return rc;
        /* Channel Mode 0 : IEEE compliant channel scheme */
        rc = at86rf215_write_subreg(lp, SR_RF09_CNM_CM, 0x0);
        if (rc)
                return rc;

	return 0;
}

#define AT86RF215_MAX_ED_LEVELS 0xF /* 16 registers */
/* TODO: These values are so random, they should be edited. */
static const s32 at86rf215_ed_levels[AT86RF215_MAX_ED_LEVELS + 1] = {
	-9800, -9600, -9400, -9200, -9000, -8800, -8600, -8400, -8200, -8000,
	-7800, -7600, -7400, -7200, -7000, -6800
};

/* This function set the channel and the page +  set the timeout to recalibrate
 * c.a.d,    partir de ce moment, qu'on comptera 5 minutes avant la recalibration.
 * Jiffies : Kernel Internal value
 * TODO: WHY : + 10 ? */
static int at86rf215_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct at86rf215_local *lp = hw->priv;
	int rc;

	rc = lp->data->set_channel(lp, page, channel);
	/* Wait for PLL */
	usleep_range(lp->data->t_channel_switch,
		     lp->data->t_channel_switch + 10);

	lp->cal_timeout = jiffies + AT86RF215_CAL_LOOP_TIMEOUT;
	return rc;
}

#define AT86RF215_MAX_TX_POWERS 0x1F /* 32 registres */
/* PLease check datasheet page 50, register concerned: TXPWR */
static const s32 at86rf215_powers[AT86RF215_MAX_TX_POWERS +1] =
{ 3100, 3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200, 2100, 2000, 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200, 100, 0};

/* change  the corresponding register with the suitable power value */
static int at86rf2xx_set_txpower(struct at86rf215_local *lp, s32 mbm)
{
	u32 i;

	for (i = 0; i < lp->hw->phy->supported.tx_powers_size; i++)
		if (lp->hw->phy->supported.tx_powers[i] == mbm)
			return at86rf215_write_subreg(lp, SR_RF09_PAC_TXPWR, i);

	return -EINVAL;
}

/* Set the transmitter power */
static int at86rf215_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct at86rf215_local *lp = hw->priv;

	return lp->data->set_txpower(lp, mbm);
}

/* The following function search for the CCA level set ( in the function parameters )
 * if it is supported. Then affected to the registre RG_BBC0_AMEDT.
 * PS : CCA shall report a busy medium upon detecting any energy above the ED threshold. */
static int at86rf215_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct at86rf215_local *lp = hw->priv;
	u32 i;

	for (i = 0; i < hw->phy->supported.cca_ed_levels_size; i++)
		if (hw->phy->supported.cca_ed_levels[i] == mbm)
			return __at86rf215_write(lp, RG_BBC0_AMEDT, i);

	return -EINVAL;
}

/* Promiscuous mode : The Automatic Acknowledgement shoud be disabled (resp.
enabled) and the promiscuous mode shoud be enabled (resp. disabled). */
static int at86rf215_set_promiscuous_mode(struct ieee802154_hw *hw,
					  const bool		on)
{
	struct at86rf215_local *lp = hw->priv;
	int rc;

	if (on) {
		rc = at86rf215_write_subreg(lp, SR_BBC0_AMCS_AACK, 1);
		if (rc < 0)
			return rc;

		rc = at86rf215_write_subreg(lp, SR_BBC0_AFC0_PM, 1);
		if (rc < 0)
			return rc;
	} else {
		rc = at86rf215_write_subreg(lp, SR_BBC0_AMCS_AACK, 0);
		if (rc < 0)
			return rc;

		rc = at86rf215_write_subreg(lp, SR_BBC0_AFC0_PM, 0);
		if (rc < 0)
			return rc;
	}

	return 0;
}

/* These functions represent callbacks that the 802.15.4 module will call later */
static const struct ieee802154_ops at86rf215_ops = {
	/* It is mendatory to implement these functions. */
	.owner			= THIS_MODULE,
	.xmit_async		= at86rf215_xmit,
	.ed			= at86rf215_ed,
	.set_channel		= at86rf215_channel,
	.start			= at86rf215_start,
	.stop			= at86rf215_stop,
	/* The following functions are optional. */
	.set_txpower		= at86rf215_set_txpower,
	.set_cca_ed_level	= at86rf215_set_cca_ed_level,
	.set_promiscuous_mode	= at86rf215_set_promiscuous_mode,
};

/* Datasheet : page 189 (Transition time) */
/* TODO: There are more transition times that maybe will be added later. */
static struct at86rf215_chip_data at86rf215_data = {
	.t_sleep_cycle		= 500,  /* from leaving P_ON state to clk available
	                                 * (tDEEP_SLEEP_TRXOFF) */
	.t_channel_switch	= 100,  /* Duration of channel switch within frequency band.
	                                 * I chose the MAX. (tPLL_CH_SW) */
	.t_reset_to_off		= 1,    /* tRESET_TRXOFF */
	.t_off_to_aack		= 90,   /* tTRXOFF_TXPREP */
	.t_off_to_tx_on		= 90,   /* tTRXOFF_RX */
	.t_sleep_to_off		= 1,    /* tSLEEP_TRXOFF */
	.t_frame		= 4096, /* ? */
	.t_p_ack		= 545,
	.rssi_base_val		= -117,
	.set_channel		= at86rf215_set_channel,
	.set_txpower		= at86rf2xx_set_txpower,
};

static int at86rf215_hw_init(struct at86rf215_local *lp)
{
	int rc, irq_type, irq_pol = IRQ_ACTIVE_HIGH; /* Could be IRQ_ACTIVE_LOW */
	unsigned int val;

	/* Check if we're in state TRXOFF */
	rc = __at86rf215_read(lp, RG_RF09_STATE, &val);
	if ((rc) || (val != RF_TRXOFF_STATUS)) {
		printk(KERN_DEBUG "Hardware Initialisation: FAILED!");
		return rc;
	}
	/* Configuring the IRQ pin polarity. */
	irq_type = irq_get_trigger_type(lp->spi->irq);
	if (irq_type == IRQ_TYPE_EDGE_FALLING || irq_type == IRQ_TYPE_LEVEL_LOW)
		irq_pol = IRQ_ACTIVE_LOW;
	rc = at86rf215_write_subreg(lp, SR_RF_CFG_IRQP, irq_pol);
	if (rc) {
		printk(KERN_DEBUG "irq failed");
		return rc;
	}

}

static int at86rf215_config(struct at86rf215_local *lp)
{
	int rc;
	unsigned int status;

	/* TODO: Check what this does. */
/*      rc = at86rf215_write_subreg(lp, SR_RX_SAFE_MODE, 1);
 *      if (rc)
 *              return rc;
 */
        rc = __at86rf215_read(lp, RG_RF09_STATE, &status);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_STATE:Something went wrong while writing in config regs");
                return rc;
        }
	printk(KERN_DEBUG "RG_RF09_STATE = %x", status);
        rc = __at86rf215_write(lp, RG_RF09_IRQM, 0x1F);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_IRQM:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_CCF0L, 0x20);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_CCF0L:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_CCF0H, 0x8D);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_CCF0H:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_CS, 0x30);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_CS:Something went wrong while writing in config regs");
                return rc;
	}
        rc = __at86rf215_write(lp, RG_RF09_CNL, 0x03);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_CNL:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_RXBWC, 0x09);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_RXBWC:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_RXDFE, 0x83);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_RXDFE:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_EDD, 0x7A);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_EDD:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_TXCUTC, 0x0B);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_TXCUTC:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_TXDFE, 0x83);
        if (rc){
                printk(KERN_DEBUG "RG_RF09_TXDFE:Something went wrong while writing in config regs");
                return rc;
        }
        rc = __at86rf215_write(lp, RG_RF09_PAC, 0x7C);

	if (rc){
		printk(KERN_DEBUG "RG_RF09_PAC:Something went wrong while writing in config regs");
		return rc;
	}
        rc = __at86rf215_write(lp, RG_BBC0_IRQM, 0x1f);
        if (rc){
                printk(KERN_ALERT "RG_BBC0_IRQM cant be written.");
        }
        rc = __at86rf215_write(lp, RG_BBC0_PC, 0x56);
        if (rc){
                printk(KERN_ALERT "RG_BBC0_PC cant be written.");
		return rc;
        }
        rc = __at86rf215_write(lp, RG_BBC0_OFDMPHRTX,0x03);
        if (rc){
                printk(KERN_ALERT "RG_BBC0_OFDMPHRTX cant be written.");
		return rc;
        }
	/* TODO: Check if this is necessary */
	/* CLKM changes are applied immediately
	 * (In case there are CLKM clock rate modifications)*/
/*        rc = at86rf215_write_subreg(lp, SR_CLKM_SHA_SEL, 0x00);
 *      if (rc)
 *              return rc;
 *      /* Turn CLKM Off*/
/*        rc = at86rf215_write_subreg(lp, SR_CLKM_CTRL, 0x00);
 *      if (rc)
 *              return rc;
 *      // Wait the next SLEEP cycle
 *      usleep_range(lp->data->t_sleep_cycle,
 *                   lp->data->t_sleep_cycle + 100);
 */
	/* xtal_trim w dvdd reading values is deleted for now */

	return rc;
}

/* Check if device tree definition for the spi device is correct. */
static int at86rf215_get_pdata(struct spi_device *spi, int *rstn)
{
	struct at86rf215_platform_data *pdata = spi->dev.platform_data;

	/* dev.of_node :associated device tree node */
	if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) {
		if (!pdata)
			return -ENOENT;

		*rstn = pdata->rstn;
		return 0;
	}

	/* Get RESET GPIO Number */
	*rstn = of_get_named_gpio(spi->dev.of_node, "reset-gpio", 0);

	return 0;
}

static int at86rf215_detect_device(struct at86rf215_local *lp)
{
	unsigned int part, version;
	int rc;

	pr_info("[Detecting]: detect_device function is being called ..\n");

	rc = __at86rf215_read(lp, RG_RF_VN, &version);
	if (rc)
		return rc;
	printk(KERN_DEBUG "[Detecting]: AT86RF215 version: %x", version);

	rc = __at86rf215_read(lp, RG_RF_PN, &part);
	if (rc)
		return rc;
	/* printk (KERN_DEBUG "[Detecting]: AT86RF215 part number: %x", part); */

/* TODO: Check this stupid test.
 *      if ( (version != 1) || (version != 3)) {
 *              dev_err(&lp->spi->dev, "VN doesn't exist ( version : %x)\n", version);
 *              return -EINVAL;
 *      }
 *      if ((part != 52) | (part != 53) || (part != 54)) {
 *              dev_err(&lp->spi->dev, "PN doesn't exist ( Part Number : %x)\n", part);
 *              return -EINVAL;
 *      }
*/

	/* Please check mac802154.h */
	lp->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM |           /* Tx will add FCS automatically */
			IEEE802154_HW_RX_OMIT_CKSUM |           /* Rx will add FCS automatically */
			IEEE802154_HW_CSMA_PARAMS |             /* It supports csma parameters (p172)
	                                                         * (max_be,min_be,backoff exponents) */
			IEEE802154_HW_FRAME_RETRIES |           /* It supports ARET frame retries setting. */
			IEEE802154_HW_AFILT |                   /* It supports hardware address filter setting. */
			IEEE802154_HW_PROMISCUOUS;              /* It supports promiscuous mode setting. */
	/* Please check cfg802154.h */
	lp->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER |            /* It supports TXPWR setting */
			     WPAN_PHY_FLAG_CCA_ED_LEVEL |       /* It supports cca ED threshold */
			     WPAN_PHY_FLAG_CCA_MODE;            /* It supports cca mode setting. */

	/* NL802154_CCA_ENERGY :Energy above threshold */
	/* (CCA-ED) is supported only, please refer to Datasheet (p148) */
	lp->hw->phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY);
	lp->hw->phy->supported.cca_opts =
	BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);
	lp->hw->phy->cca.mode = NL802154_CCA_ENERGY;

	lp->data = &at86rf215_data;

	/* 868 MHz BPSK 802.15.4-2003 */
	lp->hw->phy->supported.channels[0] |= 1;
	/* 915 MHz BPSK 802.15.4-2003 */
	lp->hw->phy->supported.channels[0] |= 0x7fe;
	/* 2.4 GHz O-QPSK 802.15.4-2003 */
	lp->hw->phy->supported.channels[0] |= 0x7FFF800;
	/* 868 MHz ASK 802.15.4-2006 */
	lp->hw->phy->supported.channels[1] |= 1;
	/* 915 MHz ASK 802.15.4-2006 */
	lp->hw->phy->supported.channels[1] |= 0x7fe;
	/* 868 MHz O-QPSK 802.15.4-2006 */
	lp->hw->phy->supported.channels[2] |= 1;
	/* 915 MHz O-QPSK 802.15.4-2006 */
	lp->hw->phy->supported.channels[2] |= 0x7fe;

	/* Select page and channel by default. */
	lp->hw->phy->current_channel = 5;
	lp->hw->phy->current_page = 0;

	/* symbol_duration: la duree d'un symbole PSDU module et code */
	lp->hw->phy->symbol_duration = 4; /* rf215 (ttx_start_delay), rf230 (tTR10) */
	lp->hw->phy->supported.tx_powers = at86rf215_powers;
	lp->hw->phy->supported.tx_powers_size = ARRAY_SIZE(at86rf215_powers);
	lp->hw->phy->supported.cca_ed_levels = at86rf215_ed_levels;
	lp->hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(
		at86rf215_ed_levels);

	/* Define the ED threshold + the transmitter power */
	lp->hw->phy->cca_ed_level = lp->hw->phy->supported.cca_ed_levels[7];
	lp->hw->phy->transmit_power = lp->hw->phy->supported.tx_powers[0];

	return rc;
}

static int at86rf215_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct at86rf215_local *lp;
	int rc, rstn, irq_type;
	unsigned int status, val;

	pr_info("[Probing]: AT86RF215 probe function is called ..\n");

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	pr_info( "[Probing]: Checking if device tree is well configured ..\n");
	rc = at86rf215_get_pdata(spi, &rstn);
	if (rc < 0) {
		dev_err(&spi->dev, "failed to parse platform_data : %d .\n Please check your DT. ", rc);
		return rc;
	}

	pr_info( "[Probing]: Checking whether gpio PINs configured in the DT could be used ..\n");
	if (gpio_is_valid(rstn)) { /* This function turns 0 if gpio is valid */
		/* request a single GPIO with initial setup IF NOT */
		pr_info("[Probing]: requesting a RESET PIN for GPIO ..");
		rc = devm_gpio_request_one(&spi->dev, rstn, GPIOF_OUT_INIT_HIGH,
					   "rstn");
		if (rc)
			return rc;
	}

	if (gpio_is_valid(rstn)) {
		pr_info("[Probing]: The board is being reset.");
		udelay(1);
		gpio_set_value_cansleep(rstn, 0);
		udelay(1);
		gpio_set_value_cansleep(rstn, 1);
		usleep_range(120, 240);
	}

	/* Allocate memory for a new hardware device. This must be called once for each
	 * hardware device. It returns the address of the pointer lp->hw .*/
	hw = ieee802154_alloc_hw(sizeof(*lp), &at86rf215_ops);
	if (!hw)
		//printk(KERN_ALERT "[Probing]: The hardware couldn't be allocated: %d", ENOMEM);
		return -ENOMEM;

	lp = hw->priv;
	lp->hw = hw;
	lp->spi = spi;
	hw->parent = &spi->dev;

	/* TODO: why would he request an extended address ( sur 8 octets ) ? We'll see LATER. */
	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	/* This function define SPI Protocol specifications. */
	lp->regmap = devm_regmap_init_spi(spi, &at86rf215_regmap_spi_config);
	if (IS_ERR(lp->regmap)) {
		rc = PTR_ERR(lp->regmap);
		dev_err(&spi->dev,
			"[Probing]: Failed to allocate register map: %d\n", rc);
		goto free_dev;
	}

	/* TODO: The following may be edited */
	at86rf215_setup_spi_messages(lp, &lp->state);   /* where are we */
	at86rf215_setup_spi_messages(lp, &lp->tx);      /* where do we wanna go */

	rc = at86rf215_detect_device(lp);
	if (rc) {
		printk(KERN_DEBUG "Device detecting failed.");
		goto free_dev;
	}

	/* This function initialize a dynamically allocated completion pointer
	 * "lp->state_complete" to completion structure that is to be initialized */
	init_completion(&lp->state_complete);

	spi_set_drvdata(spi, lp); /* spi->dev->driver_data = lp */

	rc = at86rf215_hw_init(lp);
	if (rc) {
		printk(KERN_ALERT "at86rf215_hw_init FAILED.");
		goto free_dev;
	}

	irq_type = irq_get_trigger_type(spi->irq);
	if (!irq_type){
		printk (KERN_DEBUG "Assigning an IRQ type to the IRQ line");
		irq_type = IRQF_TRIGGER_HIGH;
	}

	/* Request the IRQ and associate an interrupt handler with it.
	 * lp: is passed as an argument to at86rf215_isr */
	rc = devm_request_irq(&spi->dev, spi->irq, at86rf215_isr,
			      IRQF_SHARED | irq_type, dev_name(&spi->dev), lp);
	if (rc) {
		printk(KERN_ALERT "devm_request_irq FAILED.");
		goto free_dev;
	}

	/* disable_irq by default and wait for starting hardware */
	disable_irq(spi->irq);

	rc = ieee802154_register_hw(lp->hw);
	if (rc) {
		printk(KERN_ALERT "Unable to register the device");
		return rc;
	}
	printk(KERN_ALERT "Device REGISTRED !");

	/* Tranceiver reset */
	/*TODO: Why this reset do NOT reset the registers values ?*/
        /* Read irq status register to reset irq line. */
/*        rc = __at86rf215_read(lp, RG_RF09_IRQS, &status);
        if (rc) {
                printk(KERN_ALERT "read_subreg FAILED.");
                goto free_dev;
        }
	printk(KERN_DEBUG "Moving to reset state.");
	enable_irq(lp->spi->irq);
	rc = __at86rf215_write(lp, RG_RF09_CMD, RF_RESET_STATUS);
	disable_irq(lp->spi->irq);
        if (rc) {
                printk(KERN_ALERT "RESET STATE change FAILED.");
                goto free_dev;
        }
*/
	/*End of tranceiver reset*/

	rc = at86rf215_config(lp);
	if (rc){
                printk(KERN_ALERT "at86rf215_config FAILED.");
                goto free_dev;
        }

	at86rf215_tx(hw);
	return rc;

free_dev:
	printk(KERN_ALERT "free_dev!");
	ieee802154_free_hw(lp->hw);

	return rc;
}



static int at86rf215_remove(struct spi_device *spi)
{
	struct at86rf215_local *lp = spi_get_drvdata(spi);

	pr_info("[Removing]: AT86RF215 remove function is called ..\n");
	/* mask all at86rf215 irq's */
	//__at86rf215_write(lp, RG_RF09_IRQM, 0x0000);
	ieee802154_unregister_hw(lp->hw);
	ieee802154_free_hw(lp->hw);
	dev_dbg(&spi->dev, "unregistered at86rf215\n");
	return 0;
}

static const struct of_device_id at86rf215_of_match[] = {
	{ .compatible = "atmel,at86rf215", },
	{ },
};
MODULE_DEVICE_TABLE(of, at86rf215_of_match);


static const struct spi_device_id at86rf215_device_id[] = {
	{ .name = "at86rf215", },
	{ },
};
MODULE_DEVICE_TABLE(spi, at86rf215_device_id);

static struct spi_driver at86rf215_driver = {
	.id_table		= at86rf215_device_id, /* List of SPI devices supported by this drive */
	.driver			={
		/* of_match_ptr :  Find and read an array of u8 from a property.
		 * of_match_table : The open firmware table. */
		.of_match_table = of_match_ptr(at86rf215_of_match),
		.name		= "at86rf215",
	},
	.probe			= at86rf215_probe,
	.remove			= at86rf215_remove,
};

module_spi_driver(at86rf215_driver);

MODULE_DESCRIPTION("AT86RF215 Transceiver Driver");
MODULE_LICENSE("GPL v2");
