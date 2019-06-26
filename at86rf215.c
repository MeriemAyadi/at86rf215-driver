/*
 * AT86RF215 driver
*/

/*
 * General Explanations:
 *** spi_device :  is used to interchange data between an SPI slave
 * (usually a discrete chip) and CPU memory.
 *** platform_data : Linux uses platform_data to point to board-specific structures describing devices and how they are connected to the SoC.
 * In our case, it's the spi platform : &spi0
 *** dev.of_node : associated device tree node .
 * IN our case, it's the node of : at86rf215@0{ .
 */

/*
 * Notes:
 * I should check the sleep_gpio and reset_gpio in device tree later.
 * 
*/


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


//The following structure is used by the function "at86rf215_get_pdata" and was declared in <linux/spi/at86rf230.h> for AT86RF230 driver.
struct at86rf215_platform_data {
        int rstn;
        int slp_tr;
        int dig2;
        u8 xtal_trim;
};

#define AT86RF2XX_MAX_BUF		(127 + 3) 
/* tx retries to access the TX_ON state // RG_TRX_STATE : 0x02
 * if it's above then force change will be started.
 *
 * We assume the max_frame_retries (7) value of 802.15.4 here.
 */
#define AT86RF2XX_MAX_TX_RETRIES	7
/* We use the recommended 5 minutes timeout to recalibrate */
#define AT86RF2XX_CAL_LOOP_TIMEOUT	(5 * 60 * HZ)

struct at86rf215_chip_data {
        u16 t_sleep_cycle;
        u16 t_channel_switch;
        u16 t_reset_to_off;
        u16 t_off_to_aack;
        u16 t_off_to_tx_on;
        u16 t_off_to_sleep;
        u16 t_sleep_to_off;
        u16 t_frame;
        u16 t_p_ack;
        int rssi_base_val;

        int (*set_channel)(struct at86rf215_local *, u8, u8);
        int (*set_txpower)(struct at86rf215_local *, s32);
};


struct at86rf215_state_change {
	struct at86rf215_local *lp;
	int irq;

	struct hrtimer timer;
	struct spi_message msg;
	struct spi_transfer trx;
	u8 buf[AT86RF2XX_MAX_BUF];

	void (*complete)(void *context);
	u8 from_state;
	u8 to_state;

	bool free;
};

struct at86rf215_trac {
	u64 success;
	u64 success_data_pending;
	u64 success_wait_for_ack;
	u64 channel_access_failure;
	u64 no_ack;
	u64 invalid;
};

struct at86rf215_local {
	struct spi_device *spi;

	struct ieee802154_hw *hw;
	struct at86rf215_chip_data *data;
	struct regmap *regmap;
	int slp_tr;
	bool sleep;

	struct completion state_complete;
	struct at86rf215_state_change state;

	unsigned long cal_timeout;
	bool is_tx;
	bool is_tx_from_off;
	u8 tx_retry;
	struct sk_buff *tx_skb;
	struct at86rf215_state_change tx;

	struct at86rf215_trac trac;
};

static struct at86rf215_chip_data at86rf215_data = {
        .t_sleep_cycle = 330,
        .t_channel_switch = 11,
        .t_reset_to_off = 26,
        .t_off_to_aack = 80,
        .t_off_to_tx_on = 80,
        .t_off_to_sleep = 35,
        .t_sleep_to_off = 1000,
        .t_frame = 4096,
        .t_p_ack = 545,
        .rssi_base_val = -94,
//        .set_channel = at86rf215_set_channel,
//        .set_txpower = at86rf215_set_txpower,
};


// ************************************* The AT86RF215 operations definition  ************************************* //

/*
static void at86rf230_write_frame(void *context)
{
        struct at86rf230_state_change *ctx = context;
        struct at86rf230_local *lp = ctx->lp;
        struct sk_buff *skb = lp->tx_skb;
        u8 *buf = ctx->buf;
        int rc;

        lp->is_tx = 1;

        buf[0] = CMD_FB | CMD_WRITE; // CMD_FB : 0X20  ,  CMD_WRITE : 0x40
        buf[1] = skb->len + 2;
        memcpy(buf + 2, skb->data, skb->len);
        ctx->trx.len = skb->len + 2;
        ctx->msg.complete = at86rf230_write_frame_complete;
        rc = spi_async(lp->spi, &ctx->msg);
        if (rc) {
                ctx->trx.len = 2;
                at86rf230_async_error(lp, ctx, rc);
        }
}


static void at86rf230_async_state_change_start(void *context)
{
        struct at86rf230_state_change *ctx = context;
        struct at86rf230_local *lp = ctx->lp;
        u8 *buf = ctx->buf;
        const u8 trx_state = buf[1] & TRX_STATE_MASK; // TRX_STATE_MASK : 0x1F

        // Check for "possible" STATE_TRANSITION_IN_PROGRESS 
        if (trx_state == STATE_TRANSITION_IN_PROGRESS) { // STATE_TRANSITION_IN_PROGRESS : 0x1F
                udelay(1);
                at86rf230_async_read_reg(lp, RG_TRX_STATUS, ctx, at86rf230_async_state_change_start); // RG_TRX_STATUS : 0x01
                return;
        }

        // Check if we already are in the state which we change in 
        if (trx_state == ctx->to_state) {
                if (ctx->complete)
                        ctx->complete(context);
                return;
        }

        // Set current state to the context of state change 
        ctx->from_state = trx_state;

        // Going into the next step for a state change which do a timing relevant delay.
        at86rf230_async_write_reg(lp, RG_TRX_STATE, ctx->to_state, ctx, at86rf230_async_state_delay); // RG_TRX_STATE : 0x02
}


static void at86rf215_async_state_change(struct at86rf215_local *lp, struct at86rf215_state_change *ctx, const u8 state, void (*complete)(void *context))
{
	// Initialization for the state change context 
	ctx->to_state = state;
	ctx->complete = complete;
	at86rf215_async_read_reg(lp, RG_TRX_STATUS, ctx,at86rf215_async_state_change_start); // RG_TRX_STATUS : 0x01
}

static void at86rf215_xmit_tx_on(void *context)
{
        struct at86rf215_state_change *ctx = context;
        struct at86rf215_local *lp = ctx->lp;

        at86rf215_async_state_change(lp, ctx, STATE_TX_ARET_ON, at86rf215_write_frame); // STATE_TX_ARET_ON : 0x19
}
*/
// ********************************************************************************************************************** //

static int at86rf215_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{ return 0;
}


static int at86rf215_ed(struct ieee802154_hw *hw, u8 *level)
{
        WARN_ON(!level);
        *level = 0xbe; // to check 
        return 0;
}



static int
at86rf215_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
        struct at86rf215_local *lp = hw->priv;
        int rc;
/*
        rc = lp->data->set_channel(lp, page, channel);
        // Wait for PLL/
/*        usleep_range(lp->data->t_channel_switch,
                     lp->data->t_channel_switch + 10);

        lp->cal_timeout = jiffies + AT86RF2XX_CAL_LOOP_TIMEOUT;
*/        return rc;
}


static int at86rf215_start(struct ieee802154_hw *hw)
{
return 0;
}


static void at86rf215_stop(struct ieee802154_hw *hw)
{
}


static const struct ieee802154_ops at86rf215_ops = {
        .owner = THIS_MODULE,
        .xmit_async = at86rf215_xmit,
        .ed = at86rf215_ed,
        .set_channel = at86rf215_channel,
        .start = at86rf215_start,
        .stop = at86rf215_stop,
/*        .set_hw_addr_filt = at86rf215_set_hw_addr_filt,
        .set_txpower = at86rf215_set_txpower,
        .set_lbt = at86rf215_set_lbt,
        .set_cca_mode = at86rf215_set_cca_mode,
        .set_cca_ed_level = at86rf215_set_cca_ed_level,
        .set_csma_params = at86rf215_set_csma_params,
        .set_frame_retries = at86rf215_set_frame_retries,
        .set_promiscuous_mode = at86rf215_set_promiscuous_mode,
*/
};


//Check if device tree definition for the spi device is correct.
static int at86rf215_get_pdata(struct spi_device *spi, int *rstn, int *slp_tr, u8 *xtal_trim)
{
        struct at86rf215_platform_data *pdata = spi->dev.platform_data; 
        int ret;

        if (!IS_ENABLED(CONFIG_OF) || !spi->dev.of_node) { // dev.of_node :associated device tree node 
                if (!pdata)
                        return -ENOENT;

                *rstn = pdata->rstn;
                *slp_tr = pdata->slp_tr;
                *xtal_trim = pdata->xtal_trim;
                return 0;
        }

        *rstn = of_get_named_gpio(spi->dev.of_node, "reset-gpio", 0); // Get the GPIO Number
        *slp_tr = of_get_named_gpio(spi->dev.of_node, "sleep-gpio", 0); // Get the GPIO Number
        ret = of_property_read_u8(spi->dev.of_node, "xtal-trim", xtal_trim); // search  for a property in a node and read 8-bit value(s) from it.
        if (ret < 0 && ret != -EINVAL)
                return ret;
        return 0;
}


static bool at86rf215_reg_writeable(struct device *dev, unsigned int reg)
{
        switch (reg) {
//        case RG_.....
                return true;
        default:
                return false;
        }
}

static bool
at86rf215_reg_readable(struct device *dev, unsigned int reg)
{
        bool rc;

        /* all writeable are also readable */
        rc = at86rf215_reg_writeable(dev, reg);
        if (rc)
                return rc;

        /* readonly regs */
        switch (reg) {
	case RG_RF_PN :
	case RG_RF_VN :
                return true;
        default:
                return false;
        }
}
static bool at86rf215_reg_volatile(struct device *dev, unsigned int reg)
{
        /* can be changed during runtime */
        switch (reg) {
        //case RG_....
                return true;
        default:
                return false;
        }
}

static bool at86rf215_reg_precious(struct device *dev, unsigned int reg)
{
        /* don't clear irq line on read */
        switch (reg) {
        case RG_IRQ_STATUS:
                return true;
        default:
                return false;
        }
}


#define AT86RF215_NUMREGS 0x12E // registers number
static const struct regmap_config at86rf215_regmap_spi_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.write_flag_mask = 0x80,
	.read_flag_mask = 0x00,
	.cache_type = REGCACHE_RBTREE, // Red Black tree algorithm
	.max_register = AT86RF215_NUMREGS,
	.writeable_reg = at86rf215_reg_writeable,
	.readable_reg = at86rf215_reg_readable,
	.volatile_reg = at86rf215_reg_volatile,
	.precious_reg = at86rf215_reg_precious,
};

static void at86rf215_setup_spi_messages(struct at86rf215_local *lp, struct at86rf215_state_change *state)
{
        state->lp = lp;
        state->irq = lp->spi->irq;
        spi_message_init(&state->msg);
        state->msg.context = state;
        state->trx.len = 2;
        state->trx.tx_buf = state->buf;
        state->trx.rx_buf = state->buf;
        spi_message_add_tail(&state->trx, &state->msg);
        hrtimer_init(&state->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); // a revoir
//        state->timer.function = at86rf215_async_state_timer;
}

static inline void at86rf215_sleep(struct at86rf215_local *lp)
{
        if (gpio_is_valid(lp->slp_tr)) {
                gpio_set_value(lp->slp_tr, 1);
                usleep_range(lp->data->t_off_to_sleep,
                             lp->data->t_off_to_sleep + 10);
                lp->sleep = true;
        }
}

static inline void at86rf215_awake(struct at86rf215_local *lp)
{
        if (gpio_is_valid(lp->slp_tr)) {
                gpio_set_value(lp->slp_tr, 0);
                usleep_range(lp->data->t_sleep_to_off,
                             lp->data->t_sleep_to_off + 100);
                lp->sleep = false;
        }
}

static inline int __at86rf215_read(struct at86rf215_local *lp, unsigned int addr, unsigned int *data)
{
        bool sleep = lp->sleep;
        int ret;

        /* awake for register setting if sleep */
        if (sleep)
                at86rf215_awake(lp);

        ret = regmap_read(lp->regmap, addr, data);

        /* sleep again if was sleeping */
        if (sleep)
                at86rf215_sleep(lp);

        return ret;
}

static int at86rf215_detect_device(struct at86rf215_local *lp)
{
        unsigned int part, version;
//        const char *chip;
        int rc;
        pr_info("[Detecting]: detect_device function is being called ..\n");

        rc = __at86rf215_read(lp, RG_RF_VN, &version);
        if (rc)
                return rc;
        printk(KERN_DEBUG "AT86RF215 version: %d", version);

        rc = __at86rf215_read(lp, RG_RF_PN, &part);
        if (rc)
                return rc;
        printk (KERN_DEBUG "AT86RF215 part number:  %x",part );

/*
        if ( (version != 1) || (version != 3)) {
                dev_err(&lp->spi->dev, "Version Number doesn't exist ( version : %x)\n", version);
                return -EINVAL;
        }

        if ((part != 52) | (part != 53) || (part != 54)) {
                dev_err(&lp->spi->dev, "Part Number doesn't exist ( Part Number : %x)\n", part);
                return -EINVAL;
        }
*/

//mac802154.h
        lp->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | //Indicates that xmitter will add FCS automatically
			IEEE802154_HW_RX_OMIT_CKSUM | //Indicates that transmitter will add FCS automatically
                        IEEE802154_HW_CSMA_PARAMS | //Indicates that transceiver will support csma parameters (max_be, min_be, backoff exponents). (page 172 )
                        IEEE802154_HW_FRAME_RETRIES | //Indicates that transceiver will support ARET frame retries setting.
			IEEE802154_HW_AFILT | //Indicates that transceiver will support hardware address filter setting. / ????
                        IEEE802154_HW_PROMISCUOUS; //Indicates that transceiver will support promiscuous mode setting.
//cfg802154.h
        lp->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER | //Indicates that transceiver will support transmit power setting. ( La puissance du signal envoyé) Supports ??
                             WPAN_PHY_FLAG_CCA_ED_LEVEL | //Indicates that transceiver will support cca ed level setting. ( The ED threshold )
                             WPAN_PHY_FLAG_CCA_MODE; //Indicates that transceiver will support cca mode setting. 

// NL802154_CCA_ENERGY :Energy above threshold
	lp->hw->phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY); //" (CCA-ED) is supported only" , Datasheet : page 148

        lp->hw->phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) |
                BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);

        lp->hw->phy->cca.mode = NL802154_CCA_ENERGY;

	chip = "at86rf215";
        lp->data = &at86rf215_data;
        lp->hw->phy->supported.channels[0] = 0x00007FF;
/*        lp->hw->phy->supported.channels[2] = 0x00007FF;
        lp->hw->phy->current_channel = 5; // 
        lp->hw->phy->symbol_duration = 25;
        lp->hw->phy->supported.lbt = NL802154_SUPPORTED_BOOL_BOTH;
        lp->hw->phy->supported.tx_powers = at86rf212_powers;
        lp->hw->phy->supported.tx_powers_size = ARRAY_SIZE(at86rf212_powers);
        lp->hw->phy->supported.cca_ed_levels = at86rf212_ed_levels_100;
        lp->hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(at86rf212_ed_levels_100);
*/
	return 0;
}

static int at86rf215_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw; //IEEE 802.15.4 hardware device
        struct at86rf215_local *lp; // ?
        unsigned int status;
        int rc, irq_type, rstn, slp_tr;
        cu8 xtal_trim = 0;
        pr_info("AT86RF215 probe function is called ..\n");

        if (!spi->irq) {
                dev_err(&spi->dev, "no IRQ specified\n");
                return -EINVAL;
        }

	pr_info("[Probing]: Checking whether device tree is well configured ..\n");
        rc = at86rf215_get_pdata(spi, &rstn, &slp_tr, &xtal_trim);
        if (rc < 0) {
                dev_err(&spi->dev, "failed to parse platform_data : %d .\n Please check your device tree. ", rc);
                return rc;
        }

        pr_info("[Probing]: Checking whether gpio PINs configured in the device tree could be used ..\n");

        if (gpio_is_valid(rstn)) { // This function turns 0 if gpio is valid
		// request a single GPIO with initial setup IF NOT
                pr_info("[Probing]: requesting a RESET PIN for GPIO ..");
                rc = devm_gpio_request_one(&spi->dev, rstn, GPIOF_OUT_INIT_HIGH, "rstn");

                if (rc){
                        return rc;
			}
        }

        if (gpio_is_valid(slp_tr)) {
		// request a single GPIO with initial setup IF NOT
                rc = devm_gpio_request_one(&spi->dev, slp_tr, GPIOF_OUT_INIT_LOW, "slp_tr");
		pr_info("[Probing]: requesting a SLEEP PIN for GPIO ..");

                if (rc){
                        return rc;
                        }
        }

        if (gpio_is_valid(rstn)) {
	        pr_info("[Probing]: The board is being reset.");
                udelay(1);
                gpio_set_value_cansleep(rstn, 0);
                udelay(1);
                gpio_set_value_cansleep(rstn, 1);
                usleep_range(120, 240);
        }

        hw = ieee802154_alloc_hw(sizeof(*lp), &at86rf215_ops); //Allocate memory for a new hardware device. This must be called once for each hardware device.

        if (!hw){
		printk(KERN_ALERT "The hardware couldn't be allocated: %d", ENOMEM); // Out Of memory
                return -ENOMEM;
		}

        lp = hw->priv;
        lp->hw = hw;
        lp->spi = spi;
        lp->slp_tr = slp_tr;
        hw->parent = &spi->dev;
        ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

        lp->regmap = devm_regmap_init_spi(spi, &at86rf215_regmap_spi_config);
        if (IS_ERR(lp->regmap)) {
                rc = PTR_ERR(lp->regmap);
                dev_err(&spi->dev, "Failed to allocate register map: %d\n",
                        rc);
                goto free_dev;
        }

        at86rf215_setup_spi_messages(lp, &lp->state);
        at86rf215_setup_spi_messages(lp, &lp->tx);

        rc = at86rf215_detect_device(lp);
        if (rc < 0)
                goto free_dev;

free_dev:
        ieee802154_free_hw(lp->hw);

	return 0; 
}

static int at86rf215_remove(struct spi_device *spi)
{
	pr_info("AT86RF215 remove function is called ..\n");
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

//Faisant appel à la structure "spi_driver" sans spi.h
static struct spi_driver at86rf215_driver = {
        .id_table = at86rf215_device_id, // List of SPI devices supported by this drive
        .driver = { 
        .of_match_table = of_match_ptr(at86rf215_of_match), // of_match_ptr :  Find and read an array of u8 from a property. // of_match_table : The open firmware table
        .name   = "at86rf215",
        },
        .probe      = at86rf215_probe,
        .remove     = at86rf215_remove,
};

module_spi_driver(at86rf215_driver);

MODULE_DESCRIPTION("AT86RF215 Transceiver Driver");
MODULE_LICENSE("GPL v2");
