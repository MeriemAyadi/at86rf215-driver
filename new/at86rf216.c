/* AT86RF215 driver */

/*
 * General Explanations:
 *** spi_device :  is used to interchange data between an SPI slave
 * (usually a discrete chip) and CPU memory.
 *** platform_data : Linux uses platform_data to point to board-specific structures describing devices and how they are connected to the SoC.
 * In our case, it's the spi platform : &spi0
 *** dev.of_node : associated device tree node .
 * In our case, it's the node of : at86rf215@0{ .
 */

/*
 * Notes:
 * - I should check the sleep_gpio and reset_gpio in device tree later.
 *
*/

#include <linux/delay.h>
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

struct at86rf215_local;
/* at86rf2xx chip depend data.
 * All timings are in us.
 */

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

#define AT86RF215_MAX_BUF               (127 + 3)
/* tx retries to access the TX_ON state // RG_TRX_STATE : 0x02
 * if it's above then force change will be started.
 *
 * We assume the max_frame_retries (7) value of 802.15.4 here.
 */
#define AT86RF215_MAX_TX_RETRIES        7
/* We use the recommended 5 minutes timeout to recalibrate */
#define AT86RF215_CAL_LOOP_TIMEOUT      (5 * 60 * HZ)

struct at86rf215_state_change {
        struct at86rf215_local *lp;
        int irq;

        struct hrtimer timer;
        struct spi_message msg;
        struct spi_transfer trx;
        u8 buf[AT86RF215_MAX_BUF];

        void (*complete)(void *context);
        u8 from_state;
        u8 to_state;

        bool free;
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

//        struct at86rf215_trac trac;
};

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

static const struct ieee802154_ops at86rf215_ops = {
        .owner = THIS_MODULE,
/*        .xmit_async = at86rf215_xmit,
        .ed = at86rf215_ed,
        .set_channel = at86rf215_channel,
        .start = at86rf215_start,
        .stop = at86rf215_stop,
        .set_hw_addr_filt = at86rf215_set_hw_addr_filt,
        .set_txpower = at86rf215_set_txpower,
        .set_lbt = at86rf215_set_lbt,
        .set_cca_mode = at86rf215_set_cca_mode,
        .set_cca_ed_level = at86rf215_set_cca_ed_level,
        .set_csma_params = at86rf215_set_csma_params,
        .set_frame_retries = at86rf215_set_frame_retries,
        .set_promiscuous_mode = at86rf215_set_promiscuous_mode,
*/
};


static struct at86rf215_chip_data at86rf215_data = {
        .t_sleep_cycle = 500, // time after leaving P_ON state (us) ==> to clk signal available ( concerned time : tDEEP_SLEEP_TRXOFF )
        .t_channel_switch = 100, // Duration of channel switch within frequency band. I chose the MAX. ( concerned time : tPLL_CH_SW )
        .t_reset_to_off = 1, // tRESET_TRXOFF
        .t_off_to_aack = 90, // tTRXOFF_TXPREP
        .t_off_to_tx_on = 90, // tTRXOFF_RX
        //.t_off_to_sleep = ?, // Apparament, sa valeur n'existe pas.
        .t_sleep_to_off = 1, // tSLEEP_TRXOFF
        .t_frame = 4096,  //?
        .t_p_ack = 545,
        .rssi_base_val = -117,
        .set_channel = at86rf215_set_channel,
        .set_txpower = at86rf2xx_set_txpower,
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
        ret = of_property_read_u8(spi->dev.of_node, "xtal-trim", xtal_trim); // search  for the property in a node (from the device tree) and read 8-bit value(s) from it.
        if (ret < 0 && ret != -EINVAL)
                return ret;
        return 0;
}

static int at86rf215_detect_device(struct at86rf215_local *lp)
{
        unsigned int part, version;
        const char *chip;
        int rc;

        pr_info("[Detecting]: detect_device function is being called ..\n");

        rc = __at86rf215_read(lp, RG_RF_VN, &version);
        if (rc)
                return rc;
        //printk(KERN_DEBUG "[Detecting]: AT86RF215 version: %x", version);

        rc = __at86rf215_read(lp, RG_RF_PN, &part);
        if (rc)
                return rc;
        //printk (KERN_DEBUG "[Detecting]: AT86RF215 part number: %x", part);
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

        lp->hw->phy->supported.cca_opts = BIT(NL802154_CCA_OPT_ENERGY_CARRIER_AND) | BIT(NL802154_CCA_OPT_ENERGY_CARRIER_OR);

        lp->hw->phy->cca.mode = NL802154_CCA_ENERGY;

        lp->data = &at86rf215_data;
//?     lp->hw->phy->supported.channels[0] = 0x00007FF;
//?     lp->hw->phy->supported.channels[2] = 0x00007FF;
//?     lp->hw->phy->current_channel = 5;
        lp->hw->phy->symbol_duration = 4; //at86rf215 (ttx_start_delay)  , at86rf230 (tTR10 )  /C’est la durée d’un symbole PSDU modulé et codé/ ==>  TO CHECK AGAIN
//?        lp->hw->phy->supported.lbt = NL802154_SUPPORTED_BOOL_BOTH; //bool states for bool capability entry ? both true and false. ??
        lp->hw->phy->supported.tx_powers = at86rf215_powers;
        lp->hw->phy->supported.tx_powers_size = ARRAY_SIZE(at86rf215_powers);
        lp->hw->phy->supported.cca_ed_levels = at86rf215_ed_levels;
        lp->hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(at86rf215_ed_levels);

//Define the ED threshold + the transmitter power
        lp->hw->phy->cca_ed_level = lp->hw->phy->supported.cca_ed_levels[7]; //laquelle choisir ?
        lp->hw->phy->transmit_power = lp->hw->phy->supported.tx_powers[0]; //laquelle choisir ?

        dev_info(&lp->spi->dev, "[Detecting]: Detected At86rf215 chip version %d\n", version);

        return rc;
}


static int at86rf215_probe(struct spi_device *spi)
{
        struct ieee802154_hw *hw; //IEEE 802.15.4 hardware device
        struct at86rf215_local *lp;
        unsigned int status, stat;
        int rc, irq_type, rstn, slp_tr,err;
        u8 xtal_trim = 0;
        pr_info("[Probing]: AT86RF215 probe function is called ..\n");

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
                //printk(KERN_ALERT "[Probing]: The hardware couldn't be allocated: %d", ENOMEM); // Out Of memory
                return -ENOMEM;
                }
        lp = hw->priv;
        lp->hw = hw;
        lp->spi = spi;
        lp->slp_tr = slp_tr;
        hw->parent = &spi->dev;
        ieee802154_random_extended_addr(&hw->phy->perm_extended_addr); // why would he request an extended address ( sur 8 octets ) ? We'll see LATER.

        lp->regmap = devm_regmap_init_spi(spi, &at86rf215_regmap_spi_config); // This function define SPI Protocol specifications.
        if (IS_ERR(lp->regmap)) {
                rc = PTR_ERR(lp->regmap);
                dev_err(&spi->dev, "[Probing]: Failed to allocate register map: %d\n", rc);
                goto free_dev;
        }

        at86rf215_setup_spi_messages(lp, &lp->state);
        at86rf215_setup_spi_messages(lp, &lp->tx);

        rc = at86rf215_detect_device(lp);
        if (rc < 0)
                goto free_dev;

        init_completion(&lp->state_complete); //init_completion - Initialize a dynamically allocated completion pointer "lp->state_complete" to completion structure that is to be initialized

        spi_set_drvdata(spi, lp); // spi->dev->driver_data = lp
        rc = at86rf215_hw_init(lp, xtal_trim);

        err=__at86rf215_read(lp, RG_RF09_CMD, &stat);
        //printk (KERN_DEBUG "RG_RF09_CMD = %x", stat);

        if (rc) {
                printk (KERN_DEBUG "The function hw_init faaaaaaaaaaaaaaaaaaaaaaaaailed");
                goto free_dev;
        }
free_debugfs:
        at86rf215_debugfs_remove();

free_dev:
        ieee802154_free_hw(lp->hw);

        return 0;
}


static int at86rf215_remove(struct spi_device *spi)
{
        pr_info("[Removing]: AT86RF215 remove function is called ..\n");
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

