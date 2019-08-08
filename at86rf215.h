/*
 * AT86RF230/RF231 driver
 *
 * Copyright (C) 2009-2012 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written by:
 * Dmitry Eremin-Solenikov <dbaryshkov@gmail.com>
 * Alexander Smirnov <alex.bluesman.smirnov@gmail.com>
 */

#ifndef _AT86RF230_H
#define _AT86RF230_H

#define RG_TRX_STATUS	(0x01)
#define SR_TRX_STATUS		0x01, 0x1f, 0
#define SR_RESERVED_01_3	0x01, 0x20, 5
#define SR_CCA_STATUS		0x01, 0x40, 6
#define SR_CCA_DONE		0x01, 0x80, 7
#define RG_TRX_STATE	(0x02)
#define SR_TRX_CMD		0x02, 0x1f, 0
#define SR_TRAC_STATUS		0x02, 0xe0, 5
#define RG_TRX_CTRL_0	(0x03)
#define SR_CLKM_CTRL		0x03, 0x07, 0
#define SR_CLKM_SHA_SEL		0x03, 0x08, 3
#define SR_PAD_IO_CLKM		0x03, 0x30, 4
#define SR_PAD_IO		0x03, 0xc0, 6
#define RG_TRX_CTRL_1	(0x04)
#define SR_IRQ_POLARITY		0x04, 0x01, 0
#define SR_IRQ_MASK_MODE	0x04, 0x02, 1
#define SR_SPI_CMD_MODE		0x04, 0x0c, 2
#define SR_RX_BL_CTRL		0x04, 0x10, 4
#define SR_TX_AUTO_CRC_ON	0x04, 0x20, 5
#define SR_IRQ_2_EXT_EN		0x04, 0x40, 6
#define SR_PA_EXT_EN		0x04, 0x80, 7
#define RG_PHY_TX_PWR	(0x05)
#define SR_TX_PWR_23X		0x05, 0x0f, 0
#define SR_PA_LT_230		0x05, 0x30, 4
#define SR_PA_BUF_LT_230	0x05, 0xc0, 6
#define SR_TX_PWR_212		0x05, 0x1f, 0
#define SR_GC_PA_212		0x05, 0x60, 5
#define SR_PA_BOOST_LT_212	0x05, 0x80, 7
#define RG_PHY_RSSI	(0x06)
#define SR_RSSI			0x06, 0x1f, 0
#define SR_RND_VALUE		0x06, 0x60, 5
#define SR_RX_CRC_VALID		0x06, 0x80, 7
#define RG_PHY_ED_LEVEL	(0x07)
#define SR_ED_LEVEL		0x07, 0xff, 0
#define RG_PHY_CC_CCA	(0x08)
#define SR_CHANNEL		0x08, 0x1f, 0
#define SR_CCA_MODE		0x08, 0x60, 5
#define SR_CCA_REQUEST		0x08, 0x80, 7
#define RG_CCA_THRES	(0x09)
#define SR_CCA_ED_THRES		0x09, 0x0f, 0
#define SR_RESERVED_09_1	0x09, 0xf0, 4
#define RG_RX_CTRL	(0x0a)
#define SR_PDT_THRES		0x0a, 0x0f, 0
#define SR_RESERVED_0a_1	0x0a, 0xf0, 4
#define RG_SFD_VALUE	(0x0b)
#define SR_SFD_VALUE		0x0b, 0xff, 0
#define RG_TRX_CTRL_2	(0x0c)
#define SR_OQPSK_DATA_RATE	0x0c, 0x03, 0
#define SR_SUB_MODE		0x0c, 0x04, 2
#define SR_BPSK_QPSK		0x0c, 0x08, 3
#define SR_OQPSK_SUB1_RC_EN	0x0c, 0x10, 4
#define SR_RESERVED_0c_5	0x0c, 0x60, 5
#define SR_RX_SAFE_MODE		0x0c, 0x80, 7
#define RG_ANT_DIV	(0x0d)
#define SR_ANT_CTRL		0x0d, 0x03, 0
#define SR_ANT_EXT_SW_EN	0x0d, 0x04, 2
#define SR_ANT_DIV_EN		0x0d, 0x08, 3
#define SR_RESERVED_0d_2	0x0d, 0x70, 4
#define SR_ANT_SEL		0x0d, 0x80, 7
#define RG_IRQ_MASK	(0x0e)
#define SR_IRQ_MASK		0x0e, 0xff, 0
#define RG_IRQ_STATUS	(0x0f)
#define SR_IRQ_0_PLL_LOCK	0x0f, 0x01, 0
#define SR_IRQ_1_PLL_UNLOCK	0x0f, 0x02, 1
#define SR_IRQ_2_RX_START	0x0f, 0x04, 2
#define SR_IRQ_3_TRX_END	0x0f, 0x08, 3
#define SR_IRQ_4_CCA_ED_DONE	0x0f, 0x10, 4
#define SR_IRQ_5_AMI		0x0f, 0x20, 5
#define SR_IRQ_6_TRX_UR		0x0f, 0x40, 6
#define SR_IRQ_7_BAT_LOW	0x0f, 0x80, 7

#define RG_VREG_CTRL	(0x10)
#define SR_RESERVED_10_6	0x10, 0x03, 0
#define SR_DVDD_OK		0x10, 0x04, 2
#define SR_DVREG_EXT		0x10, 0x08, 3
#define SR_RESERVED_10_3	0x10, 0x30, 4
#define SR_AVDD_OK		0x10, 0x40, 6
#define SR_AVREG_EXT		0x10, 0x80, 7
#define RG_BATMON	(0x11)
#define SR_BATMON_VTH		0x11, 0x0f, 0
#define SR_BATMON_HR		0x11, 0x10, 4
#define SR_BATMON_OK		0x11, 0x20, 5
#define SR_RESERVED_11_1	0x11, 0xc0, 6
#define RG_XOSC_CTRL	(0x12)
#define SR_XTAL_TRIM		0x12, 0x0f, 0
#define SR_XTAL_MODE		0x12, 0xf0, 4
#define RG_RX_SYN	(0x15)
#define SR_RX_PDT_LEVEL		0x15, 0x0f, 0
#define SR_RESERVED_15_2	0x15, 0x70, 4
#define SR_RX_PDT_DIS		0x15, 0x80, 7
#define RG_XAH_CTRL_1	(0x17)
#define SR_RESERVED_17_8	0x17, 0x01, 0
#define SR_AACK_PROM_MODE	0x17, 0x02, 1
#define SR_AACK_ACK_TIME	0x17, 0x04, 2
#define SR_RESERVED_17_5	0x17, 0x08, 3
#define SR_AACK_UPLD_RES_FT	0x17, 0x10, 4
#define SR_AACK_FLTR_RES_FT	0x17, 0x20, 5
#define SR_CSMA_LBT_MODE	0x17, 0x40, 6
#define SR_RESERVED_17_1	0x17, 0x80, 7
#define RG_FTN_CTRL	(0x18)
#define SR_RESERVED_18_2	0x18, 0x7f, 0
#define SR_FTN_START		0x18, 0x80, 7
#define RG_PLL_CF	(0x1a)
#define SR_RESERVED_1a_2	0x1a, 0x7f, 0
#define SR_PLL_CF_START		0x1a, 0x80, 7
#define RG_PLL_DCU	(0x1b)
#define SR_RESERVED_1b_3	0x1b, 0x3f, 0
#define SR_RESERVED_1b_2	0x1b, 0x40, 6
#define SR_PLL_DCU_START	0x1b, 0x80, 7
//#define RG_PART_NUM	(0x1c)
//#define SR_PART_NUM		0x1c, 0xff, 0
//#define RG_VERSION_NUM	(0x1d)
//#define SR_VERSION_NUM		0x1d, 0xff, 0
//#define RG_MAN_ID_0	(0x1e)
//#define SR_MAN_ID_0		0x1e, 0xff, 0
//#define RG_MAN_ID_1	(0x1f)
//#define SR_MAN_ID_1		0x1f, 0xff, 0
#define RG_SHORT_ADDR_0	(0x20)
#define SR_SHORT_ADDR_0		0x20, 0xff, 0
#define RG_SHORT_ADDR_1	(0x21)
#define SR_SHORT_ADDR_1		0x21, 0xff, 0
//#define RG_PAN_ID_0	(0x22)
//#define SR_PAN_ID_0		0x22, 0xff, 0
//#define RG_PAN_ID_1	(0x23)
//#define SR_PAN_ID_1		0x23, 0xff, 0
#define RG_IEEE_ADDR_0	(0x24)
#define SR_IEEE_ADDR_0		0x24, 0xff, 0
#define RG_IEEE_ADDR_1	(0x25)
#define SR_IEEE_ADDR_1		0x25, 0xff, 0
#define RG_IEEE_ADDR_2	(0x26)
#define SR_IEEE_ADDR_2		0x26, 0xff, 0
#define RG_IEEE_ADDR_3	(0x27)
#define SR_IEEE_ADDR_3		0x27, 0xff, 0
#define RG_IEEE_ADDR_4	(0x28)
#define SR_IEEE_ADDR_4		0x28, 0xff, 0
#define RG_IEEE_ADDR_5	(0x29)
#define SR_IEEE_ADDR_5		0x29, 0xff, 0
#define RG_IEEE_ADDR_6	(0x2a)
#define SR_IEEE_ADDR_6		0x2a, 0xff, 0
#define RG_IEEE_ADDR_7	(0x2b)
#define SR_IEEE_ADDR_7		0x2b, 0xff, 0
#define RG_XAH_CTRL_0	(0x2c)
#define SR_SLOTTED_OPERATION	0x2c, 0x01, 0
#define SR_MAX_CSMA_RETRIES	0x2c, 0x0e, 1
#define SR_MAX_FRAME_RETRIES	0x2c, 0xf0, 4
#define RG_CSMA_SEED_0	(0x2d)
#define SR_CSMA_SEED_0		0x2d, 0xff, 0
#define RG_CSMA_SEED_1	(0x2e)
#define SR_CSMA_SEED_1		0x2e, 0x07, 0
#define SR_AACK_I_AM_COORD	0x2e, 0x08, 3
#define SR_AACK_DIS_ACK		0x2e, 0x10, 4
#define SR_AACK_SET_PD		0x2e, 0x20, 5
#define SR_AACK_FVN_MODE	0x2e, 0xc0, 6
#define RG_CSMA_BE	(0x2f)
#define SR_MIN_BE		0x2f, 0x0f, 0
#define SR_MAX_BE		0x2f, 0xf0, 4

//#define CMD_REG_MASK	0x3f
//#define CMD_WRITE	0x80
// These should be modified later.
#define CMD_FB		0x20
#define CMD_REG         0x80


#define IRQ_BAT_LOW	BIT(7)
#define IRQ_TRX_UR	BIT(6)
#define IRQ_AMI		BIT(5)
#define IRQ_CCA_ED	BIT(4)
#define IRQ_TRX_END	BIT(3)
#define IRQ_RX_START	BIT(2)
#define IRQ_PLL_UNL	BIT(1)
#define IRQ_PLL_LOCK	BIT(0)

#define IRQ_ACTIVE_HIGH	0
#define IRQ_ACTIVE_LOW	1

#define STATE_P_ON		0x00	/* BUSY */
#define STATE_BUSY_RX		0x01
#define STATE_BUSY_TX		0x02
#define STATE_FORCE_TRX_OFF	0x03
#define STATE_FORCE_TX_ON	0x04	/* IDLE */
/* 0x05 */				/* INVALID_PARAMETER */
#define STATE_RX_ON		0x06
/* 0x07 */				/* SUCCESS */
#define STATE_TRX_OFF		0x08
#define STATE_TX_ON		0x09
/* 0x0a - 0x0e */			/* 0x0a - UNSUPPORTED_ATTRIBUTE */
#define STATE_SLEEP		0x0F
#define STATE_PREP_DEEP_SLEEP	0x10
#define STATE_BUSY_RX_AACK	0x11
#define STATE_BUSY_TX_ARET	0x12
#define STATE_RX_AACK_ON	0x16
#define STATE_TX_ARET_ON	0x19
#define STATE_RX_ON_NOCLK	0x1C
#define STATE_RX_AACK_ON_NOCLK	0x1D
#define STATE_BUSY_RX_AACK_NOCLK 0x1E
#define STATE_TRANSITION_IN_PROGRESS 0x1F

//#define TRX_STATE_MASK		(0x1F)
#define TRAC_MASK(x)		((x & 0xe0) >> 5)

#define TRAC_SUCCESS			0
#define TRAC_SUCCESS_DATA_PENDING	1
#define TRAC_SUCCESS_WAIT_FOR_ACK	2
#define TRAC_CHANNEL_ACCESS_FAILURE	3
#define TRAC_NO_ACK			5
#define TRAC_INVALID			7








/* Custom registers added */

#define RG_RF_PN            (0x000d)
//#define SR_RF_PN             0x000d, 0xffff, 0
#define RG_RF_VN            (0x000e)
//#define SR_RF_VN             0x000e, 0xffff, 0

#define CMD_REG_MASK         0x3f
#define CMD_WRITE            0x80
#define CMD_READ             0x00
#define CMD_REG_MSB          0xff00
#define CMD_REG_LSB          0x00ff

/************** State machine registers *************/

/* STATUS registers:
 * Current state */
#define RG_RF09_CMD         (0x0103)

#define RF_NOP_STATUS        0X0
#define RF_SLEEP_STATUS      0X1
#define RF_TRXOFF_STATUS     0x2
#define RF_TXPREP_STATUS     0x3
#define RF_TX_STATUS         0x4
#define RF_RX_STATUS         0x5
#define RF_RESET_STATUS      0x7

/* STATE registers:
 * Which state we wanna reach */
#define RG_RF09_STATE      (0x0102)

#define STATE_RF_TRXOFF     0x2
#define STATE_RF_TXPREP     0x3
#define STATE_RF_TX         0x4
#define STATE_RF_RX         0x5
#define STATE_RF_TRANSITION 0x6
#define STATE_RF_RESET      0x7

/*************** IRQ registers ******************/

// Interrupt signalling
#define RG_RF_CFG          (0x0006)         //It contains bits to configure the IRQ behavior.
#define SR_RF_CFG_DRV          0x0006, 0x03, 0 //It configure the pads driver strength of the pins IRQ, MISO, and the frontend control (i.e. FEA09, FEB09, FEA24, FEB24) pins : 2, 4, 6, 8 mA.
#define SR_RF_CFG_IRQP      0x0006, 0x04, 2 //It configures the IRQ pin polarity. ( active high or low )
#define SR_RF_CFG_IRQMM     0x0006, 0x08, 3 //It configures the IRQ mask mode. 

// Radio mode : interruption registers
#define RG_RF09_IRQM    (0x0100)         //The register RFn_IRQM contains the radio IRQ mask.
//#define SR_RF09_IRQM     0x0100, 0xff, 0
#define RG_RF09_IRQS    (0x0000)         //A bit set to 1 indicates that the corresponding IRQ has occurred.
#define SR_IRQS_0_WAKEUP 0x0000, 0x01, 0 //Used when the procedure from state SLEEP/DEEP_SLEEP or power-up or RESET procedure is completed.
#define SR_IRQS_1_TRXRDY 0x0000, 0x02, 1 //Used when the command TXPREP is written to the register RFn_CMD and transceiver reaches the state TXPREP.
#define SR_IRQS_2_EDC    0x0000, 0x04, 2 //Used when a single or continuous energy measurement is completed. WARNING:It is not set if the automatic energy measurement mode is used.
#define SR_IRQS_3_BATLOW 0x0000, 0x08, 3 //When the battery monitor detects a voltage at EVDD that is below the threshold voltage.
#define SR_IRQS_4_TRXERR 0x0000, 0x10, 4 //When a transceiver error is detected.
#define SR_IRQS_5_IQIFSF 0x0000, 0x20, 5 //When the I/Q data interface synchronization fails.

#define IRQS_0_WAKEUP        BIT(0)
#define IRQS_1_TRXRDY        BIT(1)
#define IRQS_2_EDC           BIT(2)
#define IRQS_3_BATLOW        BIT(3)
#define IRQS_4_TRXERR        BIT(4)
#define IRQS_5_IQIFSF        BIT(5)

// Baseband mode : interruption registers
#define RG_BBC0_IRQM    (0x0300)                   //BBC0_IRQS contains the baseband IRQ status
//#define SR_BBC0_IRQM     0x0300, 0xff, 0
#define RG_BBC0_IRQS    (0x0002)
#define SR_IRQS_0_RXFS   0x0002, 0x01, 0           //This interrupt is issued if a valid PHY header is detected during frame receive.
#define SR_IRQS_1_RXFE   0x0002, 0x02, 1           //The IRQ RXFE is issued at the end of a successful frame reception.
#define SR_IRQS_2_RXAM   0x0002, 0x04, 2           //This interrupt occurs during frame receive if the Address Filter is enabled and if the received frame is recognized as matching.
#define SR_IRQS_3_RXEM   0x0002, 0x08, 3           //This interrupt occurs during frame receive if the Address Filter is enabled and if the received frame is recognized as extended.
#define SR_IRQS_4_TXFE   0x0002, 0x10, 4           //The IRQ_TXFE is issued when a frame is completely transmitted.
#define SR_IRQS_5_AGCH   0x0002, 0x20, 5           //The interrupt AGCH is issued during frame receive if a preamble of the selected PHY is detected.
#define SR_IRQS_6_AGCR   0x0002, 0x40, 6           //The interrupt AGCR is issued during frame receive if a receive process is finished.
#define SR_IRQS_7_FBLI   0x0002, 0x80, 7           //The interrupt FBLI can be used to monitor the receive frame buffer level.

#define IRQS_0_RXFS        BIT(0)
#define IRQS_1_RXFE        BIT(1)
#define IRQS_2_RXAM        BIT(2)
#define IRQS_3_RXEM        BIT(3)
#define IRQS_4_TXFE        BIT(4)
#define IRQS_5_AGCH        BIT(5)
#define IRQS_6_AGCR        BIT(6)
#define IRQS_7_FBLI        BIT(7)

/***************** Auto Mode: IEEE MAC Support ***************/

/** 1. Frame filter  **/
#define RG_BBC0_PC          (0x301)
#define SR_BBC0_PC_PT        0x301,0x03, 0 //It sets the PHY type (BB_PHYOFF, BB_MRFSK, BB_MROFDM, BB_MROQPSK)
#define SR_BBC0_PC_BBEN      0x301,0x04, 2 //It enables the baseband.
#define SR_BBC0_PC_FCST      0x301,0x08, 3 //It configures the used Frame Check Sequence Type. ( 2 or 4 butes ? )
#define SR_BBC0_PC_TXAFCS    0x301,0x10, 4 //It is set to 1 during transmission, the internal calculated FCS (type dependent of FCST) is inserted into the last 2 or 4 PSDU octets, respectively.
#define SR_BBC0_PC_FCSOK     0x301,0x20, 5 //It indicates whether the FCS of a detected frame is valid or not
#define SR_BBC0_PC_FCSFE     0x301,0x40, 6 //It configures the filter function of the FCS check. If it is set to 1, an IRQ RXFE occurs ONLY if the frame has a valid FCS.
#define SR_BBC0_PC_CTX       0x301,0x80, 7 //Continuous transmission mode

/** 2. AACK + From Tx to Rx + CCA **/
#define RG_BBC0_AMCS        (0x0440) // Auto Mode Configuration and Status
#define SR_BBC0_AMCS_TX2RX   0x0440, 0X01, 0 //The transceiver switches automatically to state RX if a transmit is completed.
#define SR_BBC0_AMCS_CCATX   0x0440, 0X02, 1 //CCA Measurement and automatic Transmit: If this bit is set to 1, the auto mode feature CCA with automatic transmit is enabled.
#define SR_BBC0_AMCS_CCAED   0x0440, 0X04, 2 //CCA Energy Detection Result: indicates the status of the result of the last CCA measurement. It is updated with "the finished ED measurement", while the procedure CCATX is active.
#define SR_BBC0_AMCS_AACK    0x0440, 0X08, 3 //Auto Acknowledgement: If this bit is set to 1, the automatic ACK feature is enabled.
#define SR_BBC0_AMCS_AACKS   0x0440, 0X10, 4 //Auto Acknowledgement Source: The automatick ACK is either sent by the transceiver respective to IEEE Std 802.15.4-2006 or from the transmit frame buffer.
#define SR_BBC0_AMCS_AACKDR  0x0440, 0X20, 5 //Auto Acknowledgement Data Rate: if set to 1, the automatic ACK is sent using the modulation settings of the received frame. If not, the automatic ACK is transmitted using the current PHY settings.
#define SR_BBC0_AMCS_AACKFA  0x0440, 0X40, 6 //Auto Acknowledgement FCS Adaption : if set to 1, the FCS type si derived from the FCS type of the received frame. Otherwise, from sub-register PC.FCST (Frame Check Sequence Type)
#define SR_BBC0_AMCS_AACKFT  0x0440, 0X80, 7 //Auto Acknowledgement Frame Transmit

#define RG_BBC0_AMEDT       (0x0441)         //Auto Mode Energy Detection Threshold:it contains the ED threshold for a CCA measurement. It is stored as a signed number in a range of [-127..128].
#define RG_BBC0_AMAACKPD    (0x0442)         //Auto Mode Automatic ACK Pending Data: This register configures the behaviour of the pending data bit of an automatic acknowledgement frame.
#define RG_BBC0_AMAACKTL    (0x0443)         //The transceiver switches automatically to state RX if a transmit is completed. (low byte)
#define RG_BBC0_AMAACKTH    (0x0444)         //The transceiver switches automatically to state RX if a transmit is completed. (high byte)


/********** RESET register **********/
#define RG_RST (0x0005) //Resetting the entire device via an SPI command. To do that, write 0x07 to this register.

/********* Clock Output ************/
#define RG_RF_CLKO    (0x0007)
#define SR_RF_CLKO_OS  0x0007, 0x07, 0 //It configures the clock output frequency of the CLKO output signal: 1 ==> 26 MHz , or 0 ofc.
#define SR_RF_CLKO_DRV 0x0007, 0x18, 3 //It configures the CLKO pad driver strength : 2 ==> 8 mA


/******** Serial I/Q Data Interface ********/

#define RG_IQIFC0        (0x000A)         //I/Q Data Interface Configuration Register 0
#define SR_IQIFC0_EEC     0x000A, 0x01, 0 //If this bit is set to 1, the transmit start and finish cannot be controlled by SPI commands.
#define SR_IQIFC0_CMV1V2  0x000A, 0x02, 1 //the common mode voltage of the I/Q data interface signals: 1 ==> 4mA
#define SR_IQIFC0_CMV     0x000A, 0x0C, 2 //The sub-register configures the common mode voltage of the I/Q data interface signals: 150mV ==> 300mV
#define SR_IQIFC0_DRV     0x000A, 0x30, 4 //The sub-register configures the I/Q data interface driver output current.
#define SR_IQIFC0_SF      0x000A, 0x40, 6 //he bit indicates whether the data stream of the I/Q data interface is synchronized correctly
#define SR_IQIFC0_EXTLB   0x000A, 0x80, 7 //The bit enables the external loopback functionality.

#define RG_IQIFC1        (0x000B)         //The register configures the skew behavior, the chip mode of the I/Q data interface and contains the status of the I/Q data interface receiver.
#define SR_IQIFC1_SKEWDRV 0x000B, 0x03, 0 //The register values define the time from the RXCLKn/p clock edge to the next RXDxxn/p signal edge.
#define SR_IQIFC1_CHPM    0x000B, 0x70, 4 //It configures the "working mode" of the chip and define which parts (RF, baseband, I/Q IF) are in operation.
#define SR_IQIFC1_FAILSF  0x000B, 0x80, 7 //This bit indicates that the LVDS receiver is in failsafe mode. The failsafe mode is entered if the LVDS receiver is not driven by an LVDS driver.

#define RG_IQIFC2        (0x000C)         //The register contains the status of the I/Q data interface deserializer.
#define SR_IQIFC2_SYNC    0x000C, 0X80, 7

/*******  Module Description *******/
/** 1) Transmitter Fronted **/
// Transmitter Filter Cutoff Control and PA Ramp Time
#define RG_RF09_TXCUTC            (0x0112)         //Transmitter filter and PA ramp time.
#define SR_RF09_TXCUTC_LPFCUT      0x0112, 0x0f, 0 //The sub-register configures the transmitter low pass filter cut-off frequency.
#define SR_RF09_TXCUTC_PARAMP      0x0112, 0xC0, 6 //Power Amplifier Ramp Time
//Transmitter TX Digital Frontend
#define RG_RF09_TXDFE             (0x0113)
#define SR_RF09_TXDFE_SR           0x0113, 0x0f, 0
#define SR_RF09_TXDFE_DM           0x0113, 0x10, 4
#define SR_RF09_TXDFE_RCUT         0x0113, 0xe0, 5
//Transmitter Power Amplifier Control
#define RG_RF09_PAC               (0x0114)
#define SR_RF09_PAC_TXPWR          0x0114, 0x1f, 0
#define SR_RF09_PAC_PACUR          0x0114, 0x60, 5
//Transceiver Auxiliary Settings
#define RG_RF09_AUXS              (0x0101)
#define SR_RF09_AUXS_PAVC          0x0101, 0x03, 0
#define SR_RF09_AUXS_AVS           0x0101, 0x04, 2
#define SR_RF09_AUXS_AVEN          0x0101, 0x08, 3
#define SR_RF09_AUXS_AVEXT         0x0101, 0x10, 4
#define SR_RF09_AUXS_AGCMAP        0x0101, 0x06, 5
#define SR_RF09_AUXS_EXTLNABYP     0x0101, 0x80, 7
//Transceiver I/Q Data Interface Configuration Register 0 (already defined : RG_IQIFC0)

/** 2) Receiver Fronted **/
//Receiver Filter Bandwidth Control
#define RG_RF09_RXBWC             (0x0109)
#define SR_RF09_RXBWC_BW           0X0109, 0x0f, 0
#define SR_RF09_RXBWC_IFS          0X0109, 0x10, 4
#define SR_RF09_RXBWC_IFI          0X0109, 0x20, 5
//Receiver Digital Frontend
#define RG_RF09_RXDFE             (0x010A)
#define SR_RF09_RXDFE_SR           0x010A, 0x0f,0
#define SR_RF09_RXDFE_RCUT         0x010A, 0xe0,5
//Receiver AGC Control 0
#define RG_RF09_AGCC
#define SR_RF09_AGCC_EN
#define SR_RF09_AGCC_FRZC
#define SR_RF09_AGCC_FRZS
#define SR_RF09_AGCC_RST
#define SR_RF09_AGCC_AVGS
#define SR_RF09_AGCC_AGCI
//Receiver AGCG
#define RG_RF09_AGCS
#define SR_RF09_AGCS_GCW
#define SR_RF09_AGCS_TGT
//Received Signal Strength Indicator
#define RG_RF09_RSSI
//Energy Detection Configuration
#define RG_RF09_EDC
//Receiver Energy Detection Averaging Duration
#define RG_RF09_EDD      (0x10f)
#define SR_RF09_EDD_DTB
#define SR_RF09_EDD_DF
//Receiver Energy Detection Value
#define RG_RF09_EDV
/** 3) Frequency Synthesizer (PLL) **/
//Channel Spacing
#define RG_RF09_CS       (0x0104)
//Channel Center Frequency F0 Low Byte
#define RG_RF09_CCF0L    (0x0105)
//Channel Center Frequency F0 High Byte
#define RG_RF09_CCF0H    (0x0106)
//Channel Number Low Byte
#define RG_RF09_CNL      (0x0107)
//Channel Mode and Channel Number High Bit
#define RG_RF09_CNM      (0x0108)
#define SR_RF09_CNM_CNH   0x0108, 0x01, 0
#define SR_RF09_CNM_CM    0x0108, 0xC0, 6
//Transceiver PLL
#define RG_RF09_PLL
#define SR_RF09_LS
#define SR_RF09_LBW
//PLL center frequency value
#define RG_RF_PLLCF
/** 4) Crystal Oscillator and TCXO **/
#define RG_RF_XOC
#define SR_RF_XOC_TRIM
#define SR_RF_XOC_FS
/** 5) External Frontend Control **/
//Transceiver Auxiliary Settings || Voltage Regulator
//(see (1) )
//External Frontend Control Pad Configuration
#define RG_RF09_PADFE
/** 7) Battery Monitor (BATMON) **/
#define RG_RF_BMDVC
#define SR_RF_BMDVC_BMVTH
#define SR_RF_BMDVC_BMHR
#define SR_RF_BMDVC_BMHR_BMS
/** 8) Analog Calibrations **/
/** 9) Baseband Core **/
/* See 1) Frame filter */
//These will be defined later
/** 10) MR-FSK PHY **/
/** 11) MR-OFDM PHY **/
#define RG_BBC0_OFDMPHRTX  (0x030C)
/** 12) O-QPSK PHY **/
/** 13) Frame Buffer **/
#define RG_BBC0_TXFLL      (0x0306)
#define RG_BBC0_TXFLH      (0x0307)
#define SR_BBC0_TXFLH       0x0307, 0x07, 0
#define RG_BBC0_FBTXS      (0x2800)
#define RG_BBC0_FBTXE	   (0x2FFE)
/** 14) Frame Check Sequence ( see frame filter ) **/
/** 15) IEEE MAC Support **/
#define RG_BBC0_AFC0       (0x320)
#define SR_BBC0_AFC0_PM     0x320, 0x10, 4
/** 16) Random Number Generator **/
/** 17) Phase Measurement Unit**/
/** 18) Timestamp Counter **/


#define RG_RF24_IRQM      (0x0200)
#define RG_EXAMPLE        (0x0117)
#define RG_EXAMPLE2       (0X02000)
#endif /* !_AT86RF230_H */

