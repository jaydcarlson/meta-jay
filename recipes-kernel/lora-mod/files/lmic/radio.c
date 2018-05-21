/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * Copyright (c) 2016 Solvz, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Adapted by Bruce Carlson, Signetik, LLC. for Solvz, Inc. 
 *******************************************************************************/

#include "lmic.h"
#include "radio.h"
#include <linux/slab.h>     // for GFP_ATOMIC
#include <linux/delay.h>    // for msleep

// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"



// ---------------------------------------- 
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F 
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11 
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12 
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13 
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14 
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15 
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16 
#define LORARegRxpacketCntValueLsb                 0x17 
#define LORARegModemStat                           0x18 
#define LORARegPktSnrValue                         0x19 
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A 
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B 
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C 
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D 
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E 
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F 
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20 
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21 
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22 
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23 
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24 
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define RegPaDac                                   0x5A // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  0x00
#define SX1272_MC2_SF7  0x70
#define SX1272_MC2_SF8  0x80
#define SX1272_MC2_SF9  0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125  0x00
#define SX1272_MC1_BW_250  0x40
#define SX1272_MC1_BW_500  0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON        0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01 
                                                    
// sx1276 RegModemConfig2          
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3          
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34
//#define LORA_MAC_PREAMBLE                  0x12     // DEBUG - TO TALK WITH THE SX1272 EVAL BOARDS

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif



// ---------------------------------------- 
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06 
#define OPMODE_CAD       0x07 

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

#define MAP_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP       0x30  // --11----
#define MAP_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT   0x08  // ----10--


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default


// RegModemStat bits:    
#define MODEMSTAT_MODEM_CLEAR   0x10
#define MODEMSTAT_HEADER_VALID  0x08
#define MODEMSTAT_RX_ONGOING    0x04
#define MODEMSTAT_SIGNAL_SYNC   0x02
#define MODEMSTAT_SIGNAL_DETECT 0x01


// pasted in some code from lmic.c:

#if defined(CFG_us915) // ========================================

#define maxFrameLen(dr) ((dr)<=DR_SF11CR ? maxFrameLens[(dr)] : 0xFF)
const u1_t maxFrameLens [] = { 24,66,142,255,255,255,255,255,  66,142 };

const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

#define pow2dBm(mcmd_ladr_p1) ((s1_t)(30 - (((mcmd_ladr_p1)&MCMD_LADR_POW_MASK)<<1)))

#endif // ================================================




// end code from lmic.c

// RADIO STATE
// (initialized by radio_init(), used by radio_rand1())
static u1_t randbuf[16];


#ifdef CFG_sx1276_radio
#define LNA_RX_GAIN (0x20|0x1)
#elif CFG_sx1272_radio
#define LNA_RX_GAIN (0x20|0x03)
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif



static void writeBuf (struct hal_radio *r, u1_t addr, xref2u1_t buf, u1_t len) {
    hal_spi_write(r, addr, buf, len);
}


static int writeReg (struct hal_radio *r, u1_t addr, u1_t data ) 
{
    return hal_spi_write(r, addr, &data, 1);
}

static u1_t readReg (struct hal_radio *r, u1_t addr) {
    u1_t val = 0;
    hal_spi_read(r, addr, &val, sizeof(val));
    return val;
}

/** Submit request to read 8-bit register asynchronously
    Function will return before read is complete.  *result will be written
    later.  *result is not valid until run_on_completion is called.
    Reads and writes are queued together and performed in the order receieved 
    into queue.
 */
static void readReg_async (struct hal_radio *r, u1_t addr, u1_t *result,
                            void (*run_on_completion)(struct hal_radio *, int)) 
{
    hal_spi_read_async(r, addr, result, 1, run_on_completion, GFP_ATOMIC);
}

/** Submit request to read buffer asynchronously
    Function will return before read is complete.  *buffer will be written
    later.  *buffer is not valid until run_on_completion is called.
    Reads and writes are queued together and performed in the order receieved 
    into queue.
 */
static void readBuf_async (struct hal_radio *r, u1_t addr, u1_t *buffer, u1_t len,
                            void (*run_on_completion)(struct hal_radio *, int)) 
{
    hal_spi_read_async(r, addr, buffer, len, run_on_completion, GFP_ATOMIC);
}

/** Submit request to write 8-bit register asynchronously
    Function will return before write is complete.  register will be written
    later.  *result is not valid until run_on_completion is called.
    Reads and writes are queued together and performed in the order receieved 
    into queue.
 */
static void writeReg_async (struct hal_radio *r, u1_t addr, u1_t new_value, 
                            void (*run_on_completion)(struct hal_radio *, int)) 
{
    hal_spi_write_async(r, addr, &new_value, sizeof(new_value), run_on_completion, GFP_ATOMIC);
}


static void opmode (struct hal_radio *r, u1_t mode) {
    writeReg(r, RegOpMode, (readReg(r, RegOpMode) & ~OPMODE_MASK) | mode);
}

static void opmodeLora(struct hal_radio *r) {
    u1_t u = OPMODE_LORA;
#ifdef CFG_sx1276_radio
    u |= 0x8;   // TBD: sx1276 high freq
#endif
    writeReg(r, RegOpMode, u);
}

////static void opmodeFSK(void) {
////    u1_t u = 0;
////#ifdef CFG_sx1276_radio
////    u |= 0x8;   // TBD: sx1276 high freq
////#endif
////    writeReg(r, RegOpMode, u);
////}

// configure LoRa modem (cfg1, cfg2)
static void configLoraModem (struct hal_radio *r) {
    sf_t sf = getSf(r->rps);

#ifdef CFG_sx1276_radio
#error SX1276 radio is no longer supported
////        u1_t mc1 = 0, mc2 = 0, mc3 = 0;
////
////        switch (getBw(r->rps)) {
////        case BW125: mc1 |= SX1276_MC1_BW_125; break;
////        case BW250: mc1 |= SX1276_MC1_BW_250; break;
////        case BW500: mc1 |= SX1276_MC1_BW_500; break;
////        default:
////            ASSERT(0);
////        }
////        switch( getCr(r->rps) ) {
////        case CR_4_5: mc1 |= SX1276_MC1_CR_4_5; break;
////        case CR_4_6: mc1 |= SX1276_MC1_CR_4_6; break;
////        case CR_4_7: mc1 |= SX1276_MC1_CR_4_7; break;
////        case CR_4_8: mc1 |= SX1276_MC1_CR_4_8; break;
////        default:
////            ASSERT(0);
////        }
////
////        if (getIh(r->rps)) {
////            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
////            writeReg(r, LORARegPayloadLength, getIh(r->rps)); // required length
////        }
////        // set ModemConfig1
////        writeReg(r, LORARegModemConfig1, mc1);
////
////        mc2 = (SX1272_MC2_SF7 + ((sf-1)<<4));
////        if (getNocrc(r->rps) == 0) {
////            mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
////        }
////        writeReg(r, LORARegModemConfig2, mc2);
////        
////        mc3 = SX1276_MC3_AGCAUTO;
////        if ((sf == SF11 || sf == SF12) && getBw(r->rps) == BW125) {
////            mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
////        }
////        writeReg(r, LORARegModemConfig3, mc3);
#elif CFG_sx1272_radio
        u1_t mc1 = (getBw(r->rps)<<6);
        switch (getBw(r->rps)) {
            case  0: printk(KERN_DEBUG "configLoraModem BW = 125 kHz\n"); break;
            case  1: printk(KERN_DEBUG "configLoraModem BW = 250 kHz\n"); break;
            case  2: printk(KERN_DEBUG "configLoraModem BW = 500 kHz\n"); break;
            default: printk(KERN_DEBUG "configLoraModem BW = ??? kHz\n"); break;
        }

        switch( getCr(r->rps) ) {
        case CR_4_5: mc1 |= SX1272_MC1_CR_4_5; printk(KERN_DEBUG "radio config CR_4_5\n"); break;
        case CR_4_6: mc1 |= SX1272_MC1_CR_4_6; printk(KERN_DEBUG "radio config CR_4_6\n"); break;
        case CR_4_7: mc1 |= SX1272_MC1_CR_4_7; printk(KERN_DEBUG "radio config CR_4_7\n"); break;
        case CR_4_8: mc1 |= SX1272_MC1_CR_4_8; printk(KERN_DEBUG "radio config CR_4_8\n"); break;
        default: printk(KERN_DEBUG "radio config CR unknown\n");
        }
        
        if ((sf == SF11 || sf == SF12) && getBw(r->rps) == BW125) {
            mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
        }

        // LoRaWAN uses explicit header mode, so this is not used:
//        if (getIh(r->rps)) {
//            mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
//            printk(KERN_INFO "configLoraModem Ih = %d\n", getIh(r->rps));
//            writeReg(r, LORARegPayloadLength, getIh(r->rps)); // required length - todo: I don't think this is used for receive (Datasheet 4.1.2.3)
//        }
        
        // With explicit headers, the PAYLOAD_CRCON bit is apparently only used
        //  by the transmitter (the receiver uses the explicit header instead).      
        // LoRaWAN downlink messages do not use payload CRC.
//        if (getNocrc(r->rps) == 0) {
//            printk(KERN_INFO "configLoraModem payload crc ON\n");
//            mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
//        }

        // set ModemConfig1
        printk(KERN_DEBUG "configLoraModem mc1 = %d\n", mc1);
        writeReg(r, LORARegModemConfig1, mc1);
        
        // set ModemConfig2 (sf, AgcAutoOn=1 SymbTimeoutHi=00)
        printk(KERN_DEBUG "configLoraModem sf = %d\n", 7 + sf-1);
        writeReg(r, LORARegModemConfig2, (SX1272_MC2_SF7 + ((sf-1)<<4)) | 0x04);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void configLoraModem_async (struct hal_radio *r) {
    sf_t sf = getSf(r->rps);

#ifdef CFG_sx1276_radio
#error SX1276 radio is no longer supported
#elif CFG_sx1272_radio
        u1_t mc1 = (getBw(r->rps)<<6);

        switch( getCr(r->rps) ) {
        case CR_4_5: mc1 |= SX1272_MC1_CR_4_5; break;
        case CR_4_6: mc1 |= SX1272_MC1_CR_4_6; break;
        case CR_4_7: mc1 |= SX1272_MC1_CR_4_7; break;
        case CR_4_8: mc1 |= SX1272_MC1_CR_4_8; break;
        }
        
        if ((sf == SF11 || sf == SF12) && getBw(r->rps) == BW125) {
            mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
        }
        
        if (getNocrc(r->rps) == 0) {
            mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
        }
        
        if (getIh(r->rps)) {
            mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
            writeReg_async(r, LORARegPayloadLength, getIh(r->rps), NULL); // required length
        }
        // set ModemConfig1
        writeReg_async(r, LORARegModemConfig1, mc1, NULL);
        
        // set ModemConfig2 (sf, AgcAutoOn=1 SymbTimeoutHi=00)
        writeReg_async(r, LORARegModemConfig2, (SX1272_MC2_SF7 + ((sf-1)<<4)) | 0x04, NULL);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}



static inline u4_t divide_64_by_32(u8_t x, u4_t base)
{
    do_div(x, base);        // !!!!!!!  do_div WRITES RESULT TO x !!!!!!  (it's magic!)
    if (x > 0xffffffff) {
        return 0xffffffff;
    }
    return x;
}

static void configChannel (struct hal_radio *r) {
    printk(KERN_DEBUG " configChannel frequency = %d\n", r->freq);
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u4_t frf = divide_64_by_32(((u8_t)r->freq << 19), 32000000);
    writeReg(r, RegFrfMsb, (u1_t)(frf>>16));
    writeReg(r, RegFrfMid, (u1_t)(frf>> 8));
    writeReg(r, RegFrfLsb, (u1_t)(frf>> 0));
}

static void configChannel_async (struct hal_radio *r) {
    printk(KERN_DEBUG " configChannel_async frequency = %u\n", r->freq);
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u4_t frf = divide_64_by_32(((u8_t)r->freq << 19), 32000000);
    writeReg_async(r, RegFrfMsb, (u1_t)(frf>>16), NULL);
    writeReg_async(r, RegFrfMid, (u1_t)(frf>> 8), NULL);
    writeReg_async(r, RegFrfLsb, (u1_t)(frf>> 0), NULL);
}


static void configPower (struct hal_radio *r) {
#ifdef CFG_sx1276_radio
////    // no boost used for now
////    s1_t pw = (s1_t)r->tx_power;
////    if(pw >= 17) {
////        pw = 15;
////    } else if(pw < 2) {
////        pw = 2;
////    }
////    // check board type for BOOST pin
////    writeReg(r, RegPaConfig, (u1_t)(0x80|(pw&0xf)));
////    writeReg(r, RegPaDac, readReg(r, RegPaDac)|0x4);
////
#elif CFG_sx1272_radio
    // set PA config (2-17 dBm using PA_BOOST)
    s1_t pw = (s1_t)r->tx_power;
    if(pw > 17) {
        pw = 17;
    } else if(pw < 2) {
        pw = 2;
    }
    // 0x80 here selects the PA_BOOST output pin : 
    writeReg(r, RegPaConfig, (u1_t)(0x80|(pw-2)));
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void txfsk (struct hal_radio *r) {
////    // select FSK modem (from sleep mode)
////    writeReg(r, RegOpMode, 0x10); // FSK, BT=0.5
////    ASSERT(readReg(r, RegOpMode) == 0x10);
////    // enter standby mode (required for FIFO loading))
////    opmode(r, OPMODE_STANDBY);
////    // set bitrate
////    writeReg(r, FSKRegBitrateMsb, 0x02); // 50kbps
////    writeReg(r, FSKRegBitrateLsb, 0x80);
////    // set frequency deviation
////    writeReg(r, FSKRegFdevMsb, 0x01); // +/- 25kHz
////    writeReg(r, FSKRegFdevLsb, 0x99);
////    // frame and packet handler settings
////    writeReg(r, FSKRegPreambleMsb, 0x00);
////    writeReg(r, FSKRegPreambleLsb, 0x05);
////    writeReg(r, FSKRegSyncConfig, 0x12);
////    writeReg(r, FSKRegPacketConfig1, 0xD0);
////    writeReg(r, FSKRegPacketConfig2, 0x40);
////    writeReg(r, FSKRegSyncValue1, 0xC1);
////    writeReg(r, FSKRegSyncValue2, 0x94);
////    writeReg(r, FSKRegSyncValue3, 0xC1);
////    // configure frequency
////    configChannel(r);
////    // configure output power
////    configPower(r);
////
////    // set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
////    writeReg(r, RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TXNOP);
////
////    // initialize the payload size and address pointers    
////    writeReg(r, FSKRegPayloadLength, r->tx_data_len+1); // (insert length byte into payload))
////
////    // download length byte and buffer to the radio FIFO
////    writeReg(r, RegFifo, r->tx_data_len);
////    writeBuf(r, RegFifo, r->tx_frame, r->tx_data_len);
////
////    // enable antenna switch for TX
////    hal_pin_rxtx(r, 1);
////    
////    // now we actually start the transmission
////    opmode(r, OPMODE_TX);
}

static void txlora (struct hal_radio *r) {
    // select LoRa modem (from sleep mode)
    //writeReg(r, RegOpMode, OPMODE_LORA);
    opmodeLora(r);
    ASSERT((readReg(r, RegOpMode) & OPMODE_LORA) != 0);

    // enter standby mode (required for FIFO loading))
    opmode(r, OPMODE_STANDBY);
    // configure LoRa modem (cfg1, cfg2)
    configLoraModem(r);
    // configure frequency
    configChannel(r);
    // configure output power
    writeReg(r, RegPaRamp, (readReg(r, RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    configPower(r);
    // set sync word
    writeReg(r, LORARegSyncWord, LORA_MAC_PREAMBLE);
    
    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(r, RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(r, LORARegIrqFlags, 0xFF);
    // mask all IRQs but TxDone
    writeReg(r, LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);
    // initialize the payload size and address pointers    
    writeReg(r, LORARegFifoTxBaseAddr, 0x00);
    writeReg(r, LORARegFifoAddrPtr, 0x00);
    writeReg(r, LORARegPayloadLength, r->tx_data_len);
       
    // download buffer to the radio FIFO
    writeBuf(r, RegFifo, r->tx_frame, r->tx_data_len);
    
    printk(KERN_DEBUG "txlora pre-transmit registers:\n");
    printk_lora_registers(r);
    
    // enable antenna switch for TX
    hal_pin_rxtx(r, 1);
    
    // now we actually start the transmission
    opmode(r, OPMODE_TX);
}

// start transmitter (buf=r->tx_frame, len=r->tx_data_len)
static void starttx (struct hal_radio *r) {
    u1_t opmode = readReg(r, RegOpMode);
    // This radio_is_transmitting check should not be necessary because 
    //  radio_lock should remain locked until previous transmission is done.
    bool radio_is_transmitting = 
        ((opmode & OPMODE_MASK) == OPMODE_TX);
    if (radio_is_transmitting) {
        printk(KERN_WARNING " radio op mode = %d\n", opmode);
    }
    ASSERT( ! radio_is_transmitting);
    if(getSf(r->rps) == FSK) { // FSK modem
        txfsk(r);
    } else { // LoRa modem
        txlora(r);
    }
    // the radio will go back to STANDBY mode as soon as the TX is finished
    // the corresponding IRQ will inform us about completion.
}

enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static const u1_t rxlorairqmask[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK,
    [RXMODE_RSSI]   = 0x00,
};

void printk_lora_registers(struct hal_radio *r)
{
    int i;
    for (i=0; i<64; i+=8) {
        printk(KERN_DEBUG "radio registers[0x%02x] = %02x %02x %02x %02x %02x %02x %02x %02x \n", i, readReg(r, i + 0), readReg(r, i + 1), readReg(r, i + 2), readReg(r, i + 3), readReg(r, i + 4), readReg(r, i + 5), readReg(r, i + 6), readReg(r, i + 7));
    }
}

// start LoRa receiver (result=r->rx_packet)
static void rxlora (struct hal_radio *r, u1_t rxmode) {
    // select LoRa modem (from sleep mode)
    opmodeLora(r);
    ASSERT((readReg(r, RegOpMode) & OPMODE_LORA) != 0);
    // enter standby mode (warm up))
    opmode(r, OPMODE_STANDBY);
    // don't use MAC settings at startup
    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
        writeReg(r, LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
        writeReg(r, LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    } else { // single or continuous rx mode
        // configure LoRa modem (cfg1, cfg2)
        configLoraModem(r);
        // configure frequency
        configChannel(r);
    }
    // set LNA gain
    writeReg(r, RegLna, LNA_RX_GAIN); 
    // set max payload size
    writeReg(r, LORARegPayloadMaxLength, 64);
    // Leave LORARegInvertIQ alone to avoid strange problems.
    // set symbol timeout (for single rx)
//    writeReg(r, LORARegSymbTimeoutLsb, LMIC.rxsyms);
    // set sync word
    printk(KERN_DEBUG " rxlora SyncWord = %8d\n", LORA_MAC_PREAMBLE);
    writeReg(r, LORARegSyncWord, LORA_MAC_PREAMBLE);
    
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    writeReg(r, RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(r, LORARegIrqFlags, 0xFF);
    // enable required radio IRQs
    writeReg(r, LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]);

    printk_lora_registers(r);
    
    // enable antenna switch for RX
    hal_pin_rxtx(r, 0);

    // now instruct the radio to receive
//    if (rxmode == RXMODE_SINGLE) { // single rx
//        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
//        opmode(r, OPMODE_RX_SINGLE);
//    } else { // continous rx (scan or rssi)
        opmode(r, OPMODE_RX); 
//    }
}

/** Start LoRa receiver (result=r->rx_packet)
    Expect antenna switch to be set to receive as soon as this is called.
 */
static void rxlora_async (struct hal_radio *r, u1_t rxmode) {
    // select LoRa modem (from sleep mode)
    u1_t opmode = OPMODE_LORA;
#ifdef CFG_sx1276_radio
    opmode |= 0x8;   // TBD: sx1276 high freq
#endif
    writeReg_async(r, RegOpMode, opmode, NULL);
    
    // enter standby mode (warm up))
    opmode |= OPMODE_STANDBY;
    writeReg_async(r, RegOpMode, opmode, NULL);
    // don't use MAC settings at startup
    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
        writeReg_async(r, LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1, NULL);
        writeReg_async(r, LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2, NULL);
    } else { // single or continuous rx mode
        // configure LoRa modem (cfg1, cfg2)
        configLoraModem_async(r);
        // configure frequency
        configChannel_async(r);
    }
    // set LNA gain
    writeReg_async(r, RegLna, LNA_RX_GAIN, NULL); 
    // set max payload size
    writeReg_async(r, LORARegPayloadMaxLength, 64, NULL);
    // Leave LORARegInvertIQ alone to avoid strange problems.
    // set symbol timeout (for single rx)
//    writeReg_async(r, LORARegSymbTimeoutLsb, LMIC.rxsyms, NULL);
    // set sync word
    writeReg_async(r, LORARegSyncWord, LORA_MAC_PREAMBLE, NULL);
    
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    writeReg_async(r, RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP, NULL);
    // clear all radio IRQ flags
    writeReg_async(r, LORARegIrqFlags, 0xFF, NULL);
    // enable required radio IRQs
    writeReg_async(r, LORARegIrqFlagsMask, ~rxlorairqmask[rxmode], NULL);

    // enable antenna switch for RX
    // Note that because the register writes are _async, this antenna switch
    // will take effect before the register writes.  That seems ok because
    //  caller should have waited for tx complete before calling us.
    hal_pin_rxtx(r, 0);

    // now instruct the radio to receive
//    if (rxmode == RXMODE_SINGLE) { // single rx
//        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
//        opmode(r, OPMODE_RX_SINGLE);
//    } else { // continous rx (scan or rssi)
    opmode = (opmode & ~OPMODE_MASK) | OPMODE_RX;
    writeReg_async(r, RegOpMode, opmode, NULL);
//    }
}

static void rxfsk (struct hal_radio *r, u1_t rxmode) {
////    // only single rx (no continuous scanning, no noise sampling)
////    ASSERT( rxmode == RXMODE_SINGLE );
////    // select FSK modem (from sleep mode)
////    //writeReg(r, RegOpMode, 0x00); // (not LoRa)
////    opmodeFSK();
////    ASSERT((readReg(r, RegOpMode) & OPMODE_LORA) == 0);
////    // enter standby mode (warm up))
////    opmode(r, OPMODE_STANDBY);
////    // configure frequency
////    configChannel(r);
////    // set LNA gain
////    //writeReg(r, RegLna, 0x20|0x03); // max gain, boost enable
////    writeReg(r, RegLna, LNA_RX_GAIN);
////    // configure receiver
////    writeReg(r, FSKRegRxConfig, 0x1E); // AFC auto, AGC, trigger on preamble?!?
////    // set receiver bandwidth
////    writeReg(r, FSKRegRxBw, 0x0B); // 50kHz SSb
////    // set AFC bandwidth
////    writeReg(r, FSKRegAfcBw, 0x12); // 83.3kHz SSB
////    // set preamble detection
////    writeReg(r, FSKRegPreambleDetect, 0xAA); // enable, 2 bytes, 10 chip errors
////    // set sync config
////    writeReg(r, FSKRegSyncConfig, 0x12); // no auto restart, preamble 0xAA, enable, fill FIFO, 3 bytes sync
////    // set packet config
////    writeReg(r, FSKRegPacketConfig1, 0xD8); // var-length, whitening, crc, no auto-clear, no adr filter
////    writeReg(r, FSKRegPacketConfig2, 0x40); // packet mode
////    // set sync value
////    writeReg(r, FSKRegSyncValue1, 0xC1);
////    writeReg(r, FSKRegSyncValue2, 0x94);
////    writeReg(r, FSKRegSyncValue3, 0xC1);
////    // set preamble timeout
////    writeReg(r, FSKRegRxTimeout2, 0xFF);//(LMIC.rxsyms+1)/2);
////    // set bitrate
////    writeReg(r, FSKRegBitrateMsb, 0x02); // 50kbps
////    writeReg(r, FSKRegBitrateLsb, 0x80);
////    // set frequency deviation
////    writeReg(r, FSKRegFdevMsb, 0x01); // +/- 25kHz
////    writeReg(r, FSKRegFdevLsb, 0x99);
////    
////    // configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
////    writeReg(r, RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TIMEOUT);
////
////    // enable antenna switch for RX
////    hal_pin_rxtx(r, 0);
////    
////    // now instruct the radio to receive
////    hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
////    opmode(r, OPMODE_RX); // no single rx mode available in FSK
}

static void startrx (struct hal_radio *r, u1_t rxmode) {
    ASSERT( (readReg(r, RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    if(getSf(r->rps) == FSK) { // FSK modem
        rxfsk(r, rxmode);
    } else { // LoRa modem
        rxlora(r, rxmode);
    }
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}


static void startrx_async (struct hal_radio *r, u1_t rxmode) {
    if(getSf(r->rps) == FSK) { // FSK modem
        printk(KERN_WARNING __FILE__ " startrx_async : rxfsk_async not supported\n");
        rxfsk(r, rxmode);
    } else { // LoRa modem
        rxlora_async(r, rxmode);
    }
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}


static void radio_reset(struct hal_radio *r) 
{
    printk(KERN_DEBUG "radio_reset\n");
    // manually reset radio
#ifdef CFG_sx1276_radio
    hal_pin_rst(r, 0); // drive RST pin low
#else
    hal_pin_rst(r, 1); // drive RST pin high
#endif
    hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us
    hal_pin_rst(r, 2); // configure RST pin floating!
    hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

    // Radio does not start up in LoRa mode.  
    // Switching to LoRa mode makes LoRa registers available.
    opmodeLora(r);  
    opmode(r, OPMODE_SLEEP);
    
    // set up any radio registers that need to be set up:
    
    // Writing to LORARegInvertIQ causes strange problems including 
    //  transmitter not transmitting or transmitting 60 dB attenuated..  
    // (this may have been due to previously reading original_InvertIQ when
    //  not in LoRa mode).
    // Verified: InvertIQ affects RX, but not TX.
    // When InvertIQ bit is set, this radio does not receive from another 
    // SX1272, but it can still be heard by another SX1272.
    // use non-inverted I/Q signal for our gateway so we can receive traffic from other SX1272s.
    writeReg(r, LORARegInvertIQ, readReg(r, LORARegInvertIQ) & ~(1<<6));
}

// Init radio
// Get random seed from wideband noise rssi
// note that radio_reset will happen after this, so any radio settings should 
// be done there instead of here.
void radio_init (struct hal_radio *r) {
    printk(KERN_DEBUG "radio_init\n");
    
    hal_radio_lock(r);
    
    r->last_rx_datarate = -1;
    r->last_rx_channel = -1;
    r->rx_handler = NULL;
    
    // Should we set up any extra channels? (see LMIC_setupChannel)

    radio_reset(r);

    opmode(r, OPMODE_SLEEP);

    // some sanity checks, e.g., read version number
    u1_t v = readReg(r, RegVersion);
#ifdef CFG_sx1276_radio
    ASSERT(v == 0x12 ); 
#elif CFG_sx1272_radio
    ASSERT(v == 0x22);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif

    // seed 15-byte randomness via noise rssi
    rxlora(r, RXMODE_RSSI);
    while( (readReg(r, RegOpMode) & OPMODE_MASK) != OPMODE_RX ); // continuous rx
    int i;
    for(i=1; i<16; i++) {
        int j;
        for(j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(r, LORARegRssiWideband) & 0x01) == (readReg(r, LORARegRssiWideband) & 0x01) );
            randbuf[i] = (randbuf[i] << 1) | b;
        }
    }
    randbuf[0] = 16; // set initial index
  
#ifdef CFG_sx1276mb1_board
////    // chain calibration
////    writeReg(r, RegPaConfig, 0);
////    
////    // Launch Rx chain calibration for LF band
////    writeReg(r, FSKRegImageCal, (readReg(r, FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
////    while((readReg(r, FSKRegImageCal)&RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING){ ; }
////
////    // Sets a Frequency in HF band
////    u4_t frf = 868000000;
////    writeReg(r, RegFrfMsb, (u1_t)(frf>>16));
////    writeReg(r, RegFrfMid, (u1_t)(frf>> 8));
////    writeReg(r, RegFrfLsb, (u1_t)(frf>> 0));
////
////    // Launch Rx chain calibration for HF band 
////    writeReg(r, FSKRegImageCal, (readReg(r, FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
////    while((readReg(r, FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) { ; }
#endif /* CFG_sx1276mb1_board */

    opmodeLora(r);  
    opmode(r, OPMODE_SLEEP);

    r->original_InvertIQ = readReg(r, LORARegInvertIQ);     // save this for later
    
    hal_radio_unlock(r);
}

void radio_shutdown(struct hal_radio *r)
{
    msleep(100);                        // allow a pending packet to be transmitted
    hal_radio_lock(r);
    radio_reset(r);
    opmode(r, OPMODE_SLEEP);
    hal_radio_unlock(r);
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t radio_rand1 (void) {
    u1_t i = randbuf[0];
    ASSERT( i != 0 );
    if( i==16 ) {
        os_aes(AES_ENC, randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = randbuf[i++];
    randbuf[0] = i;
    return v;
}

u1_t radio_rssi (struct hal_radio *r) {
    hal_radio_lock(r);
    u1_t x = readReg(r, LORARegRssiValue);
    hal_radio_unlock(r);
    return x;
}

static const u2_t LORA_RXDONE_FIXUP[] = {
    [FSK]  =     us2osticks(0), // (   0 ticks)
    [SF7]  =     us2osticks(0), // (   0 ticks)
    [SF8]  =  us2osticks(1648), // (  54 ticks)
    [SF9]  =  us2osticks(3265), // ( 107 ticks)
    [SF10] =  us2osticks(7049), // ( 231 ticks)
    [SF11] = us2osticks(13641), // ( 447 ticks)
    [SF12] = us2osticks(31189), // (1022 ticks)
};

static inline int smaller(int a, int b)
{
    return (a < b) ? a : b;
}


#define NUM_UPSTREAM_125KHZ_CHANNELS    64
#define NUM_UPSTREAM_500KHZ_CHANNELS     8
#define NUM_DOWNSTREAM_500KHZ_CHANNELS   8
// adapted function from lmic.c updateTx():
/** Map channel number to frequency
 *  @param channel:
              0 - 63 : upstream 125 kHz channels starting from US915_125kHz_UPFBASE 
                                               with spacing of US915_125kHz_UPFSTEP
             64 - 71 : upstream 500 kHz channels starting from US915_500kHz_UPFBASE
                                               with spacing of US915_500kHz_UPFSTEP
             72 - 79 : downstream 500 kHz channels starting from US915_500kHz_DNFBASE
                                                 with spacing of US915_500kHz_DNFSTEP
             80+ : (unsupported) extra channels with frequencies per LMIC.xchFreq[chnl-72];
 *  @return 0 on success
 */
static int get_channel_freq (struct hal_radio *r, uint8_t channel) {
    int x = channel;
    if( x < NUM_UPSTREAM_125KHZ_CHANNELS ) {    // up-stream 125 kHz channels
        r->freq = US915_125kHz_UPFBASE + x*US915_125kHz_UPFSTEP;
        return 0;
    }
    x -= NUM_UPSTREAM_125KHZ_CHANNELS;
    
    if( x < NUM_UPSTREAM_500KHZ_CHANNELS ) {  // up-stream 500 kHz channels
        r->freq = US915_500kHz_UPFBASE + x*US915_500kHz_UPFSTEP;
        return 0;
    }
    x -= NUM_UPSTREAM_500KHZ_CHANNELS;
    
    if( x < NUM_DOWNSTREAM_500KHZ_CHANNELS) { // down-stream 500 kHz channels
        r->freq = US915_500kHz_DNFBASE + x*US915_500kHz_DNFSTEP;
        return 0;
    }
    x -= NUM_DOWNSTREAM_500KHZ_CHANNELS;
    
// These lines are disabled because extra channels are currently not supported:
//    if( channel < MAX_XCHANNELS ) {
//        r->freq = LMIC.xchFreq[x];      
    printk(KERN_WARNING "unsupported radio channel %d\n", channel);
    r->freq = 0; 
    return -EINVAL;
}

///** Map channel number to bandwidth (in kHz)
// *  @param channel - see comments above in get_channel_freq()
// *  @return channel bandwidth in kHz, or 0 for invalid channel
// */
///static int get_channel_bw (uint8_t channel) {
///    int x = channel;
///    if( x < NUM_UPSTREAM_125KHZ_CHANNELS ) {    // up-stream 125 kHz channels
///        return 125;
///    }
///    x -= NUM_UPSTREAM_125KHZ_CHANNELS;
///    
///    if( x < NUM_UPSTREAM_500KHZ_CHANNELS ) {  // up-stream 500 kHz channels
///        return 500;
///    }
///    x -= NUM_UPSTREAM_500KHZ_CHANNELS;
///    
///    if( x < NUM_DOWNSTREAM_500KHZ_CHANNELS) { // down-stream 500 kHz channels
///        return 500;
///    }
///    x -= NUM_DOWNSTREAM_500KHZ_CHANNELS;
///    
///// These lines are disabled because extra channels are currently not supported:
/////    if( channel < MAX_XCHANNELS ) {
/////        r->freq = LMIC.xchFreq[x];      
///    return 0;
///}


// adapted function from lmic.c updateTx():
static uint8_t radio_power(uint8_t channel, uint8_t requested_tx_power)
{
    if( channel < NUM_UPSTREAM_125KHZ_CHANNELS ) {
        return smaller(30, requested_tx_power);
    } else {
        return smaller(26, requested_tx_power);
    }
}



void radio_resume_receiving_async(struct hal_radio *r)
{
    // note that rxlora() sets LORARegPayloadMaxLength to 64 bytes.  
    
    if ((r->last_rx_datarate < 0) || (r->last_rx_channel < 0)) {
        return; // no valid last data rate and channel
    }
 
    // This is a bit of a mess because it seems like the channel bandwidth
    //  really ought to be determined from the channel.  But LMIC allows the
    //  channel bandwidth to be set independently of channel.  So the user
    //  must be careful to select channel+bandwidth that follow the LoRaWAN 
    //  spec.
    //  
    // rps sets LORA mode, Spreading factor, bandwidth, coding rate, etc.
    // I think "up" is for the gateway when receiving
    r->rps = updr2rps(r->last_rx_datarate);
    int freq_result = get_channel_freq(r, r->last_rx_channel); 
    if (freq_result < 0) {  // channel is invalid:
        // leave the radio idle
        return;
    }
    
    startrx_async(r, RXMODE_SCAN); // buf=rx_packet
}

void radio_start_receiving(struct hal_radio *r, int channel, dr_t data_rate)
{
    printk(KERN_DEBUG "radio_start_receiving\n");
    r->last_rx_datarate = data_rate;
    r->last_rx_channel = channel;
    
    // note that rxlora() sets LORARegPayloadMaxLength to 64 bytes.  
    
    if ((r->last_rx_datarate < 0) || (r->last_rx_channel < 0)) {
        return; // no valid last data rate and channel
    }
 
    // rps sets LORA mode, Spreading factor, bandwidth, coding rate, etc.
    // I think "up" is for the gateway when receiving
    r->rps = updr2rps(r->last_rx_datarate);
    int freq_result = get_channel_freq(r, r->last_rx_channel); 
    if (freq_result < 0) {  // channel is invalid:
        // leave the radio idle
        return;
    }

    os_radio(r, RADIO_RXON);   // RXON rather than RX.  RX receives a single packet if before time out.
}


// These sub-parts of the radio irq handler call each other often via spi 
// completion call-backs:
// This complexity avoids blocking while waiting for spi transactions.

void handle_radio_irq_get_lock(void *radio, unsigned long interrupt_time);
void handle_radio_irq_rx_or_tx(struct hal_radio *r, int prev_radio_msg_status);
void handle_radio_irq_receive(struct hal_radio *r, int prev_radio_msg_status);
void handle_radio_irq_got_packet(struct hal_radio *r, int prev_radio_msg_status);
void handle_radio_irq_finish(struct hal_radio *r);
void handle_radio_irq_unlock(struct hal_radio *r, int prev_radio_msg_status);


// called by hal ext IRQ handler
// (radio goes back to receive to standby mode after tx/rx operations - put it back receiving)
// This assumes the radio IRQ is edge-triggered.  This is necessary because
// this function does not always clear the radio interrupt flag immediately.
void radio_irq_handler(struct hal_radio *r)
{
    ostime_t now = os_getTime();
    
    handle_radio_irq_get_lock( (void *) r, now);
}

/**
    interrupt context - no sleeping
    @param radio is to be cast to a struct hal_radio *
 */
void handle_radio_irq_get_lock(void *radio, unsigned long interrupt_time)
{
    struct hal_radio *r;
    r = (struct hal_radio *) radio;
    int got_radio_lock = hal_radio_try_lock(r);     
    if (got_radio_lock) {
        // Radio lock has just been acquired - we must release when done with radio.
        // We are also depending on the radio lock to assure only one thread 
        //  is accessing r->irq_handler_data 
        memset(&r->irq_handler_data, 0, sizeof(r->irq_handler_data));
        r->irq_handler_data.interrupt_time = interrupt_time;
        readReg_async(r, LORARegIrqFlags, &r->irq_handler_data.irq_flags,
                      &handle_radio_irq_rx_or_tx);
        // handle_radio_irq_rx_or_tx will be called when the register read is done..
    } else {
        printk(KERN_INFO "handle_radio_irq_get_lock deferring until radio becomes available.\n");
        // call self again later when lock becomes available.
        hal_radio_do_after_unlock(r, handle_radio_irq_get_lock, interrupt_time);
    }
}

/**
    interrupt context - no sleeping
    Now the LORARegIrqFlags has been read into r->irq_handler_data.irq_flags
 */
void handle_radio_irq_rx_or_tx(struct hal_radio *r, int prev_radio_msg_status)
{
    if (prev_radio_msg_status < 0) {
        r->irq_handler_data.irq_flags = 0;  // error reading irq_flags.  clear flags.
        // (errors statistics are counted by hal)
    }
    if( r->irq_handler_data.irq_flags & IRQ_LORA_TXDONE_MASK ) {
        // Since we are a gateway I don't think we need to know when the 
        // transmission completed. but if we do, here is where we could handle 
        // it.  It is in r->irq_handler_data.interrupt_time.
//        r->txend = now - us2osticks(43); // TXDONE FIXUP
        handle_radio_irq_finish(r);
    } else if( r->irq_handler_data.irq_flags & IRQ_LORA_RXDONE_MASK ) {
        // save exact rx time
        if(getBw(r->rps) == BW125) {
            r->irq_handler_data.interrupt_time -= LORA_RXDONE_FIXUP[getSf(r->rps)];
        }

        readReg_async(r, LORARegModemConfig1     , &r->irq_handler_data.ModemConfig1     , NULL);
        readReg_async(r, LORARegPayloadLength    , &r->irq_handler_data.PayloadLength    , NULL);
        readReg_async(r, LORARegRxNbBytes        , &r->irq_handler_data.RxNbBytes        , NULL);
        readReg_async(r, LORARegPktSnrValue      , &r->irq_handler_data.PktSnrValue      , NULL);
        readReg_async(r, LORARegPktRssiValue     , &r->irq_handler_data.PktRssiValue     , NULL);
        readReg_async(r, LORARegFifoRxCurrentAddr, &r->irq_handler_data.FifoRxCurrentAddr, 
                      &handle_radio_irq_receive);  
    } else {
        handle_radio_irq_finish(r);
    }
}

/**
    interrupt context - no sleeping
    Now the LORARegs above have been read into r->irq_handler_data.
 */
void handle_radio_irq_receive(struct hal_radio *r, int prev_radio_msg_status)
{
    int data_len = 0;
    if (prev_radio_msg_status < 0) {
        r->rx_packet.status = prev_radio_msg_status;
        // leave data_len == 0
    } else {
        data_len = (r->irq_handler_data.ModemConfig1 & SX1272_MC1_IMPLICIT_HEADER_MODE_ON) ?
            r->irq_handler_data.PayloadLength : r->irq_handler_data.RxNbBytes;
        data_len = smaller(data_len, HAL_MAX_FRAME_LEN);
    }
    // SX1272 datasheet says the FIFO is automatically cleared on transition to receive
    // so we shouldn't need to clear it here, even if we can't read it all into buffer.
    memset(&(r->rx_packet), 0, sizeof(r->rx_packet));
    r->rx_packet.channel = r->last_rx_channel;
    r->rx_packet.data_rate = r->last_rx_datarate;
    r->rx_packet.SNR  = r->irq_handler_data.PktSnrValue;             // SNR [dB] * 4
    r->rx_packet.RSSI = r->irq_handler_data.PktRssiValue - 125 + 64; // RSSI [dBm] (-196...+63)
    r->rx_packet.arrival_time = r->irq_handler_data.interrupt_time;
    r->rx_packet.data_bytes = data_len;
    // read the PDU and inform the MAC that we received something
    // set FIFO read address pointer
    writeReg_async(r, LORARegFifoAddrPtr, r->irq_handler_data.FifoRxCurrentAddr, NULL); 
    // now read the FIFO
    readBuf_async(r, RegFifo, r->rx_packet.data, data_len,
                  &handle_radio_irq_got_packet);
}

/**
    interrupt context - no sleeping
    Now the received packet has just been read into rx_packet_data
 */
void handle_radio_irq_got_packet(struct hal_radio *r, int prev_radio_msg_status)
{
    if (prev_radio_msg_status < 0) {    // error reading packet.
        r->rx_packet.status = prev_radio_msg_status;
        r->rx_packet.data_bytes = 0;  
    }
    if (r->rx_handler != NULL) {
        r->rx_handler(r->rx_handler_data, &r->rx_packet);
    }
    handle_radio_irq_finish(r);
}

void handle_radio_irq_finish(struct hal_radio *r)
{
    // mask all radio IRQs
    writeReg_async(r, LORARegIrqFlagsMask, 0xFF, NULL);
    // clear radio IRQ flags
    writeReg_async(r, LORARegIrqFlags, 0xFF, NULL);
    
    radio_resume_receiving_async(r);
    // Now wait until all these SPI transfers are done before unlocking radio.
    // Do a dummy read here only so we get the completion callback when all SPI
    // transfers are complete.
    readReg_async(r, RegOpMode, &r->irq_handler_data.scratch, handle_radio_irq_unlock);
}

void handle_radio_irq_unlock(struct hal_radio *r, int prev_radio_msg_status)
{
    hal_radio_unlock(r);
}


void radio_set_rx_handler(struct hal_radio *r, 
                          void (*rx_handler)(void *, struct hal_radio_packet *),
                          void *rx_handler_data)
{
    r->rx_handler = rx_handler;
    r->rx_handler_data = rx_handler_data;
}

void os_radio (struct hal_radio *r, u1_t mode) {
    hal_radio_lock(r);
    switch (mode) {
      case RADIO_RST:
        // put radio to sleep
        opmode(r, OPMODE_SLEEP);
        break;

      case RADIO_TX:
        // transmit frame now
        starttx(r); // buf=r->tx_frame, len=r->tx_data_len
        break;
      
//      case RADIO_RX:
//        // receive frame now (exactly at rxtime)
//        startrx(r, RXMODE_SINGLE); // buf=r->rx_packet, time=LMIC.rxtime, timeout=LMIC.rxsyms
//        break;
//
      case RADIO_RXON:
        // start scanning for beacon now
        startrx(r, RXMODE_SCAN); // buf=r->rx_packet
        break;
    }
    hal_radio_unlock(r);
}


int radio_get_num_channels(void)
{
    return NUM_UPSTREAM_125KHZ_CHANNELS 
         + NUM_UPSTREAM_500KHZ_CHANNELS 
         + NUM_DOWNSTREAM_500KHZ_CHANNELS 
         + MAX_XCHANNELS;
}

/** Are we in the middle of transmitting a packet? 
 */
static bool radio_is_transmitting_lock(struct hal_radio *r)
{
    hal_radio_lock(r);
    u1_t opmode = readReg(r, RegOpMode);
    hal_radio_unlock(r);
    return ((opmode & OPMODE_MASK) == OPMODE_TX);
}
/** Are we in the middle of receiving a packet? 
 */
static bool radio_is_receiving_lock(struct hal_radio *r)
{
    hal_radio_lock(r);
    u1_t status = readReg(r, LORARegModemStat);
    hal_radio_unlock(r);
    
    // The MODEMSTAT_RX_ONGOING flag seems to be set whenever the radio is in 
    //  receive mode, whether or not it is actually receiving anything.
    // The MODEMSTAT_MODEM_CLEAR flag seems to be cleared when the radio is in
    //  receive mode.
    
    printk(KERN_DEBUG "radio_is_receiving status = 0x%02x, rx = %d\n", status,    
            (status & MODEMSTAT_SIGNAL_DETECT) ||
            (status & MODEMSTAT_SIGNAL_SYNC));
    return ((status & MODEMSTAT_SIGNAL_DETECT) ||
            (status & MODEMSTAT_SIGNAL_SYNC));
}

int radio_tx(struct hal_radio *r, int channel, dr_t data_rate, s1_t tx_power, void *data, int len)
{
    // Before switching the antenna switch to the transmitter we must wait until
    // we have the radio.  This is effected by the blocking communication here.
    // if this code ever changes to asynchronous spi communication then we will
    // need to be more explicit with the antenna switch timing.
    
    if (radio_is_transmitting_lock(r) || radio_is_receiving_lock(r)) {
        return -EBUSY;
    }
    if (len > sizeof(r->tx_frame)) {
        return -E2BIG;
    }
    memcpy(&r->tx_frame, data, len);
    r->tx_data_len = len;
    
    // Setting rps sets LORA, Spreading factor, bandwidth, coding rate, etc.
    // These settings are defined in lorabase.h
    // I think "dn" is for the gateway when transmitting
    r->rps = dndr2rps(data_rate);
    int freq_result = get_channel_freq(r, channel); 
    if (freq_result < 0) {  // channel is invalid:
        return freq_result;
    }

    r->tx_power = radio_power(channel, tx_power);             // set frequency and power
    os_radio(r, RADIO_TX);
    return 0;
}


/** Can we talk to the radio?
    @return true if radio detected
    Assumes the radio is not initialized.
 */
bool radio_detect (struct hal_radio *r)
{
    bool detected = false;
    hal_radio_lock(r);

    radio_reset(r);

    u1_t u = OPMODE_LORA | OPMODE_SLEEP;
#ifdef CFG_sx1276_radio
    u |= 0x8;   // TBD: sx1276 high freq
#endif
    int write_result = writeReg(r, RegOpMode, u);

    if (write_result < 0) {
        printk(KERN_WARNING "radio_detect failed to write radio register.\n");
    }
    
    opmode(r, OPMODE_SLEEP);  // maybe not necessary

    u1_t v = readReg(r, RegVersion);
#ifdef CFG_sx1276_radio
    detected = (v == 0x12 ); 
#elif CFG_sx1272_radio
    detected = (v == 0x22);   // This could change if the chip changes, but hopefully then the chip part number would change and this driver would be updated.
#endif

    opmode(r, OPMODE_SLEEP);  // maybe not necessary

    hal_radio_unlock(r);
    
    if (detected && (write_result >= 0)) {
        printk(KERN_DEBUG "lora radio detected.\n");
    } else {
        printk(KERN_DEBUG "lora radio NOT detected.\n");
    }
    
    return detected && (write_result >= 0);
}
    
    
