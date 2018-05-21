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

// intentionally before guard #include 
#define HAS_ostick_conv 0   // required by oslmic - ok to do this multiple times. 
 
 
#ifndef _hal_hpp_
#define _hal_hpp_

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

struct hal_radio;       // forward declaration before including oslmic.h

#include "oslmic.h"
#include "lorabase.h"
#include "deferred.h"


// hal radio structure - this is made based on the above info 

struct hal_gpio {
    long gpio_num;                  // <0 indicates not allocated (long because gpio functions require unsigned int, but we also want to store negative.)
    int irq_num;                    // <0 indicates not allocated
    bool active_high;               
    char name[40];
};

#define HAL_MAX_FRAME_LEN 256       // How large of packets (bytes) this drivers supports 
                                    // The SX1272 buffer is 256 bytes.
                                    // The SX1272 can supposedly handle arbitrarily long packets, but this driver might not.
    
struct irq_handler_data {
   // the hal radio lock assures that these are only accessed by one thread at a time.  
   ostime_t interrupt_time;
   u1_t irq_flags;
   u1_t ModemConfig1;
   u1_t PayloadLength;
   u1_t RxNbBytes;
   u1_t PktSnrValue;
   u1_t PktRssiValue;
   u1_t FifoRxCurrentAddr;
   u1_t scratch;            // irq handler needs a place to store a dummy read result
}; 
    
/// hal_radio_packet is much like lora_raw_full_packet plus a status field.
struct hal_radio_packet {
    int         status;         // < 0 for error
    uint32_t    arrival_time;
    int         channel;
    int         data_rate;
    int         RSSI;
    int         SNR;
    int         data_bytes;
    uint8_t     data[HAL_MAX_FRAME_LEN]; 
};

struct hal_radio {
    char name[40];
    // Per-radio lock on the queuing of spi messages one transaction from interrupting another :
    // We would like to use a mutex here, but the documentation says we
    //  can't use mutex_trylock in an interrupt, even though it doesn't sleep.
    volatile long unsigned int radio_lock;    // 0 = unlocked.  1 = locked.  use test_and_set_bit and clear_bit to access
    
    struct deferred_task run_after_unlock;

    struct spi_device *spi_dev;
    
    struct hal_gpio reset_gpio;
    struct hal_gpio antenna_sw_tx;
    struct hal_gpio radio_DIO0;
    
//    // extra channels (these from lmic.h)
//    u4_t        xchFreq[MAX_XCHANNELS];    // extra channel frequencies (if device is behind a repeater)
//    u2_t        xchDrMap[MAX_XCHANNELS];   // extra channel datarate ranges  ---XXX: ditto
    
    // rps, tx_power, and freq are for passing parameters in to os_radio and 
    //  similar functions in radio.c.  They are not intended for other use.
    rps_t   rps;        // from LMIC rps - encodes spreading factor, bandwidth, code rate, ih, nocrc
    uint8_t tx_power;   // from LMIC struct txPow
    uint32_t freq;      // from LMIC freq

    ostime_t tx_end;            
    uint8_t tx_frame[HAL_MAX_FRAME_LEN];
    int tx_data_len;
    
    struct hal_radio_packet rx_packet;
    
    void (*rx_handler)(void *handler_data, struct hal_radio_packet *);
    void *rx_handler_data;
    
    dr_t last_rx_datarate;
    int  last_rx_channel;
    
    struct irq_handler_data irq_handler_data;

    uint8_t original_InvertIQ;
    
    long spi_error_count;
    int  last_spi_error;
    long spi_timeout_count;
    long alloc_error_count;
};


/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
int hal_init (void);

/*
 * shut down hardware.
 */
void hal_shutdown(void);

/** Tell the hal about the radio
 */
void hal_init_radio(struct hal_radio *r, struct spi_device *spi_dev);

/** Request the radio resources from the system
 */
int hal_request_radio(struct hal_radio *r);

void hal_release_radio(struct hal_radio *r);


/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx(struct hal_radio *r, u1_t val);

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst(struct hal_radio *r, u1_t val);

/*
 * Lock the spi before using
 * Each call to hal_radio_lock (or successful hal_radio_try_lock) must be paired with a call to hal_radio_unlock!
 */
void hal_radio_lock(struct hal_radio *r);

/*
 * Lock the spi before using, but don't sleep.  Return 1 if spi lock acquired / 0 if not
 * Each call to hal_radio_lock (or successful hal_radio_try_lock) must be paired with a call to hal_radio_unlock!
 */
int hal_radio_try_lock(struct hal_radio *r);

/*
 * Unlock spi after using. 
 * Each call to hal_radio_lock (or successful hal_radio_try_lock) must be paired with a call to hal_radio_unlock!
 */
void hal_radio_unlock(struct hal_radio *r);

/*
 * Schedule a function call to be made when the (currently busy) lock is being released
 * (this function may be called from interrupt handler)
 */
void hal_radio_do_after_unlock(struct hal_radio *r,
                               void (*task)(void *, unsigned long),
                               unsigned long task_data);

/*
 * perform 8-bit SPI transaction with radio.
 * write address and then read/write len bytes from/to buf
 */
int hal_spi_write (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len);
int hal_spi_read (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len);
void hal_spi_read_async (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len,
                         void (*run_on_completion)(struct hal_radio *, int),
                         gfp_t kalloc_flags);
void hal_spi_write_async (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len,
                          void (*run_on_completion)(struct hal_radio *, int),
                          gfp_t kalloc_flags);


/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void hal_sleep (void);

/*
 * return 32-bit system time in ticks.
 */
u4_t hal_ticks (void);

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void hal_waitUntil (u4_t time);

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
u1_t hal_checkTimer (u4_t targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed(char *file_name, long line_number);


// diagnostic statistics - read only:
extern long  hal_fail_count;
extern char *hal_last_fail_file;
extern long  hal_last_fail_line;



#endif // _hal_hpp_
