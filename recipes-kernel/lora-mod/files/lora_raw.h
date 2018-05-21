
#ifndef LORA_RAW_DRIVER_H
#define LORA_RAW_DRIVER_H

// LoRa radio driver
// This module talks SPI to a SemTech SX1272 LoRa radio transceiver.
// This module allows only one concurrent open handle (read or write)
// The functions below are exported for use by other kernel modules.
// lora_raw_chardev is a kernel module that provides example usage.
// lora_raw_chardev creates a char device in /dev/ corresponding to each radio.


#include <linux/types.h>

#include "lmic/hal.h"
#include "lora_timer.h"

#define LORA_RAW_MAX_PACKET_DATA HAL_MAX_FRAME_LEN


struct packet_buffer {
    struct list_head list;  // contains pointers to next and previous items in linked list
    ssize_t count;          // how many chars of data[] are used 
    char data[1];           // variable-length-field
};


struct packet_buffer *pb_create(size_t char_count, gfp_t malloc_flags);


struct lora_raw_packet {
    uint32_t    arrival_time;
    int         channel;
    int         data_rate;
    int         RSSI;
    int         SNR;
    int         data_bytes;
    uint8_t     data[];                // variable length
};

// lora_raw_full_packet must match lora_raw_packet in all fields except data !
struct lora_raw_full_packet {
    uint32_t    arrival_time;
    int         channel;
    int         data_rate;
    int         RSSI;
    int         SNR;
    int         data_bytes;
    uint8_t     data[LORA_RAW_MAX_PACKET_DATA];                // variable length
};


struct lrhandle {
    void *radio;
    int  key;
};
typedef struct lrhandle lrhandle_t;

/** Open a handle to a LoRa radio
    @param radio_name identifies the radio to be opened
    @return handle.  On success handle.key ( >= 0) 
            < 0 on error 
            EBUSY - the resource is in use / not available
 */
lrhandle_t lora_raw_open(char *radio_name);


/** Transmit a packet via LoRa radio
    @param handle   obtained from open
    @param data     bytes to send
    @param len      number of bytes to send
    @return  0 on success (packet is queued to be sent soon)
            -1 on error (system is busy EBUSY).
*/            
int lora_raw_tx(lrhandle_t handle, void *data, int len);

/** Receive a packet from LoRa radio
    @param handle   obtained from open
    @param packet   storage for packet (caller should allocate sizeof(struct 
                        lora_raw_packet) + LORA_RAW_MAX_PACKET_DATA bytes)
    @return  N on success (N = number of packets remaining)
            -1 on error
            -EFBIG:  packet too large (large packet has been discarded)
            -EAGAIN: no data ready
*/
int lora_raw_rx(lrhandle_t handle, struct lora_raw_packet *packet);


#define LORA_CB_EVENT_RX 1
typedef void (*lora_raw_cb)(void *context, int event);

/** Register an event_handler for when a packet is received
 *  @return 0 on success, < 0 on failure
 */
void lora_raw_set_callback(lrhandle_t handle, lora_raw_cb event_handler, void *event_handler_context);

/** Close handle to LoRa radio
    @param handle   obtained from open
 */
int lora_raw_close(lrhandle_t handle);


/** A packet has been received 
    Radio interrupt handler should call this when a packet has just been received
 */ 
void lora_raw_received_packet(void *radio, struct hal_radio_packet *p);



/** Set radio transmit settings
    User must be careful to select channel+bandwidth that follow the LoRaWAN spec.

    @param channel valid values are:
              0 - 63 : upstream 125 kHz channels starting from US915_125kHz_UPFBASE 
                                               with spacing of US915_125kHz_UPFSTEP
             64 - 71 : upstream 500 kHz channels starting from US915_500kHz_UPFBASE
                                               with spacing of US915_500kHz_UPFSTEP
             72 - 79 : downstream 500 kHz channels starting from US915_500kHz_DNFBASE
                                                 with spacing of US915_500kHz_DNFSTEP
             80+ : (unsupported) extra channels with frequencies per LMIC.xchFreq[chnl-72];
    @param data_rate  valid values are: (from  _dr_us915_t)
                   DR_SF10 = 0, DR_SF9 = 1, DR_SF8 = 2, DR_SF7 = 3, DR_SF8C = 4
                   // Devices behind a router:
                   DR_SF12CR = 8, DR_SF11CR = 9, DR_SF10CR = 10, DR_SF9CR = 11, 
                   DR_SF8CR = 12, DR_SF7CR = 13
    @param tx_power_dBm     for SX1272, valid range is 2 to 17 dBm
    @return 0 on success / < 0 on failure
 */
int lora_raw_set_tx_channel(lrhandle_t handle, int channel, dr_t data_rate, int8_t tx_power_dBm);

/** set radio receive settings
    User must be careful to select channel+bandwidth that follow the LoRaWAN spec.
    @return 0 on success / < 0 on failure
 */
int lora_raw_set_rx_channel(lrhandle_t handle, int channel, dr_t data_rate);


#define LORA_RAW_RADIO_LIST_DELIMITER ','
/** Get a list of radio names 
 *  Returns null-terminated string containing radio names separated by LORA_RAW_RADIO_LIST_DELIMITER
 *  Returns NULL on error
 *  Caller is responsible to free the returned memory with kzfree.
 */
char *lora_raw_radio_list(void);



#endif
