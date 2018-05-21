
#include <linux/module.h>

MODULE_DESCRIPTION("LoRa raw radio driver");
// Copyright 2016 Solvz, Inc.
MODULE_AUTHOR("Bruce Carlson, Signetik, LLC.");
// MODULE_ALIAS

// For now tell the kernel that we are GPL even though we might 
// not release it as GPL because we have to say GPL to use the gpio module.
MODULE_LICENSE("GPL");

// This module talks SPI to a SemTech SX1272 LoRa radio transceiver.
// This module allows only one concurrent open handle (read or write)

// excerpts below adapted from lmic are:
/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"



#include <linux/random.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/slab.h>         // for kzalloc, etc.
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/spi/spi.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/string.h>


#include "lora_raw.h"
#include "lora_timer.h"

#include "lmic/oslmic.h"
#include "lmic/lorabase.h"
#include "lmic/radio.h"

// NOTE: when compiling radio.c, make sure to define CFG_sx1272_radio


#define DRIVER_NAME "lora_raw"

#define LORA_RAW_MAX_OPEN_HANDLES 1000000     // no practical limit, but avoid overflow 


/////////////////////////////////////////////////////////////////
// Packet Buffer - linked list of variable-length buffers

struct pblist {
    struct list_head list;  // linked list of packet_buffers    
    long node_cnt;          // count of nodes in list
};

// diagnostic statistics : 
static atomic_t pb_nodes_alloc      = ATOMIC_INIT(0);    // how many packet buffers have been allocated? This will overflow.  Would like to use long long here, but must be atomic..
static atomic_t pb_alloc_failures   = ATOMIC_INIT(0);    // how many allocation attempts failed?         This will overflow.  Would like to use long long here, but must be atomic..
static atomic_t pb_nodes_freed      = ATOMIC_INIT(0);    // how many packet buffers have been freed?     This will overflow.  Would like to use long long here, but must be atomic..


void pb_init(struct packet_buffer *pb, size_t char_count)
{
    if (pb == NULL) {
        return;
    }
    INIT_LIST_HEAD(&pb->list);
    pb->count = char_count;
}

/** Allocate a packet_buffer
    @return pointer to the packet buffer, or <= 0 on failure.  
    packet buffer is initialized with zeroes.
    free the packet buffer with pb_destroy()
 */
struct packet_buffer *pb_create(size_t char_count, gfp_t malloc_flags)
{
    struct packet_buffer * pb;
    pb = kzalloc(sizeof(struct packet_buffer) + char_count * sizeof(char), malloc_flags);
    if (pb <= 0) {
        atomic_inc(&pb_alloc_failures);
        return pb;
    }
    atomic_inc(&pb_nodes_alloc);
    pb_init(pb, char_count);
    return pb;
}

/** destroy a single packet buffer structure
 */
void pb_destroy(struct packet_buffer *pb)
{
    if (pb == NULL) {
        return;
    }
    atomic_inc(&pb_nodes_freed);
    kzfree(pb);
}

static void pblist_init(struct pblist *pblist)
{
    if (pblist == NULL) {
        return;
    }
    INIT_LIST_HEAD(&pblist->list);
    pblist->node_cnt = 0;
}

/** Are there no nodes in pblist?
 */
static int pblist_is_empty(struct pblist *pblist)
{
    if (pblist == NULL) {
        return 1;
    }
    return list_empty(&pblist->list);
}

/** Delete the head node (oldest data)
 */
static void pblist_delete_head(struct pblist *pblist)
{
    if (pblist == NULL) {
        return;
    }
    if ( ! pblist_is_empty(pblist)) {
        struct packet_buffer *first_node;
        first_node = (struct packet_buffer *) pblist->list.next;
        list_del(pblist->list.next);
        pblist->node_cnt--;
        pb_destroy(first_node);
    } else {
        // no nodes to delete.  
        ;
        if (pblist->node_cnt != 0) {
            printk(KERN_ERR DRIVER_NAME " Error: pblist node_cnt mismatch: should be 0, but is %ld.  Clearing.\n", pblist->node_cnt);
            pblist->node_cnt = 0;
        }
    }
}

/** free the whole list 
 */
void pblist_free(struct pblist *pblist)
{
    while ( ! pblist_is_empty(pblist))
    {
        pblist_delete_head(pblist);
    }
}

/** Add new packet buffer to pblist tail
 */
void pblist_add_packet(struct pblist *pblist, struct packet_buffer *packet_to_add)
{
    if ((pblist == NULL) || (packet_to_add == NULL)) {
        return;
    }
    list_add_tail((struct list_head *) packet_to_add, &pblist->list);
    pblist->node_cnt++;
}
/** Remove the oldest packet buffer from pblist
    @return pointer to packet buffer removed from list or NULL if no data is available
    NOTE: CALLER IS RESPONSIBLE TO FREE THE RETURNED PACKET BUFFER. (use pb_destroy)
 */
struct packet_buffer *pblist_remove_packet(struct pblist *pblist)
{
    if (pblist_is_empty(pblist)) {
        return NULL;
    }
    struct list_head *p;
    p = pblist->list.next;
    list_del(p);
    return (struct packet_buffer *) p;
}


/////////////////////////////////////////////////////////////////



#if defined(CHANNEL_64)
#define LORA_RAW_DEFAULT_RX_CHANNEL    64       // default to the first 500 kHz upstream channel
#define LORA_RAW_DEFAULT_TX_CHANNEL    64       // default to the first 500 kHz upstream channel
//#define LORA_RAW_DEFAULT_TX_CHANNEL    72       // default to the first 500 kHz downstream channel
#define LORA_RAW_DEFAULT_DATA_RATE     12       // LORA_RAW_DEFAULT_DATA_RATE+1 is used as an index into _DR2RPS_CRC.  12+1 -> SF8
#else
#define LORA_RAW_DEFAULT_RX_CHANNEL    0       // default to the first 500 kHz upstream channel
#define LORA_RAW_DEFAULT_TX_CHANNEL    0       // default to the first 500 kHz upstream channel
//#define LORA_RAW_DEFAULT_TX_CHANNEL    72       // default to the first 500 kHz downstream channel
#define LORA_RAW_DEFAULT_DATA_RATE     3       // LORA_RAW_DEFAULT_DATA_RATE+1 is used as an index into _DR2RPS_CRC.  3+1 -> SF7
#endif
                                                // User must be careful to select channel+bandwidth that follow the LoRaWAN spec.
                                                
                                                // increasing SF gives longer range but lower data rate



struct lora_raw_radio {

    struct list_head list;

    struct mutex radio_lock;        // prevent concurrent open/close/tx/rx/etc.  This is distinct from radio.radio_lock
    int open_handles;
    int open_handle_key;           // >=0 is an active handle number. -1 for no open handles.
	bool got_module_reference;
    int hal_got_radio;

    lora_raw_cb event_handler;
    void *event_handler_context;
    
    int  tx_channel;
    dr_t tx_data_rate;
    s1_t tx_power;

    struct pblist rx_buffer;
    spinlock_t rx_buffer_lock;
    struct tasklet_struct receive_event_tasklet;

    struct hal_radio radio;
    long tx_packets;
    long rx_packets;
};


LIST_HEAD(radio_list);
DEFINE_MUTEX(radio_list_lock);



// find this radio name in radio_list or return NULL
static struct lora_raw_radio *get_radio_by_name(char *name)
{
    struct lora_raw_radio *ret = NULL;
    mutex_lock(&radio_list_lock);
    struct lora_raw_radio *r;
    list_for_each_entry(r, &radio_list, list) {
        if (strcmp(name, r->radio.name) == 0) {
            ret = r;
            break;
        }
    }
    mutex_unlock(&radio_list_lock);
    return ret;
}
                                    
// on error return NULL
static struct lora_raw_radio *get_radio_by_handle(lrhandle_t handle)
{
    struct lora_raw_radio *r;
    if ((handle.radio == NULL) || (handle.key < 0)) {
        return NULL;
    }
    r = (struct lora_raw_radio *) handle.radio;
    if (r->open_handle_key != handle.key) {
        return NULL;
    }
    return r;
}

static int radio_handle_is_open(struct lora_raw_radio *r)
{
    return (r->open_handles > 0);
}

static void radio_clean_up(struct lora_raw_radio *r);
static void lora_raw_receive_event_handler(unsigned long data);
static int radio_start_up(struct lora_raw_radio *r);

/** Open the named radio
    @return an lrhandle_t that references the radio.  result.key < 0 indicates error.
 */
lrhandle_t lora_raw_open(char *radio_name)
{
    printk(KERN_DEBUG DRIVER_NAME " open\n");

    struct lora_raw_radio *r;
    r = get_radio_by_name(radio_name);
    if (r == NULL) {
        return (lrhandle_t) { NULL, -ENODEV };
    }
   
    int result = 0;
    mutex_lock(&r->radio_lock);
    if (r->open_handles >= LORA_RAW_MAX_OPEN_HANDLES) {
        result = -EBUSY;        // (only so many open handles allowed)
    } else {
        r->open_handles++;
        if (r->open_handles <= 1) {             // if radio not already open
            r->open_handles = 1;
            result = radio_start_up(r);
            if (result < 0) {
                radio_clean_up(r);
            }
        }
    }
    mutex_unlock(&r->radio_lock);
    if (result < 0) {
        return (lrhandle_t) { NULL, result };
    }
    return (lrhandle_t) { r, r->open_handle_key };
}

/** 
 * assumes radio_lock is already held.
 */
static int radio_start_up(struct lora_raw_radio *r)
{
    printk(KERN_DEBUG DRIVER_NAME " radio_start_up\n");
    
    // check out a reference to this module so it doesn't get unloaded while open
    r->got_module_reference = try_module_get(THIS_MODULE);
    if ( ! r->got_module_reference) {
        return -ENXIO;
    }

    // now we are the only thread that made it here
    // pick a random key so that only the thread that opened can read and write.
    get_random_bytes(&r->open_handle_key, sizeof(r->open_handle_key));
    if (r->open_handle_key < 0) {          // shouldn't happen but just in case
        r->open_handle_key = -r->open_handle_key; 
    }

    // allocate buffers
    pblist_init(&r->rx_buffer);
    spin_lock_init(&r->rx_buffer_lock);
    tasklet_init(&r->receive_event_tasklet, &lora_raw_receive_event_handler, (unsigned long) r);

	// initialize some fields that should be reset each time we open:
	r->event_handler = NULL;
    r->event_handler_context = NULL;
    r->tx_channel   = LORA_RAW_DEFAULT_TX_CHANNEL;
    r->tx_data_rate = LORA_RAW_DEFAULT_DATA_RATE;
    r->tx_power     = 0;    						// default to low power until directed otherwise 

    r->hal_got_radio = 0;
    int hal_result = hal_request_radio(&r->radio);
    if (hal_result < 0) {
        return -EIO;
    }
    r->hal_got_radio = 1;

    // make sure we can still talk to radio
    bool radio_detected = radio_detect(&r->radio);
    if ( ! radio_detected) {
		printk(KERN_ERR DRIVER_NAME " failed to detect radio\n");
        return -EIO;
    }

    printk(KERN_INFO DRIVER_NAME " initializing radio\n");

    radio_init(&r->radio);

    radio_set_rx_handler(&(r->radio), &lora_raw_received_packet, &(r->radio));

    printk(KERN_INFO DRIVER_NAME " start receiving\n");

    radio_start_receiving(&(r->radio), LORA_RAW_DEFAULT_RX_CHANNEL, LORA_RAW_DEFAULT_DATA_RATE);
    
    return 0;
}

// called on release
int lora_raw_close(lrhandle_t handle)
{
    printk(KERN_DEBUG DRIVER_NAME " close\n");

    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }

    mutex_lock(&r->radio_lock);
    if (r->open_handles > 1) {  // if not the last open handle
        r->open_handles--;
    } else {        
        radio_clean_up(r);      // open handles must be > 0 for radio_clean_up to work
        r->open_handles = 0;    // redundant for clarity
    } 
    mutex_unlock(&r->radio_lock);
    printk(KERN_DEBUG DRIVER_NAME " close done.  open handles = %d\n", r->open_handles);
    return 0;
}

/** 
 * assumes radio_lock is already held.
 */
static void radio_clean_up(struct lora_raw_radio *r)
{
    printk(KERN_DEBUG DRIVER_NAME " radio_clean_up\n");
    if (r->open_handles > 0) {          // if handle is open then a radio needs to be cleaned up.
        if (r->hal_got_radio) {
            r->hal_got_radio = 0;

            radio_shutdown(&(r->radio));
            hal_release_radio(&(r->radio));
        }
        
        tasklet_kill(&r->receive_event_tasklet);
        r->event_handler = NULL;
        r->event_handler_context = NULL;
        unsigned long irq_flags;
        spin_lock_irqsave(&r->rx_buffer_lock, irq_flags);
        pblist_free(&r->rx_buffer);
        spin_unlock_irqrestore(&r->rx_buffer_lock, irq_flags);
        r->open_handle_key = -1;
        r->open_handles = 0;
        
        if (r->got_module_reference) {
            r->got_module_reference = false;
            module_put(THIS_MODULE);
        }
    }
}

/** set radio transmit settings
    @param channel - for details see comments on get_channel_freq in radio.c
    @param data_rate  valid values are: (from  _dr_us915_t)
                   DR_SF10 = 0, DR_SF9 = 1, DR_SF8 = 2, DR_SF7 = 3, DR_SF8C = 4
                   // Devices behind a router:
                   DR_SF12CR = 8, DR_SF11CR = 9, DR_SF10CR = 10, DR_SF9CR = 11, 
                   DR_SF8CR = 12, DR_SF7CR = 13
    @param tx_power_dBm     for SX1272, valid range is 2 to 17 dBm
    @return 0 on success / < 0 on failure
 */
int lora_raw_set_tx_channel(lrhandle_t handle, int channel, dr_t data_rate, int8_t tx_power_dBm)
{
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }
    if (channel >= radio_get_num_channels()) {
        return -EINVAL;
    }
    if ( ! validDR(data_rate)) {
        return -EINVAL;
    }
    mutex_lock(&r->radio_lock);
    r->tx_channel = channel;
    r->tx_data_rate = data_rate;
    r->tx_power = tx_power_dBm;
    mutex_unlock(&r->radio_lock);
    return 0;
}

/** set radio receive settings
    @return 0 on success / < 0 on failure
 */
int lora_raw_set_rx_channel(lrhandle_t handle, int channel, dr_t data_rate)
{
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }
    if (channel >= radio_get_num_channels()) {
        return -EINVAL;
    }
    if ( ! validDR(data_rate)) {
        return -EINVAL;
    }
    mutex_lock(&r->radio_lock);
    radio_start_receiving(&(r->radio), channel, data_rate);
    mutex_unlock(&r->radio_lock);
    return 0;
}




int lora_raw_tx(lrhandle_t handle, void *data, int len)
{
    printk(KERN_DEBUG DRIVER_NAME " TX\n");
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }
    mutex_lock(&r->radio_lock);
    int result = radio_tx(&(r->radio), r->tx_channel, r->tx_data_rate, r->tx_power, data, len);
    if (result >= 0) {
        r->tx_packets++;
    }
    mutex_unlock(&r->radio_lock);
    printk(KERN_DEBUG DRIVER_NAME " TX done\n");
    return result;
}

bool lora_packet_is_ready(lrhandle_t handle)
{
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }
    unsigned long irq_flags;
    spin_lock_irqsave(&r->rx_buffer_lock, irq_flags);
    bool empty = pblist_is_empty(&r->rx_buffer);
    spin_unlock_irqrestore(&r->rx_buffer_lock, irq_flags);
    return ! empty;
}


/**
    This function does not sleep so that it may be called from interrupt handler
 */
int lora_raw_rx(lrhandle_t handle, struct lora_raw_packet *packet)
{
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return -ENODEV;
    }
    int result = 0;
    int packets_remaining = 0;
    unsigned long irq_flags;
    spin_lock_irqsave(&r->rx_buffer_lock, irq_flags);
    if (pblist_is_empty(&r->rx_buffer)) {
        result = -EAGAIN;                 // try back later
        spin_unlock_irqrestore(&r->rx_buffer_lock, irq_flags);
    } else {
        struct packet_buffer *pb;
        pb = pblist_remove_packet(&r->rx_buffer);
        packets_remaining = r->rx_buffer.node_cnt;
        spin_unlock_irqrestore(&r->rx_buffer_lock, irq_flags);
        struct lora_raw_packet *lp;
        lp = (struct lora_raw_packet *) pb->data;
        if (lp->data_bytes < 0) {
            lp->data_bytes = 0;
        }
        if (lp->data_bytes > LORA_RAW_MAX_PACKET_DATA) {
            lp->data_bytes = LORA_RAW_MAX_PACKET_DATA;
            result = -E2BIG;    // packet truncated
        }
        memcpy(packet, lp, sizeof(struct lora_raw_packet) + lp->data_bytes);
        pb_destroy(pb);
    }
    if (result < 0) {
        return result;
    }
    return packets_remaining;   // may occasionally appear wrong due to concurrently added or removed packets
}

/**
 *  @param event_handler will be called from tasklet, so must not sleep
 */
void lora_raw_set_callback(lrhandle_t handle, lora_raw_cb event_handler, void *event_handler_context)
{
    struct lora_raw_radio *r;
    r = get_radio_by_handle(handle);
    if (r == NULL) {
        return;
    }
    r->event_handler = event_handler;
    r->event_handler_context = event_handler_context;
}



/** event handler gets called as tasklet after receive interrupt
 */
static void lora_raw_receive_event_handler(unsigned long data)
{
	if (data == 0) {
        printk(KERN_ERR DRIVER_NAME " lora_raw_receive_event_handler called with invalid data.\n");
		return; 	// 
	}
    struct lora_raw_radio *r;
    r = (struct lora_raw_radio *) data;
    if (r->event_handler != NULL) {
        r->event_handler(r->event_handler_context, LORA_CB_EVENT_RX);
    }    
}

/** A packet has been received 
    Radio interrupt handler should call this when a packet has just been received
 */ 
void lora_raw_received_packet(void *hal_radio_voidp, struct hal_radio_packet *p)
{
    struct hal_radio *hal_radio;
    hal_radio = (struct hal_radio *) hal_radio_voidp;
    struct packet_buffer *pb = pb_create(sizeof(struct lora_raw_packet) + p->data_bytes, GFP_ATOMIC);
    if (pb <= 0) {
        printk(KERN_WARNING "%s:%s failed to allocate packet buffer.  Discarding packet\n", __FILE__, __FUNCTION__);
        hal_radio->alloc_error_count++;
    } else {
        struct lora_raw_packet *lp;
        lp = (struct lora_raw_packet *) pb->data;
        lp->arrival_time = p->arrival_time;
        lp->channel = p->channel;
        lp->data_rate = p->data_rate;
        lp->data_bytes = p->data_bytes;
        memcpy(lp->data, p->data, p->data_bytes);
        // read rx quality parameters
        lp->SNR = p->SNR;
        lp->RSSI = p->RSSI;
	printk(KERN_DEBUG "%s: RX packet %d bytes, (%d,%d)\n", __FUNCTION__, p->data_bytes, lp->SNR, lp->RSSI);

        struct lora_raw_radio *r;
        r = container_of(hal_radio, struct lora_raw_radio, radio);
        unsigned long irq_flags;
        spin_lock_irqsave(&r->rx_buffer_lock, irq_flags);
        pblist_add_packet(&r->rx_buffer, pb);
        spin_unlock_irqrestore(&r->rx_buffer_lock, irq_flags);
        tasklet_schedule(&r->receive_event_tasklet);
        r->rx_packets++;
    }
}

static void radio_list_destroy(void)
{
    mutex_lock(&radio_list_lock);
    while ( ! list_empty(&radio_list)) {
        struct lora_raw_radio *r;
        r = list_entry(radio_list.next, struct lora_raw_radio, list);
        list_del(radio_list.next);
        mutex_lock(&r->radio_lock);
        radio_clean_up(r);
        mutex_unlock(&r->radio_lock);
        kzfree(r);
    }
    mutex_unlock(&radio_list_lock);
}




static bool lora_raw_detect_radio(struct lora_raw_radio *r);


int lora_raw_spi_probe(struct spi_device *spi_dev)
{
    printk(KERN_DEBUG DRIVER_NAME " probe\n");

    struct lora_raw_radio *radio;
    radio = kzalloc(sizeof(*radio), GFP_KERNEL);
    if ((radio == NULL) || (IS_ERR(radio))) {
        printk(KERN_ERR DRIVER_NAME " failed to allocate memory for radio\n");
        return PTR_ERR(radio);
    }

    mutex_init(&radio->radio_lock);
    radio->open_handles = 0;
	radio->open_handle_key = -1;

	radio->event_handler = NULL;
    radio->event_handler_context = NULL;
    radio->tx_channel   = LORA_RAW_DEFAULT_TX_CHANNEL;
    radio->tx_data_rate = LORA_RAW_DEFAULT_DATA_RATE;
    radio->tx_power     = 0;    						// default to low power until directed otherwise 

	radio->got_module_reference = false;

	hal_init_radio(&(radio->radio), spi_dev);

	bool radio_detected = lora_raw_detect_radio(radio);
	if ( ! radio_detected) {
        printk(KERN_ERR DRIVER_NAME " failed to detect radio\n");
        kzfree(radio);
        return -EIO;
    }
    printk(KERN_INFO DRIVER_NAME " radio detected\n");
    
    // store radio in spi so we can get at radio later
    spi_set_drvdata(spi_dev, radio);  
    
    INIT_LIST_HEAD(&radio->list);
    mutex_lock(&radio_list_lock);
    list_add_tail(&radio->list, &radio_list);
    mutex_unlock(&radio_list_lock);
    return 0;
}


int lora_raw_spi_remove(struct spi_device *spi)
{
    printk(KERN_DEBUG DRIVER_NAME " remove\n");

    struct lora_raw_radio *radio;
    radio = spi_get_drvdata(spi);   // probe stored radio in drvdata

	if (radio_handle_is_open(radio)) {
		printk(KERN_WARNING DRIVER_NAME " can't remove: a handle to this device is still open!\n");
		return -EBUSY;		// todo: is this right?
	}

    // close/release should have already shutdown the radio and released the hardware and memory.

    mutex_lock(&radio_list_lock);
    list_del(&(radio->list));
    mutex_unlock(&radio_list_lock);

    spi_set_drvdata(spi, NULL);

    kzfree(radio);

    return 0;
}

int lora_raw_suspend(struct device *dev, pm_message_t message)
{
    printk(KERN_INFO DRIVER_NAME " suspend %d\n", message.event);

    struct lora_raw_radio *radio;
    radio = dev_get_drvdata(dev);   // probe stored radio in drvdata
    
    // Leave the radio on.  Nothing to do here.
    
    return 0;
}

int lora_raw_resume(struct device *dev)
{
    printk(KERN_INFO DRIVER_NAME " resume\n");

    struct lora_raw_radio *radio;
    radio = dev_get_drvdata(dev);   // probe stored radio in drvdata

    // Leave the radio on.  Nothing to do here.
    
    return 0;
}

/** See if we can talk to the radio
    Assumes radio->radio is initialized, but not requested
 */
static bool lora_raw_detect_radio(struct lora_raw_radio *r)
{
    bool detected = false;
    int hal_result = hal_request_radio(&(r->radio));
    if (hal_result == 0) {  // hal got radio hardware
        detected = radio_detect(&(r->radio));
    }
    hal_release_radio(&(r->radio));
    return detected;
}



static const struct of_device_id lora_raw_match_table[] = {
    { .compatible = "SemTech,SX1272" },
    {}
};

static struct spi_driver lora_raw_spi_driver = {
    .driver = {
        .name = "lora_raw",
        .owner = THIS_MODULE,
        .of_match_table = lora_raw_match_table,
        .suspend = &lora_raw_suspend,
        .resume = &lora_raw_resume,
    },
    .probe = &lora_raw_spi_probe,
    .remove = &lora_raw_spi_remove,
    // resume/suspend not implemented
};

// lora_raw proc file:

static int show_module_diags;  // we only want this for its address. 
static int show_hal_diags;     // we only want this for its address. 

// seq_file iterator functions:

static void *lr_proc_seq_get_pointer(loff_t *pos)
{
    if (*pos == 0) {
        return &show_module_diags;         // special case - first entry indicates display module diagnostics.
    }
    if (*pos == 1) {
        return &show_hal_diags;
    }
    if (list_empty(&radio_list)) {
        return NULL;
    }
    struct list_head *p = radio_list.next;
    loff_t i;
    for (i=0; i<*pos-2; i++) {
        p = p->next;
        if (p == &radio_list) { // end of list
            return NULL;
        }
    }
    return p;
}

static int radio_count(void)
{
    int i = 0;
    struct list_head *p = radio_list.next;
    while (p != &radio_list) {
        i++;
        p = p->next;
        if (i > 32767) {    // nonsense
            return -1;
        }
    }
    return i;
}

static void *lr_proc_seq_start(struct seq_file *f, loff_t *pos)
{
    mutex_lock(&radio_list_lock);
    return lr_proc_seq_get_pointer(pos);
}

static void *lr_proc_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
    (*pos)++;
    return lr_proc_seq_get_pointer(pos);
}

static void lr_proc_seq_stop(struct seq_file *f, void *v)
{
    mutex_unlock(&radio_list_lock);
}
/**
 *  @param v was returned from lr_proc_seq_start or lr_proc_seq_next
 */
static int lr_proc_seq_show(struct seq_file *f, void *v) 
{
    if (v == &show_module_diags) {
        seq_printf(f, "lora_raw statistics\n");
        seq_printf(f, "  radios                      = %12d\n", radio_count());
        seq_printf(f, "  pb_nodes allocated          = %12d\n", atomic_read(&pb_nodes_alloc));
        seq_printf(f, "  pb_nodes freed              = %12d\n", atomic_read(&pb_nodes_freed));
        seq_printf(f, "  pb_nodes outstanding        = %12d\n", atomic_read(&pb_nodes_alloc)-atomic_read(&pb_nodes_freed));
        seq_printf(f, "  pb_node allocation failures = %12d\n", atomic_read(&pb_alloc_failures));
        seq_printf(f, "  missed timer windows        = %12ld\n", lora_timer_missed_window_count);
    } else if (v == &show_hal_diags) {
        seq_printf(f, "  hal assertion failures      = %12ld\n", hal_fail_count);
        seq_printf(f, "  last hal assertion failure in file %s at line %ld\n", hal_last_fail_file, hal_last_fail_line);
    } else {
        struct lora_raw_radio *r;
        r = container_of(v, struct lora_raw_radio, list);
        seq_printf(f, "  Radio %s :\n", r->radio.name);
        seq_printf(f, "    Packets transmitted = %12ld\n", r->tx_packets);
        seq_printf(f, "    Packets received    = %12ld\n", r->rx_packets);
        seq_printf(f, "    SPI errors          = %12ld\n", r->radio.spi_error_count);
        seq_printf(f, "    last SPI error      = %12d\n",  r->radio.last_spi_error);
        seq_printf(f, "    SPI timeouts        = %12ld\n", r->radio.spi_timeout_count);
        seq_printf(f, "    allocation errors   = %12ld\n", r->radio.alloc_error_count);
    }
    return 0;
}

static struct seq_operations lr_proc_seq_ops = {
    .start = &lr_proc_seq_start,
    .next = &lr_proc_seq_next,
    .show = &lr_proc_seq_show,
    .stop = &lr_proc_seq_stop,
};

static int lr_proc_open(struct inode *inode, struct file *file) 
{
    return seq_open(file, &lr_proc_seq_ops);
}

static ssize_t lr_proc_write(struct file *file, const char __user *write_data, size_t data_len, loff_t *pointer)
{
    // On write to proc, printk radio registers to kernel log
    int i = 0;
    struct list_head *p = radio_list.next;
    while (p != &radio_list) {
        struct lora_raw_radio *r;
        r = container_of(p, struct lora_raw_radio, list);
        printk(KERN_INFO "  Radio %s :\n", r->radio.name);
        printk_lora_registers(&r->radio);
        i++;
        p = p->next;
        if (p == radio_list.next) { // if we are back to the beginning of the list
            break;
        }
        if (i > 32767) {    // nonsense
            return -1;
        }
    }
    return data_len;
}


static struct file_operations lora_raw_proc_fops = {
    .owner = THIS_MODULE,
    .open = lr_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = seq_release,
    .write = lr_proc_write,
};




char *lora_raw_radio_list(void)
{
    mutex_lock(&radio_list_lock);
    int list_len = 1;             // allow 1 char for null-terminator
    struct lora_raw_radio *r;
    list_for_each_entry(r, &radio_list, list) {
        list_len += strlen(r->radio.name) + 1;
    }
    char *rl;
    rl = kzalloc(list_len, GFP_KERNEL);
    if (rl != NULL) {
    	char *p;
    	p = rl;
    	list_for_each_entry(r, &radio_list, list) {
            int nlen = strlen(r->radio.name);
        	memcpy(p, r->radio.name, nlen);
        	p += nlen;
            *p = LORA_RAW_RADIO_LIST_DELIMITER;
            p++;
    	}
        *p = '\0';
    }
    mutex_unlock(&radio_list_lock);
    return rl;
}


//  Driver Init / Exit


static int driver_registered = 0;
static struct proc_dir_entry *proc_file = NULL;

static void driver_cleanup(void);


static __init int lora_raw_driver_init(void)
{
    printk(KERN_INFO DRIVER_NAME " init\n");
    
    // radio_list initialized empty at compile time

    int hal_result = hal_init();
    if (hal_result < 0) {
        return hal_result;
    }
    
    // register this driver as able to handle lora_raw spi devices

    int register_result = spi_register_driver(&lora_raw_spi_driver);
    if (register_result < 0) {
        printk(KERN_ERR DRIVER_NAME " spi_register_driver failed.\n");
        driver_cleanup();
        return register_result;
    }
    driver_registered = 1;

    proc_file = proc_create("lora_raw", 0, NULL, &lora_raw_proc_fops);
    if (proc_file == NULL) {
        printk(KERN_WARNING DRIVER_NAME " failed to create proc file.\n");
        // we can continue on without the proc file.
    }
    
    return 0;
}

static void lora_raw_driver_exit(void)
{
    printk(KERN_INFO DRIVER_NAME " exit\n");

    driver_cleanup();
}

static void driver_cleanup(void)
{
    proc_remove(proc_file);
    
    if (driver_registered) {
        driver_registered = 0;
        spi_unregister_driver(&lora_raw_spi_driver);
    }
    
    radio_list_destroy();
    hal_shutdown();
}

module_init(lora_raw_driver_init);
module_exit(lora_raw_driver_exit);

EXPORT_SYMBOL(lora_raw_radio_list);
EXPORT_SYMBOL(lora_raw_open);
EXPORT_SYMBOL(lora_raw_set_tx_channel);
EXPORT_SYMBOL(lora_raw_set_rx_channel);
EXPORT_SYMBOL(lora_raw_set_callback);
EXPORT_SYMBOL(lora_raw_tx);
EXPORT_SYMBOL(lora_packet_is_ready);
EXPORT_SYMBOL(lora_raw_rx);
EXPORT_SYMBOL(lora_raw_close);


