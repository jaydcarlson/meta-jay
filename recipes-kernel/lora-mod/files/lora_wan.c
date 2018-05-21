
// Copyright 2016 Solvz, Inc.
// All rights reserved.

#include <linux/module.h>
MODULE_AUTHOR("Bruce Carlson, Signetik, LLC.");

// This loadable linux kernel module implements a custom subset of the LoRa WAN specification version 1.0.
// This module uses the Solvz lora_raw module, which must be loaded before this is loaded.


// This module is not necesarily released under GPL, but we must include 
// the following line order to use certain kernel libraries (including mutex, etc.):
MODULE_LICENSE("GPL");

#define DRIVER_NAME "lora_wan"

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "lora_wan.h"
#include "lora_wan_packet.h"
#include "lora_raw.h"


// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"



// how long a packet can sit in each buffer before being destroyed:
// todo: perhaps make these configurable from menuconfig?
#define LORA_WAN_RX_EXPIRATION_JIFFIES             (1L * 60L * 60L * HZ)  
#define LORA_WAN_TX_EXPIRATION_JIFFIES        (1 * 24L * 60L * 60L * HZ)
#define LORA_WAN_TRANSMIT_LIST_EXPIRATION_JIFFIES               (3 * HZ)

// these could go into a struct lora_wan:

static DEFINE_MUTEX(lora_wan_lock); // Guard against concurrent access to lora_wan buffers and lora_wan_handles
static int lora_wan_handles = 0;    // How many outstanding open handles are there?
static lrhandle_t lora_raw_handle;  // Handle to our lora_raw radio device
// tx_buffer is a list of data the user has requested to transmit.  They are 
//  waiting to be scheduled for transmission.
static LIST_HEAD(tx_buffer);        // list of struct tx_buffer_packet, in order by time received from user - earliest first. 
static LIST_HEAD(rx_buffer);        // list of struct lwpacket: lora_wan has received these packets - they need to be returned back to user.
static LIST_HEAD(transmit_list);    // list of struct lwpacket: packets about to be transmitted.  The only reason for this list is to ensure that these packets get freed.
static DEFINE_SPINLOCK(tx_buffer_lock); // prevent concurrent access on tx_buffer
static DEFINE_SPINLOCK(rx_buffer_lock); // prevent concurrent access on rx_buffer
static DEFINE_SPINLOCK(transmit_list_lock); // prevent concurrent access on transmit_list

// todo: display error counters from /proc file ?
static long alloc_error_count = 0; 
static long packet_fragment_count = 0;
static long unexpected_rx_error_count = 0;
static int last_unexpected_rx_error = 0;
static long dropped_transmit_packets = 0;
static long truncated_rx_packets = 0;


struct tx_buffer_packet {
  struct list_head list;
  
  int32_t expiration_jiffies;               // packets past expiration are to be destroyed.
  
  struct lw_user_packet packet;
};






static int  lora_wan_open_comm(void);
static void lora_wan_close_comm(void);
static int  get_lora_raw_radio_count(void);
static int  open_lora_raw_radio(void);
static void close_lora_raw_radio(void);
static void lora_wan_event_handler(void *context, int event);




static int lora_wan_open(void)
{
    int result = 0;
    mutex_lock(&lora_wan_lock);    
    lora_wan_handles++;
    if (lora_wan_handles <= 1) {
        lora_wan_handles = 1;
        printk(KERN_INFO DRIVER_NAME " opening first handle\n");

        result = lora_wan_open_comm();
        if (result < 0) {
            lora_wan_close_comm();
            lora_wan_handles = 0;
        }
    }
    mutex_unlock(&lora_wan_lock);
    return result;
}

int lora_wan_close(void)
{
    mutex_lock(&lora_wan_lock);    
    lora_wan_handles--;
    if (lora_wan_handles <= 0) {
        lora_wan_handles = 0;
        printk(KERN_INFO DRIVER_NAME " closing last handle\n");
        lora_wan_close_comm();
    }
    mutex_unlock(&lora_wan_lock);
    return 0;
}

static int lora_wan_open_comm(void)
{
    if (get_lora_raw_radio_count() <= 0) {
        printk(KERN_ERR DRIVER_NAME " open_comm : no lora radio available\n");
        return -ENODEV;
    } else {
        INIT_LIST_HEAD(&tx_buffer);
        INIT_LIST_HEAD(&rx_buffer);
        INIT_LIST_HEAD(&transmit_list);
        
        int result = open_lora_raw_radio();
        if (result < 0) {
            printk(KERN_ERR DRIVER_NAME " open_comm : failed to open radio\n");
            return result;
        }
        lora_raw_set_callback(lora_raw_handle, lora_wan_event_handler, &lora_raw_handle);
    }
    return 0;
}

static void free_tx_buffer(void);
static void free_rx_buffer(void);
static void free_transmit_list(void);

/** Undo whatever was done by lora_wan_open_comm - tear down 
 */
static void lora_wan_close_comm(void)
{
    close_lora_raw_radio();
    
    free_tx_buffer();
    free_rx_buffer();
    free_transmit_list();
}

static void free_tx_buffer(void)
{
    unsigned long irq_state;
    spin_lock_irqsave(&tx_buffer_lock, irq_state);
    int i = 0;
    while ( ! list_empty(&tx_buffer)) {
        struct tx_buffer_packet *p;
        p = list_first_entry(&tx_buffer, struct tx_buffer_packet, list);
        list_del(&p->list);
        kzfree(p);
        i++;
        if (i > 100000000) {
            printk(KERN_WARNING DRIVER_NAME " lora_wan_close_comm : too many tx_buffer entries\n");
            break;
        }
    }
    spin_unlock_irqrestore(&tx_buffer_lock, irq_state);
    // todo : change this to KERN_DEBUG:
    printk(KERN_INFO DRIVER_NAME " lora_wan_close_comm : discarded %d tx_buffer entries\n", i);
    INIT_LIST_HEAD(&tx_buffer);
}
static void free_rx_buffer(void)
{
    unsigned long irq_state;
    spin_lock_irqsave(&rx_buffer_lock, irq_state);
    int i = 0;
    while ( ! list_empty(&rx_buffer)) {
        struct lwpacket *p;
        p = list_first_entry(&rx_buffer, struct lwpacket, list);
        list_del(&p->list);
        kzfree(p);
        i++;
        if (i > 100000000) {
            printk(KERN_WARNING DRIVER_NAME " lora_wan_close_comm : too many rx_buffer entries\n");
            break;
        }
    }
    spin_unlock_irqrestore(&rx_buffer_lock, irq_state);
    // todo : change this to KERN_DEBUG:
    printk(KERN_INFO DRIVER_NAME " lora_wan_close_comm : discarded %d rx_buffer entries\n", i);
    INIT_LIST_HEAD(&rx_buffer);
}
static void free_transmit_list(void)
{
    unsigned long irq_state;
    spin_lock_irqsave(&transmit_list_lock, irq_state);
    int i = 0;
    while ( ! list_empty(&transmit_list)) {
        struct lwpacket *p;
        p = list_first_entry(&transmit_list, struct lwpacket, list);
        list_del(&p->list);
        kzfree(p);
        i++;
        if (i > 100000000) {
            printk(KERN_WARNING DRIVER_NAME " lora_wan_close_comm : too many transmit_list entries\n");
            break;
        }
    }
    spin_unlock_irqrestore(&transmit_list_lock, irq_state);
    // todo : change this to KERN_DEBUG:
    printk(KERN_INFO DRIVER_NAME " lora_wan_close_comm : discarded %d transmit_list entries\n", i);
    INIT_LIST_HEAD(&transmit_list);
}

static int get_lora_raw_radio_count(void)
{
    int count = 0;
    const char delimiters[] = { LORA_RAW_RADIO_LIST_DELIMITER, '\0' };
    char *list;
    list = lora_raw_radio_list();
    if (list == NULL) {
        return 0;
    }
    char *p;
    p = list;
    while (1) {
        char *radio_name;
        radio_name = strsep(&p, delimiters);
        if (radio_name == NULL) {
            break;
        }
        if (radio_name[0] != '\0') {
            count++;
        }
    }
    kzfree(list);
    return count;
}

static int open_lora_raw_radio(void)
{
    const char delimiters[] = { LORA_RAW_RADIO_LIST_DELIMITER, '\0' };
    char *list;
    list = lora_raw_radio_list();
    printk(KERN_DEBUG DRIVER_NAME " lora_raw_radio_list returned %s\n", list);
    if (list == NULL) {
        return -ENODEV;
    }
    char *p;
    p = list;
    int result = -ENODEV;
    while (1) {
        char *radio_name;
        radio_name = strsep(&p, delimiters);
        if (radio_name == NULL) {
            break;
        }
        if (radio_name[0] != '\0') {
            lora_raw_handle = lora_raw_open(radio_name);
            if (lora_raw_handle.key < 0) {
                printk(KERN_WARNING DRIVER_NAME " failed to open radio %s.  error = %d\n", radio_name, lora_raw_handle.key);
                result = lora_raw_handle.key;  // save error code
                continue;       // try another radio if possible
            } else {
                printk(KERN_WARNING DRIVER_NAME " opened radio %s.\n", radio_name);
                result = 0;     // success
                break;
            }
        }
    }
    kzfree(list);
    return result;
}

static void close_lora_raw_radio(void)
{
    lora_raw_close(lora_raw_handle);
}





static void handle_receive_event(void);
static void handle_received_packet(struct lwpacket *packet);
struct lwpacket *get_transmit_packet(uint32_t device_id);
void tx_packet_add(struct lwpacket *packet);
void tx_packet_delete(struct lwpacket *packet);
static int lora_wan_transmit_window(uint32_t transmit_packet_uint32);



/**
 *  Must not sleep (to be called from interrupt or tasklet)
 *  Could conceivably be called reentrantly
 */
static void lora_wan_event_handler(void *context, int event)
{
    // for now we only support one radio but later on we can use context to tell us which radio the event is from. 
    // context gets passed in when event hander is registered as callback
    switch (event) {
        case LORA_CB_EVENT_RX:  // just receved a packet.
                handle_receive_event();
            break;
        default:
            printk(KERN_WARNING DRIVER_NAME " unhandled event %d in lora_wan_event_handler\n", event);
    }
}

/**
 *  Must not sleep (to be called from tasklet)
 */
static void handle_receive_event(void)
{
    static volatile unsigned long already_executing = 0;    // Make sure we don't re-enter this block if another IRQ happens
    if (test_and_set_bit(0, &already_executing) == 0) {     // If nobody is already executing : 
        while (1) {
            struct lwpacket *packet = lwpacket_create();
            if (packet == NULL) {
                printk(KERN_WARNING DRIVER_NAME " handle_receive_event failed to allocate packet\n");
                alloc_error_count++;
                break;
            }
            int result = lora_raw_rx(lora_raw_handle, (struct lora_raw_packet *) &packet->lr_packet);
            if (result == -EFBIG) {     // packet too big for buffer - only partial packet returned.
                printk(KERN_WARNING DRIVER_NAME " handle_receive_event discarding packet fragment\n");
                packet_fragment_count++;
                break;                
            }
            if (result < 0) {
                printk(KERN_WARNING DRIVER_NAME " handle_receive_event unexpected error = %d\n", result);
                unexpected_rx_error_count++;
                last_unexpected_rx_error = result;
                break;
            }
            packet->expiration_jiffies = jiffies + LORA_WAN_RX_EXPIRATION_JIFFIES;
            handle_received_packet(packet);
            
            if (result == 0) {  // no more packets to read.  
                break;
            }
            // if result > 0 then keep reading...
        }
        clear_bit(0, &already_executing);
    } else {
        printk(KERN_WARNING DRIVER_NAME " handle_receive_event skipped reentrant call.  Packet may be delayed.\n");
    }
}

/** The radio has received a packet
 *  Must not sleep (to be called from tasklet)
 */
static void handle_received_packet(struct lwpacket *packet)
{
    uint32_t device_id = lwpacket_get_device_id(packet);
    uint32_t arrival_time = packet->lr_packet.arrival_time;
    unsigned long irq_state;
    spin_lock_irqsave(&rx_buffer_lock, irq_state);
    list_add_tail(&packet->list, &rx_buffer);
    spin_unlock_irqrestore(&rx_buffer_lock, irq_state);
    
    // todo: call a higher level receive event handler so that it has a chance to 
    // process the received packet and issue a response before we prepare the 
    // response for transmission.
    
    // prepare response for transmission
    struct lwpacket *tx_packet = get_transmit_packet(device_id);
    if (tx_packet != NULL) {
        tx_packet_add(tx_packet);       // must add to list before scheduling so that the scheduled callback can remove it from list.
    
        // schedule transmit slots
        uint32_t transmit_time = lora_timer_future_time(arrival_time, 1 * LORA_TIMER_COUNTS_PER_SECOND);
        int timer_result = lora_timer_schedule_callback_with_retry(transmit_time, &lora_wan_transmit_window, (uint32_t) tx_packet);
        if (timer_result < 0) {
            printk(KERN_WARNING DRIVER_NAME " handle_receive_event failed to schedule transmit windows. error = %d.  TX packet dropped.\n", timer_result);
            // ignore result code - nothing we can do
            tx_packet_delete(tx_packet);
            dropped_transmit_packets++;
        } else {
            // future improvement : This could be reworked so that outgoing 
            //  traffic is only removed from tx_buffer after it has actually 
            //  transmitted (or perhaps even acknowledged).
        }
    }
}

/** Add a packet to transmit_list
 */
void tx_packet_add(struct lwpacket *packet)
{
    INIT_LIST_HEAD(&packet->list);
    packet->expiration_jiffies = jiffies + LORA_WAN_TRANSMIT_LIST_EXPIRATION_JIFFIES;
    unsigned long saved_irq_state;
    spin_lock_irqsave(&transmit_list_lock, saved_irq_state);
    list_add_tail(&packet->list, &transmit_list);
    spin_unlock_irqrestore(&transmit_list_lock, saved_irq_state);
}

void tx_packet_delete(struct lwpacket *packet)
{
    unsigned long saved_irq_state;
    spin_lock_irqsave(&transmit_list_lock, saved_irq_state);
    list_del(&packet->list);
    spin_unlock_irqrestore(&transmit_list_lock, saved_irq_state);
    kzfree(&packet->list);
}

/** It is time to transmit.  Transmit then remove the packet from its list and free it.
 *  Must not sleep (to be called from tasklet)
 *  @return 0 on success / < 0 on failure
 */
static int lora_wan_transmit_window(uint32_t transmit_packet_uint32)
{
    struct lwpacket *tx_packet;
    tx_packet = (struct lwpacket *) transmit_packet_uint32;
    
    // todo: should we set tx channel, datarate, etc. ?
    
    int tx_result = lora_raw_tx(lora_raw_handle, tx_packet->lr_packet.data, tx_packet->lr_packet.data_bytes);
    
    if (tx_result < 0) {
        return tx_result;
    }
    // else: packet is transmitted.
    tx_packet_delete(tx_packet);
    return 0;    
}

static void lw_user_packet_to_lwpacket(struct lwpacket *source, struct lw_user_packet *dest);


/** Get the next packet to be transmitted to device
 *  @return pointer to the packet which this function creates, or NULL if no packet to transmit. 
 *  Caller assumes responsibility to free the returned packet.
 *  Must not sleep.
 *  Future enhancement : figure out how to do this without (locking tx_buffer and disabling interrupts) continuously for so long..
 */
struct lwpacket *get_transmit_packet(uint32_t device_id)
{
    // First make sure we can get memory.
    struct lwpacket *x = NULL;
    x = kzalloc(sizeof(*x), GFP_ATOMIC);
    if (x == NULL) {
        alloc_error_count++;
        return NULL;
    }
    // lock tx_buffer while accessing to prevent concurrent access
    unsigned long saved_irq_state;
    spin_lock_irqsave(&tx_buffer_lock, saved_irq_state);   
//    gather data to transmit from tx_buffer
    struct tx_buffer_packet *found = NULL;
    struct tx_buffer_packet *p;
    list_for_each_entry(p, &tx_buffer, list) {
        if (p->packet.device_id == device_id) {
            found = p;
            list_del(&found->list);
            break;
        }
    }
    spin_unlock_irqrestore(&tx_buffer_lock, saved_irq_state);
    if (found == NULL) {
        kfree(x);
        return NULL;
    }
    lw_user_packet_to_lwpacket(x, &found->packet);
    kzfree(found);
    return x;
}




// todo: Perhaps we could rework list deletion so that items are only deleted 
// when we know they aren't going to be modified (including their list head.  
// This would enable us to do garbage collection 
// without locking the list.  If a list item is deactivated and its next and 
// prev are also deactivated out then it is safe to remove it even though someone
// might be adding or removing somewhere else in the list.
// Probably need to do deactivation first, then wait till next time to do removal.
// That way we don't fight with get_transmit_packet over a packet.
// If we do that we should add a new field for "active" or similar.

// todo: call check_garbage from a timer? or from user calls?

void collect_garbage(void);
#define GARBAGE_CHECK_INTERVAL_JIFFIES (10L * HZ)

void check_garbage(void)
{
    static long last_check_time;
    static bool need_to_init = true;
    if (need_to_init) {
        need_to_init = false;
        last_check_time = jiffies;
    }
    if ((long) jiffies - last_check_time > GARBAGE_CHECK_INTERVAL_JIFFIES) {
        last_check_time = jiffies;
        collect_garbage();
    }
}

void collect_garbage(void)
{
    // todo: how to do this so we aren't blocking interrupts for milliseconds?
//    result = trylock()
//    if (result >= 0) {
//        // todo: scan buffers and transmit_list for expired packets.  Remove them.
//    }
//    // if we can't get the lock right now that's ok we'll try again later
}

static void lwpacket_to_lw_user_packet(struct lw_user_packet *dest, struct lwpacket *source);


/** Read received packet
    @param packet - caller should allocate *packet before calling.  This function writes to *packet.
    @return 0 for no packet to read, > 0 for success (packet returned).  < 0 on error
 */
int lora_wan_rx(struct lw_user_packet *packet)
{
    // lock and disable interrupts to protect rx_buffer from concurrent access by this and rx event handler
    unsigned long saved_irq_state;
    spin_lock_irqsave(&rx_buffer_lock, saved_irq_state);   
    struct list_head *p = NULL;
    if ( ! list_empty(&rx_buffer)) {
        p = rx_buffer.next;
        list_del(p);
    }
    spin_unlock_irqrestore(&rx_buffer_lock, saved_irq_state);

    if (p == NULL) {
        return 0; // no packet to read
    }
    struct lwpacket *x;
    x = container_of(p, struct lwpacket, list);
    lwpacket_to_lw_user_packet(packet, x);
    kzfree(x);
    return 1;
}

static int smaller(int a, int b)
{ 
    return (a < b) ? a : b;
}

static void lwpacket_to_lw_user_packet(struct lw_user_packet *dest, struct lwpacket *source)
{
    // todo: add fields here as fields are added to lw_user_packet:
    dest->device_id = lwpacket_get_device_id(source);
    int source_bytes = lwpacket_get_data_bytes(source);

    // todo: strip off protocol layers and just return the application data
    int bytes_to_copy = smaller(source_bytes, sizeof(dest->data));
    if (bytes_to_copy < source_bytes) {
        printk(KERN_WARNING DRIVER_NAME " lwpacket_to_lw_user_packet : packet doesn't fit in lw_user_packet!?  packet truncated.\n");
        // lw_user_packet data should be the same size as lwpacket data.
        truncated_rx_packets++;
    }
    dest->data_bytes = bytes_to_copy;
    memcpy(dest->data, source->lr_packet.data, bytes_to_copy);      
}

static void lw_user_packet_to_lwpacket(struct lwpacket *source, struct lw_user_packet *dest)
{
    // todo
}


/** Send a packet to device
    @return >= 0 for success (packet buffered for transmission).  < 0 on error
 */
int lora_wan_tx(struct lw_user_packet *packet)
{
    struct tx_buffer_packet *p;
    p = kzalloc(sizeof(*p), GFP_KERNEL);
    if (p == NULL) {
        return -ENOMEM;
    }
    memcpy(&p->packet, packet, sizeof(p->packet));
    INIT_LIST_HEAD(&p->list);
    p->expiration_jiffies = jiffies + LORA_WAN_TX_EXPIRATION_JIFFIES;
    
    // lock and disable interrupts to protect tx_buffer from concurrent access by this and tx event handler
    unsigned long saved_irq_state;
    spin_lock_irqsave(&tx_buffer_lock, saved_irq_state);   
    list_add_tail(&p->list, &tx_buffer);
    spin_unlock_irqrestore(&tx_buffer_lock, saved_irq_state);
    return 0;                       // success.  now we wait for transmit window to send it.
}








//static struct proc_dir_entry *proc_file = NULL;

static void driver_cleanup(void);


static __init int lora_wan_driver_init(void)
{
    printk(KERN_INFO DRIVER_NAME " init\n");

    mutex_init(&lora_wan_lock);
    lora_wan_handles = 0;
    
    if (get_lora_raw_radio_count() <= 0) {
        printk(KERN_WARNING DRIVER_NAME " warning : no lora_raw radios found.\n");
    }
    
    // exported lora_wan functions are available now that the driver is initialized.

//    proc_file = proc_create("lora_wan", 0, NULL, &lora_wan_proc_fops);
//    if (proc_file == NULL) {
//        printk(KERN_WARNING DRIVER_NAME " failed to create proc file.\n");
//        // we can continue on without the proc file.
//    }
    
    return 0;
}

static void lora_wan_driver_exit(void)
{
    printk(KERN_INFO DRIVER_NAME " exit\n");

    driver_cleanup();
}

static void driver_cleanup(void)
{
    if (lora_wan_handles > 0) {
        printk(KERN_WARNING DRIVER_NAME " cleanup warning : %d open handles\n", lora_wan_handles);
    }
    
//    proc_remove(proc_file);
}

module_init(lora_wan_driver_init);
module_exit(lora_wan_driver_exit);


EXPORT_SYMBOL(lora_wan_open);
//EXPORT_SYMBOL(lora_wan_set_tx_channel);
//EXPORT_SYMBOL(lora_wan_set_rx_channel);
//EXPORT_SYMBOL(lora_wan_set_callback);
EXPORT_SYMBOL(lora_wan_tx);
//EXPORT_SYMBOL(lora_packet_is_ready);
EXPORT_SYMBOL(lora_wan_rx);
EXPORT_SYMBOL(lora_wan_close);



