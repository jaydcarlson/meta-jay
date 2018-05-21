
// lora_timer module
// Copyright (C) 2016 Solvz, Inc.
// All rights reserved.
// Written by Bruce Carlson, Signetik, LLC.
//
// This module implements a timer for the lora_raw driver.
// The timer is implemented using an i.MX6 hardware timer (EPIT)
// Callbacks can be scheduled to run at a certain time.

// We are not using the epit.c module because that module does not properly 
//  handle use of both EPIT peripherals at the same time.


#include <linux/vmalloc.h>      // for ioremap_nocache
#include <linux/ioport.h>       // for request_region
#include <asm/io.h>             
#include <linux/interrupt.h>    // for request_irq
#include <asm/delay.h>          // for udelay
#include <linux/slab.h>         // for kzalloc, etc.
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/delay.h>


#include "lora_timer.h"

// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"



#define LORA_TIMER_CLOCK_INPUT_HZ   66000000        
/// the lora timer counts at the clock frequency divided by LORA_TIMER_PRESCALE:

#define SCHEDULE_LATE_TOLERANCE                50   // how late  is tolerable - in lora_timer counts.  Divide by LORA_TIMER_COUNTS_PER_SECOND to convert to seconds.
#define SCHEDULE_EARLY_TOLERANCE               10   // how early is tolerable - in lora_timer counts.  Divide by LORA_TIMER_COUNTS_PER_SECOND to convert to seconds.

#define SCHEDULE_NUMBER_OF_RETRIES              1   // how many times a missed window can be postponed.
#define SCHEDULE_RETRY_INTERVAL (LORA_TIMER_COUNTS_PER_SECOND)


#define LORA_TIMER_PRESCALE (LORA_TIMER_CLOCK_INPUT_HZ / LORA_TIMER_COUNTS_PER_SECOND)
#if LORA_TIMER_PRESCALE > 4096
#error the EPIT prescaler only goes up to 4096.
#endif


// EPIT peripheral (Reference Manual section 25.6)
#define EPIT_LEN    0x00000100 

// EPIT register          offset from EPITx_BASE
#define EPIT_CR           0x00     
#define EPIT_SR           0x04
#define EPIT_LR           0x08
#define EPIT_CMPR         0x0c
#define EPIT_CNR          0x10

// EPIT control register (CR) bits (Reference Manual section 25.6.1)
#define EPIT_CR_CLKSRC_SHIFT        24
#define EPIT_CR_CLKSRC_M            (3 << EPIT_CR_CLKSRC_SHIFT)
#define EPIT_CR_CLKSRC_OFF          (0 << EPIT_CR_CLKSRC_SHIFT)
#define EPIT_CR_CLKSRC_PERIPHERAL   (1 << EPIT_CR_CLKSRC_SHIFT)
#define EPIT_CR_CLKSRC_HIGH_REF     (2 << EPIT_CR_CLKSRC_SHIFT)
#define EPIT_CR_CLKSRC_LOW_REF      (3 << EPIT_CR_CLKSRC_SHIFT)
#define EPIT_CR_OM_SHIFT            22
#define EPIT_CR_OM_M                (3 << EPIT_CR_OM_SHIFT)
#define EPIT_CR_OM_OFF              (0 << EPIT_CR_OM_SHIFT)
#define EPIT_CR_OM_TOGGLE           (1 << EPIT_CR_OM_SHIFT)
#define EPIT_CR_OM_CLEAR            (2 << EPIT_CR_OM_SHIFT)
#define EPIT_CR_OM_SET              (3 << EPIT_CR_OM_SHIFT)
#define EPIT_CR_STOPEN              (1 << 21)
#define EPIT_CR_WAITEN              (1 << 19)
#define EPIT_CR_DBGEN               (1 << 18)
#define EPIT_CR_IOVW                (1 << 17)
#define EPIT_CR_SWR                 (1 << 16)
#define EPIT_CR_PRESCALAR_SHIFT     4
#define EPIT_CR_PRESCALAR_M         (0xfff << EPIT_CR_PRESCALAR_SHIFT)
#define EPIT_CR_RLD                 (1 << 3)
#define EPIT_CR_OCIEN               (1 << 2)
#define EPIT_CR_ENMOD               (1 << 1)
#define EPIT_CR_EN                  (1 << 0)

#define EPIT_SR_OCIF                1




static void             *epit_base = NULL;
static struct resource  *epit_mem = NULL;
static uint32_t         epit_base_addr = 0;
static int              epit_irq = -1;

long             lora_timer_missed_window_count = 0;


/** Set the masked bits of timer register at timer_register_offset to values
    Interrupts are disabled during read-modify-write
    @param timer_register_offset should not include epit_base
 */
static inline void set_timer_bits_atomic(uint32_t timer_register_offset, uint32_t mask, uint32_t values)
{
    values &= mask;
    uint32_t not_mask = ~mask;
    void *reg = epit_base + timer_register_offset;
    unsigned long saved_flags;
    local_irq_save(saved_flags);    // this disables interrupts on current processor
	uint32_t orig = __raw_readl(reg);
	__raw_writel((orig & not_mask) | values, reg);
    local_irq_restore(saved_flags); 
}
static inline void set_timer_bits(uint32_t timer_register_offset, uint32_t mask, uint32_t values)
{
    values &= mask;
    uint32_t not_mask = ~mask;
    void *reg = epit_base + timer_register_offset;
	uint32_t orig = __raw_readl(reg);
	__raw_writel((orig & not_mask) | values, reg);
}
static inline void set_timer_word(uint32_t timer_register_offset, uint32_t new_val)
{
	__raw_writel(new_val, epit_base + timer_register_offset);
}
static inline uint32_t get_timer_word(uint32_t timer_register_offset)
{
	return __raw_readl(epit_base + timer_register_offset);
}


static inline void lora_timer_disable_interrupt(void)
{
    //EPIT_CR.OCIEN = 0;
    set_timer_bits_atomic(EPIT_CR, EPIT_CR_OCIEN, 0);
}

static inline void lora_timer_enable_interrupt(void)
{
    //EPIT_CR.OCIEN = 1;
    set_timer_bits_atomic(EPIT_CR, EPIT_CR_OCIEN, 0xffffffff);
}

static inline void lora_timer_clear_interrupt(void)
{
    // clear output compare match flag:
    //EPIT_EPIT_SR.OCIF = 1;        // write 1 to clear status register
    set_timer_bits_atomic(EPIT_SR, EPIT_SR_OCIF, 0xffffffff);
}

static void lora_timer_force_interrupt(void);
int lora_timer_request_epit(uint32_t new_epit_base_addr, int irq);


static irqreturn_t lora_timer_int_handler(int irq, void *dev_id);


void lora_timer_reset(void)
{
    lora_timer_disable_interrupt();
    //EPIT_CR.SWR = 1;    // software reset of the EPIT module
    set_timer_bits(EPIT_CR, EPIT_CR_SWR, 0xffffffff);
    wmb();
    udelay(1);              // give it a moment to reset (might not be necessary)
}

static char lora_timer_name[] = "LoRa Timer";

int lora_timer_init(struct device_node *dt_timer)
{
    if (dt_timer == NULL) {
        printk(KERN_ERR " lora_timer error - invalid timer - check lr-timer in device tree.\n");
        return -ENXIO;
    }
        
    u32 new_epit_base_addr;
    u32 new_epit_len;
    int dt_status = of_property_read_u32_index(dt_timer, "reg", 0, &new_epit_base_addr);
    if (dt_status < 0) {
        printk(KERN_ERR " lora_timer error %d reading timer base address from device tree\n", dt_status);
        return -ENXIO;
    }
    dt_status = of_property_read_u32_index(dt_timer, "reg", 1, &new_epit_len);
    if (dt_status < 0) {
        printk(KERN_ERR " lora_timer error %d reading timer register length from device tree\n", dt_status);
        return -ENXIO;
    }

    int new_epit_irq = of_irq_get(dt_timer, 0);
    if (new_epit_irq < 0) {
        printk(KERN_ERR " lora_timer error %d reading irq number from device tree.\n", new_epit_irq);
        return new_epit_irq;
    }
    
    // get an EPIT :
    // this sets epit_base and epit_irq
    // too bad c functions can't return multiple values
    
    lora_timer_request_epit(new_epit_base_addr, new_epit_irq);  // It would be proper to also pass in new_epit_len.

    if (epit_base == NULL) {    // no EPIT
        printk(KERN_ERR "Lora_Timer can't access EPIT.\n");
        return -EBUSY;
    }
    
    printk(KERN_DEBUG " Lora_Timer using EPIT at base 0x%08x and interrupt %d\n", epit_base_addr, epit_irq);
   
    
    // Start EPIT timer:
    // Apparently the Clock Control Module (CCM) is already set up to to 
    //  provide clock to the timer.

    lora_timer_reset();

    // register lora_timer_int_handler as the EPIT compare match interrupt handler
//    int irq_result = request_irq(epit_irq, &lora_timer_int_handler, SA_INTERRUPT, NULL); 
    // Steve requests we not use fast interrupts (SA_INTERRUPT) for now
    printk(KERN_DEBUG  " Lora_Timer requesting irq %d\n", epit_irq);
    
    int irq_result = request_irq(epit_irq, &lora_timer_int_handler, 0, "lora_timer_int", NULL); 
    if (irq_result < 0) {
        printk(KERN_ERR "LoRa Timer could not get EPIT interrupt\n");
        lora_timer_shutdown();
        return -EBUSY;
    }
    printk(KERN_DEBUG " Lora_Timer got irq\n");

    // set clock source (this sequence from Techical Reference section 25.5.1)

    set_timer_bits(EPIT_CR, EPIT_CR_EN, 0);   // disable EPIT before changing clock source setting
    set_timer_bits(EPIT_CR, EPIT_CR_OM_M, 0); // disable EPIT output before changing clock
    wmb();

    // set clock to peripheral clock (ipg_clk)
    set_timer_bits(EPIT_CR, EPIT_CR_CLKSRC_M, EPIT_CR_CLKSRC_PERIPHERAL); 
    
    set_timer_bits(EPIT_CR, EPIT_CR_STOPEN, 0); 
    set_timer_bits(EPIT_CR, EPIT_CR_WAITEN, 0);
    set_timer_bits(EPIT_CR, EPIT_CR_RLD, 0);    // free-run mode (count from 0xffffffff down to 0 and wrap) 
    set_timer_bits(EPIT_CR, EPIT_CR_IOVW, 0);   // ignore writes to load register
    set_timer_bits(EPIT_CR, EPIT_CR_ENMOD, 0);  // don't reload timer counter on stop/wait
    set_timer_bits(EPIT_CR, EPIT_CR_PRESCALAR_M, (LORA_TIMER_PRESCALE - 1) << EPIT_CR_PRESCALAR_SHIFT);
    lora_timer_disable_interrupt(); // interrupt will be enabled when something gets scheduled.

    lora_timer_clear_interrupt();     // clear the flag in case it might be set.
    
    // enable counter
    set_timer_bits(EPIT_CR, EPIT_CR_EN, 0xffffffff);

    // verify counter is counting:
    uint32_t before = lora_timer_time();
    mdelay(5);
    uint32_t after = lora_timer_time();
    if (after - before == 0) {
        printk(KERN_WARNING " lora_timer is not counting\n");
        return -EIO;
    }
    return 0;
}

/** request an EPIT from the kernel.
    Does nothing if we already have an EPIT (evidenced by epit_base != NULL)
    MODIFIES epit_mem, epit_base, epit_base_addr, epit_irq
    on successful exit, epit_base is != NULL.  On failed exit epit_base == NULL
 */
int lora_timer_request_epit(uint32_t new_epit_base_addr, int irq)
{
    if (epit_base != NULL) {
        return -EEXIST;
    }
    epit_irq = -1;
    epit_base_addr = 0;
    epit_mem = request_mem_region(new_epit_base_addr, EPIT_LEN, lora_timer_name);
    if (epit_mem == NULL) {
        return -EBUSY;
    }
    epit_base = ioremap_nocache(new_epit_base_addr, EPIT_LEN);
    if (epit_base  == NULL) {
        release_mem_region(new_epit_base_addr, EPIT_LEN);
        epit_mem = NULL;
        return -EIO;
    }
    epit_base_addr = new_epit_base_addr;
    epit_irq = irq;
    return 0;
}
    

void lora_timer_shutdown(void)
{
    if (epit_base > 0) {
        lora_timer_reset();
    }

    if (epit_irq > 0) {
        free_irq(epit_irq, NULL);
        epit_irq = -1;
    }
    
    if (epit_mem > 0) {
        release_mem_region(epit_base_addr, EPIT_LEN);
        epit_mem = NULL;
        epit_base_addr = 0;
    }
    
    if (epit_base > 0) {
        iounmap(epit_base);        
        epit_base = NULL;
    }
}
    
static inline uint32_t epit_to_lora_timer(uint32_t epit_time)
{
    return 0xffffffff - epit_time;          // epit counts backwards.  lora_timer counts up.
}

static inline uint32_t lora_timer_to_epit(uint32_t lora_timer_value)
{
    return 0xffffffff - lora_timer_value;   // epit counts backwards.  lora_timer counts up.
}

/** Move the base time forward in time so many counts 
    @param base_time is a time obtained from lora_timer_time()
    @param delta_t_counts is how far ahead to move (or move backwards for 
    negative delta_t_counts)
    @return the adjusted time in lora_timer counts
 */
uint32_t lora_timer_future_time(uint32_t base_time, int32_t delta_t_counts)
{
    return (uint32_t) ((int32_t) base_time + delta_t_counts);   // overflow is ok here
                                                                // not sure if all the casts are necessary
}

/** Get lora_timer time
    The lora timer counts up.
    The lora_timer is currently implemented as a 32-bit counter.
    So it overflows every 2^32 / LORA_TIMER_COUNTS_PER_SECOND seconds.
 */
uint32_t lora_timer_time(void)
{
    // return EPIT_CNR;
    return epit_to_lora_timer(__raw_readl(epit_base + EPIT_CNR));
}

/** how many lora timer counts have elapsed from start_time to end_time
 *  @return 0 for start_time == end time, 
            > 0 for start_time is before end_time, or 
            < 0 for start_time is after end_time
 *  Assumes start_time and end_time are near each other (within 2^31 ticks).
 */
int32_t lora_timer_elapsed(uint32_t start_time, uint32_t end_time)
{
    return (int32_t) (end_time - start_time);
}
/** Is time a later than time b?
    @return 1 for yes / 0 for no 
 */
int lora_time_is_later(uint32_t time_a, uint32_t time_b)
{
    return (lora_timer_elapsed(time_b, time_a) > 0);
}

static schedule_item_t *schedule;
/// schedule is a singly-linked list of callbacks, ordered by time (soonest first)

static void schedule_clean(void);


static schedule_item_t *schedule_item_create(uint32_t call_time, schedule_item_t *retry, int (*callback)(uint32_t), uint32_t callback_data)
{
    schedule_item_t *p;
    p = kzalloc(sizeof(schedule_item_t), GFP_KERNEL);
    if (p != NULL) {
        p->next = NULL;
        p->active = 1;
        p->call_time = call_time;
        p->retry = retry;
        p->callback = callback;
        p->callback_data = callback_data;
    }
    return p;
}

static void schedule_item_destroy(schedule_item_t *p)
{
    kzfree(p);
}

static schedule_item_t * int_next_schedule_item = NULL;     // for use by lora_timer_int_handler


/** try to add the schedule item to the schedule    
    @return 0 for success, < 0 for failure.
 */
static int schedule_add(schedule_item_t *x)
{
    static DEFINE_SPINLOCK(schedule_lock);
    
    schedule_clean();

    int32_t how_long_before_call = lora_timer_elapsed(lora_timer_time(), x->call_time);
    
    // If time just expired or is about to expire, call it now.
    // This attempts to avoid a race between the scheduling and the timer.
    if ((how_long_before_call > - SCHEDULE_LATE_TOLERANCE) &&
        (how_long_before_call <   SCHEDULE_EARLY_TOLERANCE)) {

        x->callback(x->callback_data);
        x->active = 0;
        schedule_item_destroy(x);
        return 0;
    }
    
    if (how_long_before_call <= 0) {    // if deadline is already long gone
        return -EBADCOOKIE;             // caller is a bad cookie.
    }
    
    // Find where in the list this item should go (list must be in order by time)
    // (It is ok for the interrupt to access the list now, but make sure two threads 
    // don't do this code at the same time.)
    spin_lock(&schedule_lock);
    schedule_item_t **p;
    p = &schedule;
    while ((*p != NULL) && (lora_time_is_later(x->call_time, (*p)->call_time))) {
        p = &((*p)->next);
    }
    x->next = *p;
    *p = x;         // add x to the schedule.
    int_next_schedule_item = schedule;      // pointer write is assumed to be atomic.  problems here if not.
    spin_unlock(&schedule_lock);
    
    lora_timer_force_interrupt();           // interrupt handler will sort out the schedule
    return 0;
}

/** Schedule a new call, return pointer to its schedule item
    @return pointer to the newly added schedule item, or NULL on error
    errors can be failure to allocate memory, or the time is already long past
 */
schedule_item_t *schedule_new_attempt(uint32_t call_time, schedule_item_t *retry, int (*callback)(uint32_t), uint32_t callback_data)
{
    schedule_item_t *x;
    x  = schedule_item_create(call_time, retry, callback, callback_data);
    if (x == NULL) {
        return NULL;
    }
    int add_result = schedule_add(x);
    if (add_result != 0) {
        schedule_item_destroy(x);
        return NULL;
    }
    return x;
}

/** Schedule a function to be called later
    Note that callback might be called before this returns.
    callback will usually be called from an interrupt handler, 
    but it could occasionally be called from a normal kernel 
    context.
    @return 0 for success
            <0 for error
 */
int lora_timer_schedule_callback(uint32_t time, int (*callback)(uint32_t), uint32_t data)
{
    schedule_item_t *x;
    x = schedule_new_attempt(time, NULL, callback, data);
    if (x == NULL) {
        return -EPIPE;  // check your basement for broken pipes.  it's either out of memory or already expired.
    }
    return 0;
}


/** Schedule 2 new calls, one being a later retry of the first
    @return 0 on success or < 0 on failure
 */
int lora_timer_schedule_callback_with_retry(uint32_t call_time, int (*callback)(uint32_t), uint32_t callback_data)
{
    schedule_item_t *first;
    schedule_item_t *second;
    uint32_t second_time = call_time + SCHEDULE_RETRY_INTERVAL;
    second = schedule_new_attempt(second_time, NULL, callback, callback_data);
    first  = schedule_new_attempt(call_time, second, callback, callback_data);
    if ((first == NULL) || (second == NULL)) {
        return -EPIPE;  // it's either out of memory or already expired.
    }
    return 0;
}


/** remove the item *p from list pointed to by p.  return a pointer to the removed item.
 */
static schedule_item_t *schedule_remove_head(schedule_item_t **p)
{
    schedule_item_t *x;
    if ((p == NULL) || (*p == NULL)) {
        return NULL;
    }
    x = *p;         // remove item from list before clearing its next
    *p = x->next;
    x->next = NULL;
    return x;
}

/** Clean the schedule - remove and destroy any items that aren't active or that we missed
 */
static void schedule_clean(void)
{
    schedule_item_t **p;
    p = &schedule;
    while (*p != NULL) {
        if ((*p)->active) {
            int32_t how_long_before_call = lora_timer_elapsed(lora_timer_time(), (*p)->call_time);

            if (how_long_before_call < - SCHEDULE_LATE_TOLERANCE) {  // we missed this window 
                (*p)->active = 0;     // first clear active so the interrupt doesn't try to use it.
                if ((*p)->retry == NULL) {    
                    lora_timer_missed_window_count++;
                }
            }
        }
        if ( ! (*p)->active) {
            schedule_item_t *x;
            x = schedule_remove_head(p);
            schedule_item_destroy(x);
        } else {
            p = &((*p)->next);
        }
    }
}



/** Interrupt handler to be called on timer compare match
 */
static irqreturn_t lora_timer_int_handler(int irq, void *dev_id)
{
    schedule_item_t *p;
    p = int_next_schedule_item;     // this effectively makes p static, except it is sometimes written by schedule_add

    lora_timer_clear_interrupt();

    int i = 0;
    while (p != NULL) {
        i++;
        if (i > 1000000) {
            printk(KERN_ERR "Error: circular list in %s %s\n", __FILE__, __FUNCTION__);
            p = NULL;
            break;
        }
        
        if ( ! p->active) {     // if it's not active skip it
            p = p->next;        
            
            continue;
        }
        
        int32_t how_long_before_call = lora_timer_elapsed(lora_timer_time(), p->call_time);
        
        // if not time yet
        if (how_long_before_call > SCHEDULE_EARLY_TOLERANCE) {
            
            break;  // EXIT LOOP
            
        }
        // otherwise it is time or past time
        
        p->active = 0;
        
        // if time to call it:
        if (how_long_before_call >= - SCHEDULE_LATE_TOLERANCE) {
            
            int callback_result = p->callback(p->callback_data);
            
            if ((callback_result >= 0) && (p->retry != NULL)) {    // this is handled, so deactivate retry (if any)
                p->retry->active = 0;  // (only one retry supported)
            }
            
        } else { // we missed this window 
            if (p->retry == NULL) {    // no more retries :
                lora_timer_missed_window_count++;
            }
        }
    };
    int_next_schedule_item = p;         // save for next time
    
    if (p != NULL) {     // more calls in schedule
        // now p has the next call to make.
        // schedule next interrupt : 
        set_timer_word(EPIT_CMPR, lora_timer_to_epit(p->call_time));
    } else {
        // It would be proper to disable timer compare interrupts here, but it won't hurt anything to leave it running.
    }

	return IRQ_HANDLED;
}
    
/** Force a lora timer interrupt to occur real soon.
    On exit lora timer interrupts are enabled.
    Funny thing is, if you call this frequently enough, you could prevent the 
     interrupt from ever executing..
 */
static void lora_timer_force_interrupt(void)
{
    lora_timer_enable_interrupt();
    
    // Interrupts are disabled to assure timer doesn't count more than one 
    // tick between read of CNR and write of EPITCMPR.  We add one additional 
    // tick in case the interrupt trigger might be sensitive only to the 
    // change in count from CMPR-1 to COMPR
    unsigned long saved_flags;
    local_irq_save(saved_flags);    // this disables interrupts on current processor
    // EPIT_EPITCMPR = EPIT_CNR - 2;   // trigger after 2 timer counts
    set_timer_word(EPIT_CMPR, get_timer_word(EPIT_CNR) - 2);
    local_irq_restore(saved_flags); 
}

EXPORT_SYMBOL(lora_timer_schedule_callback_with_retry);
