
// Copyright 2016 Solvz, Inc.
// Written by Bruce Carlson, Signetik, LLC
// 
// Hardware abstraction layer for LoRa radio (SX1272) to support LMIC v1.5
// (LMIC = IBM LoRaWAN in C)
// refer to chapter 3 of LMiC-v1.5.pdf from 
// http://www.zurich.ibm.com/pdf/lrsc/lmic-release-v1.5.zip

/* This file built on the skeleton which was
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */


/** hardware requirements:
    digital outputs
        antenna switch RX
        antenna switch TX
        SPI chip select NSS
        reset line RST
    digital inputs with interrupts
        DIO0 - radio rx/tx done
    SPI controller (this requires a dedicated SPI controller which we will 
                        control directly)
    timer (precise)
*/


#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <dt-bindings/gpio/gpio.h>      // for GPIO_ACTIVE_LOW
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/completion.h>
#include <linux/delay.h>

#include "lora_timer.h"
#include "deferred.h"

#include "lmic/oslmic.h"
#include "lmic/radio.h"

#define MODULE_NAME "lora_raw:lmic_hal"

// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"


enum gpio_dir {
    GPIO_INPUT = 0,
    GPIO_OUTPUT = 1,
};
typedef enum gpio_dir gpio_dir_t;


static int hal_get_gpio(struct hal_gpio *p, 
                        struct device_node *dt_parent,
                        char *dt_gpio_node_name,
                        gpio_dir_t requested_direction, 
                        int requested_value, 
                        irq_handler_t irq_handler, 
                        void *handler_data);
static irqreturn_t hal_DIO0_irq_handler(int irq, void *data);
static void hal_cleanup(void);


static int timer_radio_count = 0;         // how many radios using timer
static DEFINE_SPINLOCK(timer_radio_count_lock);

/** Record of assertion failures:
 */
long  hal_fail_count = 0;
char *hal_last_fail_file = "";
long  hal_last_fail_line = 0;


#define HAL_NO_IRQ NULL

/** Initialize Radio data but do not claim resources
 */
void hal_init_radio(struct hal_radio *r, struct spi_device *spi_dev)
{
    memset(r, 0, sizeof(*r));
    deferred_init_list(&r->run_after_unlock);
    r->spi_dev = spi_dev;
    printk(KERN_DEBUG "init radio_lock\n");
    r->radio_lock = 0;   // unlocked
    
    struct device_node *dt_radio;
    dt_radio = r->spi_dev->dev.of_node;
    const char *radio_name = dt_radio->name;
    strncat(r->name, radio_name, sizeof(r->name)-1);
    // if you change this, make sure you don't overflow the names:
    int max_prefix = sizeof(r->reset_gpio.name) - 10;
    strncat(r->reset_gpio.name   , radio_name, max_prefix); strcat(r->reset_gpio.name   , ".reset"); 
    strncat(r->antenna_sw_tx.name, radio_name, max_prefix); strcat(r->antenna_sw_tx.name, ".sw_tx");
    strncat(r->radio_DIO0.name   , radio_name, max_prefix); strcat(r->radio_DIO0.name   , ".DIO0" );
}

/** Claim Radio resources
 */    
int hal_request_radio(struct hal_radio *r)
{
    struct device_node *dt_radio;
    dt_radio = r->spi_dev->dev.of_node;

    int timer_init_result = 0;
    spin_lock(&timer_radio_count_lock);
    timer_radio_count++;
    if (timer_radio_count <= 1) {   // we are the first user of the timer so we need to set it up:
        timer_radio_count = 1;
        struct device_node *dt_timer;
        dt_timer = of_parse_phandle(dt_radio, "lr-timer", 0);
        timer_init_result = lora_timer_init(dt_timer);
        of_node_put(dt_timer);
    }
    spin_unlock(&timer_radio_count_lock);
    if (timer_init_result < 0) {
        hal_release_radio(r);
        return timer_init_result;
    }
    
    int result1 = hal_get_gpio(&r->reset_gpio,    dt_radio, "lr-pin_reset"    , GPIO_INPUT,  0, HAL_NO_IRQ           , r);
    int result3 = hal_get_gpio(&r->antenna_sw_tx, dt_radio, "lr-pin_transmit" , GPIO_OUTPUT, 0, HAL_NO_IRQ           , r);
    // While lr-interrupt isn't strictly a pin, it fits the same format except that it has an interrupt sense
    //  instead of active_high/low  so we can pass it in here as long as we ignore its active_high field.
    int result5 = hal_get_gpio(&r->radio_DIO0,    dt_radio, "lr-interrupt"    , GPIO_INPUT,  0, &hal_DIO0_irq_handler, r);
    if (result1 < 0) { hal_release_radio(r); return result1; }
    if (result3 < 0) { hal_release_radio(r); return result3; }
    if (result5 < 0) { hal_release_radio(r); return result5; }
    return 0;
}

int hal_init(void)
{
    return 0;
}

long dt_read_u32(struct device_node *node, char * property_name, int index)
{
    u32 x;
    int dt_status = of_property_read_u32_index(node, property_name, index, &x);
    if (dt_status < 0) {
        printk(KERN_ERR " lora_raw error %d reading %s.%s from device tree\n", dt_status, node->name, property_name);
        return dt_status;
    }
    return x;
}


/** request access to a gpio pin, set direction and interrupt handler (edge triggered)
 */
static int hal_get_gpio(struct hal_gpio *p, 
                        struct device_node *dt_parent,
                        char *dt_gpio_property_name,
                        gpio_dir_t requested_direction, 
                        int requested_value, 
                        irq_handler_t irq_handler, 
                        void *handler_data)
{ 
    struct device_node *dt_gpio;
    dt_gpio = of_parse_phandle(dt_parent, dt_gpio_property_name, 0);
    int gpio_num = of_get_named_gpio(dt_parent, dt_gpio_property_name, 0);
    if (gpio_num < 0) {
        printk(KERN_ERR MODULE_NAME " error %d getting %s.%s from device tree.\n", gpio_num, dt_parent->name, dt_gpio_property_name);
        return gpio_num;
    }
    printk(KERN_DEBUG MODULE_NAME " hal_get_gpio getting %s.%s from device tree.  gpio_num = %d\n", dt_parent->name, dt_gpio_property_name, gpio_num);
    p->active_high = (dt_read_u32(dt_parent, dt_gpio_property_name, 2) & GPIO_ACTIVE_LOW) != GPIO_ACTIVE_LOW;
    of_node_put(dt_gpio);

    p->gpio_num = -1;
    p->irq_num = -1;
    if (gpio_num < 0) {
        return 0;       // no gpio pin requested
    }
    int gpio_result = gpio_request(gpio_num, p->name);
    if (gpio_result < 0) {
        printk(KERN_ERR MODULE_NAME " failed to get gpio %d, error = %d\n", gpio_num, gpio_result);
        return gpio_result;
    }
    p->gpio_num = gpio_num;

    int gpio_dir_result;
    if (requested_direction) {
        gpio_dir_result = gpio_direction_output(p->gpio_num, requested_value);
    } else {
        gpio_dir_result = gpio_direction_input(p->gpio_num);
    }
    if (gpio_dir_result < 0) {
        printk(KERN_ERR MODULE_NAME " failed to setup gpio %d pin direction, error = %d\n", gpio_num, gpio_dir_result);
        return gpio_dir_result;
    }
    
    if (gpio_cansleep(p->gpio_num)) {
        // if you are sure that it is ok to use a sleepy gpio 
        // then disable these lines:
        printk(KERN_ERR MODULE_NAME " gpio %d can sleep!  This is not supported.\n", gpio_num);
        return -EIO;
    }

    if (irq_handler != NULL) {
        p->irq_num = gpio_to_irq(gpio_num);
        if (p->irq_num < 0) {
            printk(KERN_WARNING MODULE_NAME " IRQ number not available for gpio %ld.  error = %d.\n", p->gpio_num, p->irq_num);
            return -EBUSY;
        } else {
            int irq_result = request_irq(p->irq_num, irq_handler, IRQF_TRIGGER_RISING, p->name, handler_data);
            // interrupt handler could fire at any time now.
            if (irq_result < 0) {
                printk(KERN_WARNING MODULE_NAME " failed to get IRQ %u.  error = %d.\n", p->irq_num, irq_result);
                p->irq_num = -1;
                return -EBUSY;
            } else {
                ; // got the irq - nothing to do here since irq_num is already set.
            }
        }
    }
    return 0; //success
}

static void hal_release_gpio(struct hal_gpio *p, void *handler_data);


static void hal_cleanup(void)
{
    lora_timer_shutdown();
}

void hal_release_radio(struct hal_radio *r)
{
    spin_lock(&timer_radio_count_lock);
    timer_radio_count--;
    if (timer_radio_count <= 0) {   // we are the last user of the timer so we need to shut it down:
        timer_radio_count = 0;
        lora_timer_shutdown();
    }
    spin_unlock(&timer_radio_count_lock);

    deferred_destroy_list(&(r->run_after_unlock));
    hal_release_gpio(&r->reset_gpio   , r);         // pass in the same pointer (r) that we passed when registering.
    hal_release_gpio(&r->antenna_sw_tx, r);
    hal_release_gpio(&r->radio_DIO0   , r);
}

void hal_shutdown(void) 
{
    hal_cleanup();
}

static void hal_release_gpio(struct hal_gpio *p, void *handler_data)
{ 
    if (p->gpio_num >= 0) {
        if (p->irq_num >= 0) {
            free_irq(p->irq_num, handler_data);
            p->irq_num = -1;
        }
        gpio_free(p->gpio_num);
        p->gpio_num = -1;
    }
}


void hal_gpio_set_value(struct hal_gpio *gpio, int new_value)
{
    if (gpio->gpio_num < 0) {
        return;         // no gpio - nothing to do.
    }
    int out_value = (new_value != 0);
    if ( ! gpio->active_high) {
        out_value = ! out_value;
    }
    gpio_set_value(gpio->gpio_num, out_value);
}


void hal_failed(char *file_name, long line_number)
{
    printk(KERN_ERR "LoRa radio ASSERT failed in %s at line %ld\n", file_name, line_number);

    hal_fail_count++;
    hal_last_fail_file = file_name;
    hal_last_fail_line = line_number;

    // todo: reboot?
}

/** Set antenna RF switch for transmitting (val == 1) / receiving (val == 0)
 */
void hal_pin_rxtx(struct hal_radio *r, u1_t val)
{
    hal_gpio_set_value(&r->antenna_sw_tx, val);
}

/** Drive LoRa chip reset 
    @param val 0 = low / 1 = high / 2 = floating
 */
void hal_pin_rst(struct hal_radio *r, u1_t val)
{
    printk(KERN_DEBUG "hal_pin_rst %d\n", val);
    
    // yes the datasheet really does call for this pin to sometimes be left floating.
    // we'll try to do that by making it an input.
    if (r->reset_gpio.gpio_num < 0) {
        return;         // no gpio - nothing to do.
    }
    int direction_result;
    if ((val == 0) || (val == 1)) {
        direction_result = gpio_direction_output(r->reset_gpio.gpio_num, val);
    } else {
        direction_result = gpio_direction_input(r->reset_gpio.gpio_num);
    }
    if (direction_result < 0) {
        printk(KERN_WARNING "%s:%s failed to set LoRa RESET pin direction\n", __FILE__, __FUNCTION__);
    }
}

static irqreturn_t hal_DIO0_irq_handler(int irq, void *data)
{
    // called whenever a packet is received or done transmitting.
    radio_irq_handler((struct hal_radio *) data);
    return IRQ_HANDLED;
}

struct hal_radio_message_data {
    struct hal_radio *r;
    struct spi_message msg;
    struct spi_transfer x_addr;
    struct spi_transfer x_buf;
    void (*run_on_completion)(struct hal_radio *, int);
    struct completion *x_complete;
    int *result;
    uint8_t addr;
    uint8_t data[1];   // variable length - must be last field in struct.
};

// completion call back for spi sync and async read/write routines
static void hal_spi_complete(void *hal_radio_message_data)
{
    struct hal_radio_message_data *m;
    m = (struct hal_radio_message_data *) hal_radio_message_data;
    if (m->msg.status < 0) {
        m->r->spi_error_count++;
        m->r->last_spi_error = m->msg.status;
    }
    int *t_result;
    t_result = m->result;
    if (t_result != NULL) {
        *t_result = m->msg.status;
    }
    if (m->run_on_completion != NULL) {
        m->run_on_completion(m->r, m->msg.status);
    }
    struct completion *t_complete;
    t_complete = m->x_complete;  // read it once in case it might change. TODO: better to remove this and make the null check and the complete() atomic
    if (t_complete != NULL) {
        complete(t_complete);
    }
    kzfree(m);
}


#define SPI_TRANSFER_TIMEOUT_JIFFIES    100

/** Write address and data to radio
 *  Block until transfer complete.  Must not sleep.
 *  (this function sleeps)
 *  @return 0 for success / < 0 on failure
 */
int hal_spi_write (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len) 
{
    addr |= 0x80;               // set bit 7 for write
    // allocate transfer complete on stack here because we must check it after the completion function frees m.
    struct completion transfer_complete; // to be completed by hal_spi_transfer_complete
    int transfer_result = 0;
    struct hal_radio_message_data *m;
    init_completion(&transfer_complete);
    m = kzalloc(sizeof(struct hal_radio_message_data) + len, GFP_KERNEL);
    if (m == NULL) {
        r->alloc_error_count++;
        if (m->run_on_completion != NULL) {
            m->run_on_completion(m->r, -ENOMEM);
        }
        return -ENOMEM;
    }
    m->r = r;
    spi_message_init(&m->msg);
    m->msg.complete = hal_spi_complete;
    m->msg.context = (void *) m;
    m->x_complete = &transfer_complete;
    m->result = &transfer_result;
    m->run_on_completion = NULL;
    m->addr = addr;
    m->x_addr.tx_buf = &m->addr;
    m->x_addr.len = 1;
    spi_message_add_tail(&m->x_addr, &m->msg);
    memcpy(m->data, buf, len);
    m->x_buf.tx_buf = m->data;
    m->x_buf.len = len;
    spi_message_add_tail(&m->x_buf, &m->msg);
    int async_result = spi_async(r->spi_dev, &m->msg);
    if (async_result != 0) {
        m->msg.status = async_result;   // pass error code to error handler
        hal_spi_complete(m); 
        printk(KERN_ERR "lora_raw hal_spi_write failed.  result = %d\n", async_result);
        return async_result;
    }
    unsigned long time_remaining = wait_for_completion_timeout(
                        &transfer_complete, SPI_TRANSFER_TIMEOUT_JIFFIES);
    if (time_remaining == 0) {  // timeout:
        // SPI controller is stalled.  We don't know when or if it will 
        // every complete the transfer.  We'd really like to remove the 
        // transfer from the SPI controller queue, but there doesn't 
        // seem to be a way to do that.  So it seems the best we can do 
        // is leave the transfer memory allocated and clear any pointers 
        // that point anywhere else.  If the SPI code doesn't check for 
        // NULL then it will crash.
        m->msg.complete = NULL; /// @bug If the transfer just completed now then m may already be deallocated. (unlikely)
                                /// @bug Now m will never be deallocated. (memory leak)  This is necessary because we don't know when the transaction will finish - maybe after the driver is unloaded.
        m->x_complete = NULL; 
        m->result = NULL;
        printk(KERN_CRIT "lora_raw hal_spi_write SPI is stalled.  Abandoning outstanding transfer.  Memory will leak.\n");
        r->spi_timeout_count++;
        return -EIO;
    }
    return transfer_result;
}

/** Write address and read data from radio
 *  Block until transfer complete.  Must not sleep.
 *  (this function sleeps)
 *  @return 0 for success / < 0 on failure
 */
int hal_spi_read (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len) 
{
    memset(buf, 0, len);        // return all zeroes on failure
    
    addr &= 0x7f;              // clear bit 7 for read
    struct completion transfer_complete; // to be completed by hal_spi_transfer_complete
    int transfer_result = 0;
    struct hal_radio_message_data *m;
    init_completion(&transfer_complete);
    m = kzalloc(sizeof(struct hal_radio_message_data), GFP_KERNEL);
    if (m == NULL) {
        r->alloc_error_count++;
        if (m->run_on_completion != NULL) {
            m->run_on_completion(m->r, -ENOMEM);
        }
        printk(KERN_ERR "lora_raw hal_spi_read failed to allocate memory.\n");
        return -ENOMEM;
    }
    m->r = r;
    spi_message_init(&m->msg);
    m->msg.complete = hal_spi_complete;
    m->msg.context = (void *) m;
    m->x_complete = &transfer_complete;
    m->result = &transfer_result;
    m->run_on_completion = NULL;
    m->addr = addr;
    m->x_addr.tx_buf = &m->addr;
    m->x_addr.len = 1;
    spi_message_add_tail(&m->x_addr, &m->msg);
    m->x_buf.rx_buf = buf;
    m->x_buf.len = len;
    spi_message_add_tail(&m->x_buf, &m->msg);
    int async_result = spi_async(r->spi_dev, &m->msg);
    if (async_result != 0) {
        m->msg.status = async_result;   // pass error code to run_on_completion
        hal_spi_complete(m); 
        printk(KERN_ERR "lora_raw hal_spi_read failed.  result = %d\n", async_result);
        return async_result;
    }
    unsigned long time_remaining = wait_for_completion_timeout(
                        &transfer_complete, SPI_TRANSFER_TIMEOUT_JIFFIES);
    if (time_remaining == 0) {  // timeout:
        // SPI controller is stalled.  We don't know when or if it will 
        // every complete the transfer.  We'd really like to remove the 
        // transfer from the SPI controller queue, but there doesn't 
        // seem to be a way to do that.  So it seems the best we can do 
        // is leave the transfer memory allocated and clear any pointers 
        // that point anywhere else.  If the SPI code doesn't check for 
        // NULL then it will crash.
        m->msg.complete = NULL; /// @bug If the transfer just completed now then m may already be deallocated. (unlikely)
                                /// @bug Now m will never be deallocated. (memory leak)  This is necessary because we don't know when the transaction will finish - maybe after the driver is unloaded.
        m->x_buf.rx_buf = NULL; /// @bug Race here between spi transfer completion and this. (unlikely)
        m->x_complete = NULL; 
        m->result = NULL;
        printk(KERN_CRIT "lora_raw hal_spi_read SPI is stalled.  Abandoning outstanding transfer.  Memory will leak.\n");
        r->spi_timeout_count++;
        return -EIO;
    }
    return transfer_result;
}




/** Write address and read data from radio
 *  @param kalloc_flags should probably be either GFP_KERNEL or GFP_ATOMIC 
 *  Return before transfer complete. 
 *  On success caller must keep *buf allocated until run_on_completion runs.
 *  On error, run_on_completion is passed the error code.
 */
void hal_spi_read_async (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len,
                        void (*run_on_completion)(struct hal_radio *, int),
                        gfp_t kalloc_flags) 
{
    addr &= 0x7f;              // clear bit 7 for read
    
    struct hal_radio_message_data *m;
    m = kzalloc(sizeof(struct hal_radio_message_data), kalloc_flags);
    if (m == NULL) {
        r->alloc_error_count++;
        if (m->run_on_completion != NULL) {
            m->run_on_completion(m->r, -ENOMEM);
        }
        return;
    }
    m->r = r;
    spi_message_init(&m->msg);
    m->msg.complete = hal_spi_complete;
    m->msg.context = (void *) m;
    m->run_on_completion = run_on_completion;
    m->x_complete = NULL;
    m->result = NULL;
    m->addr = addr;
    m->x_addr.tx_buf = &m->addr;
    m->x_addr.len = 1;
    spi_message_add_tail(&m->x_addr, &m->msg);
    m->x_buf.rx_buf = buf;
    m->x_buf.len = len;
    spi_message_add_tail(&m->x_buf, &m->msg);
    int xfer_result = spi_async(r->spi_dev, &m->msg);
    if (xfer_result != 0) {
        m->msg.status = xfer_result;   // pass error code to run_on_completion
        hal_spi_complete(m); 
    }
}

/** Write address data to radio
 *  @param kalloc_flags should probably be either GFP_KERNEL or GFP_ATOMIC 
 *  Return before transfer complete. 
 *  Caller does not need to maintain buf allocated after return.
 *  On error run_on_completion is passed the error code.
 */
void hal_spi_write_async (struct hal_radio *r, u1_t addr, u1_t *buf, u1_t len,
                        void (*run_on_completion)(struct hal_radio *, int),
                        gfp_t kalloc_flags) 
{
    addr |= 0x80;               // set bit 7 for write
    
    struct hal_radio_message_data *m;
    m = kzalloc(sizeof(struct hal_radio_message_data) + len, kalloc_flags);
    if (m == NULL) {
        r->alloc_error_count++;
        if (m->run_on_completion != NULL) {
            m->run_on_completion(m->r, -ENOMEM);
        }
        return;
    }
    m->r = r;
    spi_message_init(&m->msg);
    m->msg.complete = hal_spi_complete;
    m->msg.context = (void *) m;
    m->run_on_completion = run_on_completion;
    m->x_complete = NULL;
    m->result = NULL;
    m->addr = addr;
    m->x_addr.tx_buf = &m->addr;
    m->x_addr.len = 1;
    spi_message_add_tail(&m->x_addr, &m->msg);
    memcpy(m->data, buf, len);
    m->x_buf.tx_buf = m->data;
    m->x_buf.len = len;
    spi_message_add_tail(&m->x_buf, &m->msg);
    int xfer_result = spi_async(r->spi_dev, &m->msg);
    if (xfer_result != 0) {
        m->msg.status = xfer_result;   // pass error code to run_on_completion
        hal_spi_complete(m); 
    }
}


static int radio_try_lock(struct hal_radio *r)
{
    int already_locked = test_and_set_bit(0, &r->radio_lock);
    return ! already_locked;
}

void hal_radio_lock(struct hal_radio *r)
{
    printk(KERN_DEBUG "hal_radio_lock\n");
    
    while ( ! radio_try_lock(r)) {
        msleep(5);
    }
}


/** try to get radio lock - return 1 on success / 0 on failure
 */
int hal_radio_try_lock(struct hal_radio *r)
{
    printk(KERN_DEBUG "hal_radio_trylock\n");
    return radio_try_lock(r);
}

void hal_radio_unlock(struct hal_radio *r)
{
    printk(KERN_DEBUG "hal_radio_unlock\n");
    clear_bit(0, &r->radio_lock);

    printk(KERN_DEBUG "hal_radio_unlock : running deferred tasks:\n");
    // While we were using the SPI, an interrupt may have queued up 
    // communications that need to be done now.
    deferred_run_now(&r->run_after_unlock);
}

/** Get 32-bit system time in ticks 
    it is unspecified how long a tick should be (?)
 */
u4_t hal_ticks(void)
{
    return lora_timer_time();
}

// this function copied here from oslmic.c
ostime_t os_getTime (void) {
    return hal_ticks();
}

/** wait until time (ticks) is reached
    (this is used by some hardware delays on init, and also for rx timeouts)
 */
void hal_waitUntil(u4_t time)
{
   while ( ! lora_time_is_later(os_getTime(), time)) {
       ; // wait
   }
}


/** Add a task to run right after the (currently held) lock is released
    (this function expected to run from interrupt handler)
 */
void hal_radio_do_after_unlock(struct hal_radio *r,
                              void (*task)(void *, unsigned long),
                              unsigned long task_data)
{
    deferred_add_task(&r->run_after_unlock, task, r, task_data);
}



