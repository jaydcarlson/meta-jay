
#ifndef LORA_TIMER_H
#define LORA_TIMER_H

// lora_timer module
// Copyright (C) 2016 Solvz, Inc.
// All rights reserved.
// Written by Bruce Carlson, Signetik, LLC.
//
// This module implements a timer for the lora_raw driver.
// The timer is implemented using an i.MX6 hardware timer (EPIT)
// Callbacks can be scheduled to run at a certain time.

#include <linux/types.h>
#include <linux/of.h>

#define LORA_TIMER_COUNTS_PER_SECOND 1000000        // for this to be exact, it must be a factor of LORA_TIMER_CLOCK_INPUT_HZ

void lora_timer_reset(void);
int lora_timer_init(struct device_node *dt_timer);
void lora_timer_shutdown(void);

/** Get lora_timer time
    The lora timer counts up.
    The lora_timer is currently implemented as a 32-bit counter.
    So it underflows every 2^32 / LORA_TIMER_COUNTS_PER_SECOND seconds.
 */
uint32_t lora_timer_time(void);

/** Move the base time forward in time so many counts 
    @param base_time is a time obtained from lora_timer_time()
    @param delta_t_counts is how far ahead to move (or move backwards for 
    negative delta_t_counts)
    @return the adjusted time in lora_timer counts
 */
uint32_t lora_timer_future_time(uint32_t base_time, int32_t delta_t_counts);

/** how many lora timer counts have elapsed from start_time to end_time
 */
int32_t lora_timer_elapsed(uint32_t start_time, uint32_t end_time);

/** Is time a later than time b?
    @return 1 for yes / 0 for no 
 */
int lora_time_is_later(uint32_t time_a, uint32_t time_b);

struct schedule_item;
typedef struct schedule_item schedule_item_t;

struct schedule_item {
    schedule_item_t *next;
    int             active;                 // this schedule item is 1 (active) or 0 (inactive, awaiting destruction)
    uint32_t        call_time;
    schedule_item_t *retry;                 // points to a later scheduled retry so we can unschedule both if this succeeds.
    int           (*callback)(uint32_t);
    uint32_t        callback_data;
};

/** Schedule a function to be called later
    Note that callback might be called before this returns.
    callback will usually be called from an interrupt handler, 
    but it could occasionally be called from a normal kernel 
    context.
    @return 0 for success
            <0 for error
 */
int lora_timer_schedule_callback(uint32_t time, int (*callback)(uint32_t), uint32_t data);


/** Schedule a callback to be called either at call_time or SCHEDULE_RETRY_INTERVAL later
    (callback will only be called once if it succeeds).
    @return 0 on success or < 0 on failure
 */
int lora_timer_schedule_callback_with_retry(uint32_t call_time, int (*callback)(uint32_t), uint32_t callback_data);

/** Diagnostics: (read-only)
 */
extern long             lora_timer_missed_window_count;


#endif
