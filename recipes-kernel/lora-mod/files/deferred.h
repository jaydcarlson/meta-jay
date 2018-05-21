
// Copyright (C) 2016 Solvz, Inc.
// All rights reserved.
// Written by Bruce Carlson, Signetik, LLC.


#ifndef DEFERRED_H
#define DEFERRED_H

#include <linux/spinlock.h>

/** list of deferred tasks
    note that the lock field of only the head node is used.
 */
struct deferred_task {
    struct list_head list;
    void (*task)(void *, unsigned long);
    void *task_data1;
    unsigned long task_data2;
    spinlock_t lock;
};



void deferred_init_list(struct deferred_task *task_list);

void deferred_add_task(struct deferred_task *task_list, 
                       void (*task)(void *, unsigned long),
                       void *task_data1,
                       unsigned long task_data2);

void deferred_run_now(struct deferred_task *task_list);
                       
void deferred_destroy_list(struct deferred_task *task_list);







#endif



