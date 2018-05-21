

// Copyright (C) 2016 Solvz, Inc.
// All rights reserved.
// Written by Bruce Carlson, Signetik, LLC.
//
// This module implements a list of deferred tasks (function calls) to be run
// later.


#include <linux/list.h>
#include <linux/slab.h>

#include "deferred.h"


void deferred_init_list(struct deferred_task *task_list)
{
    INIT_LIST_HEAD(&(task_list->list));
    spin_lock_init(&task_list->lock);
    task_list->task = NULL;
    task_list->task_data1 = NULL;
    task_list->task_data2 = 0;
}

/** Add a deferred task to a list of deferred tasks
    (this function expected to run from interrupt handler)
 */
void deferred_add_task(struct deferred_task *task_list, 
                       void (*task)(void *, unsigned long),
                       void *task_data1,
                       unsigned long task_data2)
{
    unsigned long irq_flags;
    struct deferred_task *t;
    t = kzalloc(sizeof(*t), GFP_ATOMIC);
    INIT_LIST_HEAD(&t->list);
    t->task = task;
    t->task_data1 = task_data1;
    t->task_data2 = task_data2;
    spin_lock_irqsave(&task_list->lock, irq_flags);
    list_add_tail(&t->list, &task_list->list);
    spin_unlock_irqrestore(&task_list->lock, irq_flags);
}

/** Destroy the list of deferred tasks list and free the memory
    (note that the first item, *task_list, is not freed (assumed to be static)
 */
void deferred_destroy_list(struct deferred_task *task_list) 
{
    unsigned long irq_flags;
    spin_lock_irqsave(&task_list->lock, irq_flags);
    while ( ! list_empty(&task_list->list)) {
        struct deferred_task *task;
        task = list_entry(task_list->list.next, struct deferred_task, list);
        list_del(&task->list);
        kzfree(task);
    }
    spin_unlock_irqrestore(&task_list->lock, irq_flags);
}

/** Run deferred tasks and remove them from the list and free their memory
    (*task_list is not run or freed (assumed to be only the list head), 
    but every other item in its list is run and freed.)
    
    If one of these deferred tasks reschedules itself (into task_list), it 
    will run again later, not now.
 */
void deferred_run_now(struct deferred_task *task_list) 
{
    // First save task_list as x and then clear task_list so it can 
    // be repopulated immediately if a task calls deferred_add_task
    struct deferred_task x;
    unsigned long irq_flags;
    spin_lock_irqsave(&task_list->lock, irq_flags);
    memcpy(&x, task_list, sizeof(x));
    // now clear out task_list.  (deferred_init_list(task_list);  except for the spinlock)
    INIT_LIST_HEAD(&(task_list->list));
    task_list->task = NULL;
    task_list->task_data1 = NULL;
    task_list->task_data2 = 0;
    spin_unlock_irqrestore(&task_list->lock, irq_flags); 
    
    if ((x.list.next == NULL) || (x.list.next == &(task_list->list))) { // empty list
        return;
    }
    printk(KERN_INFO "deferred_run_now\n");
    // repair x.list, which was broken by relocating:
    x.list.next->prev = &x.list;
    x.list.prev->next = &x.list;
    
    while ( ! list_empty(&x.list)) {   // while deferred tasks need to run
        struct deferred_task *task;
        task = list_entry(x.list.next, struct deferred_task, list);
        if (task->task != NULL) {
            task->task(task->task_data1, task->task_data2);
        }
        list_del(&task->list);
        kzfree(task);
    }
    printk(KERN_INFO "deferred_run_now done\n");
}
