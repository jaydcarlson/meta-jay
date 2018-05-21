/*
 * This kernel module provides a char dev for the Raw lora driver
 *  interface at /dev/
 *
 * Copyright (C) 2015 Signetic, LLC
 *  Bruce Carlson
 *  
 * Adapted from spibridge, which was 
 * adapted from spidev.c, which was 
 *      Copyright (C) 2006 SWAPP
 *	     Andrea Paterniani <a.paterniani@swapp-eng.it>
 *      Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
//#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/cdev.h>     // char dev structure
#include <linux/completion.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

#include "lora_raw.h"



// lrcd_ stands for lora_raw_chardev_


// tell gcc it's ok to declare variables in line with code (like c99):
#pragma GCC diagnostic ignored "-Wdeclaration-after-statement"

#define DRIVER_NAME                 "lora_raw_chrdev"


static size_t smaller_size_t(size_t a, size_t b)
{
    return (a < b) ? a : b;
}


/////////////////////////////////////////////////////////////////

/*
 * This module allocates a character device number dynamically.  
 * You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/lora_raw device
 * node.
 * As of the time of this writing the lora_raw driver supports only one
 * concurrent open handle.  This module also supports only one concurrent
 * open handle.  Enforcement is by the lora_raw driver.  If the lora_raw
 * driver changes to support concurrent handles then this driver will also
 * need to change.
 */
struct radio_data {

    struct list_head list;
    char name[80];

	dev_t			    devt;                   // linux file syste device number
    int                 devt_is_allocated;

    lrhandle_t          lora_raw_handle;
    
    struct cdev         cdevice;
    int                 cdevice_is_added;       // != 0 indicates we have a cdevice registered
    
    struct completion   packet_is_ready;
    
//    struct mutex        ioctl_lock;             // prevent concurrent ioctl accesses
};                                              

LIST_HEAD(radio_list); // list of struct radio_data

#define FIRST_MINOR_DEVICE_NUMBER   0


/*-------------------------------------------------------------------------*/


/** Read packet from lora_raw
	@param user_buf must be allocated by caller
	@param count is max number of bytes to be written to *user_buf
    @return number of bytes read, or < 0 on error
*/
static ssize_t
lrcd_read(struct file *filp, char __user *user_buf, size_t count, loff_t *f_pos)
{
    struct radio_data *r;
    r = (struct radio_data *) filp->private_data;
    struct lora_raw_full_packet packet;
    int result = -EAGAIN;
    // this loop may need some future improvement, but watch out for concurrency with receiving packets and competions...

    if (filp->f_flags & O_NONBLOCK) {
        try_wait_for_completion(&r->packet_is_ready);       // decrement the completion counter
    } else {
        wait_for_completion_interruptible(&r->packet_is_ready);
    }
    result = lora_raw_rx(r->lora_raw_handle, (struct lora_raw_packet *) &packet);
    if (result == -EAGAIN) {                 // no data to be read
        return 0;
    }
    if (result < 0) {
        return result;
    }
    int packet_bytes_with_metadata = sizeof(packet) - sizeof(packet.data) + packet.data_bytes;
    int bytes_to_copy = smaller_size_t(packet_bytes_with_metadata, count);
    long bytes_not_copied = copy_to_user(user_buf, &packet, bytes_to_copy);
    if (bytes_not_copied != 0) {
        return -EFAULT;
    }
    if (packet.data_bytes > count) {
        // This packet is too long for buffer.  The extra bytes are gone forever.
        return -EFBIG;
    }
    return bytes_to_copy;
}

/** Send packet via lora_raw 
    @return number of bytes written, or < 0 on error
*/
static ssize_t
lrcd_write(struct file *filp, const char __user *user_buf,
		size_t count, loff_t *f_pos)
{
    uint8_t buffer[LORA_RAW_MAX_PACKET_DATA];
    if (count > sizeof(buffer)) {
        return -EFBIG;
    }
    long bytes_not_copied = copy_from_user(buffer, user_buf, count);
    if (bytes_not_copied != 0) {
        return -EFAULT;
    }
    struct radio_data *r;
    r = (struct radio_data *) filp->private_data;
    int result = lora_raw_tx(r->lora_raw_handle, buffer, count);
    if (result < 0) {
        return result;
    }
    return count;
}

static long
lrcd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
#if 0
	int			err = 0;
	int			retval = 0;
	struct radio_data	*d;
	struct spi_device	*spi;
	u32			tmp;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* Take a reference to spi controller to 
     * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	d = filp->private_data;
    if (d == NULL) {
        return -EFAULT;
    }
    
    // lock here to prevent concurrent SPI_IOC_WR_* from morphing
	//    data fields while SPI_IOC_RD_* reads them
    mutex_lock(&d->ioctl_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;

	default:
        retval = -ENOTTY;
		break;
	}

    mutex_unlock(&d->ioctl_lock);
	return retval;
#endif
    return -EFAULT; // not yet implemented
}

#ifdef CONFIG_COMPAT
static long
lrcd_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return lrcd_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define lrcd_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

struct radio_data *get_radio_by_device_number(dev_t devt)
{
    struct radio_data *r;
    list_for_each_entry(r, &radio_list, list) {
        if (r->devt == devt) {
            return r;
        }
    }
    return NULL;
}

/** This is to be called when lora_raw receives a packet 
 */
void lrcd_event_handler_callback(void *context, int event)
{
    struct radio_data *r = context;
    if (event == LORA_CB_EVENT_RX) {
        complete(&r->packet_is_ready);
    }
}

static int lrcd_open(struct inode *inode, struct file *filp)
{
    struct radio_data *r;
    r = get_radio_by_device_number(inode->i_rdev);
    filp->private_data = r;
    lrhandle_t handle = lora_raw_open(r->name);
    if (handle.key < 0) {
        return handle.key;  // return error code
    }
    r->lora_raw_handle = handle;
    lora_raw_set_callback(handle, lrcd_event_handler_callback, r);
    return 0;
}

static int lrcd_release(struct inode *inode, struct file *filp)
{
    struct radio_data *r;
    r = filp->private_data;
    int result = lora_raw_close(r->lora_raw_handle);
//    r->lora_raw_handle.key = -1;      // can't throw away handle because there might be other open handles
    return result;
}

static const struct file_operations lrcd_fops = {
	.owner =	THIS_MODULE,
	.write =	lrcd_write,
	.read =		lrcd_read,
	.unlocked_ioctl = lrcd_ioctl,
	.compat_ioctl = lrcd_compat_ioctl,
	.open =		lrcd_open,
	.release =	lrcd_release,
	.llseek =	no_llseek,
};




/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/lora_raw character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *lrcd_class;



/*-------------------------------------------------------------------------*/

void radio_cleanup(struct radio_data *r);

/** 
    @return pointer to newly created radio, or NULL on error
 */
struct radio_data *radio_create(char *name)
{
    printk(KERN_INFO DRIVER_NAME " radio_create %s\n", name);
    
    struct radio_data *r;
    r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (r == NULL) {
		printk(KERN_ERR DRIVER_NAME " failed to allocate memory for %s\n", name);
		// increment allocation failure counter?
		return NULL;
	}
    
    INIT_LIST_HEAD(&r->list);
    strncpy(r->name, name, sizeof(r->name));
    r->name[sizeof(r->name)-1] = '\0';

    init_completion(&r->packet_is_ready);
    
//	mutex_init(&d->ioctl_lock);

    // get a device number 
   
    int alloc_device_number_result = alloc_chrdev_region(&r->devt, FIRST_MINOR_DEVICE_NUMBER, 1, name);
    if (alloc_device_number_result < 0) {
        printk(KERN_ERR DRIVER_NAME " failed to allocate device number. err = %d\n", alloc_device_number_result);
        radio_cleanup(r);
        return NULL;
    }
    r->devt_is_allocated = 1;
    
    struct device *dev;
    struct device *parent = NULL;  // perhaps this should point to the lora_raw device?
    dev = device_create(lrcd_class, parent, r->devt, r, name);
    if (IS_ERR(dev)) {
        printk(KERN_ERR DRIVER_NAME " failed to create device. err = %ld\n", PTR_ERR(dev));
        radio_cleanup(r);
        return NULL;
    }

    // add a character device to the system:
    
    cdev_init(&r->cdevice, &lrcd_fops);
    r->cdevice.owner = THIS_MODULE;
    int device_add_result = cdev_add(&r->cdevice, r->devt, 1);
    if (device_add_result < 0) {
        printk(KERN_ERR DRIVER_NAME " failed to add device.  result = %d\n", device_add_result);
        radio_cleanup(r);
        return NULL;
    };
    r->cdevice_is_added = 1;

    printk(KERN_DEBUG DRIVER_NAME " radio_create success - device on-line\n");
    return r;
}

void radio_destroy(struct radio_data *r) 
{
    
    printk(KERN_DEBUG DRIVER_NAME " radio_destroy\n");

    radio_cleanup(r);
}

void radio_cleanup(struct radio_data *r)
{
    // Undo everything done in create
    
    // todo: before removing cdevice, do we need to wait until cdevice is not in use?
    
    if (r->cdevice_is_added) {
        r->cdevice_is_added = 0;
        // remove the device
        cdev_del(&r->cdevice);
    }
    
	/* prevent new opens */
	device_destroy(lrcd_class, r->devt);

    if (r->devt_is_allocated) {
        r->devt_is_allocated = 0;
        // free the device number
        unregister_chrdev_region(r->devt, 1);
    }

    if ( ! completion_done(&r->packet_is_ready)) {
        printk(KERN_NOTICE"Waiting for lora_raw_chardev readers to go away..\r\n");
        do {
            uint32_t j;
            for (j=0; j<1000; j++) {
                complete(&r->packet_is_ready);
            }
            msleep(100);
        } while ( ! completion_done(&r->packet_is_ready));
    }
    
    kzfree(r);

    printk(KERN_DEBUG "radio_cleanup done.\n");
}






/** Get a list of radios from lora_raw driver and make a corresponding chardev 
 *  for each radio
 */
static int make_radios(void)
{
    int count = 0;
    const char delimiters[] = { LORA_RAW_RADIO_LIST_DELIMITER, '\0' };
    char *list;
    
    list = lora_raw_radio_list();
    printk(KERN_DEBUG DRIVER_NAME " lora_raw_radio_list returned %s\n", list);
    
    if (list == NULL) {
        return 0;
    }
    char *p;
    p = list;
    while (1) {
        char 
        *radio_name;
        radio_name = strsep(&p, delimiters);
        if (radio_name == NULL) {
            break;
        }
        if (radio_name[0] != '\0') {
            struct radio_data *r;
            r = radio_create(radio_name);
            if (r == NULL) {
                printk(KERN_WARNING DRIVER_NAME " failed to create radio %s\n", radio_name);
            } else {
                list_add(&r->list, &radio_list);
                count++;
            }
        }
    }
    kzfree(list);
    return count;
}

static void destroy_radios(void)
{
    while ( ! list_empty(&radio_list)) {
        struct radio_data *r;
        r = list_first_entry(&radio_list, struct radio_data, list);
        list_del(&r->list);
        radio_destroy(r);
    }
}


/*-------------------------------------------------------------------------*/


#define DEVICE_NAME                 "lora_raw_chardev"


static int my_class_created = 0;
static int radio_count = 0;

static void driver_cleanup(void);

static int __init lrcd_init(void)
{
    printk(KERN_DEBUG DRIVER_NAME " init\n");

    // create lrcd_class.  radio_create() will use this to create device fs entries.
    
	lrcd_class = class_create(THIS_MODULE, "lora_raw");
	if (IS_ERR(lrcd_class)) {
        printk(KERN_ERR DRIVER_NAME " failed to create class\n");
        driver_cleanup();
		return PTR_ERR(lrcd_class);
	}
    my_class_created = 1;
    
    INIT_LIST_HEAD(&radio_list);    
    radio_count = make_radios();
    
    if (radio_count <= 0) {
        printk(KERN_INFO DRIVER_NAME " lora_raw reports no radios\n");
    }
    return 0;
}
module_init(lrcd_init);

static void __exit lrcd_exit(void)
{
    printk(KERN_INFO DRIVER_NAME " exit\n");

    driver_cleanup();

    printk(KERN_DEBUG DRIVER_NAME " exit complete\n");
}
module_exit(lrcd_exit);

static void driver_cleanup(void)
{
    destroy_radios();
    
    if (my_class_created) {
        my_class_created = 0;
        class_destroy(lrcd_class);
    }
}


MODULE_AUTHOR("Bruce Carlson <bcarlson@signetik.com");
MODULE_DESCRIPTION("User mode Raw LoRa interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:lora_raw_chrdev");
