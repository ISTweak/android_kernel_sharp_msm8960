/**
 * TSIF driver client
 *
 * Character device that, being read
 * returns stream of TSIF packets.
 *
 * Copyright (c) 2009-2011, The Linux Foundation. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>       /* Needed by all modules */
#include <linux/kernel.h>       /* Needed for KERN_INFO */
#include <linux/cdev.h>
#include <linux/err.h>          /* IS_ERR etc. */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sched.h>        /* TASK_INTERRUPTIBLE */

#include <linux/uaccess.h>        /* copy_to_user */

#include <linux/tsif_api.h>


#define TSIF_NON_DMA_TRANSFER 0


#if TSIF_NON_DMA_TRANSFER
extern int tsif_start_S(void *cookie);
extern int tsif_stop_S(void *cookie);
extern int tsif_read_S(unsigned char *buf, unsigned short count, unsigned short *result);
#endif

#if defined( CONFIG_DTV_TUNERDEV ) || defined( CONFIG_DTV_TUNERDEV_DL12 )
#include <mach/gpio.h>
#define GPIO_TSIF_DATA_PORTNO	77
#define GPIO_TSIF_EN_PORTNO		76
#define GPIO_TSIF_CLK_PORTNO	75
#define IOC_TSIF_GIPIO_CTRL		0x0001
#endif

struct tsif_chrdev {
	struct cdev cdev;
	struct device *dev;
	wait_queue_head_t wq_read;
	void *cookie;
	/* mirror for tsif data */
	void *data_buffer;
	unsigned buf_size_packets; /**< buffer size in packets */
	unsigned ri, wi;
	enum tsif_state state;
	unsigned rptr;
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	int index;
#endif
};

static ssize_t tsif_open(struct inode *inode, struct file *file)
{
	int rc;
	struct tsif_chrdev *the_dev =
	       container_of(inode->i_cdev, struct tsif_chrdev, cdev);
	if (!the_dev->cookie)  /* not bound yet */
		return -ENODEV;
	file->private_data = the_dev;
#if TSIF_NON_DMA_TRANSFER
	rc = tsif_start_S(the_dev->cookie);
#else
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	rc = tsif_start(the_dev->cookie, the_dev->index);
#else
	rc = tsif_start(the_dev->cookie);
#endif
#endif
	if (rc)
		return rc;
	tsif_get_info(the_dev->cookie, &the_dev->data_buffer,
		      &the_dev->buf_size_packets);
	the_dev->rptr = 0;
	return nonseekable_open(inode, file);
}

static ssize_t tsif_release(struct inode *inode, struct file *filp)
{
	struct tsif_chrdev *the_dev = filp->private_data;
#if TSIF_NON_DMA_TRANSFER
	tsif_stop_S(the_dev->cookie);
#else
	tsif_stop(the_dev->cookie);
#endif
	return 0;
}

static ssize_t tsif_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *f_pos)
{
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	unsigned char *ptr_copy_from;
	int ritmp;
	int i;
	tsif_2nd_buffer_t *p2nd_buffer = &tsif_2nd_buffer;
#endif
#if TSIF_NON_DMA_TRANSFER
	unsigned short	result;
	tsif_read_S(buf, count/192, &result);
	return result*192;
#else
	int avail = 0;
	int wi;
	struct tsif_chrdev *the_dev = filp->private_data;
	tsif_get_state(the_dev->cookie, &the_dev->ri, &the_dev->wi,
		       &the_dev->state);
		       

#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	if(p2nd_buffer->wi == p2nd_buffer->ri){
		//printk("%s:%dbuffer empty.\n", __FILE__, __LINE__);
		return 0;
	}

	wi = (p2nd_buffer->wi > p2nd_buffer->ri) ? p2nd_buffer->wi : p2nd_buffer->num_of_each_tsbuffer * p2nd_buffer->cnt_data_buffer_ptr;
	
	avail=((wi - p2nd_buffer->ri)* TSIF_PKT_SIZE < count) ? (wi - p2nd_buffer->ri) * TSIF_PKT_SIZE : count;

	for(i = 0; i < avail / TSIF_PKT_SIZE; i++){
		ritmp = (p2nd_buffer->ri + i) % (p2nd_buffer->num_of_each_tsbuffer * p2nd_buffer->cnt_data_buffer_ptr);
		ptr_copy_from = tsif_pcktloc_from_num(p2nd_buffer, ritmp);
		if(ptr_copy_from){
			if (copy_to_user(buf+i*TSIF_PKT_SIZE, ptr_copy_from, TSIF_PKT_SIZE)){
				printk("%s:%d !! copy_to_user failed.\n", __FILE__, __LINE__);
				return -EFAULT;
			}
		}else{
			printk("%s:%d inconsistent ri %d.\n", __FILE__, __LINE__, ritmp);
			return -EFAULT;
		}
	}

	p2nd_buffer->rptr = (p2nd_buffer->rptr + avail) % (TSIF_PKT_SIZE * p2nd_buffer->num_of_each_tsbuffer * p2nd_buffer->cnt_data_buffer_ptr);
	p2nd_buffer->ri = p2nd_buffer->rptr / TSIF_PKT_SIZE;

    return avail;

#else
	/* consistency check */
	if (the_dev->ri != (the_dev->rptr / TSIF_PKT_SIZE)) {
		dev_err(the_dev->dev,
			"%s: inconsistent read pointers: ri %d rptr %d\n",
			__func__, the_dev->ri, the_dev->rptr);
		the_dev->rptr = the_dev->ri * TSIF_PKT_SIZE;
	}
	/* ri == wi if no data */
	if (the_dev->ri == the_dev->wi) {
		/* shall I block waiting for data? */
		if (filp->f_flags & O_NONBLOCK) {
			if (the_dev->state == tsif_state_running) {
				return -EAGAIN;
			} else {
				/* not running -> EOF */
				return 0;
			}
		}
		if (wait_event_interruptible(the_dev->wq_read,
		      (the_dev->ri != the_dev->wi) ||
		      (the_dev->state != tsif_state_running))) {
			/* got signal -> tell FS to handle it */
			return -ERESTARTSYS;
		}
		if (the_dev->ri == the_dev->wi) {
			/* still no data -> EOF */
			return 0;
		}
	}
	/* contiguous chunk last up to wi or end of buffer */
	wi = (the_dev->wi > the_dev->ri) ?
		the_dev->wi : the_dev->buf_size_packets;
	avail = min(wi * TSIF_PKT_SIZE - the_dev->rptr, count);
	if (copy_to_user(buf, the_dev->data_buffer + the_dev->rptr, avail))
		return -EFAULT;
	the_dev->rptr = (the_dev->rptr + avail) %
		(TSIF_PKT_SIZE * the_dev->buf_size_packets);
	the_dev->ri = the_dev->rptr / TSIF_PKT_SIZE;
	*f_pos += avail;
	tsif_reclaim_packets(the_dev->cookie, the_dev->ri);
	return avail;
#endif	// CONFIG_DTV_TUNERDEV_DL12

#endif
}


#if defined( CONFIG_DTV_TUNERDEV ) || defined( CONFIG_DTV_TUNERDEV_DL12 )
static long tsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int param;
	
	if (copy_from_user(&param, (int*)arg, sizeof(int))) {
		printk("%s:%d  !!!tsif_ioctl copy_from_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
		return -EFAULT;
	}

	switch ( cmd ) {
	case IOC_TSIF_GIPIO_CTRL:
		if ( param ) {
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_DATA_PORTNO, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_EN_PORTNO	, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_CLK_PORTNO	, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
		} else {
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_DATA_PORTNO, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE );
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_EN_PORTNO	, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE );
			gpio_tlmm_config( GPIO_CFG(GPIO_TSIF_CLK_PORTNO	, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE );
		}
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}
#endif


static void tsif_notify(void *data)
{
	struct tsif_chrdev *the_dev = data;
	tsif_get_state(the_dev->cookie, &the_dev->ri, &the_dev->wi,
		       &the_dev->state);
	wake_up_interruptible(&the_dev->wq_read);
}

static const struct file_operations tsif_fops = {
	.owner   = THIS_MODULE,
	.read    = tsif_read,
	.open    = tsif_open,
	.release = tsif_release,
#if defined( CONFIG_DTV_TUNERDEV ) || defined( CONFIG_DTV_TUNERDEV_DL12 )
	.unlocked_ioctl = tsif_ioctl,
#endif
};

static struct class *tsif_class;
static dev_t tsif_dev;  /**< 1-st dev_t from allocated range */
static dev_t tsif_dev0; /**< next not yet assigned dev_t */

static int tsif_init_one(struct tsif_chrdev *the_dev, int index)
{
	int rc;
	pr_info("%s[%d]\n", __func__, index);
	cdev_init(&the_dev->cdev, &tsif_fops);
	the_dev->cdev.owner = THIS_MODULE;
	init_waitqueue_head(&the_dev->wq_read);
	rc = cdev_add(&the_dev->cdev, tsif_dev0++, 1);
	the_dev->dev = device_create(tsif_class, NULL, the_dev->cdev.dev,
				     the_dev, "tsif%d", index);
	if (IS_ERR(the_dev->dev)) {
		rc = PTR_ERR(the_dev->dev);
		pr_err("device_create failed: %d\n", rc);
		goto err_create;
	}
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	the_dev->index = index;
	if (index!=0) {
		return 0;
	}
#endif
	the_dev->cookie = tsif_attach(index, tsif_notify, the_dev);
	if (IS_ERR(the_dev->cookie)) {
		rc = PTR_ERR(the_dev->cookie);
		pr_err("tsif_attach failed: %d\n", rc);
		goto err_attach;
	}
	/* now data buffer is not allocated yet */
	tsif_get_info(the_dev->cookie, &the_dev->data_buffer, NULL);
	dev_info(the_dev->dev,
		 "Device %d.%d attached to TSIF, buffer size %d\n",
		 MAJOR(the_dev->cdev.dev), MINOR(the_dev->cdev.dev),
		 the_dev->buf_size_packets);
	return 0;
err_attach:
	device_destroy(tsif_class, the_dev->cdev.dev);
err_create:
	cdev_del(&the_dev->cdev);
	return rc;
}

#if defined( CONFIG_DTV_TUNERDEV_DL12 )
static void tsif_exit_one(struct tsif_chrdev *the_dev, int index)
#else
static void tsif_exit_one(struct tsif_chrdev *the_dev)
#endif
{
	dev_info(the_dev->dev, "%s\n", __func__);
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	if (index==0) {
		tsif_detach(the_dev->cookie);
	}
#else
	tsif_detach(the_dev->cookie);
#endif
	device_destroy(tsif_class, the_dev->cdev.dev);
	cdev_del(&the_dev->cdev);
}

#if defined( CONFIG_DTV_TUNERDEV_DL12 )
#define TSIF_NUM_DEVS 2 /**< support this many devices */
#else
#define TSIF_NUM_DEVS 1 /**< support this many devices */
#endif

struct tsif_chrdev the_devices[TSIF_NUM_DEVS];

static int __init mod_init(void)
{
	int rc;
	int instance;
	rc = alloc_chrdev_region(&tsif_dev, 0, TSIF_NUM_DEVS, "tsif");
	if (rc) {
		pr_err("alloc_chrdev_region failed: %d\n", rc);
		goto err_devrgn;
	}
	tsif_dev0 = tsif_dev;
	tsif_class = class_create(THIS_MODULE, "tsif");
	if (IS_ERR(tsif_class)) {
		rc = PTR_ERR(tsif_class);
		pr_err("Error creating tsif class: %d\n", rc);
		goto err_class;
	}
	instance = tsif_get_active();
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	if (instance >= 0) {
		int i ;
		for (i=0; i<TSIF_NUM_DEVS; i++) {
			rc = tsif_init_one(&the_devices[i], i);
			if (rc)
				goto err_init1;
			if (i!=0) {
				the_devices[i].cookie = the_devices[0].cookie ;
				the_devices[i].data_buffer = the_devices[0].data_buffer ;
			}
		}
	}
#else
	if (instance >= 0)
		rc = tsif_init_one(&the_devices[0], instance);
#endif
	else
		rc = instance;
	if (rc)
		goto err_init1;
	return 0;
err_init1:
	class_destroy(tsif_class);
err_class:
	unregister_chrdev_region(tsif_dev, TSIF_NUM_DEVS);
err_devrgn:
	return rc;
}

static void __exit mod_exit(void)
{
#if defined( CONFIG_DTV_TUNERDEV_DL12 )
	int i ;
	for (i=0; i<TSIF_NUM_DEVS; i++) {
		tsif_exit_one(&the_devices[i], i);
	}
#else
	tsif_exit_one(&the_devices[0]);
#endif
	class_destroy(tsif_class);
	unregister_chrdev_region(tsif_dev, TSIF_NUM_DEVS);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("TSIF character device interface");
MODULE_LICENSE("GPL v2");

