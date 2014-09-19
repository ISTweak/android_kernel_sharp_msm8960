/*
 * Copyright (C) 2012 Sharp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* -------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
/* -------------------------------------------------------------------- */
#define FREYJA_KER_BASEMINOR		(0)
#define FREYJA_KER_MINORCOUNT		(1)
#define FREYJA_KER_DRVNAME		"freyja"
#define FREYJA_CLASS_NAME		"cls_freyja"
#define FREYJA_PARAM_SIZE 		(512)
#define freyja_printk			if(0)printk
/* -------------------------------------------------------------------- */
typedef struct{
	int major;
	dev_t dev;
	struct cdev freyja_cdev;
	struct class* freyja_classp;
	int status;
}freyja_info_type;
freyja_info_type freyja_info;
/* -------------------------------------------------------------------- */
typedef struct _freyja_command
{
	int cmd;
	int ret;
	char param[FREYJA_PARAM_SIZE];
}freyja_command_t;
/* -------------------------------------------------------------------- */
static int		freyja_open(struct inode* inode, struct file* filp);
static int		freyja_close(struct inode* inode, struct file* filp);
static ssize_t		freyja_read(struct file* filp, char* buf, size_t count, loff_t* pos);
static ssize_t		freyja_write(struct file* filp, const char* buf, size_t count, loff_t* pos);
/* -------------------------------------------------------------------- */
static struct file_operations freyja_Ops = {
	.owner   = THIS_MODULE,
	.read    = freyja_read,
	.write   = freyja_write,
	.open    = freyja_open,
	.release = freyja_close,
};
/* -------------------------------------------------------------------- */
static struct semaphore 	freyja_sem;
static wait_queue_head_t	freyja_process_q;
static wait_queue_head_t	freyja_read_q;
static wait_queue_head_t	freyja_write_q;
static freyja_command_t 	freyja_command;
static volatile int		freyja_state = 0;
/* -------------------------------------------------------------------- */
int freyja_command_process(char* param)
{
	int ret = 0;

	freyja_printk("[0] : freyja_command_process start\n");

    if(down_interruptible(&freyja_sem))return -1;

    memset(&freyja_command, 0, sizeof(freyja_command_t));

    freyja_command.cmd = 0;
		
    memcpy(freyja_command.param, param, sizeof(freyja_command.param));

    freyja_state = 1;

    up(&freyja_sem);

    freyja_printk("[0] : wake_up_interruptible(&freyja_read_q);\n");

    wake_up_interruptible(&freyja_read_q);

    freyja_printk("[0] : wait_event_interruptible(freyja_process_q, freyja_state == 3);\n");

    /* TODO: tune timeout period. */
    ret = wait_event_interruptible_timeout(freyja_process_q, freyja_state == 3, HZ * 10 );

    freyja_printk("[5] : awake. ret = %d, freyja_command.ret = %d\n", ret, freyja_command.ret);

    if ((!ret)||(freyja_command.ret)) { /* timeout or frey returns error */
        u32 *param32 = (u32*)&param[32];
        printk(KERN_WARNING "freyja: %s: wait_event timeout.\n", __func__);

        param[1] = 1;
        param32[0] &= ~0xff; param32[0] |= 0x2a;
        param32[1] &= ~0x40000000;
    }
    else {
        freyja_printk("[6] : memcpy from freyja_command to param.\n");

        memcpy(param, freyja_command.param, sizeof(freyja_command.param));
    }

    if(down_interruptible(&freyja_sem))return -1;

    ret = freyja_command.ret;
    freyja_state = 0;

    up(&freyja_sem);

    if(ret != 0)
        return -1;

	freyja_printk("[6] : freyja_command_process end\n");

	return 0;
}
/* -------------------------------------------------------------------- */
static int __init freyja_ker_init(void)
{
	int sdResult;
	struct device* devp;

	freyja_info.major = -1;
	freyja_info.freyja_classp = NULL;
	freyja_info.status = 0;
	
	freyja_info.dev = MKDEV(freyja_info.major, 0);
	
	sdResult = alloc_chrdev_region( &freyja_info.dev, FREYJA_KER_BASEMINOR, FREYJA_KER_MINORCOUNT, FREYJA_KER_DRVNAME );
	if( sdResult < 0 ){
		return -1;
	}
	freyja_info.major = sdResult;
	

	cdev_init( &freyja_info.freyja_cdev, &freyja_Ops );
	freyja_info.freyja_cdev.owner = THIS_MODULE;
	freyja_info.freyja_cdev.ops = &freyja_Ops;

	sdResult = cdev_add(&freyja_info.freyja_cdev, freyja_info.dev, FREYJA_KER_MINORCOUNT);
	if( sdResult < 0 ){
		return -1;
	}

	
	freyja_info.freyja_classp = class_create( THIS_MODULE, FREYJA_CLASS_NAME );
	if (IS_ERR(freyja_info.freyja_classp)){
		return -1;
	}

	devp = device_create( freyja_info.freyja_classp, NULL, freyja_info.dev, NULL, FREYJA_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 ){
		return -1;
	}
	
	sema_init(&freyja_sem, 1);

	init_waitqueue_head(&freyja_process_q);
	init_waitqueue_head(&freyja_read_q);
	init_waitqueue_head(&freyja_write_q);

	return 0;
}
/* -------------------------------------------------------------------- */
static void __exit freyja_ker_term( void )
{
	if( freyja_info.major < 0){
		return;
	}
	
	device_destroy( freyja_info.freyja_classp, freyja_info.dev );
	class_destroy( freyja_info.freyja_classp );
	freyja_info.freyja_classp = NULL;

	cdev_del( &freyja_info.freyja_cdev );
	unregister_chrdev_region( freyja_info.dev, FREYJA_KER_MINORCOUNT );
	freyja_info.major = -1;

	return;
}
/* -------------------------------------------------------------------- */
/*  */
/* -------------------------------------------------------------------- */
static int freyja_open(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static int freyja_close(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t freyja_read(struct file* filp, char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;
	int r;

	freyja_printk("[3] : freyja_read\n");

	r = wait_event_interruptible(freyja_read_q, freyja_state == 1);

	if(r != 0)
	{
		return 0;
	}

	freyja_printk("[3] : awake\n");

	do
	{
		if(down_interruptible(&freyja_sem))return 0;

		if(copy_to_user(buf, &freyja_command, sizeof(freyja_command_t)))break;

		ret = sizeof(freyja_command_t);
	}
	while(0);

	freyja_state = 2;

	up(&freyja_sem);

	freyja_printk("[3] : wake_up_interruptible(&freyja_write_q);\n");

	wake_up_interruptible(&freyja_write_q);

	return ret;
}
/* -------------------------------------------------------------------- */
static ssize_t freyja_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;

	freyja_printk("[4] : freyja_write\n");

	wait_event_interruptible(freyja_write_q, freyja_state == 2);

	freyja_printk("[4] : awake\n");

	do
	{
		if(down_interruptible(&freyja_sem))return 0;

		if(copy_from_user(&freyja_command, buf, sizeof(freyja_command_t)))break;

		ret = sizeof(freyja_command_t);
	}
	while(0);

	freyja_state = 3;

	up(&freyja_sem);

	freyja_printk("[4] : wake_up_interruptible(&freyja_rw_q);\n");

	wake_up_interruptible(&freyja_process_q);

	return ret;
}
/* -------------------------------------------------------------------- */
module_init( freyja_ker_init );
module_exit( freyja_ker_term );
/* -------------------------------------------------------------------- */
MODULE_AUTHOR("SHARP");
MODULE_DESCRIPTION("freyja device");
MODULE_LICENSE("GPL");
/* -------------------------------------------------------------------- */
