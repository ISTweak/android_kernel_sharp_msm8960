/* drivers/usb/gadget/android.c
 *
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
 * Copyright (C) 2013 SHARP CORPORATION
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android.h>

#ifdef CONFIG_USB_ANDROID_SH_CUST
#include <linux/ratelimit.h>
#include <mach/usb_gadget_xport.h>
#include "sharp/sh_smem.h"
#include "sharp/sh_boot_manager.h"
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"
#ifdef CONFIG_USB_ANDROID_SH_CUST
#include "sh_string.c"
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#include "f_diag.c"
#ifndef CONFIG_USB_ANDROID_SH_CUST
#include "f_rmnet_smd.c"
#include "f_rmnet_sdio.c"
#include "f_rmnet_smd_sdio.c"
#include "f_rmnet.c"
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#include "f_audio_source.c"
#ifdef CONFIG_USB_ANDROID_SH_UMS
#include "f_sh_msc.c"
#endif /* CONFIG_USB_ANDROID_SH_UMS */
#ifdef CONFIG_LISMO_BUFFER_CONTROL
#include "lismo_buf_control.c"
#endif /* CONFIG_LISMO_BUFFER_CONTROL */
#if !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD)
#include "f_mass_storage.c"
#endif /* !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD) */
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_rmnet_ctrl_smd.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"
#ifndef CONFIG_USB_ANDROID_SH_CUST
#include "u_bam.c"
#include "u_ctrl_hsuart.c"
#include "u_data_hsuart.c"
#include "f_serial.c"
#else /* CONFIG_USB_ANDROID_SH_CUST */
#include "f_sh_mdlm.c"
#include "f_obex.c"
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#include "f_acm.c"
#include "f_adb.c"
#ifndef CONFIG_USB_ANDROID_SH_CUST
#include "f_ccid.c"
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"
#ifndef CONFIG_USB_ANDROID_SH_CUST
#include "u_bam_data.c"
#include "f_mbim.c"
#endif /* CONFIG_USB_ANDROID_SH_CUST */
#include "u_uac1.c"
#include "f_uac1.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	struct android_usb_platform_data *pdata;

	bool connected;
	bool sw_connected;
	char pm_qos[5];
	struct pm_qos_request pm_qos_req_dma;
	struct work_struct work;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	int charge_enable;
	int is_serial_set;
	bool unlocked;

	struct switch_dev sdev;
#endif /* CONFIG_USB_ANDROID_SH_CUST */
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

#ifdef CONFIG_USB_ANDROID_SH_CUST
#define D_SH_SERIAL_SETUP_PORT_OBEX		(0)
#define D_SH_SERIAL_SETUP_PORT_MDLM		(1)
#define D_SH_SERIAL_SETUP_PORT_NUM		(2)

#define SWITCH_NAME_RNDIS			"rndis_switch"

#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifndef CONFIG_USB_ANDROID_SH_CUST
/* move to sh_string.c */
static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];
#endif /* CONFIG_USB_ANDROID_SH_CUST */

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG               = __constant_cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
};

enum android_device_state {
	USB_DISCONNECTED,
	USB_CONNECTED,
	USB_CONFIGURED,
};

#ifdef CONFIG_USB_ANDROID_SH_CUST
static struct device *sysfs_device = NULL;

typedef enum {
	BOOT_MODE_ERR = -1,
	BOOT_MODE_NORMAL = 0,
	BOOT_MODE_TESTMODE,
	BOOT_MODE_SOFTWARE_UPDATE,
} android_boot_mode;

static android_boot_mode boot_mode_hold;

static android_boot_mode get_boot_mode_from_smem(void)
{
	sharp_smem_common_type *p_smem_addr = NULL;
	int smem_boot_mode = 0;
	int smem_softupdateflg = 0;
	android_boot_mode boot_mode = BOOT_MODE_NORMAL;

	/* smem */
	p_smem_addr = sh_smem_get_common_address();
	if ( !p_smem_addr ) {
		pr_err("%s: sh_smem_get_common_address failed\n", __func__);
		return BOOT_MODE_ERR;
	}

	smem_boot_mode = p_smem_addr->sh_boot_mode;
	smem_softupdateflg = p_smem_addr->shusb_softupdate_mode_flag;

	/* judge boot mode */
	switch ( smem_boot_mode ) {
		case 0xFFFF:
			if ( !smem_softupdateflg ) {
				boot_mode = BOOT_MODE_NORMAL;
			} else {
				boot_mode = BOOT_MODE_SOFTWARE_UPDATE;
			}
			break;
		case 0x40:
			if ( !smem_softupdateflg ) {
				boot_mode = BOOT_MODE_TESTMODE;
			} else {
				boot_mode = BOOT_MODE_SOFTWARE_UPDATE;
			}
			break;
		case 0x41:
		case 0x42:
			boot_mode = BOOT_MODE_TESTMODE;
			break;
		default :
			boot_mode = BOOT_MODE_NORMAL;
			break;
	}
	return boot_mode;
}

static int get_diag_enable_from_smem(void)
{
	sharp_smem_common_type *p_smem_addr = NULL;
	int smem_diag_enable = 0;

	/* smem */
	p_smem_addr = sh_smem_get_common_address();
	if ( !p_smem_addr ) {
		pr_err("%s: sh_smem_get_common_address failed\n", __func__);
		return 0;
	}
	smem_diag_enable = p_smem_addr->shusb_qxdm_ena_flag;
#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s: diag_enable smem:%d\n", __func__, smem_diag_enable);
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	return !!smem_diag_enable;
}

static int get_charge_enable_from_smem(void)
{
	sharp_smem_common_type *p_smem_addr = NULL;
	int smem_charge_enable = 0;

	/* smem */
	p_smem_addr = sh_smem_get_common_address();
	if ( !p_smem_addr ) {
		pr_err("%s: sh_smem_get_common_address failed\n", __func__);
		return 0;
	}
	smem_charge_enable = p_smem_addr->shusb_usb_charge_ena_flag;
#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s: charge_enable smem:%d\n", __func__, smem_charge_enable);
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	return !!smem_charge_enable;
}

static bool is_offcharge_bootmode;

static bool get_offcharge_bootmode_from_smem(void)
{
	bool is_offcharge = false;
	unsigned short bootmode = sh_boot_get_bootmode();

	/* get boot mode */
	switch (bootmode) {
		case SH_BOOT_O_C:
		case SH_BOOT_U_O_C:
			is_offcharge = true;
			break;
		default:
			break;
	}
#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("boot mode 0x%04x\n", bootmode);
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	return is_offcharge;
}

static ssize_t print_switch_name_rndis(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME_RNDIS);
}

static ssize_t print_switch_state_rndis(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->state ? "enable" : "disable");
}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static void android_pm_qos_update_latency(struct android_dev *dev, int vote)
{
	struct android_usb_platform_data *pdata = dev->pdata;
	u32 swfi_latency = 0;
	static int last_vote = -1;

	if (!pdata || vote == last_vote
		|| !pdata->swfi_latency)
		return;

	swfi_latency = pdata->swfi_latency + 1;
	if (vote)
		pm_qos_update_request(&dev->pm_qos_req_dma,
				swfi_latency);
	else
		pm_qos_update_request(&dev->pm_qos_req_dma,
				PM_QOS_DEFAULT_VALUE);
	last_vote = vote;
}

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;
	int pm_qos_vote = -1;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
		next_state = USB_CONFIGURED;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
		next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
		if (dev->connected && strncmp(dev->pm_qos, "low", 3))
			pm_qos_vote = 1;
		else if (!dev->connected || !strncmp(dev->pm_qos, "low", 3))
			pm_qos_vote = 0;
	}
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (pm_qos_vote != -1)
		android_pm_qos_update_latency(dev, pm_qos_vote);

	if (uevent_envp) {
		/*
		 * Some userspace modules, e.g. MTP, work correctly only if
		 * CONFIGURED uevent is preceded by DISCONNECT uevent.
		 * Check if we missed sending out a DISCONNECT uevent. This can
		 * happen if host PC resets and configures device really quick.
		 */
		if (((uevent_envp == connected) &&
		      (last_uevent != USB_DISCONNECTED)) ||
		    ((uevent_envp == configured) &&
		      (last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
								disconnected);
			msleep(20);
		}
		/*
		 * Before sending out CONFIGURED uevent give function drivers
		 * a chance to wakeup userspace threads and notify disconnect
		 */
		if (uevent_envp == configured)
			msleep(50);

		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		last_uevent = next_state;
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}
}

static void android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (WARN_ON(!dev->disable_depth))
		return;

	if (--dev->disable_depth == 0) {
		usb_add_config(cdev, &android_config_driver,
					android_bind_config);
		usb_gadget_connect(cdev->gadget);
	}
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &android_config_driver);
	}
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

struct adb_data {
	bool opened;
	bool enabled;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (!is_offcharge_bootmode) {
		/* Disable the gadget until adbd is ready */
		if (!data->opened)
			android_disable(dev);
	}
#else /* CONFIG_USB_ANDROID_SH_CUST */
	/* Disable the gadget until adbd is ready */
	if (!data->opened)
		android_disable(dev);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
}

static void adb_android_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!data->opened)
		android_enable(dev);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

static void adb_ready_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
}


/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

#ifndef CONFIG_USB_ANDROID_SH_CUST
/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
};

/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
};

/*rmnet transport string format(per port):"ctrl0,data0,ctrl1,data1..." */
#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char buf[MAX_XPORT_STR_LEN], *b;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);
		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				err = frmnet_init_port(ctrl_name, data_name);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
};


/* MBIM - used with BAM */
#define MAX_MBIM_INSTANCES 1

static int mbim_function_init(struct android_usb_function *f,
					 struct usb_composite_dev *cdev)
{
	return mbim_init(MAX_MBIM_INSTANCES);
}

static void mbim_function_cleanup(struct android_usb_function *f)
{
	fmbim_cleanup();
}

static int mbim_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return mbim_bind_config(c, 0);
}

static struct android_usb_function mbim_function = {
	.name		= "usb_mbim",
	.cleanup	= mbim_function_cleanup,
	.bind_config	= mbim_function_bind_config,
	.init		= mbim_function_init,
};

/* PERIPHERAL AUDIO */
static int audio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return audio_bind_config(c);
}

static struct android_usb_function audio_function = {
	.name		= "audio",
	.bind_config	= audio_function_bind_config,
};
#endif /* CONFIG_USB_ANDROID_SH_CUST */


/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

#ifdef CONFIG_USB_ANDROID_SH_CUST
static ssize_t diag_enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_diag_enable_from_smem());
}

/* unlock */
static ssize_t unlocked_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->unlocked);
}

static ssize_t unlocked_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	int unlocked = 0;

	sscanf(buff, "%d", &unlocked);
	if (unlocked && !dev->unlocked) {
		dev->unlocked = true;
	} else if (!unlocked && dev->unlocked) {
		dev->unlocked = false;
	} else {
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_info("unlocked_store: already %s\n",
				dev->unlocked ? "unlocked" : "locked");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	}
	return size;
}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
#ifdef CONFIG_USB_ANDROID_SH_CUST
static struct device_attribute dev_attr_diag_enable =
	__ATTR(enable, S_IRUGO, diag_enable_show, NULL);
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static struct device_attribute *diag_function_attributes[] = {
	&dev_attr_clients,
#ifdef CONFIG_USB_ANDROID_SH_CUST
	&dev_attr_diag_enable,
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	NULL
};

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}

	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

/* SERIAL */
#ifdef CONFIG_USB_ANDROID_SH_CUST
#define MAX_GUID_NUM		16
#define MAX_GUID_STR_NUM	(MAX_GUID_NUM * 2)
static unsigned char mdlm_guid[MAX_GUID_NUM];

int sh_serial_setup(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	if (!dev->is_serial_set) {
		ret = gserial_setup(cdev->gadget, D_SH_SERIAL_SETUP_PORT_NUM);
		dev->is_serial_set = true;
	}

	return ret;
}
void sh_serial_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	if (dev->is_serial_set) {
		gserial_cleanup();
		dev->is_serial_set = false;
	}
}
static int sh_serial_mdlm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return sh_serial_setup(cdev);

}
int sh_serial_mdlm_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{

	set_guid_value(mdlm_guid);
	return gser_bind_config(c, D_SH_SERIAL_SETUP_PORT_MDLM);
}

static void sh_serial_mdlm_function_cleanup(struct android_usb_function *f)
{
	sh_serial_cleanup(f);
}

static ssize_t mdlm_guid_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		mdlm_guid[0], mdlm_guid[1], mdlm_guid[2], mdlm_guid[3],
		mdlm_guid[4], mdlm_guid[5], mdlm_guid[6], mdlm_guid[7],
		mdlm_guid[8], mdlm_guid[9], mdlm_guid[10], mdlm_guid[11],
		mdlm_guid[12], mdlm_guid[13], mdlm_guid[14], mdlm_guid[15]);
}

static ssize_t mdlm_guid_store(struct device *device, 
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int len;
	int i;
	char str[3];

	memset(str, 0x00, sizeof(str));

	len = strlen(buff);
	if(len < MAX_GUID_STR_NUM) {
		return -EINVAL;
	}

	for (i = 0; i < MAX_GUID_NUM; i++) {
		memcpy(str, buff, 2);
		mdlm_guid[i] = (unsigned char)simple_strtoul(str, NULL, 16);
		buff += 2;
	}
	return size;
}

static DEVICE_ATTR(guid, S_IRUGO | S_IWUSR, mdlm_guid_show, mdlm_guid_store);

static struct device_attribute *sh_serial_mdlm_function_attributes[] = {
	/* define the device attributes in sh_string.c */
	&dev_attr_mdlm_iInterface,
	&dev_attr_guid,
	NULL
};

static struct android_usb_function mdlm_function = {
	.name		= "mdlm",
	.init		= sh_serial_mdlm_function_init,
	.cleanup	= sh_serial_mdlm_function_cleanup,
	.bind_config	= sh_serial_mdlm_bind_config,
	.attributes	= sh_serial_mdlm_function_attributes,
};

static int sh_serial_obex_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{

	return sh_serial_setup(cdev);

}
int sh_serial_obex_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{

	return obex_bind_config(c, D_SH_SERIAL_SETUP_PORT_OBEX);
}

static void sh_serial_obex_function_cleanup(struct android_usb_function *f)
{
	sh_serial_cleanup(f);
}

static struct device_attribute *sh_serial_obex_function_attributes[] = {
	/* define the device attributes in sh_string.c */
	&dev_attr_obex_iInterface,
	NULL
};

static struct android_usb_function obex_function = {
	.name		= "obex",
	.init		= sh_serial_obex_function_init,
	.cleanup	= sh_serial_obex_function_cleanup,
	.bind_config	= sh_serial_obex_bind_config,
	.attributes	= sh_serial_obex_function_attributes,
};

#else /* CONFIG_USB_ANDROID_SH_CUST */

static char serial_transports[32];	/*enabled FSERIAL ports - "tty[,sdio]"*/
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int serial_initialized = 0, ports = 0;

	if (serial_initialized)
		goto bind_config;

	serial_initialized = 1;
	strlcpy(buf, serial_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = gserial_init_port(ports, name);
			if (err) {
				pr_err("serial: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = gser_bind_config(c, i);
		if (err) {
			pr_err("serial: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

#endif /* CONFIG_USB_ANDROID_SH_CUST */

/* ACM */
static char acm_transports[32];	/*enabled ACM ports - "tty[,sdio]"*/
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static struct device_attribute *acm_function_attributes[] = {
#ifdef CONFIG_USB_ANDROID_SH_SERIALS
		&dev_attr_acm_iInterface,
#endif /* CONFIG_USB_ANDROID_SH_SERIALS */
		&dev_attr_acm_transports, NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int acm_initialized, ports;

	if (acm_initialized)
		goto bind_config;

	acm_initialized = 1;
	strlcpy(buf, acm_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = acm_init_port(ports, name);
			if (err) {
				pr_err("acm: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = acm_port_setup(c);
	if (err) {
		pr_err("acm: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = acm_bind_config(c, i);
		if (err) {
			pr_err("acm: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}
static struct android_usb_function acm_function = {
#ifdef CONFIG_USB_ANDROID_SH_CUST
	.name		= "modem",
#else /* CONFIG_USB_ANDROID_SH_CUST */
	.name		= "acm",
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

#ifndef CONFIG_USB_ANDROID_SH_CUST
/* CCID */
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static int mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
#ifdef CONFIG_USB_ANDROID_SH_CUST
	.ctrlrequest	= mtp_function_ctrlrequest,
#endif /* CONFIG_USB_ANDROID_SH_CUST */
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	/* "Wireless" RNDIS; auto-detected by Windows */
	bool	wceis;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	_android_dev->sdev.name = SWITCH_NAME_RNDIS;
	_android_dev->sdev.print_name = print_switch_name_rndis;
	_android_dev->sdev.print_state = print_switch_state_rndis;
	if (switch_dev_register(&_android_dev->sdev) < 0) {
		pr_err("%s: switch_dev_register(%s) failed.\n", __func__, _android_dev->sdev.name);
		kfree(f->config);
		return -ENODEV;
	}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
#endif /* CONFIG_USB_DEBUG_SH_LOG */

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

#ifdef CONFIG_USB_ANDROID_SH_CUST
	ret = rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID, rndis->manufacturer);
	if (!ret)
		switch_set_state(&_android_dev->sdev, 1);
	return ret;
#else /* CONFIG_USB_ANDROID_SH_CUST */
	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
#ifdef CONFIG_USB_ANDROID_SH_CUST
	switch_set_state(&_android_dev->sdev, 0);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;

	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	u32 i, tmp_addr[ETH_ALEN];
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    &tmp_addr[0], &tmp_addr[1],
		    &tmp_addr[2], &tmp_addr[3],
		    &tmp_addr[4], &tmp_addr[5]) == ETH_ALEN) {
		for (i = 0; i < ETH_ALEN; i++) {
			rndis->ethaddr[i] = (u8)tmp_addr[i];
		}
		return size;
	}
#else /* CONFIG_USB_ANDROID_SH_CUST */
	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


#if !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD)
struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int i;
	const char *name[2];

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.nluns = 1;
	name[0] = "lun";
	if (dev->pdata->cdrom) {
		config->fsg.nluns = 2;
		config->fsg.luns[1].cdrom = 1;
		config->fsg.luns[1].ro = 1;
		config->fsg.luns[1].removable = 0;
		name[1] = "lun0";
	}

	config->fsg.luns[0].removable = 1;
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	config->fsg.luns[0].cdrom = 1;
	config->fsg.luns[0].ro = 1;
	config->fsg.release = 0x0100;
	config->fsg.can_stall = 1;
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	for (i = 0; i < config->fsg.nluns; i++) {
		err = sysfs_create_link(&f->dev->kobj,
					&common->luns[i].dev.kobj,
					name[i]);
		if (err)
			goto error;
	}

	config->common = common;
	f->config = config;
	return 0;
error:
	for (; i > 0 ; i--)
		sysfs_remove_link(&f->dev->kobj, name[i-1]);

	fsg_common_release(&common->ref);
	kfree(config);
	return err;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	char *line_feed;
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	strncpy(config->common->inquiry_string, buf, USB_INQUIRY_STRING_LEN);
	config->common->inquiry_string[USB_INQUIRY_STRING_LEN-1] = 0x00;
	line_feed = strstr(config->common->inquiry_string, "\n");
	if(line_feed) *line_feed = 0x00;
#else /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	/* define the device attributes in sh_string.c */
	&dev_attr_cd_iInterface,
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	NULL
};

static struct android_usb_function mass_storage_function = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
	.name		= "mass_storage_cd",
#else /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	.name		= "mass_storage",
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};
#endif /* !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD) */


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

#ifdef CONFIG_USB_ANDROID_SH_UMS
static int msc_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return msc_setup(cdev, f->dev);
}

static void msc_function_cleanup(struct android_usb_function *f)
{
	msc_cleanup();
}

static int msc_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return msc_bind_config(c);
}

static ssize_t msc_inquiry_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", _msc_dev->msc_inquiry_string);
}

static ssize_t msc_inquiry_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *line_feed;
	if (size >= sizeof(_msc_dev->msc_inquiry_string))
		return -EINVAL;
	strncpy(_msc_dev->msc_inquiry_string, buf, USB_MSC_INQUIRY_STRING_LEN);
	_msc_dev->msc_inquiry_string[USB_MSC_INQUIRY_STRING_LEN-1] = 0x00;
	line_feed = strstr(_msc_dev->msc_inquiry_string, "\n");
	if(line_feed) *line_feed = 0x00;
	return size;
}
static DEVICE_ATTR(msc_inquiry_string, S_IRUGO | S_IWUSR, msc_inquiry_show, msc_inquiry_store);

static struct device_attribute *msc_function_attributes[] = {
	&dev_attr_msc_inquiry_string,
	/* define the device attributes in sh_string.c */
	&dev_attr_msc_iInterface,
	NULL
};

static struct android_usb_function msc_function = {
	.name		= "mass_storage",
	.init		= msc_function_init,
	.cleanup	= msc_function_cleanup,
	.bind_config	= msc_function_bind_config,
	.attributes	= msc_function_attributes,
};
#endif /* CONFIG_USB_ANDROID_SH_UMS */

static struct android_usb_function *supported_functions[] = {
#ifdef CONFIG_USB_ANDROID_SH_CUST
	&rndis_function,
	&accessory_function,
	&audio_source_function,
	&obex_function,
	&mdlm_function,
	&acm_function,
#ifdef CONFIG_USB_ANDROID_SH_UMS
	&msc_function,
#endif /* CONFIG_USB_ANDROID_SH_UMS */
#if !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD)
	&mass_storage_function,
#endif /* !defined(CONFIG_USB_ANDROID_SH_UMS) || defined(CONFIG_USB_ANDROID_MASS_STORAGE_CD) */
	&mtp_function,
	&ptp_function,
	&adb_function,
	&diag_function,
	NULL
#else /* CONFIG_USB_ANDROID_SH_CUST */
	&mbim_function,
	&audio_function,
	&rmnet_smd_function,
	&rmnet_sdio_function,
	&rmnet_smd_sdio_function,
	&rmnet_function,
	&diag_function,
	&serial_function,
	&adb_function,
	&ccid_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
	&rndis_function,
	&mass_storage_function,
	&accessory_function,
	&audio_source_function,
	NULL
#endif /* CONFIG_USB_ANDROID_SH_CUST */
};

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		} else
			continue;

		if (f->cleanup)
			f->cleanup(f);

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++))
				device_remove_file(f->dev, attr);
		}
	}
}

static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 1; /* index 0 is for android0 device */

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		if (!f->dev_name) {
			err = -ENOMEM;
			goto err_out;
		}
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			f->dev = NULL;
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_init;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_attrs;
		}
	}
	return 0;

err_attrs:
	for (attr = *(attrs -= 2); attrs != f->attributes; attr = *(attrs--))
		device_remove_file(f->dev, attr);
	if (f->cleanup)
		f->cleanup(f);
err_init:
	device_destroy(android_class, f->dev->devt);
err_create:
	f->dev = NULL;
	kfree(f->dev_name);
err_out:
	android_cleanup_functions(dev->functions);
	return err;
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;
#ifdef CONFIG_USB_ANDROID_SH_MTP
	u8 need = 0;
	u8 count = 0;
#endif /* CONFIG_USB_ANDROID_SH_MTP */

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
#ifdef CONFIG_USB_ANDROID_SH_MTP
		count++;
		if (!strncmp(f->name , "mtp", strlen("mtp"))) {
			need = 1;
		}
#endif /* CONFIG_USB_ANDROID_SH_MTP */
	}
#ifdef CONFIG_USB_ANDROID_SH_MTP
	if (need == 1 && count < 4)
		setup_os_descriptor(count);
#endif /* CONFIG_USB_ANDROID_SH_MTP */
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	if (!strcmp(name, "diag") && !get_diag_enable_from_smem()) {
		pr_info("%s: diag cannot enable\n", __func__);
		return 0;
	}
	else if (!strcmp(name, "adb") && BOOT_MODE_TESTMODE == boot_mode_hold && !get_diag_enable_from_smem()) {
		pr_info("%s: adb cannot enable testmode\n", __func__);
		return 0;
	}
	else if (!strcmp(name, "adb") && BOOT_MODE_SOFTWARE_UPDATE == boot_mode_hold && !get_diag_enable_from_smem()) {
		pr_info("%s: adb cannot enable softupdate\n", __func__);
		return 0;
	}
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

#ifndef CONFIG_USB_ANDROID_SH_CUST
static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(android_config_driver.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	if (enable)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	return size;
}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	int enabled = 0;

	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/*
		 * Update values in composite driver's copy of
		 * device descriptor.
		 */
#ifdef CONFIG_USB_ANDROID_SH_CUST
		if (!dev->unlocked) {
#ifdef CONFIG_USB_DEBUG_SH_LOG
			pr_info("enable_store: locked \n");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
			mutex_unlock(&dev->mutex);
			return size;
		}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifndef CONFIG_USB_ANDROID_SH_CUST
		cdev->next_string_id = 0;
#endif /* CONFIG_USB_ANDROID_SH_CUST */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
		android_enable(dev);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
#ifdef CONFIG_USB_ANDROID_SH_CUST
		/* wait to prevent disconnect-failure */
		msleep(50);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	} else {
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t pm_qos_show(struct device *pdev,
			   struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%s\n", dev->pm_qos);
}

static ssize_t pm_qos_store(struct device *pdev,
			   struct device_attribute *attr,
			   const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	strlcpy(dev->pm_qos, buff, sizeof(dev->pm_qos));

	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

#ifdef CONFIG_USB_ANDROID_SH_CUST
static ssize_t bootmode_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	int bm = 0;

	switch ( boot_mode_hold ) {
		case BOOT_MODE_NORMAL:
			bm = 0;
			break;
		case BOOT_MODE_TESTMODE:
			bm = 1;
			break;
		case BOOT_MODE_SOFTWARE_UPDATE:
			bm = 2;
			break;
		default :
			bm = -1;
			break;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", bm);
}

#endif /* CONFIG_USB_ANDROID_SH_CUST */

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#ifndef CONFIG_USB_ANDROID_SH_CUST
/* move to sh_string.c */
#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	strlcpy(buffer, buf, sizeof(buffer));				\
	strim(buffer);							\
	return size;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);
#endif /* CONFIG_USB_ANDROID_SH_CUST */

DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
#ifndef CONFIG_USB_ANDROID_SH_CUST
/* move to sh_string.c */
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(pm_qos, S_IRUGO | S_IWUSR,
		pm_qos_show, pm_qos_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
#ifdef CONFIG_USB_ANDROID_SH_CUST
static DEVICE_ATTR(bootmode, S_IRUGO, bootmode_show, NULL);
static DEVICE_ATTR(unlocked, S_IRUGO | S_IWUSR, unlocked_show, unlocked_store);
#else
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifdef CONFIG_USB_ANDROID_SH_CUST
static ssize_t android_show_charge_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct android_dev *adev = dev_get_drvdata(dev);
	int value;

	if (!adev)
		return -ENODEV;

	value = snprintf(buf, PAGE_SIZE, "%d\n", adev->charge_enable);

	return value;
}

static ssize_t android_store_charge_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct android_dev *adev = dev_get_drvdata(dev);
	int value;
	int ret = 0;

	if (!adev)
		return -ENODEV;

	if ((size == 0) || (buf == NULL))
		return -EOPNOTSUPP;

	if (size >= 3)
		return -EOPNOTSUPP;

	if ((size == 2) && (*(buf+1) != 0x0a) && (*(buf+1) != 0x00))
		return -EOPNOTSUPP;

	if (*buf == '1') {
		adev->charge_enable = 1;
		value = size;
	}
	else if (*buf == '0') {
		adev->charge_enable = 0;
		value = size;
	}
	else {
		value = -EOPNOTSUPP;
	}

	if (value > 0) {
		if (adev->cdev && adev->cdev->gadget) {
			if (adev->charge_enable) 
				ret = usb_gadget_clear_selfpowered(adev->cdev->gadget);
			else
				ret = usb_gadget_set_selfpowered(adev->cdev->gadget);

			if (ret < 0)
				printk(KERN_ERR 
					"android_store_charge_enable: cant set(%d) \n",ret);
		}
	}

	return value;
}


static DEVICE_ATTR(charge_enable, ((S_IRUSR | S_IWUSR) | (S_IRGRP | S_IWGRP) | S_IROTH),
		android_show_charge_enable, android_store_charge_enable);

static int android_sysfs_init(struct device *dev)
{
	int			retval;

	if (dev) {
		sysfs_device = dev;
		retval = device_create_file(sysfs_device, &dev_attr_charge_enable);
		if (retval != 0)
			dev_err(sysfs_device,
				"failed to create sysfs entry(charge_enable):"
				"err:(%d)\n", retval);
	}

	return 0;
}

static void android_sysfs_cleanup(void)
{
	if (sysfs_device) {
		device_remove_file(sysfs_device, &dev_attr_charge_enable);
	}

}
#endif /* CONFIG_USB_ANDROID_SH_CUST */

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_pm_qos,
	&dev_attr_state,
#ifdef CONFIG_USB_ANDROID_SH_CUST
	&dev_attr_bootmode,
	&dev_attr_unlocked,
#else
	&dev_attr_remote_wakeup,
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int smem_charge_enable = 0;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	/*
	 * Start disconnected. Userspace will connect the gadget once
	 * it is done configuring the functions.
	 */
	usb_gadget_disconnect(gadget);

#ifndef CONFIG_USB_ANDROID_SH_CUST
	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strlcpy(manufacturer_string, "Android",
		sizeof(manufacturer_string) - 1);
	strlcpy(product_string, "Android", sizeof(product_string) - 1);
	strlcpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget_is_otg(cdev->gadget))
		android_config_driver.descriptors = otg_desc;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	dev->cdev = cdev;

#ifdef CONFIG_USB_ANDROID_SH_CUST
	if ( BOOT_MODE_TESTMODE == boot_mode_hold ) {
		usb_gadget_force_fullspeed(gadget);
	}

	/* charge enable */
	smem_charge_enable = get_charge_enable_from_smem();
	dev->charge_enable = !!smem_charge_enable;
	printk(KERN_ERR "%s dev->charge_enable=%d \n", __func__, dev->charge_enable);

	if (dev->charge_enable)
		usb_gadget_clear_selfpowered(gadget);
	else
		usb_gadget_set_selfpowered(gadget);
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	manufacturer_string[0] = '\0';
	product_string[0] = '\0';
	serial_string[0] = '0';
	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
	.max_speed	= USB_SPEED_SUPER
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);
	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int ret = 0;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	int result = 0;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	dev->pdata = pdata;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}

	if (pdata)
		composite_driver.usb_core_id = pdata->usb_core_id;

	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android "
				 "composite driver\n", __func__);
		goto err_probe;
	}

	/* pm qos request to prevent apps idle power collapse */
	if (pdata && pdata->swfi_latency)
		pm_qos_add_request(&dev->pm_qos_req_dma,
			PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	strlcpy(dev->pm_qos, "high", sizeof(dev->pm_qos));

#ifdef CONFIG_USB_ANDROID_SH_CUST
	dev_set_drvdata(&pdev->dev, _android_dev);

	result = android_sysfs_init(&pdev->dev);
	if (result)
		pr_info("%s: android_sysfs_init failed\n", __func__);
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	return ret;
err_probe:
	android_destroy_device(dev);
err_dev:
	class_destroy(android_class);
	return ret;
}

static int android_remove(struct platform_device *pdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;

	android_destroy_device(dev);
	class_destroy(android_class);
	usb_composite_unregister(&android_usb_driver);
	if (pdata && pdata->swfi_latency)
		pm_qos_remove_request(&dev->pm_qos_req_dma);

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
	.probe = android_probe,
	.remove = android_remove,
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		return -ENOMEM;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	dev->is_serial_set = false;
	/* judge boot mode */
	boot_mode_hold = get_boot_mode_from_smem();
	is_offcharge_bootmode = get_offcharge_bootmode_from_smem();
#endif /* CONFIG_USB_ANDROID_SH_CUST */
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	_android_dev = dev;

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;

	ret = platform_driver_register(&android_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		kfree(dev);
	}

#ifdef CONFIG_LISMO_BUFFER_CONTROL
	lun_init();
#endif /* CONFIG_LISMO_BUFFER_CONTROL */

	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
#ifdef CONFIG_USB_ANDROID_SH_CUST
	android_sysfs_cleanup();
#endif /* CONFIG_USB_ANDROID_SH_CUST */

	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
