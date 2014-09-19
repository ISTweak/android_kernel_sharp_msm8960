/* drivers/sharp/shspamp/shspamp_ak7811.c
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <sharp/shspamp.h>

#define	SHSPAMP_MUTEN_MUTE		0x00
#define	SHSPAMP_MUTEN_UNMUTE	0x01

static struct i2c_client *this_client;
static struct shspamp_platform_data *pdata;

static int is_on;
static DEFINE_MUTEX(spk_amp_lock);
static int shspamp_opened;

static int shspamp_i2c_write(int reg, u8 data)
{
    int rc;
    u8 buf[2];
    struct i2c_msg msg[] = {
    {
        .addr = this_client->addr,
        .flags= 0,
        .len  = 2,
        .buf  = buf,
        },
    };

    buf[0] = reg;
    buf[1] = data;
    rc = i2c_transfer(this_client->adapter, msg, 1);
    if(rc != 1){
        dev_err(&this_client->dev,
                "shspamp_i2c_write FAILED: writing to reg %d\n", reg);
        rc = -1;
    }
    return rc;
}

#if 0
static int shspamp_i2c_read(char *rxData, int length)
{
    struct i2c_msg msgs[] = {
        {
            .addr = this_client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };
    pr_err("{shspamp} %s\n", __func__);

    msgs.addr  = this_client->addr;
    msgs.flags = 0;
    msgs.len   = length;
    msgs.buf   = (unsigned char*)rxData;

    if (i2c_transfer(this_client->adapter, &msgs, 1) < 0) {
        pr_err("%s: I2C transfer error\n", __func__);
        return -EIO;
    }

#if DEBUG
    do {
        int i = 0;
        for (i = 0; i < length; i++)
            pr_info("%s: rx[%d] = %2x\n",
                __func__, i, rxData[i]);
    } while(0);
#endif
    return 0;
}
#endif
static int shspamp_open(struct inode *inode, struct file *file)
{
    int rc = 0;
    pr_debug("{shspamp} %s\n", __func__);

    mutex_lock(&spk_amp_lock);

    if (shspamp_opened) {
        pr_err("%s: busy\n", __func__);
        rc = -EBUSY;
        goto done;
    }

    shspamp_opened = 1;
done:
    mutex_unlock(&spk_amp_lock);
    return rc;
}
static int shspamp_release(struct inode *inode, struct file *file)
{

    pr_debug("{shspamp} %s\n", __func__);

    mutex_lock(&spk_amp_lock);
    shspamp_opened = 0;
    mutex_unlock(&spk_amp_lock);
    return 0;
}
static struct file_operations shspamp_fops = {
    .owner   = THIS_MODULE,
    .open    = shspamp_open,
    .release = shspamp_release,
};
static struct miscdevice shspamp_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shspamp",
    .fops = &shspamp_fops,
};
void shspamp_set_speaker_amp(int on)
{
    int ret =0;
    struct pm_gpio param = {
        .direction      = PM_GPIO_DIR_OUT,
        .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
        .output_value   = 1,
        .pull           = PM_GPIO_PULL_NO,
        .vin_sel        = PM_GPIO_VIN_S4,
        .out_strength   = PM_GPIO_STRENGTH_MED,
        .function       = PM_GPIO_FUNC_NORMAL,
    };

    pr_debug("{shspamp} %s \n", __func__);

    if (!pdata) {
        pr_err("%s: no platform data!\n", __func__);
        return;
    }
    mutex_lock(&spk_amp_lock);
    if (on && !is_on) {
        ret = gpio_request(pdata->gpio_shspamp_spk_en, "shspamp");
        if (ret) {
            pr_err("%s: Failed to request gpio %d\n", __func__,
                    pdata->gpio_shspamp_spk_en);
            goto err_free_gpio;
        }
        ret = pm8xxx_gpio_config(pdata->gpio_shspamp_spk_en, &param);
        if (ret){
            pr_err("%s: Failed to configure gpio %d\n", __func__,
                    pdata->gpio_shspamp_spk_en);
            goto err_free_gpio;
        }
        else{
            gpio_direction_output(pdata->gpio_shspamp_spk_en, 1);
        }
        shspamp_i2c_write(0x02, SHSPAMP_MUTEN_MUTE);   /* SLEEPN bit = 0 */
        shspamp_i2c_write(0x03, 0x14);                 /* [PRGAIN] GAIN:+19db */
        shspamp_i2c_write(0x04, 0x01);                 /* [MODE] MODE:Loud(ALC) */
        shspamp_i2c_write(0x06, 0x0d);                 /* [LOUD] HDRM:+2dB, Loud:LD1 */
/* [Protect Off] [start] */
        shspamp_i2c_write(0x1f, 0x96);                 /* Step1:Into TestMode */
        shspamp_i2c_write(0x1A, 0x02);                 /* Step2:Protect Off */
/* [Protect Off] [end] */
        shspamp_i2c_write(0x02, SHSPAMP_MUTEN_UNMUTE); /* SLEEPN bit = 1 */
        is_on = 1;
        msleep(28);
    } else if (!on && is_on) {
        shspamp_i2c_write(0x02, SHSPAMP_MUTEN_MUTE);
        is_on = 0;
        ret = gpio_request(pdata->gpio_shspamp_spk_en, "shspamp");
        if (ret) {
        pr_err("%s: Failed to request gpio %d\n", __func__,
            pdata->gpio_shspamp_spk_en);
        goto err_free_gpio;
        }
        ret = pm8xxx_gpio_config(pdata->gpio_shspamp_spk_en, &param);
        if (ret){
            pr_err("%s: Failed to configure gpio %d\n", __func__,
                pdata->gpio_shspamp_spk_en);
            goto err_free_gpio;
        }
        else{
            gpio_direction_output(pdata->gpio_shspamp_spk_en, 0);
        }
        pr_debug("%s: OFF\n", __func__);
    }

err_free_gpio:
    mutex_unlock(&spk_amp_lock);
    gpio_free(pdata->gpio_shspamp_spk_en);
    return;
}

static int shspamp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    pr_debug("{shspamp} %s\n", __func__);
    pdata = client->dev.platform_data;

    if (!pdata) {
        ret = -EINVAL;
        pr_err("%s: platform data is NULL\n", __func__);
        goto err_no_pdata;
    }

    this_client = client;

    ret = misc_register(&shspamp_device);
    if (ret) {
        pr_err("%s: shspamp_device register failed\n", __func__);
    }

    return 0;

err_no_pdata:
    return ret;
}
static int shspamp_remove(struct i2c_client *client)
{
    this_client = i2c_get_clientdata(client);
    return 0;
}

static const struct i2c_device_id shspamp_id[] = {
    { SHSPAMP_I2C_NAME, 0 },
    { }
};

static struct i2c_driver shspamp_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = SHSPAMP_I2C_NAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shspamp_probe,
    .id_table = shspamp_id,
    .remove   = shspamp_remove,
};

static int __init shspamp_init(void)
{
    pr_debug("{shspamp} %s\n", __func__);
    return i2c_add_driver(&shspamp_driver);
}

module_init(shspamp_init);

MODULE_DESCRIPTION("shspamp speaker amp driver");
MODULE_LICENSE("GPL");
