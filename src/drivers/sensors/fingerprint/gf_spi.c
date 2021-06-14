/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
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
 */
#define pr_fmt(fmt)     KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include "gf_spi.h"
#include <linux/msm_drm_notify.h>
#include <linux/atomic.h>

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 12

#define WAKELOCK_HOLD_TIME 3000 /* in ms */

#define GF_SPIDEV_NAME     "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME            "goodix_fp"
#define GF_INPUT_NAME       "qwerty"    /*"goodix_fp" */

#define CHRD_DRIVER_NAME    "goodix_fp_spi"
#define CLASS_NAME          "goodix_fp"

#define N_SPI_MINORS        32  /* ... up to 256 */
static int SPIDEV_MAJOR;
atomic_t g_notify_fod = ATOMIC_INIT(0);
volatile int g_id_fod = 10;
atomic_t g_fg_down = ATOMIC_INIT(0);
EXPORT_SYMBOL(g_notify_fod);

extern enum DEVICE_HWID g_ASUS_hwID;
extern int g_bl_lvl; //asus jacob add for fod
extern bool panelOff; //asus jacob add for fod
extern int lastFps; //asus jacob add for fod
extern bool g_enter_AOD; //asus jacob add for fod
extern unsigned int g_frame_count; //asus jacob add for fod

extern void set_fod_ir_drop(bool drop);
extern void set_panel_idle(bool enter);
extern void get_tcon_cmd(char cmd, int rlen);
extern bool g_panel_power_state_idle;
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
//static struct wake_lock fp_wakelock;
static struct wakeup_source fp_wakelock;
static struct gf_dev gf;

struct completion fod_fbc_comp;//asus jacob add for WA frame not update

static struct gf_key_map maps[] = {
    { EV_KEY, GF_KEY_INPUT_HOME },
    { EV_KEY, GF_KEY_INPUT_MENU },
    { EV_KEY, GF_KEY_INPUT_BACK },
    { EV_KEY, GF_KEY_INPUT_POWER },
#if defined(SUPPORT_NAV_EVENT)
    { EV_KEY, GF_NAV_INPUT_UP },
    { EV_KEY, GF_NAV_INPUT_DOWN },
    { EV_KEY, GF_NAV_INPUT_RIGHT },
    { EV_KEY, GF_NAV_INPUT_LEFT },
    { EV_KEY, GF_KEY_INPUT_CAMERA },
    { EV_KEY, GF_NAV_INPUT_CLICK },
    { EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
    { EV_KEY, GF_NAV_INPUT_LONG_PRESS },
    { EV_KEY, GF_NAV_INPUT_HEAVY },
#endif
};


/* --- asus jacob add for test --- */
#if 1
static void FOD_AUTH(struct work_struct *work) {
    char msg = 0;
    struct gf_dev *gf_dev = &gf;

    printk("[JK][SPI] FOD_AUTH, g_notify_fod = %d\n", atomic_read(&g_notify_fod));

    if (g_id_fod == 12) {
        reinit_completion(&fod_fbc_comp);		
        printk("[JK][SPI] reinit_completion\n");
    }

    if (!wait_for_completion_timeout(&fod_fbc_comp, HZ * 0.3)) {
        printk("[JK][FOD_ERR] wait_for_completion_timeout\n");
        return;
    }

    spin_lock(&gf_dev->timer_lock);
    if (gf_dev->timer_init && atomic_read(&g_notify_fod) && atomic_read(&g_fg_down)) {
        msg = GF_NET_EVENT_FG_DOWN;
        sendnlmsg(&msg);
        if (g_ASUS_hwID > 2) {
            gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);
        } else {
            gpio_set_value(gf_dev->lhbm_exi1_gpio, 1);
        }
        pr_info("Get fod exi value : exi1 = %d exi2 = %d fc = %d \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio), g_frame_count);            
        //msleep(10); remove delay 20190502
    //    __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
    }
    spin_unlock(&gf_dev->timer_lock);
    return;
}
#if 0
static void init_FOD_Timer(struct gf_dev *gf_dev, int delay_time)
{

    unsigned long expires;



    if (atomic_read(&g_notify_fod)) {
        printk("[JK] Init touch timer\n");
        if (!gf_dev->Touch_Timer_expires) {
            init_timer(&gf_dev->Touch_Timer);
            gf_dev->Touch_Timer.function = FOD_AUTH;
            gf_dev->Touch_Timer.expires = jiffies + msecs_to_jiffies(delay_time);
            add_timer(&gf_dev->Touch_Timer);
            gf_dev->Touch_Timer_expires = gf_dev->Touch_Timer.expires;
        } else {
            expires = jiffies + msecs_to_jiffies(delay_time);
            if (!expires)
                expires = 1;

            if (!gf_dev->Touch_Timer_expires || time_after(expires, gf_dev->Touch_Timer_expires)) {
                mod_timer(&gf_dev->Touch_Timer, expires);
                gf_dev->Touch_Timer_expires = expires;
            }
        }

        gf_dev->timer_init=true;

    } else {
        printk("[JK] cancel fod no need Init timer\n");
        gf_dev->timer_init=false;        
    }


    return;
}

static void del_FOD_Timer(struct gf_dev *gf_dev)
{

    if (gf_dev->Touch_Timer_expires) {
        del_timer(&gf_dev->Touch_Timer);
        gf_dev->Touch_Timer_expires = 0;
        printk("[JK] Del Disable touch timer\n");
    }

    gf_dev->timer_init=false;

    return;
}
#endif
#endif
/* --- asus jacob add for test --- */


//static int fod_position[4] = {480, 600,1950, 2110}; move to goodix touch driver

void TP_call_FOD(int id, bool down){
    char msg = 0;
    struct gf_dev *gf_dev = &gf;
    int TP_gesture = 12;
    int TP_gesture_wai_panel = 11;

    pr_info("[%s] lhbm_exi2_gpio = %d  BL_val = %d lastfps = %d id = %d \n", __func__, gf_dev->lhbm_exi2_gpio, g_bl_lvl, lastFps, id);
    if (down)
        atomic_set(&g_fg_down, 1);
    else
        atomic_set(&g_fg_down, 0);

	pr_info("[%s] fb_black = %d  panelOff = %d g_notify_fod = %d\n", __func__, gf_dev->fb_black, panelOff, atomic_read(&g_notify_fod));
    if (!gf_dev->lhbm_exi2_gpio | gf_dev->fb_black | panelOff | !atomic_read(&g_notify_fod) | !g_bl_lvl ) {
        if (id == TP_gesture) {
            if (down)
                g_id_fod = TP_gesture;
            else 
                g_id_fod = 10;            
        }
        if ( id == TP_gesture_wai_panel || !down) {

        } else {
            __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
            gf_dev->timer_init=false;
            pr_info("[%s] Do not things  \n", __func__);
            return;
        }

    }

    __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);

    if (down) {
        if (id < g_id_fod) {
            if (!g_enter_AOD && atomic_read(&g_notify_fod)) {
                pr_info("START_FOD test  FG down \n");
                g_id_fod = id;
                msg = GF_NET_EVENT_FG_DOWN;
                sendnlmsg(&msg);
                if (g_ASUS_hwID > 2) {
                    set_fod_ir_drop(false);
                    gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);
                } else {
                    gpio_set_value(gf_dev->lhbm_exi1_gpio, 1);
                }
            }
            //msleep(10); remove delay 20190502
            
        } else if (id == TP_gesture) {
            pr_info("START_FOD in AOD  FG down 1\n");
            g_id_fod = id;
            get_tcon_cmd(0x0a, 1);
            set_fod_ir_drop(false);
            if (g_panel_power_state_idle)
                set_panel_idle(false);
            schedule_delayed_work(&gf_dev->FOD_Auth_work, msecs_to_jiffies(110));
//            init_FOD_Timer(gf_dev,110);
            gf_dev->timer_init=true;
            //reinit_completion(&fod_fbc_comp);
            /*
            if (g_ASUS_hwID > 2) {

//                msleep(60);
                gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);
            } else {
                gpio_set_value(gf_dev->lhbm_exi1_gpio, 1);
            }
            //msleep(10); remove delay 20190502
            __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
            msg = GF_NET_EVENT_FG_DOWN;
            sendnlmsg(&msg);*/
        } else if (g_id_fod == TP_gesture && id == TP_gesture_wai_panel){
            pr_info("START_FOD in AOD  FG down 2\n");
            g_id_fod = id;
            set_fod_ir_drop(false);
//            set_panel_idle(false);
            //init_FOD_Timer(gf_dev, 3);
            schedule_delayed_work(&gf_dev->FOD_Auth_work, msecs_to_jiffies(3));
            gf_dev->timer_init=true;
            /*
            if (g_ASUS_hwID > 2) {
                set_fod_ir_drop(false);
//                set_panel_idle(false);
//                msleep(60);
                gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);
            } else {
                gpio_set_value(gf_dev->lhbm_exi1_gpio, 1);
            }
            //msleep(10); remove delay 20190502
            __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
            msg = GF_NET_EVENT_FG_DOWN;
            sendnlmsg(&msg);*/
        }
    } else {
        pr_info("START_FOD test  FG up \n");
        cancel_delayed_work(&gf_dev->FOD_Auth_work);
        g_id_fod = 10;
        msg = GF_NET_EVENT_FG_UP;
        set_fod_ir_drop(true);
        sendnlmsg(&msg);
        spin_lock(&gf_dev->timer_lock);
        //del_FOD_Timer(gf_dev);
        gf_dev->timer_init=false;
        if (&fod_fbc_comp != NULL)
            complete_all(&fod_fbc_comp);
        gpio_set_value(gf_dev->lhbm_exi1_gpio, 0);
        gpio_set_value(gf_dev->lhbm_exi2_gpio, 0);
        spin_unlock(&gf_dev->timer_lock);
    }
    pr_info("Get fod exi value : exi1 = %d exi2 = %d fc = %d \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio), g_frame_count);            

}
EXPORT_SYMBOL(TP_call_FOD);

static void gf_enable_irq(struct gf_dev *gf_dev)
{
    if (gf_dev->irq_enabled) {
        pr_warn("IRQ has been enabled.\n");
    } else {
        enable_irq(gf_dev->irq);
        gf_dev->irq_enabled = 1;
    }
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
    if (gf_dev->irq_enabled) {
        gf_dev->irq_enabled = 0;
        disable_irq(gf_dev->irq);
    } else {
        pr_warn("IRQ has been disabled.\n");
    }
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
    long lowest_available, nearest_low, step_size, cur;
    long step_direction = -1;
    long guess = rate;
    int max_steps = 10;

    cur = clk_round_rate(clk, rate);
    if (cur == rate)
        return rate;

    /* if we got here then: cur > rate */
    lowest_available = clk_round_rate(clk, 0);
    if (lowest_available > rate)
        return -EINVAL;

    step_size = (rate - lowest_available) >> 1;
    nearest_low = lowest_available;

    while (max_steps-- && step_size) {
        guess += step_size * step_direction;
        cur = clk_round_rate(clk, guess);

        if ((cur < rate) && (cur > nearest_low))
            nearest_low = cur;
        /*
         * if we stepped too far, then start stepping in the other
         * direction with half the step size
         */
        if (((cur > rate) && (step_direction > 0))
                || ((cur < rate) && (step_direction < 0))) {
            step_direction = -step_direction;
            step_size >>= 1;
        }
    }
    return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
    long rate;
    int rc;

    rate = spi_clk_max_rate(gf_dev->core_clk, speed);
    if (rate < 0) {
        pr_info("%s: no match found for requested clock frequency:%d",
                __func__, speed);
        return;
    }

    rc = clk_set_rate(gf_dev->core_clk, rate);
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
    pr_debug("%s: enter\n", __func__);

    data->clk_enabled = 0;
    data->core_clk = clk_get(&data->spi->dev, "core_clk");
    if (IS_ERR_OR_NULL(data->core_clk)) {
        pr_err("%s: fail to get core_clk\n", __func__);
        return -EPERM;
    }
    data->iface_clk = clk_get(&data->spi->dev, "iface_clk");
    if (IS_ERR_OR_NULL(data->iface_clk)) {
        pr_err("%s: fail to get iface_clk\n", __func__);
        clk_put(data->core_clk);
        data->core_clk = NULL;
        return -ENOENT;
    }
    return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
    int err;

    pr_debug("%s: enter\n", __func__);

    if (data->clk_enabled)
        return 0;

    err = clk_prepare_enable(data->core_clk);
    if (err) {
        pr_err("%s: fail to enable core_clk\n", __func__);
        return -EPERM;
    }

    err = clk_prepare_enable(data->iface_clk);
    if (err) {
        pr_err("%s: fail to enable iface_clk\n", __func__);
        clk_disable_unprepare(data->core_clk);
        return -ENOENT;
    }

    data->clk_enabled = 1;

    return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
    pr_debug("%s: enter\n", __func__);

    if (!data->clk_enabled)
        return 0;

    clk_disable_unprepare(data->core_clk);
    clk_disable_unprepare(data->iface_clk);
    data->clk_enabled = 0;

    return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
    pr_debug("%s: enter\n", __func__);

    if (data->clk_enabled)
        gfspi_ioctl_clk_disable(data);

    if (!IS_ERR_OR_NULL(data->core_clk)) {
        clk_put(data->core_clk);
        data->core_clk = NULL;
    }

    if (!IS_ERR_OR_NULL(data->iface_clk)) {
        clk_put(data->iface_clk);
        data->iface_clk = NULL;
    }

    return 0;
}
#endif

static void nav_event_input(struct gf_dev *gf_dev, gf_nav_event_t nav_event)
{
    uint32_t nav_input = 0;

    switch (nav_event) {
    case GF_NAV_FINGER_DOWN:
        pr_debug("%s nav finger down\n", __func__);
        break;

    case GF_NAV_FINGER_UP:
        pr_debug("%s nav finger up\n", __func__);
        break;

    case GF_NAV_DOWN:
        nav_input = GF_NAV_INPUT_DOWN;
        pr_debug("%s nav down\n", __func__);
        break;

    case GF_NAV_UP:
        nav_input = GF_NAV_INPUT_UP;
        pr_debug("%s nav up\n", __func__);
        break;

    case GF_NAV_LEFT:
        nav_input = GF_NAV_INPUT_LEFT;
        pr_debug("%s nav left\n", __func__);
        break;

    case GF_NAV_RIGHT:
        nav_input = GF_NAV_INPUT_RIGHT;
        pr_debug("%s nav right\n", __func__);
        break;

    case GF_NAV_CLICK:
        nav_input = GF_NAV_INPUT_CLICK;
        pr_debug("%s nav click\n", __func__);
        break;

    case GF_NAV_HEAVY:
        nav_input = GF_NAV_INPUT_HEAVY;
        pr_debug("%s nav heavy\n", __func__);
        break;

    case GF_NAV_LONG_PRESS:
        nav_input = GF_NAV_INPUT_LONG_PRESS;
        pr_debug("%s nav long press\n", __func__);
        break;

    case GF_NAV_DOUBLE_CLICK:
        nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
        pr_debug("%s nav double click\n", __func__);
        break;

    default:
        pr_warn("%s unknown nav event: %d\n", __func__, nav_event);
        break;
    }

    if ((nav_event != GF_NAV_FINGER_DOWN) &&
            (nav_event != GF_NAV_FINGER_UP)) {
        input_report_key(gf_dev->input, nav_input, 1);
        input_sync(gf_dev->input);
        input_report_key(gf_dev->input, nav_input, 0);
        input_sync(gf_dev->input);
    }
}

static irqreturn_t gf_irq(int irq, void *handle)
{
#if defined(GF_NETLINK_ENABLE)
    char msg = GF_NET_EVENT_IRQ;

    //wake_lock_timeout(&fp_wakelock, msecs_to_jiffies(WAKELOCK_HOLD_TIME));
    __pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
    sendnlmsg(&msg);
#elif defined(GF_FASYNC)
    struct gf_dev *gf_dev = &gf;

    if (gf_dev->async)
        kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
    printk("[FP][%s]irq~~~~~  ! \n", __func__);
    return IRQ_HANDLED;
}

static int irq_setup(struct gf_dev *gf_dev)
{
    int status;

    gf_dev->irq = gf_irq_num(gf_dev);
    status = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            "gf", gf_dev);

    if (status) {
        pr_err("failed to request IRQ:%d\n", gf_dev->irq);
        return status;
    }
    enable_irq_wake(gf_dev->irq);
    gf_dev->irq_enabled = 1;

    return status;
}

static void irq_cleanup(struct gf_dev *gf_dev)
{
    gf_dev->irq_enabled = 0;
    disable_irq(gf_dev->irq);
    disable_irq_wake(gf_dev->irq);
    free_irq(gf_dev->irq, gf_dev);
}

static void gf_kernel_key_input(struct gf_dev *gf_dev, struct gf_key *gf_key)
{
    uint32_t key_input = 0;

    if (gf_key->key == GF_KEY_HOME) {
        key_input = GF_KEY_INPUT_HOME;
    } else if (gf_key->key == GF_KEY_POWER) {
        key_input = GF_KEY_INPUT_POWER;
    } else if (gf_key->key == GF_KEY_CAMERA) {
        key_input = GF_KEY_INPUT_CAMERA;
    } else {
        /* add special key define */
        key_input = gf_key->key;
    }
    pr_info("%s: received key event[%d], key=%d, value=%d\n",
            __func__, key_input, gf_key->key, gf_key->value);

    if ((GF_KEY_POWER == gf_key->key || GF_KEY_CAMERA == gf_key->key)
            && (gf_key->value == 1)) {
        input_report_key(gf_dev->input, key_input, 1);
        input_sync(gf_dev->input);
        input_report_key(gf_dev->input, key_input, 0);
        input_sync(gf_dev->input);
    }

    if (gf_key->key == GF_KEY_HOME) {
        input_report_key(gf_dev->input, key_input, gf_key->value);
        input_sync(gf_dev->input);
    }
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = &gf;
    struct gf_key gf_key;
#if defined(SUPPORT_NAV_EVENT)
    gf_nav_event_t nav_event = GF_NAV_NONE;
#endif
    int retval = 0;
    int RstDelatTime = 0;
    u8 netlink_route = NETLINK_TEST;
    //char msg = 0;
    struct gf_ioc_chip_info info;

    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
        return -ENODEV;

    if (_IOC_DIR(cmd) & _IOC_READ)
        retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (retval)
        return -EFAULT;
    printk("[FP][%s]cmd %d  ! \n", __func__, cmd);
    switch (cmd) {
    case GF_IOC_INIT:
        printk("[FP][%s]GF_IOC_INIT \n", __func__);
        pr_debug("%s GF_IOC_INIT\n", __func__);
        if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
            pr_err("GF_IOC_INIT failed\n");
            retval = -EFAULT;
            break;
        }
        break;

    case GF_IOC_EXIT:
        printk("[FP][%s]GF_IOC_EXIT \n", __func__);
        pr_debug("%s GF_IOC_EXIT\n", __func__);
        break;

    case GF_IOC_DISABLE_IRQ:
        printk("[FP][%s]GF_IOC_DISABLE_IRQ \n", __func__);
        pr_debug("%s GF_IOC_DISABEL_IRQ\n", __func__);
        gf_disable_irq(gf_dev);
        break;

    case GF_IOC_ENABLE_IRQ:
        printk("[FP][%s]GF_IOC_ENABLE_IRQ \n", __func__);
        pr_debug("%s GF_IOC_ENABLE_IRQ\n", __func__);
        gf_enable_irq(gf_dev);
        break;

    case GF_IOC_RESET:
        pr_debug("%s GF_IOC_RESET\n", __func__);
        if (g_enter_AOD)
            RstDelatTime = 3 * (1000/30);
        else
            RstDelatTime = 3 * (1000/lastFps);

        printk("[FP][%s]GF_IOC_RESET delay time %d \n", __func__, RstDelatTime);
        if (RstDelatTime <= 7)
            RstDelatTime = 7;
        gf_hw_reset(gf_dev, RstDelatTime);
        break;

    case GF_IOC_INPUT_KEY_EVENT:
        printk("[FP][%s]GF_IOC_INPUT_KEY_EVENT \n", __func__);
        if (copy_from_user(&gf_key, (void __user *)arg, sizeof(struct gf_key))) {
            pr_err("failed to copy input key event from user to kernel\n");
            retval = -EFAULT;
            break;
        }

        gf_kernel_key_input(gf_dev, &gf_key);
        break;

#if defined(SUPPORT_NAV_EVENT)
    case GF_IOC_NAV_EVENT:
        printk("[FP][%s]GF_IOC_NAV_EVENT \n", __func__);
        pr_debug("%s GF_IOC_NAV_EVENT\n", __func__);
        if (copy_from_user(&nav_event, (void __user *)arg, sizeof(gf_nav_event_t))) {
            pr_err("failed to copy nav event from user to kernel\n");
            retval = -EFAULT;
            break;
        }

        nav_event_input(gf_dev, nav_event);
        break;
#endif

    case GF_IOC_ENABLE_SPI_CLK:
        printk("[FP][%s]GF_IOC_ENABLE_SPI_CLK \n", __func__);
        pr_debug("%s GF_IOC_ENABLE_SPI_CLK\n", __func__);
#ifdef AP_CONTROL_CLK
        gfspi_ioctl_clk_enable(gf_dev);
#else
        pr_debug("doesn't support control clock!\n");
#endif
        break;

    case GF_IOC_DISABLE_SPI_CLK:
        printk("[FP][%s]GF_IOC_DISABLE_SPI_CLK \n", __func__);
        pr_debug("%s GF_IOC_DISABLE_SPI_CLK\n", __func__);
#ifdef AP_CONTROL_CLK
        gfspi_ioctl_clk_disable(gf_dev);
#else
        pr_debug("doesn't support control clock!\n");
#endif
        break;

    case GF_IOC_ENABLE_POWER:
        printk("[FP][%s]GF_IOC_ENABLE_POWER \n", __func__);
        pr_debug("%s GF_IOC_ENABLE_POWER\n", __func__);
        gf_power_on(gf_dev);
        break;

    case GF_IOC_DISABLE_POWER:
        printk("[FP][%s]GF_IOC_DISABLE_POWER \n", __func__);
        pr_debug("%s GF_IOC_DISABLE_POWER\n", __func__);
        gf_power_off(gf_dev);
        break;

    case GF_IOC_ENTER_SLEEP_MODE:
        printk("[FP][%s]GF_IOC_ENTER_SLEEP_MODE \n", __func__);
        pr_debug("%s GF_IOC_ENTER_SLEEP_MODE\n", __func__);
        break;

    case GF_IOC_GET_FW_INFO:
        printk("[FP][%s]GF_IOC_GET_FW_INFO \n", __func__);
        pr_debug("%s GF_IOC_GET_FW_INFO\n", __func__);
        break;

    case GF_IOC_REMOVE:
        printk("[FP][%s]GF_IOC_REMOVE \n", __func__);
        pr_debug("%s GF_IOC_REMOVE\n", __func__);
        break;

    case GF_IOC_CHIP_INFO:
        printk("[FP][%s]GF_IOC_CHIP_INFO \n", __func__);
        pr_debug("%s GF_IOC_CHIP_INFO\n", __func__);
        if (copy_from_user(&info, (void __user *)arg, sizeof(struct gf_ioc_chip_info))) {
            retval = -EFAULT;
            break;
        }
        pr_info("vendor_id : 0x%x\n", info.vendor_id);
        pr_info("mode : 0x%x\n", info.mode);
        pr_info("operation: 0x%x\n", info.operation);
        break;
    case GF_IOC_START_FOD:
        printk("[FP][%s]GF_IOC_START_FOD \n", __func__);
        pr_info("START_FOD test  START \n");
        if (gf_dev->lhbm_exi2_gpio < 0) {
            pr_err("[FOD] ignore  FOD feature !\n");
        } else {
            atomic_set(&g_notify_fod, 1);
        }
        //msg = GF_NET_EVENT_FG_DOWN;
        //sendnlmsg(&msg);
        break;
    case GF_IOC_STOP_FOD:
        printk("[FP][%s]GF_IOC_STOP_FOD \n", __func__);
        pr_info("START_FOD test  STOP \n");
        atomic_set(&g_notify_fod, 0);
        if (gf_dev->lhbm_exi2_gpio) {
            set_fod_ir_drop(true);
            spin_lock(&gf_dev->timer_lock);
            //del_FOD_Timer(gf_dev);
            gf_dev->timer_init=false;
            cancel_delayed_work(&gf_dev->FOD_Auth_work);
            gpio_set_value(gf_dev->lhbm_exi1_gpio, 0);
            gpio_set_value(gf_dev->lhbm_exi2_gpio, 0);
            spin_unlock(&gf_dev->timer_lock);
            pr_info("Get fod exi value : exi1 = %d exi2 = %d fc = %d \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio), g_frame_count);            
        } else {
            printk("[FP][%s]skip FOD relay \n", __func__);
        }
        g_id_fod = 10;
        //msg = GF_NET_EVENT_FG_UP;
        //sendnlmsg(&msg);
        break;
    case GF_IOC_TEST_FOD_EXI1:
        printk("[FP][%s]GF_IOC_TEST_FOD_EXI1 %d  \n", __func__, gf_dev->lhbm_exi1_gpio);
        pr_info("Set FOD EXI1 \n");
        if (gf_dev->lhbm_exi2_gpio) {
            set_fod_ir_drop(false);
            gpio_set_value(gf_dev->lhbm_exi1_gpio, 1);
            msleep(100);
            pr_info("Get fod exi value : exi1 = %d exi2 = %d  \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio));            
        } else {
            printk("[FP][%s]skip FOD relay \n", __func__);
        }
        //msg = GF_NET_EVENT_FG_UP;
        //sendnlmsg(&msg);
        break;
    case GF_IOC_TEST_FOD_EXI2:
        printk("[FP][%s]GF_IOC_TEST_FOD_EXI2 %d \n", __func__, gf_dev->lhbm_exi2_gpio);
        pr_info("Set FOD EXI2 \n");
        if (gf_dev->lhbm_exi2_gpio) {
            set_fod_ir_drop(false);
            gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);
            msleep(100);
            pr_info("Get fod exi value : exi1 = %d exi2 = %d  \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio));            
        } else {
            printk("[FP][%s]skip FOD relay \n", __func__);
        }
        //msg = GF_NET_EVENT_FG_UP;
        //sendnlmsg(&msg);
        break;
    case GF_IOC_TEST_FOD_END:
        printk("[FP][%s]GF_IOC_TEST_FOD_END \n", __func__);
        pr_info("Clean LHBM \n");
        if (gf_dev->lhbm_exi2_gpio) {
            set_fod_ir_drop(true);
            spin_lock(&gf_dev->timer_lock);
            //del_FOD_Timer(gf_dev);
            gf_dev->timer_init=false;
            cancel_delayed_work(&gf_dev->FOD_Auth_work);
            gpio_set_value(gf_dev->lhbm_exi1_gpio, 0);
            gpio_set_value(gf_dev->lhbm_exi2_gpio, 0);
            spin_unlock(&gf_dev->timer_lock);
            pr_info("Get fod exi value : exi1 = %d exi2 = %d fc = %d \n", gpio_get_value(gf_dev->lhbm_exi1_gpio), gpio_get_value(gf_dev->lhbm_exi2_gpio), g_frame_count);            
        } else {
            printk("[FP][%s]skip FOD relay \n", __func__);
        }

        //msg = GF_NET_EVENT_FG_UP;
        //sendnlmsg(&msg);
        break;

    default:
        pr_warn("unsupport cmd:0x%x\n", cmd);
        break;
    }

    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/


static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev = &gf;
    int status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
        if (gf_dev->devt == inode->i_rdev) {
            pr_info("Found\n");
            status = 0;
            break;
        }
    }

    if (status == 0) {
        if (status == 0) {
            gf_dev->users++;
            filp->private_data = gf_dev;
            nonseekable_open(inode, filp);
            pr_info("Succeed to open device. irq = %d\n",
                    gf_dev->irq);
            if (gf_dev->users == 1) {
                status = gf_power_on(gf_dev);
                //status = gf_parse_dts(gf_dev);
                if (status)
                    goto err_parse_dt;

                status = irq_setup(gf_dev);
                if (status)
                    goto err_irq;
            }
            gf_hw_reset(gf_dev, 3);
            gf_dev->device_available = 1;
        }
    } else {
        pr_info("No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);

    return status;
err_irq:
    gf_cleanup(gf_dev);
err_parse_dt:
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret;

    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    pr_info("ret = %d\n", ret);
    return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev = &gf;
    int status = 0;

    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close?? */
    gf_dev->users--;
    if (!gf_dev->users) {
        irq_cleanup(gf_dev);
        gf_cleanup(gf_dev);

        /*power off the sensor*/
        gf_dev->device_available = 0;
        gf_power_off(gf_dev);
    }
    mutex_unlock(&device_list_lock);
    return status;
}

static const struct file_operations gf_fops = {
    .owner = THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
    .open = gf_open,
    .release = gf_release,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
        unsigned long val, void *data)
{
    struct gf_dev *gf_dev;
    //struct fb_event *evdata = data;
    struct msm_drm_notifier *evdata = data;
    unsigned int blank;
    char msg = 0;

    if (val != MSM_DRM_EARLY_EVENT_BLANK)
        return 0;
    pr_info("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
            __func__, (int)val);
    gf_dev = container_of(nb, struct gf_dev, notifier);
//  if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
    if (evdata && evdata->data && val == MSM_DRM_EARLY_EVENT_BLANK && gf_dev) {

        blank = *(int *)(evdata->data);
//        pr_info("[info] %s blank value = %d\n", __func__, blank);
        switch (blank) {
        case MSM_DRM_BLANK_POWERDOWN:
            if (gf_dev->device_available == 1) {
                gf_dev->fb_black = 1;
#if defined(GF_NETLINK_ENABLE)
                msg = GF_NET_EVENT_FB_BLACK;
                sendnlmsg(&msg);
#elif defined(GF_FASYNC)
                if (gf_dev->async)
                    kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
            if (gf_dev->lhbm_exi2_gpio) {
                pr_info("[info] Force cancel \n");
                msg = GF_NET_EVENT_FG_UP;
//                set_fod_ir_drop(true);
                if (!gpio_get_value(gf_dev->lhbm_exi2_gpio))
                    gpio_set_value(gf_dev->lhbm_exi2_gpio, 1);

                gpio_set_value(gf_dev->lhbm_exi1_gpio, 0);
                gpio_set_value(gf_dev->lhbm_exi2_gpio, 0);
                sendnlmsg(&msg);
                g_id_fod = 10;
            }

            }
            break;
        case MSM_DRM_BLANK_UNBLANK:
            if (gf_dev->device_available == 1) {
                gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
                msg = GF_NET_EVENT_FB_UNBLACK;
                sendnlmsg(&msg);
#elif defined(GF_FASYNC)
                if (gf_dev->async)
                    kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
            }
            break;
        default:
            pr_info("%s defalut\n", __func__);
            break;
        }
    }
    return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
    .notifier_call = goodix_fb_state_chg_callback,
};

static struct class *gf_class;
#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
    struct gf_dev *gf_dev = &gf;
    int status = -EINVAL;
    unsigned long minor;
    int i;

    /* Initialize the driver data */
    INIT_LIST_HEAD(&gf_dev->device_entry);
#if defined(USE_SPI_BUS)
    gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
    gf_dev->spi = pdev;
#endif
    gf_dev->irq_gpio = -EINVAL;
    gf_dev->reset_gpio = -EINVAL;
    gf_dev->pwr_gpio = -EINVAL;
    gf_dev->device_available = 0;
    gf_dev->fb_black = 0;
    gf_dev->power_init=false;
    gf_dev->lhbm_exi2_gpio=0;
    gf_dev->Touch_Timer_expires=0;
    gf_dev->timer_init=false;

    status = gf_parse_dts(gf_dev);

    INIT_DELAYED_WORK(&gf_dev->FOD_Auth_work, FOD_AUTH);

    init_completion(&fod_fbc_comp);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
        dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
                gf_dev, GF_DEV_NAME);
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
        status = -ENODEV;
        mutex_unlock(&device_list_lock);
        goto error_hw;
    }

    if (status == 0) {
        set_bit(minor, minors);
        list_add(&gf_dev->device_entry, &device_list);
    } else {
        gf_dev->devt = 0;
        goto error_hw;
    }
    mutex_unlock(&device_list_lock);

    gf_dev->input = input_allocate_device();
    if (gf_dev->input == NULL) {
        pr_err("%s, failed to allocate input device\n", __func__);
        status = -ENOMEM;
        goto error_dev;
    }
    for (i = 0; i < ARRAY_SIZE(maps); i++)
        input_set_capability(gf_dev->input, maps[i].type, maps[i].code);

    gf_dev->input->name = GF_INPUT_NAME;
    status = input_register_device(gf_dev->input);
    if (status) {
        pr_err("failed to register input device\n");
        goto error_input;
    }
    complete_all(&fod_fbc_comp);
#ifdef AP_CONTROL_CLK
    pr_info("Get the clk resource.\n");
    /* Enable spi clock */
    if (gfspi_ioctl_clk_init(gf_dev))
        goto gfspi_probe_clk_init_failed;

    if (gfspi_ioctl_clk_enable(gf_dev))
        goto gfspi_probe_clk_enable_failed;

    spi_clock_set(gf_dev, 1000000);
#endif

    gf_dev->notifier = goodix_noti_block;
    //fb_register_client(&gf_dev->notifier);
    msm_drm_register_client(&gf_dev->notifier);

    //wake_lock_init(&fp_wakelock, WAKE_LOCK_SUSPEND, "fp_wakelock");
    wakeup_source_init(&fp_wakelock, "fp_wakelock");

    spin_lock_init(&gf_dev->timer_lock);

    status = gf_power_off(gf_dev);

    status = regulator_disable(gf_dev->sdvcc);

    pr_info("version V%d.%d.%02d\n", VER_MAJOR, VER_MINOR, PATCH_LEVEL);

    return status;

#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
    gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif

error_input:
    if (gf_dev->input != NULL)
        input_free_device(gf_dev->input);
error_dev:
    if (gf_dev->devt != 0) {
        pr_info("Err: status = %d\n", status);
        mutex_lock(&device_list_lock);
        list_del(&gf_dev->device_entry);
        device_destroy(gf_class, gf_dev->devt);
        clear_bit(MINOR(gf_dev->devt), minors);
        mutex_unlock(&device_list_lock);
    }
error_hw:
    gf_dev->device_available = 0;

    return status;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
    struct gf_dev *gf_dev = &gf;

    //wake_lock_destroy(&fp_wakelock);
    wakeup_source_trash(&fp_wakelock);
    //fb_unregister_client(&gf_dev->notifier);
    msm_drm_unregister_client(&gf_dev->notifier);
    if (gf_dev->input)
        input_unregister_device(gf_dev->input);
    input_free_device(gf_dev->input);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    mutex_unlock(&device_list_lock);

    return 0;
}

static const struct of_device_id gx_match_table[] = {
    { .compatible = GF_SPIDEV_NAME },
    {},
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
    .driver = {
        .name = GF_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = gx_match_table,
    },
    .probe = gf_probe,
    .remove = gf_remove,
};

static int __init gf_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */

    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0) {
        pr_warn("Failed to register char device!\n");
        return status;
    }
    SPIDEV_MAJOR = status;
    gf_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_class)) {
        unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
        pr_warn("Failed to create class.\n");
        return PTR_ERR(gf_class);
    }
#if defined(USE_PLATFORM_BUS)
    status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
    status = spi_register_driver(&gf_driver);
#endif
    if (status < 0) {
        class_destroy(gf_class);
        unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
        pr_warn("Failed to register SPI driver.\n");
    }

#ifdef GF_NETLINK_ENABLE
    netlink_init();
#endif
    pr_info("status = 0x%x\n", status);
    return 0;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
#ifdef GF_NETLINK_ENABLE
    netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
    platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
    spi_unregister_driver(&gf_driver);
#endif
    class_destroy(gf_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
