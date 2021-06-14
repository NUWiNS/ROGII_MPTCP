/*
 * Goodix GTX5 Gesture Dirver
 *
 * Copyright (C) 2015 - 2016 Goodix, Inc.
 * Authors:  Wang Yafei <wangyafei@goodix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include "goodix_ts_core.h"
#include <asm/uaccess.h>
#include <linux/uaccess.h>

#define GSX_REG_GESTURE_DATA			0x4100
#define GSX_REG_GESTURE				0x6F68
#define GSX_VENOR_CFG_REG                       0x6D20

#define GSX_GESTURE_CMD				0x08
#define GSX_ROTATION_CMD                        0x0E
#define GSX_ROTATION_EXIT                       0x0F
#define GSX_CHARGE_MODE                         0x06
#define GSX_CHARGE_MODE_EXIT                    0x07
#define GSX_VENDOR_CFG_DOWNLOAD                 0x39 //download vendor config to FW

#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

#define GSX_KEY_DATA_LEN	37
#define GSX_GESTURE_TYPE_LEN	32
#define ZENMOTION_LEN           8

/*
 * struct gesture_module - gesture module data
 * @registered: module register state
 * @sysfs_node_created: sysfs node state
 * @gesture_type: store valied gesture type,each bit stand for a gesture
 * @gesture_data: gesture data
 * @gesture_ts_cmd: gesture command data
*/
struct gesture_module {
	atomic_t registered;
	atomic_t zen_motion;
	unsigned int kobj_initialized;
	rwlock_t rwlock;
	unsigned char gesture_type[GSX_GESTURE_TYPE_LEN];
	unsigned char gesture_data[GSX_KEY_DATA_LEN];
	struct goodix_ext_module module;
	struct goodix_ts_cmd cmd;
	//ASUS_BSP Beryl +++
	u8 zenmotion_type;
	atomic_t dclick;
	atomic_t swipeup;
	atomic_t aod_enable;
	int rotation;
	struct goodix_cfg_cmd cfg_cmd;
	struct mutex gsx_mutex;

};

static struct gesture_module *gsx_gesture; /*allocated in gesture init module*/

bool waitting_FOD = false;
bool r_allow_touch_reset = true; //rotation
bool c_allow_touch_reset = true; //charge mode
extern u8 game_cfg[13];
bool allow_report_zenmotion = true;
bool call_state = false;
bool setting_rotation = false;
int ingore_reset = 0;
EXPORT_SYMBOL(setting_rotation);
bool game_resume = false;
EXPORT_SYMBOL(game_resume);
extern bool allow_suspend;
extern bool proximity_check_status(void);
extern bool notify_touch_usbplug;
extern atomic_t g_notify_fod;
extern void TP_call_FOD(int id, bool down);
extern bool panel_on;

//static void gsx_keyL_delaywork(struct work_struct *work);
/**
 * gsx_gesture_type_show - show valid gesture type
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to output buffer
 * Returns >=0 - succeed,< 0 - failed
 */
static ssize_t gsx_gesture_type_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = 0, i, ret = 0;
	unsigned char *type;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}
	type = kzalloc(256, GFP_KERNEL);
	if (!type)
		return -ENOMEM;
	read_lock(&gsx_gesture->rwlock);
	for (i = 0; i < 256; i++) {
		if (QUERYBIT(gsx_gesture->gesture_type, i)) {
			type[count] = i;
			count++;
		}
	}
	type[count] = '\0';
	if (count > 0) {
		/* TODO 这里使用scnprintf需要确认一下是否有效 */
		ret = scnprintf(buf, PAGE_SIZE, "%s", type);
	}
	read_unlock(&gsx_gesture->rwlock);

	kfree(type);
	return ret;
}

/**
 * gsx_gesture_type_store - set vailed gesture
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to valid gesture type
 * @count: length of buf
 * Returns >0 - valid gestures, < 0 - failed
 */
static ssize_t gsx_gesture_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	int i;

	if (count <= 0 || count > 256 || buf == NULL) {
		ts_err("Parameter error");
		return -EINVAL;
	}

	write_lock(&gsx_gesture->rwlock);
	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	for (i = 0; i < count; i++)
		gsx_gesture->gesture_type[buf[i]/8] |= (0x1 << buf[i]%8);
	write_unlock(&gsx_gesture->rwlock);

	return count;
}

static ssize_t gsx_gesture_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->registered));
}

static ssize_t gsx_gesture_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int ret;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("Tmp value =%d", tmp);

	if (tmp == 1) {
		if (atomic_read(&gsx_gesture->registered)) {
			ts_debug("Gesture module has aready registered");
			return count;
		}
		ret = goodix_register_ext_module(&gsx_gesture->module);
		if (!ret) {
			ts_info("Gesture module registered!");
			atomic_set(&gsx_gesture->registered, 1);
			atomic_set(&gsx_gesture->zen_motion, 1);
		} else {
			atomic_set(&gsx_gesture->registered, 0);
			atomic_set(&gsx_gesture->zen_motion, 0);
			ts_err("Gesture module register failed");
		}
	} else if (tmp == 0) {
		if (!atomic_read(&gsx_gesture->registered)) {
			ts_debug("Gesture module has aready unregistered");
			return count;
		}
		ts_debug("Start unregistered gesture module");
		ret = goodix_unregister_ext_module(&gsx_gesture->module);
		if (!ret) {
			atomic_set(&gsx_gesture->registered, 0);
                        atomic_set(&gsx_gesture->zen_motion, 0);
			ts_info("Gesture module unregistered success");
		} else {
			atomic_set(&gsx_gesture->registered, 1);
                        atomic_set(&gsx_gesture->zen_motion, 1);
			ts_info("Gesture module unregistered failed");
		}
	} else {
		ts_err("Parameter error!");
		return -EINVAL;
	}
	return count;
}

//ASUS_BSP Beryl "ZenMotion" +++
static ssize_t gsx_zenmotion_enable_show(struct goodix_ext_module *module,
		char *buf)
{

        return scnprintf(buf, PAGE_SIZE, "%d\n", gsx_gesture->zenmotion_type);
}

static ssize_t gsx_zenmotion_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
        int tmp = 0, i = 0 , j = 0;
        u8 gesturetmp = 0;
        char gesture_buf[16] ,gesture_buf1[16];
        char cmpchar = '1';

        memset(gesture_buf, 0, sizeof(gesture_buf));
        memset(gesture_buf1, 0, sizeof(gesture_buf));
        gsx_gesture-> zenmotion_type = 0;
        sprintf(gesture_buf, "%s", buf);

	ts_info("gsx_zenmotion_enable_store %s ",gesture_buf);
	
        for (j = 0, i = count -1; i >= 0; j++, i--)
        {
            gesture_buf1[j] = gesture_buf[i];
         }
  
         if (gesture_buf1[0] == cmpchar) {
             atomic_set(&gsx_gesture->zen_motion, 1);
             ts_info("gesture_mode enable !");
        } else {
             atomic_set(&gsx_gesture->zen_motion, 0);
             ts_info("gesture_mode disable !");
        }
        
        if (atomic_read(&gsx_gesture->zen_motion) == 1) {
            for (tmp = 0; tmp < 7; tmp++) {
                if (gesture_buf1[tmp] == cmpchar) {
                    gesturetmp |= (1 << tmp);
                }
            }
            
           gsx_gesture-> zenmotion_type = gesturetmp;
	   ts_info("gesture_mode_enable type = %x !", gsx_gesture-> zenmotion_type);
	   
	} else {
	  gsx_gesture-> zenmotion_type = 0;
	  atomic_set(&gsx_gesture->zen_motion, 0);
	}
  
	return count;
}

static ssize_t gsx_dclick_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->dclick));
}

static ssize_t gsx_dclick_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
  	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("dclick =%d", tmp);

	if (tmp == 1) {
	  atomic_set(&gsx_gesture->dclick, 1);
	} else
	  atomic_set(&gsx_gesture->dclick, 0);
	return count;
  
}

static ssize_t gsx_swipeup_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->swipeup));
}

static ssize_t gsx_swipeup_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
  	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("swipeup =%d", tmp);

	if (tmp == 1) {
	  atomic_set(&gsx_gesture->swipeup, 1);
	} else
	  atomic_set(&gsx_gesture->swipeup, 0);
	return count;
  
}

static ssize_t gsx_aod_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->aod_enable));
}

static ssize_t gsx_aod_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
  	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("AOD enable =%d", tmp);

	if (tmp == 1) {
	  atomic_set(&gsx_gesture->aod_enable, 1);
	} else
	  atomic_set(&gsx_gesture->aod_enable, 0);
	return count;
  
}
//ASUS_BSP Beryl ---

/**
 * gsx_gesture_data_show - show gesture data read frome IC
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to output buffer
 * Returns >0 - gesture data length,< 0 - failed
 */
/*static ssize_t gsx_gesture_data_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = GSX_KEY_DATA_LEN;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}
	if (!buf || !gsx_gesture->gesture_data) {
		ts_info("Parameter error!");
		return -EPERM;
	}
	read_lock(&gsx_gesture->rwlock);
	memcpy(buf, gsx_gesture->gesture_data, count);
	read_unlock(&gsx_gesture->rwlock);

	return count;
}*/

static ssize_t gsx_gesture_data_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = GSX_KEY_DATA_LEN;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}
	if (!buf || (gsx_gesture->gesture_data == NULL)) {
		ts_info("Parameter error!");
		return -EPERM;
	}
	read_lock(&gsx_gesture->rwlock);

	count = scnprintf(buf, PAGE_SIZE, "Previous gesture type:0x%x\n",
			  gsx_gesture->gesture_data[2]);
	read_unlock(&gsx_gesture->rwlock);

	return count;
}

const struct goodix_ext_attribute gesture_attrs[] = {
	__EXTMOD_ATTR(type, 0666, gsx_gesture_type_show,
		gsx_gesture_type_store),
	__EXTMOD_ATTR(enable, 0666, gsx_gesture_enable_show,
		gsx_gesture_enable_store),
	__EXTMOD_ATTR(data, 0444, gsx_gesture_data_show, NULL),
	//ASUS_BSP Beryl +++ "ASUS feactures" +++
	__EXTMOD_ATTR(zenmotion, 0666, gsx_zenmotion_enable_show,
		gsx_zenmotion_enable_store),
	__EXTMOD_ATTR(dclick, 0666, gsx_dclick_enable_show,
		gsx_dclick_enable_store),
	__EXTMOD_ATTR(swipeup, 0666, gsx_swipeup_enable_show,
		gsx_swipeup_enable_store),
	__EXTMOD_ATTR(aod_enable, 0666, gsx_aod_enable_show,
		gsx_aod_enable_store)
};

static int gsx_gesture_init(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module)
{
	int i, ret;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	if (!core_data || !ts_dev->hw_ops->write || !ts_dev->hw_ops->read) {
		ts_err("Register gesture module failed, ts_core unsupported");
		goto exit_gesture_init;
	}

	gsx_gesture->cmd.cmd_reg = GSX_REG_GESTURE;
	gsx_gesture->cmd.length = 3;
	gsx_gesture->cmd.cmds[0] = GSX_GESTURE_CMD;
	gsx_gesture->cmd.cmds[1] = 0x0;
	gsx_gesture->cmd.cmds[2] = 0 - GSX_GESTURE_CMD;
	gsx_gesture->cmd.initialized = 1;

	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	memset(gsx_gesture->gesture_data, 0xff, GSX_KEY_DATA_LEN);
	
	//vendor cfg init
	gsx_gesture->cfg_cmd.cmd_reg = GSX_VENOR_CFG_REG;
	gsx_gesture->cfg_cmd.length = 12;
	memset(gsx_gesture->cfg_cmd.cmds, 0x00, sizeof(gsx_gesture->cfg_cmd.cmds));
        gsx_gesture->cfg_cmd.initialized = 1;
	
	ts_debug("Set gesture type manually");
	memset(gsx_gesture->gesture_type, 0xff, GSX_GESTURE_TYPE_LEN);
	/*gsx_gesture->gesture_type[34/8] |= (0x1 << 34%8);*/	/* 0x22 double click */
	/*gsx_gesture->gesture_type[170/8] |= (0x1 << 170%8);*/	/* 0xaa up swip */
	/*gsx_gesture->gesture_type[187/8] |= (0x1 << 187%8);*/	/* 0xbb right swip */
	/*gsx_gesture->gesture_type[171/8] |= (0x1 << 171%8);*/	/* 0xab down swip */
	/*gsx_gesture->gesture_type[186/8] |= (0x1 << 186%8);*/	/* 0xba left swip */

	if (gsx_gesture->kobj_initialized)
		goto exit_gesture_init;

	ret = kobject_init_and_add(&module->kobj, goodix_get_default_ktype(),
			&core_data->pdev->dev.kobj, "gesture");

	if (ret) {
		ts_err("Create gesture sysfs node error!");
		goto exit_gesture_init;
	}

	for (i = 0; i < sizeof(gesture_attrs)/sizeof(gesture_attrs[0]); i++) {
		if (sysfs_create_file(&module->kobj,
				&gesture_attrs[i].attr)) {
			ts_err("Create sysfs attr file error");
			kobject_put(&module->kobj);
			goto exit_gesture_init;
		}
	}
	gsx_gesture->kobj_initialized = 1;
	atomic_set(&gsx_gesture->dclick, 0);
	atomic_set(&gsx_gesture->swipeup, 0);
	atomic_set(&gsx_gesture->aod_enable, 0);
        mutex_init(&gsx_gesture->gsx_mutex);
//	INIT_DELAYED_WORK(&core_data->keyL_work, gsx_keyL_delaywork);
	proc_symlink("driver/dclick", NULL, "/sys/devices/platform/goodix_ts.0/gesture/dclick");
	proc_symlink("driver/swipeup", NULL, "/sys/devices/platform/goodix_ts.0/gesture/swipeup");
	proc_symlink("driver/gesture_type", NULL, "/sys/devices/platform/goodix_ts.0/gesture/zenmotion");
	
exit_gesture_init:
	return 0;
}
static int gsx_gesture_exit(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module)
{
	/*if (gsx_gesture->kobj_initialized)
		kobject_put(&module->kobj);
	gsx_gesture->kobj_initialized = 0;*/
	atomic_set(&gsx_gesture->registered, 0);
	atomic_set(&gsx_gesture->zen_motion, 0);
	atomic_set(&gsx_gesture->dclick, 0);
	atomic_set(&gsx_gesture->swipeup, 0);
	atomic_set(&gsx_gesture->aod_enable, 0);
	
	return 0;
}

static int report_gesture_key(struct input_dev *dev, char keycode)
{
	int r = 0;
	if (!allow_report_zenmotion)
	   r =2;

	if(call_state){
	  if (proximity_check_status()){
	    ts_info("in call state , p sensor enable, ignore any gesture event");
	    return 2;
	  }
	} 

	if(atomic_read(&gsx_gesture->aod_enable)==1) {
	    if(keycode == 'F') {	        
	        input_report_key(dev, KEY_F, 1);
		input_sync(dev);
		input_report_key(dev, KEY_F, 0);
		input_sync(dev);
		TP_call_FOD( 12, true);

		if (ingore_reset == 0) {
		    ingore_reset = 1;
		    ts_info("keycode F , set ingore_reset %d",ingore_reset);
		}
		r = 3;
	    }
	    
	    if(keycode == 'U') {	        
	        input_report_key(dev, KEY_U, 1);
		input_sync(dev);
		input_report_key(dev, KEY_U, 0);
		input_sync(dev);
	        TP_call_FOD( 12, false);
		r = 3;
	    }
	    
	    if(keycode == 'L') {	        
	        input_report_key(dev, KEY_L, 1);
		input_sync(dev);
		input_report_key(dev, KEY_L, 0);
		input_sync(dev);
		if (atomic_read(&gsx_gesture->dclick) == 1) {
		    if (ingore_reset == 0) {
		        ingore_reset = 1;
		        ts_info("double click enable, set ingore_reset %d",ingore_reset);
		    }		
		}
		r = 3;
	    }
	    if (r == 3)
	        return r;
	}
	 
	if (proximity_check_status()){
	  ts_info("P sensor enable, ignore gesture event");
	  return 2;
	}

        if (allow_report_zenmotion) {
	    switch (keycode) {
		case 'w': // w
		    if (gsx_gesture-> zenmotion_type & 1 << 1) {
			input_report_key(dev, KEY_W, 1);
			input_sync(dev);
			input_report_key(dev, KEY_W, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 's': // S
		    if(gsx_gesture-> zenmotion_type & 1 << 2) {
			input_report_key(dev, KEY_S, 1);
			input_sync(dev);
			input_report_key(dev, KEY_S, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 'e': // e
		    if(gsx_gesture-> zenmotion_type & 1 << 3){
			input_report_key(dev, KEY_E, 1);
			input_sync(dev);
			input_report_key(dev, KEY_E, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 'c': // c
		    if(gsx_gesture-> zenmotion_type & 1 << 4) {
			input_report_key(dev, KEY_C, 1);
			input_sync(dev);
			input_report_key(dev, KEY_C, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 'z': // Z
		    if(gsx_gesture-> zenmotion_type & 1 << 5){
			input_report_key(dev, KEY_Z, 1);
			input_sync(dev);
			input_report_key(dev, KEY_Z, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 'v': // V
		    if(gsx_gesture-> zenmotion_type & 1 << 6) {
			input_report_key(dev, KEY_V, 1);
			input_sync(dev);
			input_report_key(dev, KEY_V, 0);
			input_sync(dev);
			ts_info("report v");
			r = 1;
		    }
		    break;
		case 'L':
		    r = 3;
		    break;
		case 0xcc:
		    if (atomic_read(&gsx_gesture->dclick)==1) {
			input_report_key(dev, KEY_POWER, 1);
			input_sync(dev);
			input_report_key(dev, KEY_POWER, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		case 0xba:
		    if (atomic_read(&gsx_gesture->swipeup)==1) {
			input_report_key(dev, KEY_UP, 1);
			input_sync(dev);
			input_report_key(dev, KEY_UP, 0);
			input_sync(dev);
			r = 1;
		    }
		    break;
		default:
		    break;		  
	    }	  
	}
	
	return r;	
}

/*static void gsx_keyL_delaywork(struct work_struct *work){
  
  struct delayed_work *dwork = to_delayed_work(work);
  struct goodix_ts_core *core_data = container_of(dwork,
			struct goodix_ts_core, keyL_work);
  ts_info("report key L");
  input_report_key(core_data->input_dev, KEY_L, 1);
  input_sync(core_data->input_dev);
  input_report_key(core_data->input_dev, KEY_L, 0);
  input_sync(core_data->input_dev);
}*/

/**
 * gsx_gesture_ist - Gesture Irq handle
 * This functions is excuted when interrupt happended and
 * ic in doze mode.
 *
 * @core_data: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_CANCEL_IRQEVT  stop execute
 */
static int gsx_gesture_ist(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int r;
	unsigned char clear_reg = 0;
	int ret;
	unsigned char checksum = 0;	
	unsigned char temp_data[GSX_KEY_DATA_LEN];
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	
/*	if (panel_on) {
	    if ((temp_data[0] & GOODIX_GESTURE_EVENT) == GOODIX_GESTURE_EVENT){
		goto gesture_ist_exit;
	    }
	    
	    return EVT_CONTINUE;
	}
*/
	/* get ic gesture state*/
        mutex_lock(&gsx_gesture->gsx_mutex);
	ts_info("receive irq for gesture event");
	msleep(10);
	ret = ts_dev->hw_ops->read_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
				   temp_data, sizeof(temp_data));
	if (ret < 0 || ((temp_data[0] & GOODIX_GESTURE_EVENT)  == 0)) {
		ts_debug("Read gesture data faild, ret=%d, temp_data[0]=0x%x", ret, temp_data[0]);
		if (panel_on) {
		    mutex_unlock(&gsx_gesture->gsx_mutex);
		    return EVT_CONTINUE;
		} else {
		    ts_info("display not on, resend gesture command");
		    ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
		    ts_dev->hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
		    enable_irq_wake(core_data->irq);
		    mutex_unlock(&gsx_gesture->gsx_mutex);
		    return EVT_CANCEL_IRQEVT;
		}
	}

	checksum = checksum_u8(temp_data, sizeof(temp_data));
	if (checksum != 0) {
		ts_err("Gesture data checksum error:0x%x", checksum);
		ts_info("Gesture data %*ph", (int)sizeof(temp_data), temp_data);
		if(panel_on){
		  ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
	          mutex_unlock(&gsx_gesture->gsx_mutex);
		  return EVT_CANCEL_IRQEVT;;
		} else {
		    ts_info("display not on, resend gesture command");
		    ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
		    ts_dev->hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
		    enable_irq_wake(core_data->irq);
		    mutex_unlock(&gsx_gesture->gsx_mutex);
		    return EVT_CANCEL_IRQEVT;
		}
		
	}
        mutex_unlock(&gsx_gesture->gsx_mutex);
	ts_debug("Gesture data: data[0-4]0x%x, 0x%x, 0x%x, 0x%x", temp_data[0], temp_data[1],
		 temp_data[2], temp_data[3]);

	write_lock(&gsx_gesture->rwlock);
	memcpy(gsx_gesture->gesture_data, temp_data, sizeof(temp_data));
	write_unlock(&gsx_gesture->rwlock);

	if (core_data->station_insert){
	  ts_info("inserted into station , not allow zenmotion functions");
	  allow_report_zenmotion = false;
	} else
	  allow_report_zenmotion = true;
	
	if (core_data->phone_call_on)
	  call_state = true;
	else
	  call_state = false;
	
	if (QUERYBIT(gsx_gesture->gesture_type, temp_data[2])) {
/*		if (atomic_read(&gsx_gesture->dclick)==1 && temp_data[2]=='L'){
//		   ts_info("enable wakeup irq");
		   enable_irq_wake(core_data->irq);
//		   ts_info("clear_reg : GSX_REG_GESTURE_DATA");
		   ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
		   
		   if(atomic_read(&gsx_gesture->aod_enable)==1){
		      schedule_delayed_work(&core_data->keyL_work, 0.3 * HZ);
		   }
		   
		   return EVT_CANCEL_RESUME;
		}
*/		
		r = report_gesture_key(core_data->input_dev,temp_data[2]);
		if (r == 1)
		  goto gesture_ist_exit; // irq handled
		else if (r == 2){
		  goto gesture_ist_exit_without_wakeup;
		} else if (r == 3){
		  goto FOD_exit;
		}else {
		  ts_info("gesture:%x not enabled", temp_data[2]);
		  if (atomic_read(&core_data->suspended) == 1)
		      goto gesture_ist_exit_without_wakeup;
		  else
		      return EVT_CONTINUE;
		}
		
	} else {
		ts_info("Unsupported gesture:%x", temp_data[2]);
	}

gesture_ist_exit:
//        ts_info("clear_reg : GSX_REG_GESTURE_DATA, EVT_CANCEL_IRQEVT");
	ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
	return EVT_CANCEL_IRQEVT;
gesture_ist_exit_without_wakeup:
        ts_info("Do not wakeup system");
	mutex_lock(&gsx_gesture->gsx_mutex);
	ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
	ts_dev->hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
	enable_irq_wake(core_data->irq);
	mutex_unlock(&gsx_gesture->gsx_mutex);
        return EVT_CANCEL_RESUME; 
FOD_exit:
        ts_dev->hw_ops->write_trans(core_data->ts_dev, GSX_REG_GESTURE_DATA,
			      &clear_reg, 1);
	enable_irq_wake(core_data->irq);
        return EVT_CANCEL_RESUME;     
}

/**
 * gsx_gesture_before_suspend - execute gesture suspend routine
 * This functions is excuted to set ic into doze mode
 *
 * @core_data: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_IRQCANCLED  stop execute
 */
static int gsx_gesture_before_suspend(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int ret;
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;

	if (!gesture_cmd->initialized || hw_ops == NULL) {
		ts_err("Uninitialized doze command or hw_ops");
		return 0;
	}
	
	//ASUS_BSP Beryl "Zen motion"
	if ((atomic_read(&gsx_gesture->zen_motion) == 0) &&
	    (atomic_read(&gsx_gesture->dclick) == 0) &&
	    (atomic_read(&gsx_gesture->swipeup) == 0) &&
	    (atomic_read(&gsx_gesture->aod_enable) == 0)) {
	    ts_info("Zen motion not enable, going to deep sleep mode");
	    return 0;
	}
	
	if (!allow_suspend)
	  return EVT_CANCEL_SUSPEND;
	
	//going to gesture mode
	mutex_lock(&gsx_gesture->gsx_mutex);
	ts_info("Going to Gesture mode");
	gsx_gesture->cmd.length = 3;
	gsx_gesture->cmd.cmds[0] = GSX_GESTURE_CMD;
	gsx_gesture->cmd.cmds[1] = 0x00;
	gsx_gesture->cmd.cmds[2] = 0xF8;
	
	ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
	if (ret != 0) {
		ts_err("Send doze command error");
		mutex_unlock(&gsx_gesture->gsx_mutex);
		return 0;
	} else {
		ts_info("Set IC in doze mode");
		atomic_set(&core_data->suspended, 1);
		atomic_set(&core_data->dsi_suspend, 1);
		enable_irq_wake(core_data->irq); 
                mutex_unlock(&gsx_gesture->gsx_mutex);
		return EVT_CANCEL_SUSPEND;
	}
}

static int gsx_enter_rotation(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int ret = -1;
	
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	setting_rotation = true;
	gsx_gesture->rotation = core_data->rotation;
	
	if (gsx_gesture->rotation == 90) {
	  ts_info("Rotation to 90");
	  gsx_gesture->cmd.length = 3;
	  gsx_gesture->cmd.cmds[0] = GSX_ROTATION_CMD;
	  gsx_gesture->cmd.cmds[1] = 0x00;
	  gsx_gesture->cmd.cmds[2] = 0xF2;
	  r_allow_touch_reset = false;
	  
	} 
	
	if (gsx_gesture->rotation == 270) {
	  ts_info("Rotation to 270");
	  gsx_gesture->cmd.length = 3;
 	  gsx_gesture->cmd.cmds[0] = GSX_ROTATION_CMD;
	  gsx_gesture->cmd.cmds[1] = 0x01;
	  gsx_gesture->cmd.cmds[2] = 0xF1;
          r_allow_touch_reset = false;
	}
	
	if(gsx_gesture->rotation!=0)
	  ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
          
	if(ret!=0) {
	  setting_rotation = false;
	  return EVT_ROTATION_FAIL;
	}
	setting_rotation = false;
	return EVT_ROTATION_SUCCESS;
}

static int gsx_exit_rotation(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int ret = 0;
	
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	
	gsx_gesture->rotation = core_data->rotation;
	
	ts_info("Exit rotation mode");
	gsx_gesture->cmd.length = 3;
	gsx_gesture->cmd.cmds[0] = GSX_ROTATION_EXIT;
	gsx_gesture->cmd.cmds[1] = 0x00;
	gsx_gesture->cmd.cmds[2] = 0xF1;
        gsx_gesture->rotation = 0;
	r_allow_touch_reset= true;
	
	ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);

	if(ret!=0) {
	  return EVT_ROTATION_FAIL;
	}
	
	return EVT_ROTATION_SUCCESS;
}

static int gsx_charge_mode(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
  	int ret = 0;
	
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	
  	if (atomic_read(&core_data->charge_mode) == 1){
	    ts_info("Enter to charge mode");
	    if (core_data->station_insert) {
	      ts_info("inserted into station , not enter charge mode");
	      return EVT_CHANGE_MODE_CANCEL;
	    }

	    c_allow_touch_reset = false;
	    gsx_gesture->cmd.length = 3;
	    gsx_gesture->cmd.cmds[0] = GSX_CHARGE_MODE;
	    gsx_gesture->cmd.cmds[1] = 0x00;
	    gsx_gesture->cmd.cmds[2] = 0xFA;
	} 
	
	if (atomic_read(&core_data->charge_mode) == 0){
	    ts_info("Exit charge mode");
	    c_allow_touch_reset = true;
	    gsx_gesture->cmd.length = 3;
	    gsx_gesture->cmd.cmds[0] = GSX_CHARGE_MODE_EXIT;
	    gsx_gesture->cmd.cmds[1] = 0x00;
	    gsx_gesture->cmd.cmds[2] = 0xF9;
	}
	
	ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
	
	if(ret!=0) {
	  return EVT_CHANGE_MODE_FAIL;
	}
	
	return EVT_CHANGE_MODE_SUCCESS;
}

static int gsx_game_cfg(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
        int ret = 0,r = 0, i = 0;  
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	struct goodix_cfg_cmd *vendor_cmd = &gsx_gesture->cfg_cmd;
	
	gsx_gesture->cfg_cmd.cmd_reg = GSX_VENOR_CFG_REG;
	gsx_gesture->cfg_cmd.length = 13;
	for(i = 0 ; i < 13;i++) {
	    gsx_gesture->cfg_cmd.cmds[i] = game_cfg[i];
	}
	//set game config
	ret = hw_ops->send_vcfg_cmd(core_data->ts_dev, vendor_cmd);
	if (ret!=0) {
	    ts_err("send vendor_cmd fail");
	    r = -1;
	}
	//download vendor config to FW  
	gsx_gesture->cmd.length = 3;
	gsx_gesture->cmd.cmds[0] = GSX_VENDOR_CFG_DOWNLOAD;
	gsx_gesture->cmd.cmds[1] = 0x00;
	gsx_gesture->cmd.cmds[2] = 0xC7;
	ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
	if (ret!=0) {
	    ts_err("send vendor_cmd fail");
	    r = -1;
	}
	
	if(r == -1) {
	  return EVT_GAME_CFG_FAIL;
	}
	ts_info("Load game cfg success");
	return EVT_GAME_CFG_SUCCESS;
}


static int gsx_resume_vendor_state(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
  	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;
	int ret = 0;
	int r = 0; 
	//resume charge mode if usb cable plugged
	if (notify_touch_usbplug){

        if(core_data->station_insert) {
			ts_info("insert into station , not enter charge mode");
        } else {
	    ts_info("Usb is pluged , resume to charge mode");
	    gsx_gesture->cmd.length = 3;
	    gsx_gesture->cmd.cmds[0] = GSX_CHARGE_MODE;
	    gsx_gesture->cmd.cmds[1] = 0x00;
	    gsx_gesture->cmd.cmds[2] = 0xFA;
	    ret = hw_ops->send_cmd(core_data->ts_dev, gesture_cmd);
	    if(ret!=0) { 
	      ts_err("Resume to charge mode failed");
	      r = -1;
	    }
	    atomic_set(&core_data->charge_mode, 1);
        }
	}
	
	if (atomic_read(&core_data->charge_mode) == 1 && !notify_touch_usbplug){
	    // set charge mode to 0 because usb unpluged when suspend
	    ts_info("Usb is unpluged , reset charge mode state to 0");
	    atomic_set(&core_data->charge_mode, 0);
	}	 
	if(core_data->game_mode)
	  game_resume = true;
    return r;
}

static struct goodix_ext_module_funcs gsx_gesture_funcs = {
	.irq_event = gsx_gesture_ist,
	.init = gsx_gesture_init,
	.exit = gsx_gesture_exit,
	.before_suspend = gsx_gesture_before_suspend,
	.enter_rotation = gsx_enter_rotation,
	.exit_rotation = gsx_exit_rotation,
	.charge_mode = gsx_charge_mode,
	.game_cfg = gsx_game_cfg,
	.after_resume = gsx_resume_vendor_state
};

static int __init goodix_gsx_gesture_init(void)
{
	/* initialize core_data->ts_dev->gesture_cmd*/
	int result;
	ts_info("gesture module init");
	gsx_gesture = kzalloc(sizeof(struct gesture_module), GFP_KERNEL);
	if (!gsx_gesture)
		result = -ENOMEM;
	gsx_gesture->module.funcs = &gsx_gesture_funcs;
	gsx_gesture->module.priority = EXTMOD_PRIO_GESTURE;
	gsx_gesture->module.name = "Goodix_gsx_gesture";
	gsx_gesture->module.priv_data = gsx_gesture;
	gsx_gesture->kobj_initialized = 0;
	atomic_set(&gsx_gesture->registered, 0);
	rwlock_init(&gsx_gesture->rwlock);
	result = goodix_register_ext_module(&(gsx_gesture->module));
	if (result == 0){
		atomic_set(&gsx_gesture->registered, 1);
		atomic_set(&gsx_gesture->zen_motion, 0); 
		//register gesture module but not enable zen_motion
	}

	return result;
}

static void __exit goodix_gsx_gesture_exit(void)
{
	ts_info("gesture module exit");
	if (gsx_gesture->kobj_initialized)
		kobject_put(&gsx_gesture->module.kobj);
	kfree(gsx_gesture);
	return;
}

module_init(goodix_gsx_gesture_init);
module_exit(goodix_gsx_gesture_exit);

MODULE_DESCRIPTION("Goodix gsx Touchscreen Gesture Module");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
