 /*
  * Goodix Touchscreen Driver
  * Core layer of touchdriver architecture.
  *
  * Copyright (C) 2015 - 2016 Goodix, Inc.
  * Authors:  Yulong Cai <caiyulong@goodix.com>
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
  *
  */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "goodix_ts_core.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif

#define GOOIDX_INPUT_PHYS	"goodix_ts/input0"
#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"
#define PROCNAME "driver"

extern int goodix_start_cfg_bin(struct goodix_ts_core *ts_core);

extern atomic_t g_notify_fod;
extern volatile int g_id_fod;
static u32 fod_press = 0x0000;
extern void TP_call_FOD(int id, bool down);
extern bool panel_on;
extern int ingore_reset;

//ASUS_BSP Beryl +++
struct goodix_ts_core *gts_core_data = NULL;
int init_success = 0;
//AirTrigger 
#define TOTAL_SLOT 10

bool finger_press = false;
int touch_figer_slot[TOTAL_SLOT] = {0};
static int LastATR = 0;
static int LastATL = 0;
static int id_pressed[10] = {0};
extern bool allow_hw_reset;
extern bool g_enter_AOD;
bool allow_suspend = true;
EXPORT_SYMBOL(allow_suspend);

u8 game_cfg[13]={0x0};
EXPORT_SYMBOL(game_cfg);
extern uint8_t gDongleType;
extern bool Reseting;
extern bool setting_rotation;
extern bool game_resume;
extern int station_cfg_version;
extern bool notify_touch_usbplug;
static void goodix_suspend_work(struct work_struct *work);
static void goodix_resume_work(struct work_struct *work);
//ASUS_BSP Beryl ---
struct goodix_module goodix_modules;

/**
 * __do_register_ext_module - register external module
 * to register into touch core modules structure
 */
static void  __do_register_ext_module(struct work_struct *work)
{
	struct goodix_ext_module *module =
			container_of(work, struct goodix_ext_module, work);
	struct goodix_ext_module *ext_module;
	struct list_head *insert_point = &goodix_modules.head;

	ts_info("__do_register_ext_module IN, goodix_modules.core_exit:%d", goodix_modules.core_exit);
        ts_info("register external module %s",module->name);
	
	/* waitting for core layer */
	if (!wait_for_completion_timeout(&goodix_modules.core_comp, 60 * HZ)) {
		ts_err("Module [%s] timeout", module->name);
		return;
	}

	/* driver probe failed */
	if (goodix_modules.core_exit) {
		ts_err("Can't register ext_module, core exit");
		return;
	}

	ts_info("start register ext_module");

	/* prority level *must* be set */
	if (module->priority == EXTMOD_PRIO_RESERVED) {
		ts_err("Priority of module [%s] needs to be set",
				module->name);
		return;
	}

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (ext_module == module) {
				ts_info("Module [%s] already exists",
						module->name);
				mutex_unlock(&goodix_modules.mutex);
				return;
			}
		}

		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			/* small value of priority have
			 * higher priority level*/
			if (ext_module->priority >= module->priority) {
				insert_point = &ext_module->list;
				break;
			}
		} /* else module will be inserted
		 to goodix_modules->head */
	}

	if (module->funcs && module->funcs->init) {
		if (module->funcs->init(goodix_modules.core_data,
					module) < 0) {
			ts_err("Module [%s] init error",
					module->name ? module->name : " ");
			mutex_unlock(&goodix_modules.mutex);
			return;
		}
	}

	list_add(&module->list, insert_point->prev);
	goodix_modules.count++;
	mutex_unlock(&goodix_modules.mutex);

	ts_info("Module [%s] registered,priority:%u",
			module->name,
			module->priority);
	return;
}

/**
 * goodix_register_ext_module - interface for external module
 * to register into touch core modules structure
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int goodix_register_ext_module(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;
        
	if (!goodix_modules.initilized) {
	        ts_info("goodix_modules.initilized is false");
		goodix_modules.initilized = true;
		goodix_modules.core_exit = true;
		INIT_LIST_HEAD(&goodix_modules.head);
		mutex_init(&goodix_modules.mutex);
		init_completion(&goodix_modules.core_comp);
	}

/*	if (goodix_modules.core_exit) {
		ts_err("Can't register ext_module, core exit");
		return -EFAULT;
	}
*/
	ts_info("goodix_register_ext_module IN");

	INIT_WORK(&module->work, __do_register_ext_module);
	schedule_work(&module->work);

	ts_info("goodix_register_ext_module OUT");


	return 0;
}
EXPORT_SYMBOL_GPL(goodix_register_ext_module);

/**
 * goodix_unregister_ext_module - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int goodix_unregister_ext_module(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module;
	bool found = false;

	if (!module)
		return -EINVAL;

	if (!goodix_modules.initilized)
		return -EINVAL;

	if (!goodix_modules.core_data)
		return -ENODEV;

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (ext_module == module) {
				found = true;
				break;
			}
		}
	} else {
		mutex_unlock(&goodix_modules.mutex);
		return -EFAULT;
	}

	if (!found) {
		ts_err("Module [%s] never registed",
				module->name);
		mutex_unlock(&goodix_modules.mutex);
		return -EFAULT;
	}

	list_del(&module->list);
	mutex_unlock(&goodix_modules.mutex);

	if (module->funcs && module->funcs->exit)
		module->funcs->exit(goodix_modules.core_data, module);
	goodix_modules.count--;

	ts_info("Moudle [%s] unregistered",
			module->name ? module->name : " ");
	return 0;
}
EXPORT_SYMBOL_GPL(goodix_unregister_ext_module);

static void goodix_ext_sysfs_release(struct kobject *kobj)
{
	ts_info("Kobject released!");
}

#define to_ext_module(kobj)	container_of(kobj,\
				struct goodix_ext_module, kobj)
#define to_ext_attr(attr)	container_of(attr,\
				struct goodix_ext_attribute, attr)

static ssize_t goodix_ext_sysfs_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->show)
		return ext_attr->show(module, buf);

	return -EIO;
}

static ssize_t goodix_ext_sysfs_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->store)
		return ext_attr->store(module, buf, count);

	return -EIO;
}

static const struct sysfs_ops goodix_ext_ops = {
	.show = goodix_ext_sysfs_show,
	.store = goodix_ext_sysfs_store
};

static struct kobj_type goodix_ext_ktype = {
	.release = goodix_ext_sysfs_release,
	.sysfs_ops = &goodix_ext_ops,
};

struct kobj_type *goodix_get_default_ktype(void)
{
	return &goodix_ext_ktype;
}
EXPORT_SYMBOL_GPL(goodix_get_default_ktype);

struct kobject *goodix_get_default_kobj(void)
{
	struct kobject *kobj = NULL;

	if (goodix_modules.core_data &&
			goodix_modules.core_data->pdev)
		kobj = &goodix_modules.core_data->pdev->dev.kobj;
	return kobj;
}
EXPORT_SYMBOL_GPL(goodix_get_default_kobj);

/* debug fs */
struct debugfs_buf {
	struct debugfs_blob_wrapper buf;
	int pos;
	struct dentry *dentry;
} goodix_dbg;

void goodix_msg_printf(const char *fmt, ...)
{
	va_list args;
	int r;

	if (goodix_dbg.pos < goodix_dbg.buf.size) {
		va_start(args, fmt);
		r = vscnprintf(goodix_dbg.buf.data + goodix_dbg.pos,
			 goodix_dbg.buf.size - 1, fmt, args);
		goodix_dbg.pos += r;
		va_end(args);
	}
}
EXPORT_SYMBOL_GPL(goodix_msg_printf);

static int goodix_debugfs_init(void)
{
	struct dentry *r_b;
	goodix_dbg.buf.size = PAGE_SIZE;
	goodix_dbg.pos = 0;
	goodix_dbg.buf.data = kzalloc(goodix_dbg.buf.size, GFP_KERNEL);
	if (goodix_dbg.buf.data == NULL) {
		pr_err("Debugfs init failed\n");
		goto exit;
	}
	r_b = debugfs_create_blob("goodix_ts", 0644, NULL, &goodix_dbg.buf);
	if (!r_b) {
		pr_err("Debugfs create failed\n");
		return -ENOENT;
	}
	goodix_dbg.dentry = r_b;

exit:
	return 0;
}

static void goodix_debugfs_exit(void)
{
	debugfs_remove(goodix_dbg.dentry);
	goodix_dbg.dentry = NULL;
	pr_info("Debugfs module exit\n");
}

/* show external module infomation */
static ssize_t goodix_ts_extmod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ext_module *module;
	size_t offset = 0;
	int r;

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(module, &goodix_modules.head, list) {
			r = snprintf(&buf[offset], PAGE_SIZE,
					"priority:%u module:%s\n",
					module->priority, module->name);
			if (r < 0) {
				mutex_unlock(&goodix_modules.mutex);
				return -EINVAL;
			}
			offset += r;
		}
	}

	mutex_unlock(&goodix_modules.mutex);
	return offset;
}

/* show driver infomation */
static ssize_t goodix_ts_driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t goodix_ts_chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_version chip_ver;
	int r, cnt = 0;
	if (init_success!=1)
	  return 0;
	
        if(core_data==NULL || ts_dev==NULL)
	  return cnt;
	
	cnt += snprintf(buf, PAGE_SIZE,
			"TouchDeviceName:%s\n", ts_dev->name);
	if (ts_dev->hw_ops->read_version) {
		r = ts_dev->hw_ops->read_version(ts_dev, &chip_ver);
		if (!r && chip_ver.valid) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
					"PID:%s\nVID:%02x.%02x.%02x.%02x\nSensorID:%02x\n",
					chip_ver.pid, chip_ver.vid[0],
					chip_ver.vid[1], chip_ver.vid[2],
					chip_ver.vid[3], chip_ver.sensor_id);
		}
	}

	return cnt;
}

/* show chip configuration data */
static ssize_t goodix_ts_config_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_config *ncfg = ts_dev->normal_cfg;
	u8 *data;
	int i, r, offset = 0;

	if (ncfg && ncfg->initialized && ncfg->length < PAGE_SIZE) {
		data = kmalloc(ncfg->length, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		r = ts_dev->hw_ops->read(ts_dev, ncfg->reg_base,
				&data[0], ncfg->length);
		if (r < 0) {
			kfree(data);
			return -EINVAL;
		}

		for (i = 0; i < ncfg->length; i++) {
			if (i != 0 && i % 20 == 0)
				buf[offset++] = '\n';
			offset += snprintf(&buf[offset], PAGE_SIZE - offset,
					"%02x ", data[i]);
		}
		buf[offset++] = '\n';
		buf[offset++] = '\0';
		kfree(data);
		return offset;
	}

	return -EINVAL;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	if (ts_dev->hw_ops->reset)
		ts_dev->hw_ops->reset(ts_dev);
	return count;

}

static ssize_t goodix_ts_read_cfg_show(struct device *dev,
				struct device_attribute *attr,
						char *buf)
{
	struct goodix_ts_core *core_data =
				dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int ret, i, offset;
	char *cfg_buf;
        if(init_success!=1)
	  return -EINVAL;
	
        if(core_data==NULL || ts_dev==NULL)
	  return 0;
	
	cfg_buf = kzalloc(4096, GFP_KERNEL);
	disable_irq(core_data->irq);
	if (ts_dev->hw_ops->read_config)
		ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf, 0);
	else
		ret = -EINVAL;
	enable_irq(core_data->irq);

	offset = 0;
	if (ret > 0) {
		for (i = 0; i < ret; i++) {
			if (i != 0 && i % 20 == 0)
				buf[offset++] = '\n';
			offset += snprintf(&buf[offset], 4096 - offset,
					"%02x ", cfg_buf[i]);
		}

	}
	kfree(cfg_buf);
	return ret;
}

static int goodix_ts_convert_0x_data(const u8 *buf,
				int buf_size,
				unsigned char *out_buf,
				int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}
	ts_info("***m_size:%d", m_size);

	if (m_size <= 1) {
		ts_err("cfg file ERROR, valid data count:%d\n", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X') {
			if (temp_index >= m_size) {
				ts_err("exchange cfg data error, overflow, temp_index:%d,m_size:%d\n",
						temp_index, m_size);
				return -EINVAL;
			}
			if (buf[i + 1] >= '0' && buf[i + 1] <= '9')
				out_buf[temp_index] = (buf[i + 1] - '0') << 4;
			else if (buf[i + 1] >= 'a' && buf[i + 1] <= 'f')
				out_buf[temp_index] = (buf[i + 1] - 'a' + 10) << 4;
			else if (buf[i + 1] >= 'A' && buf[i + 1] <= 'F')
				out_buf[temp_index] = (buf[i + 1] - 'A' + 10) << 4;

			if (buf[i + 2] >= '0' && buf[i + 2] <= '9')
				out_buf[temp_index] += (buf[i + 2] - '0');
			else if (buf[i + 2] >= 'a' && buf[i + 2] <= 'f')
				out_buf[temp_index] += (buf[i + 2] - 'a' + 10);
			else if (buf[i + 2] >= 'A' && buf[i + 2] <= 'F')
				out_buf[temp_index] += (buf[i + 2] - 'A' + 10);

			temp_index++;
		}
	}
	return 0;
}



static ssize_t goodix_ts_send_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct goodix_ts_core *core_data =
				dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en, r;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;

	ts_info("******IN");

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	ts_info("en:%d", en);

	disable_irq(core_data->irq);

	/*request configuration*/
	r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	} else
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);

	config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	}

	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}

	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	if (ts_dev->hw_ops->send_config)
		ts_dev->hw_ops->send_config(ts_dev, config);

exit:
	enable_irq(core_data->irq);

	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}

	ts_info("******OUT");
	return count;
}

/* show irq infomation */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n",
			core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		atomic_read(&core_data->irq_enabled) ?
		"enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
			"echo 0/1 > irq_info to disable/enable irq");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable(core_data, en);
	return count;
}
//ASUS_BSP "ASUS feactures" +++
// power on/off
static ssize_t goodix_ts_power_on_off(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
		
	int en, result;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable(core_data, en);
	
	if (core_data->power_on == 1 && en == 0) {
		result = goodix_ts_power_off(core_data);
	}
	if (core_data->power_on == 0 && en == 1)
		result = goodix_ts_power_on(core_data);
	ts_info("power operation status %d",result);
	
	return count;

}

static ssize_t goodix_ts_power_state(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n",core_data->power_on);
}

static ssize_t goodix_ts_rotation_type_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ext_module *ext_module;
	int r;
	char rotation_type[4];
	core_data->rotation = 0;

	memset(rotation_type, 0, sizeof(rotation_type));
	sprintf(rotation_type, "%s", buf);
	rotation_type[count-1] = '\0';
	
	switch(rotation_type[0]){
	      case '9':
		core_data->rotation = 90;
		break;
	      case '2':
		core_data->rotation = 270;
		break;
	      default:
		core_data->rotation = 0;
		break;	      
	}

	ts_info("rotation_type %d game_mode %d",core_data->rotation,core_data->game_mode);
	/* inform external module */
	if (core_data->rotation == 90 || core_data->rotation == 270) {
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->enter_rotation)
				continue;
                        if(!core_data->game_mode)
			        continue; // do not rotation if not in game mode
			
			r = ext_module->funcs->enter_rotation(core_data, ext_module);
			if (r == EVT_ROTATION_FAIL) {
			    ts_info("Rotation fail");
			    atomic_set(&core_data->rotation_set, 0);
			}
			
			if(r == EVT_ROTATION_SUCCESS){
			  ts_info("Set Rotation success");
			  atomic_set(&core_data->rotation_set, 1);			  
			}
		}
	}	
	mutex_unlock(&goodix_modules.mutex);
	}
		

	if (core_data->rotation == 0 && atomic_read(&core_data->rotation_set)==1) {
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->exit_rotation)
				continue;
                        if(!core_data->game_mode)
			        continue; // do not rotation if not in game mode
			
			r = ext_module->funcs->exit_rotation(core_data, ext_module);
			if (r == EVT_ROTATION_FAIL) {
			    ts_info("Exit Rotation fail");
			}
			
			if(r == EVT_ROTATION_SUCCESS){
			  ts_info("Exit Rotation success");
			  atomic_set(&core_data->rotation_set, 0);			  
			}
		}
	}	
	mutex_unlock(&goodix_modules.mutex);
	}
	
		
	return count;
}

static ssize_t goodix_ts_rotation_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n",core_data->rotation);
}

static ssize_t goodix_ts_debug_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en == 1) {
	   core_data->debug = true;
	   ts_info("debug mode enable");
	}
	else {
	   core_data->debug = false;
	   ts_info("debug mode disable");
	}
	return count;

}

static ssize_t airtrigger_touch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	bool stat = 0;
    struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (core_data->atr_enable)
		stat = true;
	else
		stat = false;

	return sprintf(buf, "%d", stat);
}


static ssize_t keymapping_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
    int id, action, x, y, random, minus;
    ts_info("keymapping cmd buf: %s len=%d\n", buf, count);


	if (count != 14) {
	   ts_info("Invalid cmd buffer");
	   return -EINVAL;
	}

    
    id = buf[0] - '0';
    action = buf[1] - '0';
    random = buf[2] - '0';
    
    minus = buf[3];
    x =  shex_to_u16(buf + 4, 4);
    if(minus == '-')
        x = -x;
    
    minus = buf[8];
    y =  shex_to_u16(buf + 9, 4);
    if(minus == '-')
        y = -y;

    ATR_touch_new(dev,  id, action,  x,  y, random);

	return count;
}
static ssize_t airtrigger_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
    int action, x, y;
    ts_info("keymapping airtrigger cmd buf: %s len=%d\n", buf, count);

    if(count != 19)
        return -EINVAL;
    if(buf[0] == 'F')
    {
        struct goodix_ts_core *core_data = dev_get_drvdata(dev);
        int i;
        
        for(i = 0; i < TOTAL_SLOT; i++)
        {
            if(touch_figer_slot[i])
            {
                input_mt_slot(core_data->input_dev, i + 10);
                input_mt_report_slot_state(core_data->input_dev, MT_TOOL_FINGER, false);
                input_sync(core_data->input_dev);
                touch_figer_slot[i] = 0;
                LastATR = LastATL = 0;
            }
        }
        core_data->atr_enable = false;
        input_report_key(core_data->input_dev, BTN_TOUCH, 0);
        input_sync(core_data->input_dev);
        ts_info("keymapping all buttons up\n");
        return count;
    }

    action = buf[1] - '0'; // handle R
    if(LastATR != action)
    {
        x =  shex_to_u16(buf + 10, 4);
        y =  shex_to_u16(buf + 14, 4);
        ATR_touch_new(dev, 11, action,  x,  y, 0);
        ts_info("keymapping airtrigger R %d %d, %d\n", action, x, y, 0);
        LastATR = action;
    }
    action = buf[0] - '0'; // handle L
    if(LastATL != action)
    {
        x =  shex_to_u16(buf + 2, 4);
        y =  shex_to_u16(buf + 6, 4);
        ATR_touch_new(dev, 12, action,  x,  y, 0);
        ts_info("keymapping airtrigger L %d %d, %d\n", action, x, y, 0);
        LastATL = action;
    }

	return count;
}

static ssize_t glove_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	
	if(core_data == NULL)
	  return snprintf(buf, PAGE_SIZE, "error\n");

        return snprintf(buf, PAGE_SIZE, "%d\n",core_data->glove_mode);
}

static ssize_t glove_mode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
       	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;
	struct goodix_ext_module *ext_module;
	int en, r;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("glove mode enable %d",en);
	
	if (!allow_hw_reset){
	  ts_info("FW is updating , not allow reset");
	  return -EINVAL;
	}  

	if(en == 0 && atomic_read(&core_data->glove_mode)!=1){
	  ts_info("previous status not in glove mode");
	  return count;
	}
	
	disable_irq(core_data->irq);
        if(en == 1) {
	    /*request configuration*/
	    r = request_firmware(&cfg_img, GOODIX_GLOVE_CFG_NAME, dev);
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_GLOVE_CFG_NAME, r);
		goto exit;
	    } else
		ts_info("cfg file [%s] is ready", GOODIX_GLOVE_CFG_NAME);

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }	    
	    atomic_set(&core_data->glove_mode, 1);
	}
	
	if(en == 0 && atomic_read(&core_data->glove_mode)==1){ // reset touch cfg 

	    if(gts_core_data->station_insert){
	      /*request configuration to station config*/
	      ts_info("station is inserted, redownload to station config");
	      r = request_firmware(&cfg_img, GOODIX_STATION_CFG_NAME, dev);
	    } else {
	      /*request configuration to phone config*/
	      r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	    }
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	    } else {
	        if(gts_core_data->station_insert){
		  ts_info("cfg file [%s] is ready", GOODIX_STATION_CFG_NAME);
		  atomic_set(&core_data->station_cfg_reload, 1);
		}else
		  ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);
	    }

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }	    
	    atomic_set(&core_data->glove_mode, 0);
	}	
	
	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}
	
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	if (ts_dev->hw_ops->send_config)
	  r = ts_dev->hw_ops->send_config(ts_dev, config);
	ts_info("send cfg result %d", r);
exit:
	enable_irq(core_data->irq);

	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}
        ts_dev->hw_ops->reset(ts_dev);
	ts_info("******OUT");
        //restore charge mode
	if (notify_touch_usbplug) {
	    ts_info("USB is plugged, restore charge mode");
	    
	    if(gts_core_data->station_insert){
	      ts_info("station is inserted , do not enter charge mode");
	      return count;
	    }
	    mutex_lock(&goodix_modules.mutex);
		
	    if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
		    if (!ext_module->funcs->charge_mode)
			continue;
		        r = ext_module->funcs->charge_mode(gts_core_data, ext_module);
			if (r == EVT_CHANGE_MODE_CANCEL || r == EVT_CHANGE_MODE_FAIL){
			    atomic_set(&gts_core_data->charge_mode, 0);
			}			  
		    }
	        }
	    mutex_unlock(&goodix_modules.mutex);	
	}
	return count;	
}

static ssize_t game_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	
	if(core_data == NULL)
	  return snprintf(buf, PAGE_SIZE, "error\n");

        return snprintf(buf, PAGE_SIZE, "%d\n",core_data->game_mode);
}

static ssize_t game_mode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
       	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ext_module *ext_module;
	
	char game_mode_type[2];
	char game_enable = '1';
	char game_exit = '0';
        bool hw_resetted = false;
	int r =0, i=0;
	bool need_reset = false;
	
	memset(game_mode_type, 0, sizeof(game_mode_type));
	sprintf(game_mode_type, "%s", buf);
	game_mode_type[count-1] = '\0';
	ts_info("game_mode_type %s ",game_mode_type);
	
//-------------------------game enter -----------------------------------------------------------------------	
	if (game_mode_type[0] == game_enable) {
	   core_data->game_mode = true;
	   ts_info("game mode enable rotation %d", core_data->rotation);
	  
	  if(core_data->rotation == 90 || core_data->rotation== 270) {
	  /* inform external module */
	  mutex_lock(&goodix_modules.mutex);
	  if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->enter_rotation)
				continue;

			r = ext_module->funcs->enter_rotation(core_data, ext_module);
			if (r == EVT_ROTATION_FAIL) {
			    ts_info("Rotation fail");
			    atomic_set(&core_data->rotation_set, 0);
			}
			if(r == EVT_ROTATION_SUCCESS){
			  ts_info("Set Rotation success");
			  atomic_set(&core_data->rotation_set, 1);
			}
		}
	  }	
	  mutex_unlock(&goodix_modules.mutex);
	  }
	}

//-------------------------game exit -----------------------------------------------------------------------
	if (game_mode_type[0] == game_exit) {
	   //normal mode
	  ts_info("game mode exit");
	  if(finger_press){
	    ts_info("release all fingers");
	    input_report_key(core_data->input_dev, BTN_TOUCH, 0);
	    for(i = 0;i<=9;i++){
	      input_mt_slot(core_data->input_dev, i);
	      input_mt_report_slot_state(core_data->input_dev, MT_TOOL_FINGER, false);
	    }
	     input_sync(core_data->input_dev);
	     finger_press=false;
	  }
	  if (core_data->atr_enable) {
	      //release air trigger key when exit game mode
	      ts_info("release air trigger key when exit game mode");
	      input_report_key(core_data->input_dev, BTN_TOUCH, 0);
	      for (i = 0 ;i <=9 ; i ++) {
	          input_mt_slot(core_data->input_dev, i+10);
	          input_mt_report_slot_state(core_data->input_dev, MT_TOOL_FINGER, false);
	      }
	      input_sync(core_data->input_dev);
	      core_data->atr_enable = false;
	   }
	   core_data->game_mode = false;
	   game_resume = false;
//---------------------------------------------------------------------------------------	   
	  if(atomic_read(&core_data->rotation_set)==1) {
	     ts_info("clear game config , exit rotation mode");
	     mutex_lock(&goodix_modules.mutex);
	     if (!list_empty(&goodix_modules.head)) {
		    list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->exit_rotation)
				continue;

			r = ext_module->funcs->exit_rotation(core_data, ext_module);
			if (r == EVT_ROTATION_FAIL) {
			    ts_info("send exit rotation fail, reset touch hw");
			    need_reset = true;
			    atomic_set(&core_data->rotation_set, 0);
			}
			
			if(r == EVT_ROTATION_SUCCESS)
			  atomic_set(&core_data->rotation_set, 0);
		    }
	      }
	      mutex_unlock(&goodix_modules.mutex);	     
	  }

	   if(atomic_read(&core_data->game_cfg_set)==1) {
	     ts_info("clear game config , reset touch ic");
	     atomic_set(&core_data->game_cfg_set, 0);
	     memset(game_cfg, 0x0, sizeof(game_cfg));
	     need_reset = true;
	   }

	   if(atomic_read(&gts_core_data->dsi_suspend)==1){
	     ts_info("Touch already suspended, ignore reset ");
	     need_reset =false;
	   }
           if (ts_dev->hw_ops->reset && need_reset){
		ts_dev->hw_ops->reset(ts_dev);
		hw_resetted = true;
	     }  
	  
	  if (hw_resetted) {
	      ts_info("HW resetted , resume previous mode");
	      mutex_lock(&goodix_modules.mutex);
	      if (!list_empty(&goodix_modules.head)) {
		    list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
			}
		    }
	      }
	      mutex_unlock(&goodix_modules.mutex);
	  }
	}
	return count;	
}

static ssize_t game_settings_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
  
        struct goodix_ts_core *core_data = dev_get_drvdata(dev);
        struct goodix_ext_module *ext_module;
        char game_settings[29];
	
	u16 touch_level = 0x0,leave_level = 0x0, first_filter = 0x0, normal_filter = 0x0;
	u8 Rcoef = 0x0, RcoefRight=0x0, touch_timer = 0x0;
	u8 checksum = 0x0;
	int r = 0;
	bool need_rotation = false;
	
	memset(game_settings, 0, sizeof(game_settings));
	sprintf(game_settings, "%s", buf);
	game_settings[count-1] = '\0';
	ts_info("game_settings %s count %d ",game_settings,count);
	
	if(count != 28){
            return -EINVAL;
	}
	
	touch_level = (u16)shex_to_u16(game_settings +0, 3);
	leave_level = (u16)shex_to_u16(game_settings +4, 3);
	first_filter = (u16)shex_to_u16(game_settings +8, 3);
	normal_filter = (u16)shex_to_u16(game_settings +12, 3);
	Rcoef = (u8)shex_to_u16(game_settings +16, 3);
	RcoefRight = (u8)shex_to_u16(game_settings +20, 3);
	touch_timer = (u8)shex_to_u16(game_settings +24, 3);
	
        game_cfg[0] = 0x0D;
	game_cfg[1] = touch_level& 0xFF00;
	game_cfg[2] = touch_level& 0x00FF;
	game_cfg[3] = leave_level& 0xFF00;
	game_cfg[4] = leave_level& 0x00FF;
	game_cfg[5] = first_filter& 0xFF00;
	game_cfg[6] = first_filter& 0x00FF;
	game_cfg[7] = normal_filter& 0xFF00;
	game_cfg[8] = normal_filter& 0x00FF;
	game_cfg[9] = Rcoef;
	game_cfg[10] = RcoefRight;
	game_cfg[11] = touch_timer;
	checksum = (u8)(0x00-(game_cfg[0]+game_cfg[1]+game_cfg[2]+game_cfg[3]+game_cfg[4]+
	  game_cfg[5]+game_cfg[6]+game_cfg[7]+game_cfg[8]+game_cfg[9]+game_cfg[10]+game_cfg[11]));

	game_cfg[12] = checksum;

        ts_info("touch_level 0x%04X, leave_level 0x%04X first_filter 0x%04X, normal_filter 0x%04X",
		touch_level,leave_level,first_filter,normal_filter);
	ts_info("Rcoef 0x%02X,RcoefRight 0x%02X touch_timer 0x%02X checksum 0x%02X",Rcoef,RcoefRight,touch_timer,checksum);

	if (setting_rotation)
	  msleep(10);
	
	ts_info("rotation_type %d game_mode %d game_resume %d",core_data->rotation,core_data->game_mode,game_resume);
        if(core_data->rotation!=0 && core_data->game_mode){
	  if(atomic_read(&core_data->rotation_set)==0)
	    need_rotation = true;
	}
	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->game_cfg)
				continue;
			if(!core_data->game_mode)	
				continue;
			
			if(game_resume || need_rotation) {
			  if(core_data->rotation == 90 || core_data->rotation== 270) {
			      r = ext_module->funcs->enter_rotation(core_data, ext_module);
			      if (r == EVT_ROTATION_FAIL) {
				  ts_info("Rotation fail");
				  atomic_set(&core_data->rotation_set, 0);
			      }
			      if(r == EVT_ROTATION_SUCCESS){
				  ts_info("Set Rotation success");
				  atomic_set(&core_data->rotation_set, 1);
			      }
			  } else 
			      ts_info("not rotation to 90 or 270");
			}
			
			r = ext_module->funcs->game_cfg(core_data, ext_module);
			if (r == EVT_GAME_CFG_FAIL) {
			    atomic_set(&core_data->game_cfg_set, 0);
			    ts_info("set game config fail");
			}
			if (r == EVT_GAME_CFG_SUCCESS){
			  atomic_set(&core_data->game_cfg_set, 1);
			  ts_info("set game config success");
			}
		}
	}
	
	mutex_unlock(&goodix_modules.mutex);

	return count;
}

static ssize_t phone_state_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("phone state %d",en);
	
	if(en == 1)
	  core_data->phone_call_on = true;
	else
	  core_data->phone_call_on = false;

	return count;
}

static ssize_t dongle_state_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
  	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("phone insert to %d",en);
	
	if(en == 2)
	  core_data->station_insert = true;
	else
	  core_data->station_insert = false;

	return count;
}

static ssize_t test_cfg_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
       	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;
	int en, r;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("update test cfg %d",en);
	
	if (!allow_hw_reset){
	  printk("FW is updating , not allow reset");
	  return -EINVAL;
	}  

	if(en == 0 && atomic_read(&core_data->testcfg)!=1){
	  ts_info("previous status not update test cfg");
	  return count;
	}

	disable_irq(core_data->irq);
        if(en == 1) {
	    /*request configuration*/
	    r = request_firmware(&cfg_img, GOODIX_TEST_CFG_NAME, dev);
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_TEST_CFG_NAME, r);
		goto exit;
	    } else
		ts_info("cfg file [%s] is ready", GOODIX_TEST_CFG_NAME);

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }
	    atomic_set(&core_data->testcfg, 1);
	}
	
	if(en == 0){ // reset touch cfg 
	    /*request configuration*/
	    r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	    } else
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }	
	    atomic_set(&core_data->testcfg, 0);
	}	
	
	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}
	
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	if (ts_dev->hw_ops->send_config)
	  r = ts_dev->hw_ops->send_config(ts_dev, config);
	ts_info("send cfg result %d", r);
exit:
	enable_irq(core_data->irq);

	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}

	ts_info("******OUT");
	
	return count;	
}

static ssize_t load_station_cfg_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
       	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;
	int en, r;
	bool resume_touch = false;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("update station cfg %d",en);
	
	if (!allow_hw_reset){
	  printk("FW is updating , not allow reset");
	  return -EINVAL;
	}  

	if(en == 0 && atomic_read(&core_data->station_cfg_reload)!=1){
	  ts_info("previous status not update to station cfg");
	  return count;
	}

	if(atomic_read(&core_data->glove_mode)==1){
	  ts_info("glove mode enabled ,do not re-download to station config");
	  return count;
	}

	if(atomic_read(&gts_core_data->dsi_suspend) == 1){
	  ts_info("touch suspended, resume touch");
	  resume_touch = true;
	  display_panel_off(0,0);
	  msleep(500);
	}
		
	mutex_lock(&core_data->load_read_cfg_lock);
	disable_irq(core_data->irq);
        if(en == 1) {
	    /*request configuration*/
	    r = request_firmware(&cfg_img, GOODIX_STATION_CFG_NAME, dev);
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_STATION_CFG_NAME, r);
		goto exit;
	    } else
		ts_info("cfg file [%s] is ready", GOODIX_STATION_CFG_NAME);

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }
	    atomic_set(&core_data->station_cfg_reload, 1);
	}
	
	if(en == 0){ // reset touch cfg 
	    /*request configuration*/
	    r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	    if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	    } else
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);

	    config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	    if (config == NULL) {
		ts_err("Memory allco err");
		goto exit;
	    }	
	    atomic_set(&core_data->station_cfg_reload, 0);
	}	
	
	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}
	
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	if (ts_dev->hw_ops->send_config)
	  r = ts_dev->hw_ops->send_config(ts_dev, config);
	ts_info("send cfg result %d", r);
exit:
	enable_irq(core_data->irq);

	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}
        mutex_unlock(&core_data->load_read_cfg_lock);

	if(!panel_on && resume_touch){
	  ts_info("suspend touch after load station because display off");
	  display_panel_off(1,0);
	}
	ts_info("******OUT");
	
	return count;	
}

static ssize_t read_station_mode_cfg_show(struct device *dev,
				struct device_attribute *attr,
						char *buf)
{  
	return snprintf(buf, PAGE_SIZE, "%02x",station_cfg_version);
}

static ssize_t disable_fod_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	
	if(core_data == NULL)
	  return snprintf(buf, PAGE_SIZE, "error\n");

        return snprintf(buf, PAGE_SIZE, "%d\n",core_data->disable_fod);
}


static ssize_t disable_fod_state_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
  	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
    ts_info("system disable fod %d",en);
	
	if(en == 1)
	  core_data->disable_fod = true;
	else
	  core_data->disable_fod = false;

	return count;
}

//ASUS_BSP "ASUS feactures" ---
static DEVICE_ATTR(extmod_info, S_IRUGO, goodix_ts_extmod_show, NULL);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_ts_driver_info_show, NULL);
static DEVICE_ATTR(chip_info, S_IRUGO, goodix_ts_chip_info_show, NULL);
static DEVICE_ATTR(config_data, S_IRUGO, goodix_ts_config_data_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, goodix_ts_reset_store);
static DEVICE_ATTR(send_cfg, S_IWUSR | S_IWGRP, NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, S_IRUGO, goodix_ts_read_cfg_show, NULL);
static DEVICE_ATTR(irq_info, S_IRUGO | S_IWUSR | S_IWGRP,
		goodix_ts_irq_info_show, goodix_ts_irq_info_store);
//ASUS_BSP "ASUS feactures" +++
static DEVICE_ATTR(power_on_off, S_IWUSR | S_IWGRP, NULL, goodix_ts_power_on_off);
static DEVICE_ATTR(power_state, S_IRUGO, goodix_ts_power_state, NULL);
static DEVICE_ATTR(rotation_type, S_IRUGO | S_IWUSR | S_IWGRP,
		goodix_ts_rotation_type_show, goodix_ts_rotation_type_store);
static DEVICE_ATTR(debug, S_IWUSR | S_IWGRP, NULL, goodix_ts_debug_store);
static DEVICE_ATTR(airtrigger_touch, S_IRUGO|S_IWUSR, airtrigger_touch_show, airtrigger_touch_store);
static DEVICE_ATTR(keymapping_touch, S_IRUGO|S_IWUSR, airtrigger_touch_show, keymapping_touch_store);
static DEVICE_ATTR(glove_mode, S_IRUGO|S_IWUSR, glove_mode_show, glove_mode_store);
static DEVICE_ATTR(game_mode, S_IRUGO | S_IWUSR | S_IWGRP, game_mode_show, game_mode_store);
static DEVICE_ATTR(game_settings, S_IRUGO | S_IWUSR | S_IWGRP, NULL, game_settings_store);
static DEVICE_ATTR(phone_state_on,S_IRUGO|S_IWUSR, NULL, phone_state_store);
static DEVICE_ATTR(dongle_state,S_IRUGO|S_IWUSR, NULL, dongle_state_store);
static DEVICE_ATTR(test_cfg, S_IRUGO|S_IWUSR, NULL, test_cfg_store);
static DEVICE_ATTR(load_station_cfg, S_IRUGO|S_IWUSR, NULL, load_station_cfg_store);
static DEVICE_ATTR(read_station_cfg, S_IRUGO, read_station_mode_cfg_show, NULL);
static DEVICE_ATTR(disable_fod_state,S_IRUGO|S_IWUSR, disable_fod_state_show, disable_fod_state_store);

//ASUS_BSP "ASUS feactures" ---
static struct attribute *sysfs_attrs[] = {
	&dev_attr_extmod_info.attr,
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_config_data.attr,
	&dev_attr_reset.attr,
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_power_on_off.attr,
	&dev_attr_power_state.attr,
	&dev_attr_rotation_type.attr,
	&dev_attr_debug.attr,
	&dev_attr_airtrigger_touch.attr,
	&dev_attr_keymapping_touch.attr,
	&dev_attr_glove_mode.attr,
	&dev_attr_game_mode.attr,
	&dev_attr_game_settings.attr,
	&dev_attr_phone_state_on.attr,
	&dev_attr_dongle_state.attr,
	&dev_attr_test_cfg.attr,
	&dev_attr_load_station_cfg.attr,
	&dev_attr_read_station_cfg.attr,
	&dev_attr_disable_fod_state.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	return sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* event notifier */
static BLOCKING_NOTIFIER_HEAD(ts_notifier_list);
/**
 * goodix_ts_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *  see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_register_notifier);

/**
 * goodix_ts_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_unregister_notifier);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_blocking_notify(enum ts_notify_event evt, void *v)
{
	return blocking_notifier_call_chain(&ts_notifier_list,
			(unsigned long)evt, v);
}
EXPORT_SYMBOL_GPL(goodix_ts_blocking_notify);



/**
 * goodix_ts_input_report - report touch event to input subsystem
 *
 * @dev: input device pointer
 * @touch_data: touch data pointer
 * return: 0 ok, <0 failed
 */
static int fod_position[4] = {440, 640,1960, 2140};

static int goodix_ts_input_report(struct input_dev *dev,
		struct goodix_touch_data *touch_data)
{
	struct goodix_ts_coords *coords = &touch_data->coords[0];
	struct goodix_ts_core *core_data = input_get_drvdata(dev);
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	unsigned int touch_num = touch_data->touch_num;
	static u32 pre_fin = 0x0000;
	static u8 pre_key = 0x00;
	int i, id;

	/*first touch down and last touch up condition*/
	if (touch_num != 0 && pre_fin == 0x0000) {
	        mutex_lock(&dev->mutex);
		/*first touch down event*/
		if (core_data->game_mode && core_data->atr_enable){
		    ts_info("atr pressed, ignore touch down event"); 
		} else {
		    ts_info("touch down");
		    input_report_key(dev, BTN_TOUCH, 1);
//		    input_sync(dev);
		    finger_press = true;
		}
		mutex_unlock(&dev->mutex);
        /*
		if (touch_num == 1 && !touch_data->pen_down && atomic_read(&g_notify_fod)) {
            TP_call_FOD(touch_data->coords[0].x, touch_data->coords[0].y);
        }*/
	} else if (touch_num == 0 && pre_fin != 0x0000) {
		/*no finger exist*/
        
		if (core_data->game_mode && core_data->atr_enable){
		    ts_info("game enable, ignore up event"); 
		} else {
		    ts_info("touch up");
    		    mutex_lock(&dev->mutex);
		    input_report_key(dev, BTN_TOUCH, 0);
		    finger_press = false;
		    mutex_unlock(&dev->mutex);    
            fod_press = 0x0000;
            if (atomic_read(&g_notify_fod)) 
                TP_call_FOD(10, false);
		}
	}  else if (atomic_read(&gts_core_data->dsi_suspend)==1) {
	        ts_info("touch suspended , release all finger data");
		input_report_key(dev, BTN_TOUCH, 0);
		for (i = 0; i < ts_bdata->panel_max_id * 2; i++) {
		    if (pre_fin & (1 << i)) {/* release touch */
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
			pre_fin &= ~(1 << i);
		    }
		}
		input_sync(dev);
		finger_press = false;
		
		ts_info("keymapping SUSPEND atr_enable=%d", core_data->atr_enable);
	        if(core_data->atr_enable) { // release airtrigger & game pad fingers   
		    input_report_key(dev, BTN_TOUCH, 0);
		    for(i = 0;i < TOTAL_SLOT ; i ++) {
		      input_mt_slot(dev, i+10);
		      input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);		      
		    }
		    input_sync(dev);
		    ts_info("keymapping release all Airtrigger");
		    LastATR = LastATL = 0;
		    core_data->atr_enable = false;
		}
		
	}

	/*report key, include tp's key and pen's key */
	if (unlikely(touch_data->have_key)) {
		for (i = 0; i < ts_bdata->panel_max_key; i++) {
			input_report_key(dev, ts_bdata->panel_key_map[i],
							touch_data->key_value & (1 << i));
		}
		pre_key = touch_data->key_value;
		/*ts_info("$$$$$$pre_key:0x%02x",pre_key);*/
	} else if (pre_key != 0x00) {
		/*ts_info("******no key, by pre_key is not ZERO! pre_key:0x%02x", pre_key);*/
		for (i = 0; i < ts_bdata->panel_max_key; i++) {
			if (pre_key & (1 << i)) {
				input_report_key(dev, ts_bdata->panel_key_map[i], 0);
				pre_key &= ~(1 << i);
				/*ts_info("******report i:%d, key:%d leave", i, ts_bdata->panel_key_map[i]);*/
			}
		}
		/*ts_info("******after, pre_key:0x%02x", pre_key);*/
	}

#if 1
	/*protocol B*/
#if 0
	/*report pen*/
	if (touch_num >= 1 && touch_data->pen_down) {
		touch_num -= 1;

		input_mt_slot(dev, ts_bdata->panel_max_id * 2);
		input_mt_report_slot_state(dev, MT_TOOL_PEN, true);

		input_report_abs(dev, ABS_MT_POSITION_X, touch_data->pen_coords[0].x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch_data->pen_coords[0].y);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, touch_data->pen_coords[0].w);
		input_report_abs(dev, ABS_MT_PRESSURE, touch_data->pen_coords[0].p);

		pre_fin |= 1 << 20;
		/*ts_info("!!!!!!report pen  DOWN,%d,%d,%d",
				touch_data->pen_coords[0].x,
				touch_data->pen_coords[0].y,
				touch_data->pen_coords[0].p);*/
	} else {
		if (pre_fin & (1 << 20)) {
			input_mt_slot(dev, ts_bdata->panel_max_id * 2);
			input_mt_report_slot_state(dev, MT_TOOL_PEN, false);
			pre_fin &= ~(1 << 20);
			/*ts_info("!!!!!!report pen LEAVE");*/
		}
	}
#endif
	/*report finger*/
	id = coords->id;
	for (i = 0; i < ts_bdata->panel_max_id * 2; i++) {
		if (touch_num && i == id) { /* this is a valid touch down event */
            if (!id_pressed[id] && !atomic_read(&g_notify_fod)) {
                ts_info("input report id:%d x:%d,y:%d,p:%d",id, coords->x, coords->y,coords->p);
            }
            if (atomic_read(&g_notify_fod) && !core_data->disable_fod) {
                if (fod_position[0] < coords->x && coords->x < fod_position[1] && fod_position[2] < coords->y && coords->y < fod_position[3]) {
                	if ((!fod_press) && g_id_fod != i) {
                        TP_call_FOD( i, true);
                        input_mt_slot(dev, i);
                        input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
                        input_sync(dev);
                        fod_press |= 1 << i;
                    }
//                } else {
//                    if (g_id_fod == id)
//                        TP_call_FOD( 10, false);                        
//                        ts_info("input report g_id_fod:%d id:%d x:%d,y:%d,p:%d", g_id_fod, id, coords->x, coords->y,coords->p);
                    pre_fin |= 1 << i;
                    id = (++coords)->id;
                    continue;
                } else if (fod_press == (1<<i)){ 
					pre_fin |= 1 << i;
					id = (++coords)->id;
					continue;
                }
            } else if (fod_press == (1<<i)){ 
					pre_fin |= 1 << i;
					id = (++coords)->id;
					continue;
            }
            mutex_lock(&dev->mutex);
			input_mt_slot(dev, id);
			id_pressed[id] = 1;
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);

			input_report_abs(dev, ABS_MT_POSITION_X, coords->x);
			input_report_abs(dev, ABS_MT_POSITION_Y, coords->y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, coords->w);
			input_report_abs(dev, ABS_MT_PRESSURE, coords->p);
            finger_press = true;
            mutex_unlock(&dev->mutex);
			pre_fin |= 1 << i;
			id = (++coords)->id;
		} else {
			if (pre_fin & (1 << i)) {/* release touch */
			  ts_info("release touch %d , game_mode %d atr_enable %d", i,core_data->game_mode,core_data->atr_enable);
			  if(core_data->game_mode && core_data->atr_enable){
			      mutex_lock(&dev->mutex);
			      ts_info("ATR pressed ignore finger up event");
				  input_mt_slot(dev, i);

				  input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
				  input_sync(dev);
				  pre_fin &= ~(1 << i);
				  ts_info("game_mode report leave:%d touchN %d pre_fin %u", i,touch_num,pre_fin);
				  if (touch_num == 0)
				    finger_press = false;
                  id_pressed[i] = 0;
				  mutex_unlock(&dev->mutex);
			  }else {
				  mutex_lock(&dev->mutex);
				  input_mt_slot(dev, i);
				  input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
				  input_sync(dev);
				  pre_fin &= ~(1 << i);
				  mutex_unlock(&dev->mutex);

				  ts_info("report leave id:%d ",i);
				  id_pressed[i] = 0;
				  if (touch_num == 0)
				    finger_press = false;
				  
                  if (atomic_read(&g_notify_fod)) {
				      if (g_id_fod == i) {
                      TP_call_FOD( 10, false);
                      fod_press = 0x0000;
                  } else if (fod_press == (1<<i))
                      fod_press = 0x0000;
                  } else if (fod_press == (1<<i))
                      fod_press = 0x0000;
                  }
			}
		}
	}
#endif

#if 0
	/*report pen use protocl A*/
	if (touch_data->pen_down) {
		/*input_report_key(dev, BTN_TOOL_PEN, 1);*/
		/*input_report_key(dev, BTN_TOUCH, 1);*/

		input_report_abs(dev, ABS_MT_POSITION_X, touch_data->pen_coords[0].x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch_data->pen_coords[0].y);

		input_report_abs(dev, ABS_MT_PRESSURE, touch_data->pen_coords[0].p);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, touch_data->pen_coords[0].w);
		/*input_report_abs(dev, ABS_MT_TRACKING_ID, touch_data->pen_coords[0].id);*/
		input_report_abs(dev, ABS_MT_TOOL_TYPE, 1);

		input_mt_sync(dev);
	} else {
		if (pre_fin & (1 << 10) && touch_num == 0) {
			/*input_report_key(dev, BTN_TOOL_PEN, 0);*/
			/*input_report_key(dev, BTN_TOUCH, 0);*/

			pre_fin &= ~(1 << 10);
		}
	}

	/* report abs */
	id = coords->id;
	for (i = 0; i < ts_bdata->panel_max_id; i++) {
		if (touch_num && i == id) { /* this is a valid touch down event */

			/*input_report_key(dev, BTN_TOUCH, 1);*/
			/*input_report_abs(dev, ABS_MT_TRACKING_ID, id);*/

			input_report_abs(dev, ABS_MT_POSITION_X, coords->x);
			input_report_abs(dev, ABS_MT_POSITION_Y, coords->y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, coords->w);
			input_report_abs(dev, ABS_MT_PRESSURE, coords->p);
			input_mt_sync(dev);

			pre_fin |= 1 << i;
			id = (++coords)->id;
		} else {
			if (pre_fin & (1 << i)) {
				/*input_mt_sync(dev);*/

				pre_fin &= ~(1 << i);
			}
		}
	}
#endif

        input_sync(dev);
	return 0;
}
void ATR_touch_new(struct device *dev, int id,int action, int x, int y, int random)
{
    static int random_x = -5, random_y = -5, random_pressure = -20, random_major = -5;
    struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct input_dev *input_dev = core_data->input_dev;
    int first_empty_slot = -1;
    int i;
    
    //ts_info("keymapping ATR_touch_new  id=%d, action=%d, x=%d, y=%d\n", id, action,  x,  y);
    mutex_lock(&input_dev->mutex);
    if(action) //press, find first slot or find last slot;
    {
        for(i = TOTAL_SLOT -1; i >= 0 ; i--)
        {
            if(first_empty_slot == -1 && touch_figer_slot[i] == 0) //find first empty slot
                first_empty_slot = i;
            if(touch_figer_slot[i] == (id + 1)) //if the last id has been pressed, keep reporting same slot
                first_empty_slot = i;
        }
        //ts_info("keymapping ATR_touch_new press found slot %d\n", first_empty_slot);
        if(first_empty_slot != -1) // found an available slot
        {
            if(touch_figer_slot[first_empty_slot] ==0)
                ts_info("keymapping report %d down x=%d ,y=%d ",first_empty_slot,x,y);
            input_mt_slot(input_dev, first_empty_slot + 10);
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
            
            if(!random)
            {
                input_report_abs(input_dev, ABS_MT_PRESSURE, 0x3f + random_pressure);
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x09 + random_major);
                input_report_abs(input_dev, ABS_MT_POSITION_X, x + random_x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y + random_y);
            }
            else
            {
                input_report_abs(input_dev, ABS_MT_PRESSURE, 0x3f + random_pressure);
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x09 + random_major);
                input_report_abs(input_dev, ABS_MT_POSITION_X, x );
                input_report_abs(input_dev, ABS_MT_POSITION_Y, y );
            }    
            
            if(!finger_press){
                //ts_info("atr touch down");
                input_report_key(input_dev, BTN_TOUCH, 1);
            } else {
                ts_info("finger pressed , ignore atr touch down");
            }
            input_sync(input_dev);
            touch_figer_slot[first_empty_slot] = id + 1; // save finger id in slot 
            core_data->atr_enable = true;
        }
    }
    else //release
    {
        for(i = TOTAL_SLOT -1; i >= 0 ; i--)
        {
            if(touch_figer_slot[i] == (id + 1)) //find the released slot
            {
                first_empty_slot = i;
                break;
            }
        }
        
        ts_info("keymapping  release slot %d\n", first_empty_slot);
        if(first_empty_slot >= 0)
        {
            input_mt_slot(input_dev, first_empty_slot + 10);
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
            input_sync(input_dev);
            touch_figer_slot[first_empty_slot] = 0;
        }
    }

    for(i = TOTAL_SLOT -1; i >= 0 ; i--)
    {
        if(touch_figer_slot[i] != 0) //find the released slot
            break;
    }   
    if(i < 0) // all button up
    {
        core_data->atr_enable = false;
        if(!finger_press)
        {
            ts_info("keymapping  all button up\n");
            input_report_key(input_dev, BTN_TOUCH, 0);
            input_sync(input_dev);
        }
    } 
    random_x += 1; if(random_x > 5) random_x = -1;
    random_y += 1; if(random_y > 5) random_y = -1;
    random_pressure += 1; if(random_pressure > 20) random_pressure = -1;
    random_major += 1; if(random_major > 5) random_major = -1;
    
    mutex_unlock(&input_dev->mutex);
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_device *ts_dev =  core_data->ts_dev;
	struct goodix_ext_module *ext_module;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	int r;

	if (Reseting)
            return IRQ_HANDLED;
	
	core_data->irq_trig_cnt++;

	if(core_data->debug ) {
	    //debug raw data
	    mutex_lock(&goodix_modules.mutex);
	    list_for_each_entry(ext_module, &goodix_modules.head, list) {
		if (!ext_module->funcs->tool_irq_event)
		    continue;
		r = ext_module->funcs->tool_irq_event(core_data, ext_module);
		if (r == EVT_CANCEL_IRQEVT) {
		    mutex_unlock(&goodix_modules.mutex);
		    return IRQ_HANDLED;
		}
	    }
	    mutex_unlock(&goodix_modules.mutex);		  
	}
	
	/* read touch data from touch device */
	r = ts_dev->hw_ops->event_handler(ts_dev, ts_event);
	if (likely(r >= 0)) {
		if (ts_event->event_type == EVENT_TOUCH) {
			/* report touch */
			goodix_ts_input_report(core_data->input_dev,
					&ts_event->event_data.touch_data);			
		}
	}
	
	if (ts_event->event_type == EVENT_GESTURE) {
	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	list_for_each_entry(ext_module, &goodix_modules.head, list) {
		if (!ext_module->funcs->irq_event)
			continue;
		r = ext_module->funcs->irq_event(core_data, ext_module);
		if (r == EVT_CANCEL_IRQEVT) {
			mutex_unlock(&goodix_modules.mutex);
			return IRQ_HANDLED;
		} else if (r == EVT_CANCEL_RESUME){
		  	mutex_unlock(&goodix_modules.mutex);
			return IRQ_HANDLED;
		}
	}
	mutex_unlock(&goodix_modules.mutex);
	}
	
	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;

	/* if ts_bdata-> irq is invalid */
	if (ts_bdata->irq <= 0) {
		core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	} else {
		core_data->irq = ts_bdata->irq;
	}

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	r = devm_request_threaded_irq(&core_data->pdev->dev,
			core_data->irq, NULL,
			goodix_ts_threadirq_func,
			ts_bdata->irq_flags | IRQF_ONESHOT,
			GOODIX_CORE_DRIVER_NAME,
			core_data);
	if (r < 0)
		ts_err("Failed to requeset threaded irq:%d", r);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return r;
}

/**
 * goodix_ts_irq_enable - Enable/Disable a irq
 * @core_data: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_enable(struct goodix_ts_core *core_data,
			bool enable)
{
	if (enable) {
		if (!atomic_cmpxchg(&core_data->irq_enabled, 0, 1)) {
			enable_irq(core_data->irq);
			ts_debug("Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&core_data->irq_enabled, 1, 0)) {
			disable_irq(core_data->irq);
			ts_debug("Irq disabled");
		}
	}

	return 0;
}
EXPORT_SYMBOL(goodix_ts_irq_enable);
/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct device *dev = NULL;
	struct goodix_ts_board_data *ts_bdata;
	int r = 0;

	ts_info("Power init");
	/* dev:i2c client device or spi slave device*/
	dev =  core_data->ts_dev->dev;
	ts_bdata = board_data(core_data);

	if (ts_bdata->avdd_name) {
		core_data->avdd = devm_regulator_get(dev,
				 ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			r = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", r);
			core_data->avdd = NULL;
			return r;
		}
	} else {
		ts_info("Avdd name is NULL");
	}

	return r;
}

/**
 * goodix_ts_power_on - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_on(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;

	ts_info("Device power on");
	if (core_data->power_on)
		return 0;

	if (core_data->avdd) {
		r = regulator_enable(core_data->avdd);
		if (!r) {
			ts_info("regulator enable SUCCESS");
			if (ts_bdata->power_on_delay_us)
				usleep_range(ts_bdata->power_on_delay_us,
						ts_bdata->power_on_delay_us);
		} else {
			ts_err("Failed to enable analog power:%d", r);
			return r;
		}
	}

	core_data->power_on = 1;
	return 0;
}

/**
 * goodix_ts_power_off - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_off(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;

	ts_info("Device power off");
	if (!core_data->power_on)
		return 0;

	if (core_data->avdd) {
		r = regulator_disable(core_data->avdd);
		if (!r) {
			ts_info("regulator disable SUCCESS");
			if (ts_bdata->power_off_delay_us)
				usleep_range(ts_bdata->power_off_delay_us,
						ts_bdata->power_off_delay_us);
		} else {
			ts_err("Failed to disable analog power:%d", r);
			return r;
		}
	}

	core_data->power_on = 0;
	return 0;
}

#ifdef CONFIG_PINCTRL
/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_ts_core *core_data)
{
	int r = 0;
        ts_info("goodix_ts_pinctrl_init start");
	/* get pinctrl handler from of node */
	core_data->pinctrl = devm_pinctrl_get(core_data->ts_dev->dev);
	if (IS_ERR_OR_NULL(core_data->pinctrl)) {
		ts_err("Failed to get pinctrl handler");
		return PTR_ERR(core_data->pinctrl);
	}

	/* active state */
	core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		r = PTR_ERR(core_data->pin_sta_active);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_ACTIVE, r);
		goto exit_pinctrl_put;
	}

	/* suspend state */
	core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		r = PTR_ERR(core_data->pin_sta_suspend);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_SUSPEND, r);
		goto exit_pinctrl_put;
	}

	ts_info("goodix_ts_pinctrl_init complete");
	return 0;
exit_pinctrl_put:
	devm_pinctrl_put(core_data->pinctrl);
	core_data->pinctrl = NULL;
	core_data->pin_sta_active = NULL;
	core_data->pin_sta_suspend = NULL; 
	return r;
}
#endif

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 *	reset_gpio and irq_gpio number are obtained from goodix_ts_device
 *  which created in hardware layer driver. e.g.goodix_xx_i2c.c
 *	A goodix_ts_device should set those two fileds to right value
 *	before registed to touch core driver.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			 board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	ts_info("before setup gpio reset = %d", gpio_get_value(ts_bdata->reset_gpio));
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->reset_gpio,
			GPIOF_OUT_INIT_HIGH,
			"ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->irq_gpio,
			GPIOF_IN,
			"ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}
	
	ts_info("reset touch ic for init");
        gpio_direction_output(ts_bdata->reset_gpio, 0);
	udelay(10000);
	gpio_direction_output(ts_bdata->reset_gpio, 1);
	msleep(100);

	ts_info("gpio reset = %d", gpio_get_value(ts_bdata->reset_gpio));
	ts_info("gpio irq = %d", gpio_get_value(ts_bdata->irq_gpio));
	return 0;
}

/**
 * goodix_input_set_params - set input parameters
 */
static void goodix_ts_set_input_params(struct input_dev *input_dev,
		struct goodix_ts_board_data *ts_bdata)
{
	int i;

	if (ts_bdata->swap_axis)
		swap(ts_bdata->panel_max_x, ts_bdata->panel_max_y);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, ts_bdata->panel_max_w, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			0, ts_bdata->panel_max_p, 0, 0);

	if (ts_bdata->panel_max_key) {
		for (i = 0; i < ts_bdata->panel_max_key; i++)
			input_set_capability(input_dev, EV_KEY,
					ts_bdata->panel_key_map[i]);
	}
}

/**
 * goodix_ts_input_dev_config - Requset and config a input device
 *  then register it to input sybsystem.
 *  NOTE that some hardware layer may provide a input device
 *  (ts_dev->input_dev not NULL).
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int r;

	input_dev = devm_input_allocate_device(&core_data->pdev->dev);
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	input_dev->name = GOODIX_CORE_DRIVER_NAME;
	input_dev->phys = GOOIDX_INPUT_PHYS;
	input_dev->id.product = 0xDEAD;
	input_dev->id.vendor = 0xBEEF;
	input_dev->id.version = 10427;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
//	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	__set_bit(KEY_W, input_dev->keybit);
	__set_bit(KEY_S, input_dev->keybit);
	__set_bit(KEY_E, input_dev->keybit);
	__set_bit(KEY_C, input_dev->keybit);
	__set_bit(KEY_Z, input_dev->keybit);
	__set_bit(KEY_V, input_dev->keybit);

//	__set_bit(BTN_TOOL_PEN, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* set input parameters */
	goodix_ts_set_input_params(input_dev, ts_bdata);

#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	/*input_mt_init_slots(input_dev, ts_bdata->panel_max_id,
			INPUT_MT_DIRECT);*/
	input_mt_init_slots(input_dev,
			ts_bdata->panel_max_id * 2 + 1,
			INPUT_MT_DIRECT);
#else
	/*input_mt_init_slots(input_dev, ts_bdata->panel_max_id);*/
	input_mt_init_slots(input_dev,
			ts_bdata->panel_max_id * 2 + 1);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_UP);	
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	/* add for FOD*/
	input_set_capability(input_dev, EV_KEY, KEY_F);
	input_set_capability(input_dev, EV_KEY, KEY_U);
	input_set_capability(input_dev, EV_KEY, KEY_L);

	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		return r;
	}

	return 0;
}

/**
 * goodix_ts_hw_init - Hardware initilize
 *  poweron - hardware reset - sendconfig
 * @core_data: pointer to touch core data
 * return: 0 intilize ok, <0 failed
 */
int goodix_ts_hw_init(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_hw_ops *hw_ops =
		ts_hw_ops(core_data);
	int r;
	
	/* reset touch device */
	if (hw_ops->reset) {
		r = hw_ops->reset(core_data->ts_dev);
		if (r < 0)
			goto exit;
	}

	/* init */
	if (hw_ops->init) {
		r = hw_ops->init(core_data->ts_dev);
		if (r < 0)
			goto exit;
	}

exit:
	/* if bus communication error occured then
	 * exit driver binding, other errors will
	 * be ignored */
	if (r != -EBUS)
		r = 0;
	return r;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd = container_of(dwork,
			struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *core = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(core);
	int r = 0;
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;

	if (ts_esd->esd_on == false)
		return;

	if (hw_ops->check_hw)
		r = hw_ops->check_hw(core->ts_dev);
	if (r < 0) {
		goodix_ts_power_off(core);
		goodix_ts_power_on(core);
		if (hw_ops->reset)
			hw_ops->reset(core->ts_dev);

		/*init static esd*/
		if (core->ts_dev->ic_type == IC_TYPE_NANJING) {
			r = hw_ops->write(core->ts_dev,
					0x8043, &data, 1);
			if (r < 0)
				ts_err("nanjing esd reset, init static esd FAILED, i2c wirte ERROR");
		}

		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			ts_err("esd reset, init dynamic esd FAILED, i2c write ERROR");

	} else {
		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			ts_err("esd init watch dog FAILED, i2c write ERROR");
	}

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on)
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
	mutex_unlock(&ts_esd->esd_mutex);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
static void goodix_ts_esd_on(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	if(core->ts_dev->reg.esd == 0)
		return;

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on == false) {
		ts_esd->esd_on = true;
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
		mutex_unlock(&ts_esd->esd_mutex);
		ts_info("Esd on");
		return;
	}
	mutex_unlock(&ts_esd->esd_mutex);
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
static void goodix_ts_esd_off(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on == true) {
		ts_esd->esd_on = false;
		cancel_delayed_work(&ts_esd->esd_work);
		mutex_unlock(&ts_esd->esd_mutex);
		ts_info("Esd off");
		return;
	}
	mutex_unlock(&ts_esd->esd_mutex);
}

/**
 * goodix_esd_notifier_callback - notification callback
 *  under certain condition, we need to turn off/on the esd
 *  protector, we use kernel notify call chain to achieve this.
 *
 *  for example: before firmware update we need to turn off the
 *  esd protector and after firmware update finished, we should
 *  turn on the esd protector.
 */
static int goodix_esd_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_esd *ts_esd = container_of(nb,
			struct goodix_ts_esd, esd_notifier);

	switch (action) {
	case NOTIFY_FWUPDATE_START:
	case NOTIFY_SUSPEND:
	case NOTIFY_ESD_OFF:
		goodix_ts_esd_off(ts_esd->ts_core);
		break;
	case NOTIFY_FWUPDATE_END:
	case NOTIFY_RESUME:
	case NOTIFY_ESD_ON:
		goodix_ts_esd_on(ts_esd->ts_core);
		break;
	}

	return 0;
}

/**
 * goodix_ts_esd_init - initialize esd protection
 */
int goodix_ts_esd_init(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	struct goodix_ts_device *dev = core->ts_dev;
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;
	int r;

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	mutex_init(&ts_esd->esd_mutex);
	ts_esd->ts_core = core;
	ts_esd->esd_on = false;
	ts_esd->esd_notifier.notifier_call = goodix_esd_notifier_callback;
	goodix_ts_register_notifier(&ts_esd->esd_notifier);

	if (dev->hw_ops->check_hw && dev->reg.esd != 0) {
		/*init static esd*/
		if (dev->ic_type == IC_TYPE_NANJING) {
			r = dev->hw_ops->write_trans(core->ts_dev,
				0x8043, &data, 1);
			if (r < 0)
				ts_err("static ESD init ERROR, i2c write failed");
		}

		/*init dynamic esd*/
		r = dev->hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			ts_err("dynamic ESD init ERROR, i2c write failed");

		goodix_ts_esd_on(core);
	}
	return 0;
}

/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to  sleep
 */
static int goodix_ts_suspend(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int r;

	ts_info("Suspend start");

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;

			r = ext_module->funcs->before_suspend(core_data, ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	/* disable irq */
	goodix_ts_irq_enable(core_data, false);

	/* let touch ic work in sleep mode */
	if (ts_dev && ts_dev->hw_ops->suspend)
		ts_dev->hw_ops->suspend(ts_dev);
	atomic_set(&core_data->suspended, 1);
        atomic_set(&gts_core_data->dsi_suspend, 1);
#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
				core_data->pin_sta_suspend);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* inform exteranl modules */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->after_suspend)
				continue;

			r = ext_module->funcs->after_suspend(core_data, ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
	/* release all the touch IDs */
	//core_data->ts_event.event_data.touch_data.touch_num = 0;
	if (core_data->cfg_group_parsed == true)
	goodix_ts_input_report(core_data->input_dev,
			&core_data->ts_event.event_data.touch_data);
    ts_info("keymapping SUSPEND atr_enable=%d", core_data->atr_enable);
	if(core_data->atr_enable) { // release airtrigger fingers
        
      input_report_key(core_data->input_dev, BTN_TOUCH, 0);
      input_sync(core_data->input_dev);
      ts_info("keymapping release all Airtrigger");
      LastATR = LastATL = 0;
      core_data->atr_enable = false;
	}
	ts_info("Suspend end");
	return 0;
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
static int goodix_ts_resume(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev =
				core_data->ts_dev;
	int r;

	ts_info("Resume start");
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->before_resume)
				continue;

			r = ext_module->funcs->before_resume(core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	atomic_set(&core_data->suspended, 0);
        atomic_set(&gts_core_data->dsi_suspend, 0);
	/* resume device */
	if (ts_dev && ts_dev->hw_ops->resume)
		ts_dev->hw_ops->resume(ts_dev);

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	goodix_ts_irq_enable(core_data, true);
out:
	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	goodix_ts_blocking_notify(NOTIFY_RESUME, NULL);
	ts_debug("Resume end");
	return 0;
}

static void goodix_suspend_work(struct work_struct *work)
{
  
        struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev = gts_core_data->ts_dev;
	int r;

	ts_info("Suspend work start");
        mutex_lock(&gts_core_data->gts_suspend_mutex);
	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */

	if(!allow_suspend || panel_on){ // touch is under resume or panel already on
	  ts_info("not allow suspend");
	  mutex_unlock(&gts_core_data->gts_suspend_mutex);
	  return;
	}

	goodix_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;
						
			r = ext_module->funcs->before_suspend(gts_core_data, ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	/* disable irq */
	goodix_ts_irq_enable(gts_core_data, false);
	
	/* let touch ic work in sleep mode */
	if (ts_dev && ts_dev->hw_ops->suspend)
	    ts_dev->hw_ops->suspend(ts_dev);
	
	atomic_set(&gts_core_data->suspended, 1);
        atomic_set(&gts_core_data->dsi_suspend, 1);
	allow_suspend = false; //already suspend
#ifdef CONFIG_PINCTRL
	if (gts_core_data->pinctrl) {
		r = pinctrl_select_state(gts_core_data->pinctrl,
				gts_core_data->pin_sta_suspend);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

out:
	/* release all the touch IDs */
	//core_data->ts_event.event_data.touch_data.touch_num = 0;
        if (gts_core_data->cfg_group_parsed == true)
	    goodix_ts_input_report(gts_core_data->input_dev,
			&gts_core_data->ts_event.event_data.touch_data);

	mutex_unlock(&gts_core_data->gts_suspend_mutex);
	ts_info("Suspend end");
	return;


}
static void goodix_resume_work(struct work_struct *work)
{
  	struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev = gts_core_data->ts_dev;
	int r;

	ts_info("Resume work start");
	mutex_lock(&gts_core_data->gts_suspend_mutex);
#ifdef CONFIG_PINCTRL
	if (gts_core_data->pinctrl) {
		r = pinctrl_select_state(gts_core_data->pinctrl,
					gts_core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif
        atomic_set(&gts_core_data->suspended, 0);
        atomic_set(&gts_core_data->dsi_suspend, 0);
	/* resume device */
	if (ts_dev && ts_dev->hw_ops->resume)
	    ts_dev->hw_ops->resume(ts_dev);

	
	goodix_ts_irq_enable(gts_core_data, true);

	allow_suspend = true;
	
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry(ext_module, &goodix_modules.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(gts_core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	goodix_ts_blocking_notify(NOTIFY_RESUME, NULL);
	ts_debug("Resume end");
	mutex_unlock(&gts_core_data->gts_suspend_mutex);
	return;

}


/*ASUS_BSP Beryl +++ */
void display_panel_off(int panel_off, int nolp)
{
        if (gts_core_data == NULL || init_success!= 1) {
	  printk("goodix touch not vaild or init fail\n"); 
	} else {
	   ts_info("display %d , touch state %d , AOD %d",panel_off, atomic_read(&gts_core_data->dsi_suspend),g_enter_AOD);
	   ts_info("phone state %d ingore_reset = %d nolp = %d",gts_core_data->phone_call_on,ingore_reset,nolp);
           if (ingore_reset == 1 && nolp ==1)
	     ingore_reset = 0;
	   
	   gts_core_data->disable_fod = false;	   
           if (panel_off == 1) { //display panel off , touch suspend
	          if(atomic_read(&gts_core_data->dsi_suspend) == 1){
		    ts_info("touch already suspend");
		    return;
		  }
	          allow_suspend = true;
		  queue_work(gts_core_data->gts_suspend_resume_wq, &gts_core_data->gts_suspend_work);
	   } else {//display panel on , touch resume
	          if (ingore_reset == 1) {
		    ts_info("ingore_reset == 1");
		    allow_suspend = false;
		    return;
		  }
	          allow_suspend = false;
		  queue_work(gts_core_data->gts_suspend_resume_wq, &gts_core_data->gts_resume_work);
	   }   	
	}
}
EXPORT_SYMBOL(display_panel_off);

void gts_usb_plugin(bool plugin)
{        
        int r;
        struct goodix_ext_module *ext_module;

        if (gts_core_data == NULL || init_success!=1) {
	    printk("goodix touch not vaild or init fail\n"); 
	    return;
	}
		
	if(atomic_read(&gts_core_data->dsi_suspend)==1){
	  ts_info("touch suspended,not enter charge mode, usb plug %d  ",plugin);
	  return;
	}
	
	if(gts_core_data->station_insert){
	  ts_info("insert into station , not enter charge mode");
	  return;
	}
	
	if(plugin) {
	        ts_info("usb plug , charge mode enable");
		mutex_lock(&goodix_modules.mutex);
		atomic_set(&gts_core_data->charge_mode, 1); //set charge_mode enable
		
		if (!list_empty(&goodix_modules.head)) {
		    list_for_each_entry(ext_module, &goodix_modules.head, list) {
		        if (!ext_module->funcs->charge_mode)
		            continue;
		        r = ext_module->funcs->charge_mode(gts_core_data, ext_module);
			if (r == EVT_CHANGE_MODE_CANCEL || r == EVT_CHANGE_MODE_FAIL){
			    atomic_set(&gts_core_data->charge_mode, 0);
			}			  
		    }
	        }
		mutex_unlock(&goodix_modules.mutex);	   
	} else {
	      ts_info("usb unplug , charge mode exit");
	      mutex_lock(&goodix_modules.mutex);
		
	      if (!list_empty(&goodix_modules.head)) {
		    list_for_each_entry(ext_module, &goodix_modules.head, list) {
		        if (!ext_module->funcs->charge_mode)
		            continue;
                        if (atomic_read(&gts_core_data->charge_mode)==0) {
			    ts_info("previous mode not in charge mode");
			    continue;
			} else {
			    atomic_set(&gts_core_data->charge_mode, 0);
			    r = ext_module->funcs->charge_mode(gts_core_data, ext_module);
			}
		    }
	      }
	      mutex_unlock(&goodix_modules.mutex);
	}	  
}
EXPORT_SYMBOL(gts_usb_plugin);
/*ASUS_BSP Beryl --- */

#ifdef CONFIG_FB
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
int goodix_ts_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;

	if (fb_event && fb_event->data && core_data) {
		if (event == FB_EARLY_EVENT_BLANK) {
			/* before fb blank */
		} else if (event == FB_EVENT_BLANK) {
			int *blank = fb_event->data;
			if (*blank == FB_BLANK_UNBLANK)
				goodix_ts_resume(core_data);
			else if (*blank == FB_BLANK_POWERDOWN)
				goodix_ts_suspend(core_data);
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * goodix_ts_earlysuspend - Early suspend function
 * Called by kernel during system suspend phrase
 */
static void goodix_ts_earlysuspend(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_lateresume - Late resume function
 * Called by kernel during system wakeup
 */
static void goodix_ts_lateresume(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_resume(core_data);
}
#endif

/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
        ts_info("goodix_ts_pm_suspend");
	return goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
        ts_info("goodix_ts_pm_resume");
	return goodix_ts_resume(core_data);
}

/**
 * goodix_generic_noti_callback - generic notifier callback
 *  for goodix touch notification event.
 */
int goodix_generic_noti_callback(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct goodix_ts_core *ts_core = container_of(self,
			struct goodix_ts_core, ts_notifier);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(ts_core);
	int r;

	switch (action) {
	case NOTIFY_FWUPDATE_END:
		if (hw_ops->init) {
			/* Firmware has been updated, we need to reinit
			 * the chip, read the sensor ID and send the
			 * correct config data based on sensor ID.
			 * The input parameters also needs to be updated.*/
			r = hw_ops->init(ts_core->ts_dev);
			if (r < 0)
				goto exit;

			goodix_ts_set_input_params(ts_core->input_dev,
					ts_core->ts_dev->board_data);
		}
		break;
	}

exit:
	return 0;

}


/**
 * goodix_ts_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = NULL;
	struct goodix_ts_device *ts_device;
	int r;
	u8 read_val = 0;

	ts_info("goodix_ts_probe IN");

	ts_device = pdev->dev.platform_data;
	if (!ts_device || !ts_device->hw_ops ||
			!ts_device->board_data) {
		ts_err("Invalid touch device");
		return -ENODEV;
	}

	core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_ts_core),
						GFP_KERNEL);
	if (!core_data) {
		ts_err("Failed to allocate memory for core data");
		return -ENOMEM;
	}

	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->ts_dev = ts_device;
	platform_set_drvdata(pdev, core_data);
	core_data->cfg_group_parsed = false;

	r = goodix_ts_power_init(core_data);
	if (r < 0)
		goto out;

	r = goodix_ts_power_on(core_data);
	if (r < 0)
		goto out;

#ifdef CONFIG_PINCTRL
	/* Pinctrl handle is optional. */
	r = goodix_ts_pinctrl_init(core_data);
	if (!r && core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* get GPIO resource */
	r = goodix_ts_gpio_setup(core_data);
	if (r < 0)
		goto out;

	/*create sysfs files*/
	goodix_ts_sysfs_init(core_data);

	r = ts_device->hw_ops->reset(ts_device);
	if (r < 0)
		goto out;

	/*i2c test*/
	r = ts_device->hw_ops->read_trans(ts_device, 0x3100,
			&read_val, 1);
	if (!r)
		ts_info("i2c test SUCCESS");
	else {
		ts_err("i2c test FAILED");
		goto out;
	}

	/*unified protocl
	 * start a thread to parse cfg_bin and init IC*/
	r = goodix_start_cfg_bin(core_data);
	if (!r) {
		ts_info("***start cfg_bin_proc SUCCESS");
	} else {
		ts_err("***start cfg_bin_proc FAILED");
		goto out;
	}

	//ASUS_BSP Beryl +++

	core_data->rotation = 0;
	core_data->debug = false;
	atomic_set(&core_data->charge_mode, 0);
	atomic_set(&core_data->glove_mode, 0);
	atomic_set(&core_data->testcfg, 0);
	core_data->atr_enable = false;
        atomic_set(&core_data->game_cfg_set, 0);
	core_data->phone_call_on = false;
	core_data->station_insert = false;
	//ASUS_BSP Jacob ++++
	core_data->disable_fod = false;
	//ASUS_BSP Jacob ----
	atomic_set(&core_data->station_cfg_reload, 0);
	atomic_set(&core_data->rotation_set, 0);
	mutex_init(&core_data->load_read_cfg_lock);
	core_data->gts_workqueue = create_singlethread_workqueue("gts_wq");
	if (NULL == core_data->gts_workqueue) {
            ts_err("failed to create gts workqueue");
        }
        mutex_init(&core_data->gts_suspend_mutex);
	
	core_data->gts_suspend_resume_wq = create_singlethread_workqueue("goodix_suspend_resume_wq");
        if (NULL == core_data->gts_suspend_resume_wq) {
	    ts_err("create suspend/resume workqueue failed");
	    r=-1;
        }

        INIT_WORK(&core_data->gts_resume_work, goodix_resume_work);
        INIT_WORK(&core_data->gts_suspend_work, goodix_suspend_work);
	proc_symlink("driver/glove", NULL, "/sys/devices/platform/goodix_ts.0/glove_mode");
	gts_core_data = core_data;
	init_success=1;
	//ASUS_BSP Beryl ---
out:
	if (r!=0) {
	  ts_err("goodix_ts_probe fail, r:%d", r);
          init_success=0;
	} 
	ts_info("goodix_ts_probe OUT, r:%d", r);
	return r;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data =
		platform_get_drvdata(pdev);

	goodix_ts_power_off(core_data);
	goodix_debugfs_exit();
	goodix_ts_sysfs_exit(core_data);
	return 0;
}

static const struct dev_pm_ops dev_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};

static const struct platform_device_id ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
		//.pm = &dev_pm_ops,
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};


static int __init goodix_ts_core_init(void)
{
	ts_info("Core layer init");

	if (!goodix_modules.initilized) {
	        ts_info("initilize goodix module");
		goodix_modules.initilized = true;
		goodix_modules.core_exit = true;
		INIT_LIST_HEAD(&goodix_modules.head);
		mutex_init(&goodix_modules.mutex);
		init_completion(&goodix_modules.core_comp);
	}
        ts_info("goodix_debugfs_init");
	goodix_debugfs_init();
	return platform_driver_register(&goodix_ts_driver);
}


static void __exit goodix_ts_core_exit(void)
{
	ts_info("Core layer exit");
	platform_driver_unregister(&goodix_ts_driver);
	return;
}

module_init(goodix_ts_core_init);
module_exit(goodix_ts_core_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Core Module");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
