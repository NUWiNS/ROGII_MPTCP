/*
ASP1690E ADC Driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
//#include <linux/hwmon.h>
//#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
//#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/kernel.h>
#include <linux/types.h>
//#include <linux/spmi.h>
//#include <linux/fs.h>
#include <linux/cdev.h>
//#include <linux/semaphore.h>
#include <linux/device.h>
//#include <linux/syscalls.h>
//#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
/*#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/wakelock.h>*/

//ASUS charger BSP : global asp1690e_READY +++
bool asp1690e_ready = false;
EXPORT_SYMBOL(asp1690e_ready);

//Define register addresses of asp1690e 0x39
#define asp1690e_raddr	0x39

struct asp1690e_data
{
	u32 asp1690e;
};

struct i2c_client *asp1690e_client;

/*
asp1690e_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/
int asp1690e_write_reg(uint8_t cmd_reg, uint8_t write_val)
{
	int ret = 0;

	printk("[BAT][CHG] asp1690e_write_reg start\n");
	//asp1690e_client->addr = asp1690e_raddr; //real SMBus address (8 bits)
	ret = i2c_smbus_write_byte_data(asp1690e_client, cmd_reg, write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, asp1690e_raddr);
	}

	return ret;
}
EXPORT_SYMBOL(asp1690e_write_reg);

/*
asp1690e_mask_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/
int asp1690e_mask_write_reg(uint8_t cmd_reg, uint8_t mask, uint8_t write_val)
{
	int ret = 0;
	uint8_t m_write_val;

	printk("[BAT][CHG] asp1690e_mask_write_reg start\n");
	//asp1690e_client->addr = asp1690e_raddr; //real SMBus address (8 bits)
	ret = i2c_smbus_read_byte_data(asp1690e_client, cmd_reg);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, asp1690e_raddr);
	}

	m_write_val = (uint8_t)ret & ~mask;
	m_write_val |= write_val & mask;
	
	ret = i2c_smbus_write_byte_data(asp1690e_client, cmd_reg, m_write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, asp1690e_raddr);
	}

	return ret;
}
EXPORT_SYMBOL(asp1690e_mask_write_reg);

/*
asp1690e_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
store_read_val  :	value be read will store here

*/
int asp1690e_read_reg(uint8_t cmd_reg, uint8_t *store_read_val)
{
	int ret = 0;

	//asp1690e_client->addr = asp1690e_raddr;
	ret = i2c_smbus_read_byte_data(asp1690e_client, cmd_reg);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, asp1690e_raddr);
	}

	*store_read_val = (uint8_t) ret;

	return ret;
}
EXPORT_SYMBOL(asp1690e_read_reg);

static ssize_t adapter_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	int ret = 0;
	u8 val[] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

	ret = asp1690e_read_reg(0x30,&val[0]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x31,&val[1]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x3C,&val[2]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x3D,&val[3]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x3E,&val[4]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x3F,&val[5]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x41,&val[6]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x42,&val[7]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x44,&val[8]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x45,&val[9]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x46,&val[10]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x47,&val[11]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	ret = asp1690e_read_reg(0x48,&val[12]);
	if (ret < 0)
		return sprintf(buf, "ERR");

	return sprintf(buf, "Reg:0x30 = 0x%xh\nReg:0x31 = 0x%xh\nReg:0x3C = 0x%xh\nReg:0x3D = 0x%xh\n"
		"Reg:0x3E = 0x%xh\nReg:0x3F = 0x%xh\nReg:0x41 = 0x%xh\nReg:0x42 = 0x%xh\nReg:0x44 = 0x%xh\n"
		"Reg:0x45 = 0x%xh\nReg:0x46 = 0x%xh\nReg:0x47 = 0x%xh\nReg:0x48 = 0x%xh\n",
		val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12]);
}

static ssize_t adc_ack_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u8 val = 0;
	int ret = 0;
	bool ack = 1;

	ret = asp1690e_read_reg(0x43, &val);
	if (ret < 0)
		ack = 0;
	else
		ack = 1;

	return sprintf(buf, "%d\n", ack);
}

static DEVICE_ATTR(adapter_value, 0664, adapter_value_show, NULL);
static DEVICE_ATTR(adc_ack, 0664, adc_ack_show, NULL);

static struct attribute *dump_reg_attrs[] = {
	&dev_attr_adapter_value.attr,
	&dev_attr_adc_ack.attr,
	NULL
};

static const struct attribute_group dump_reg_attr_group = {
	.attrs = dump_reg_attrs,
};

static int asp1690e_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct asp1690e_data *data;
	int rc;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);

	asp1690e_ready = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[BAT][CHG] %s: i2c bus does not support the asp1690e\n", __FUNCTION__);
	}

	data = devm_kzalloc(&client->dev, sizeof(struct asp1690e_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	asp1690e_client = client;
	i2c_set_clientdata(client, data);
	asp1690e_client->addr = asp1690e_raddr;

	rc = sysfs_create_group(&client->dev.kobj, &dump_reg_attr_group);
	if (rc)
		goto exit_remove;

	asp1690e_mask_write_reg(0x31,0xC0,0xC0);
	asp1690e_write_reg(0x3C,0xFF);
	asp1690e_write_reg(0x3D,0x31);
	asp1690e_write_reg(0x3E,0xFF);
	asp1690e_write_reg(0x3F,0x31);

	asp1690e_ready = 1;

	printk("[BAT][CHG] %s end\n", __FUNCTION__);

	return 0;

exit_remove:
		sysfs_remove_group(&client->dev.kobj, &dump_reg_attr_group);
	return rc;

}

static struct of_device_id asp1690e_match_table[] = {
	{ .compatible = "asp1690e",},
	{ },
};

static const struct i2c_device_id asp1690e_id[] = {
	{ "asp1690e", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asp1690e_id);

static struct i2c_driver asp1690e_driver = {
	.driver = {
		.name = "asp1690e",
		.owner		= THIS_MODULE,
		.of_match_table	= asp1690e_match_table,
	},
	.probe = asp1690e_probe,
	.id_table = asp1690e_id,
};

module_i2c_driver(asp1690e_driver);

MODULE_DESCRIPTION("asp1690e driver");
MODULE_LICENSE("GPL");

