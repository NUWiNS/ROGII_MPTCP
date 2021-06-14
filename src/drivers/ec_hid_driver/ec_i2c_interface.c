#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/msm_drm_notify.h>

#include "ec_i2c_interface.h"

extern void ec_hid_uevent(void);
extern void asus_hid_is_connected(void);
extern uint8_t gDongleType;
extern int is_porta_cc_locked;
extern int is_ec_has_removed;
extern int pogo_sync_key;
extern int ec_i2c_is_suspend;
extern int ec_i2c_ultra_mode;
extern int ec_i2c_driver_state;
extern struct mutex ec_i2c_func_mutex;
extern struct blocking_notifier_head ec_hid_event_header;
// BSP SZ +++ Lydia_Wu add for Station display
extern int lastFps;
extern int get_last_backlight_value(void);
extern int g_station_hbm_mode;
extern void asus_dp_change_state(bool mode, int type);
extern bool g_Charger_mode;
// BSP SZ --- Lydia_Wu add for Station display
char asus_gpio = 0;

// For Station HWID
extern int Station_HWID;

/************************************************************************
* Name: ec_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int ec_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
    int ret = 0;
    int i = 0;

    mutex_lock(&i2c_rw_access);
    if (writelen > 0) {
        struct i2c_msg msgs[] = {
            {
                .addr = client->addr,
                .flags = 0,
                .len = writelen,
                .buf = writebuf,
            },
        };
        for (i = 0; i < I2C_RETRY_NUMBER; i++) {
            ret = i2c_transfer(client->adapter, msgs, 1);
            if (ret < 0) {
                printk("[EC_I2C] : %s:i2c_transfer(write) error, ret=%d", __func__, ret);
            } else
                break;
        }
    }
    mutex_unlock(&i2c_rw_access);

    return ret;
}

/************************************************************************
* Name: ec_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int ec_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
    int ret = 0;
    int i = 0;

    mutex_lock(&i2c_rw_access);

    if (readlen > 0) {
        if (writelen > 0) {
            struct i2c_msg msgs[] = {
                {
                    .addr = client->addr,
                    .flags = 0,
                    .len = writelen,
                    .buf = writebuf,
                },
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };
            for (i = 0; i < I2C_RETRY_NUMBER; i++) {
                ret = i2c_transfer(client->adapter, msgs, 2);
                if (ret < 0) {
                    printk("[EC_I2C] : i2c_transfer(write) error, ret=%d!!", ret);
                } else
                    break;
            }

        } else {
            struct i2c_msg msgs[] = {
                {
                    .addr = client->addr,
                    .flags = I2C_M_RD,
                    .len = readlen,
                    .buf = readbuf,
                },
            };
            for (i = 0; i < I2C_RETRY_NUMBER; i++) {
                ret = i2c_transfer(client->adapter, msgs, 1);
                if (ret < 0) {
                    printk("[EC_I2C] : i2c_transfer(read) error, ret=%d!!", ret);
                } else
                    break;
            }
        }
    }

    mutex_unlock(&i2c_rw_access);
    return ret;
}


int i2c_get_ec_fw_ver(char *ver)
{
	int ret = 0;
	char cmd = CMD_I2C_GET_EC_FW_Version;


	printk("[EC_I2C] i2c_get_ec_fw_ver\n");
	if(ec_i2c_data)
	{
		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,ver,4);
		if(ret < 0)
		{
			printk("[EC_I2C] i2c_get_ec_fw_ver I2C error!!\n");
			ret = -1;
		}
	}
	else
	{
		printk("[EC_I2C] ec_i2c_data does not init\n");
		ret = -1;
	}

	return ret;
}

int ec_i2c_get_porta_cc_state(int *state)
{
	int ret = 0;
	char cmd = CMD_I2C_GET_PORTA_CC_STATE;
	char temp[2]={0};


	printk("[EC_I2C] ec_i2c_get_porta_cc_state\n");

	if(ec_i2c_data)
	{
		printk("[EC_I2C] ec_i2c_data has init\n");
		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,2);
		if(ret < 0)
		{
			printk("[EC_I2C] ec_i2c_get_porta_cc_state I2C error!!\n");
			ec_porta_cc.ec_i2c_get_porta_cc_state = NULL;
			return ret;
		}
	}
	else
	{
		printk("[EC_I2C] ec_i2c_data does not init\n");
		ret = -1;
	}

	*state = temp[1];
	
	printk("[EC_I2C] ec_i2c_get_porta_cc_state state is %d\n",*state);

	return ret;

}

int i2c_set_display_bl(char* brightness)
{
		int ret = 0;
		char cmd[3] = {0};

		printk("[EC_I2C] i2c_set_display_bl brightness : 0x%x,0x%x\n",brightness[0],brightness[1]);

		cmd[0] = CMD_I2C_SET_DISPLAY_BL;
		cmd[1] = brightness[0];
		cmd[2] = brightness[1];

		ret = ec_i2c_write(ec_i2c_data->client,cmd,3);
		
		return ret;
}

//BSP Yadong +++
int i2c_set_hbm(char hbm)
{
	int ret = 0;
	char cmd[2] = {0};

	printk("[EC_I2C] i2c_set_hbm hbm : 0x%x\n",hbm);

	cmd[0] = CMD_I2C_SET_HBM;
	cmd[1] = hbm;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return ret;
}
//BSP Yadong ---

int i2c_set_display_fps(char fps)
{
	int ret = 0;
	char cmd[2] = {0};

	printk("[EC_I2C] i2c_set_display_fps fps : 0x%x\n",fps);

	cmd[0] = CMD_I2C_SET_FPS;
	cmd[1] = fps;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);
	
	return ret;
}

int i2c_control_display(char on)
{
	int ret = 0;
	char cmd[2] = {0};

	printk("[EC_I2C] i2c_control_display on : 0x%x\n",on);

	cmd[0] = CMD_I2C_CONTROL_DISPLAY;
	cmd[1] = on;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);
	
	return ret;
}

int i2c_set_dp_display_id(char display_id)
{
	int ret = 0;
	char cmd[2] = {0};

	printk("[EC_I2C] i2c_set_dp_display_id display_id : 0x%x\n",display_id);

	cmd[0] = CMD_I2C_SET_DP_DISPLAY_ID;
	cmd[1] = display_id;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);
	
	return ret;
}

int i2c_to_gpio_set(u8 gpio, u8 value)
{
	int ret = 0;
	char cmd[3] = {0};

	printk("[EC_I2C] i2c_to_gpio_set : GPIO[%d] : 0x%x\n", gpio, value);
	cmd[0] = CMD_I2C_TO_SET_GPIO;
	cmd[1] = gpio;
	cmd[2] = value;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,3);
	
	return ret;
}

int i2c_get_gpio_data(char *buffer,char gpio)
{
	int ret = 0;
	char cmd = 0;
	char temp[2]={0};

	switch(gpio)
	{
		case 0x12:
			cmd = CMD_I2C_GET_GPIO12;
			break;
		case 0x22:
			cmd = CMD_I2C_GET_GPIO22;
			break;
		case 0x34:
			cmd = CMD_I2C_GET_GPIO34;
			break;
		case 0x35:
			cmd = CMD_I2C_GET_GPIO35;
			break;
		default:
			printk("[EC_I2C] : Unsupport GPIO number 0x%x!\n",gpio);
			return -1;
	}
	
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,2);

	printk("[EC_I2C] i2c_get_gpio_data : 0x%x\n",temp[1]);
	
	*buffer= temp[1];

	return ret;
}


int i2c_notify_ec_enumerate_message(char val)
{
	int ret = 0;
	char cmd[2] = {0};

	cmd[0] = CMD_I2C_NOTIFY_EC_ENUMERATE;
	cmd[1] = val;
	
	if(ec_i2c_data)
	{
		ret = ec_i2c_write(ec_i2c_data->client,cmd,2);
		if(ret < 0)
			printk("[EC_I2C] CMD_I2C_NOTIFY_EC_ENUMERATE wirte fail\n");
	}
	else
	{
		printk("[EC_I2C] ec_i2c_data does not init\n");
		ret = -1;
	}

	return ret;
}

int i2c_get_ec_ready_state(void)
{
	int ret = 0;
	char cmd = CMD_I2C_GET_EC_READY_STATE;
	char temp[2]={0};


	printk("[EC_I2C] i2c_get_ec_ready_state\n");

	if(ec_i2c_data)
	{
		printk("[EC_I2C] ec_i2c_data has init\n");
		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,2);

		gEC_init = temp[1];
	}
	else
	{
		printk("[EC_I2C] ec_i2c_data does not init\n");
		ret = -1;
	}

	printk("[EC_I2C] i2c_get_ec_ready_state gEC_init is %d\n",gEC_init);

	return ret;
}

int i2c_set_ultra_power_mode(u8 type)
{
	int ret = 0;
	char cmd[2] = {0};
	
	cmd[0] = CMD_I2C_SET_COVER_STATE;
	cmd[1] = type;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return ret;
}

int i2c_get_charger_type(int *type, short *vol, short *cur)
{
	int ret = 0;
	char cmd = 0;
	char buffer[6] = {0};

	cmd = CMD_I2C_GET_CHARGER_TYPE;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,6);
	if(ret < 0)
	{
		printk("[EC_I2C] I2C not connect \n");
		return -1 ;
	}
	
	(*type) = buffer[1];

	(*vol) = buffer[2] << 8;
	(*vol) += buffer[3];

	(*cur) = buffer[4] << 8;
	(*cur) += buffer[5];

	return ret;
}

int i2c_get_battery_cap(int *cap)
{
	int ret = 0;
	char cmd  = 0;
	char buffer[2] = {0};

	cmd = CMD_I2C_GET_BATTERY_CAP;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);
	if (ret < 0)
	{
		printk("[EC_I2C] i2c_get_battery_cap I2C not connect \n");
		return -1 ;
	}

	*cap = buffer[1];

	printk("[EC_I2C] : battery_cap is %d\n",*cap);

	return ret;
}


int i2c_get_battery_vol(int *vol)
{
	int ret = 0;
	char cmd = 0;
	char buffer[3] = {0};
	
	cmd = CMD_I2C_GET_BATTERY_VOL;

	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0)
	{
		printk("[EC_I2C] I2C not connect \n");
		return -1 ;
	}


	*vol = buffer[1] << 8;
	*vol += buffer[2];

	return ret;
}

int i2c_get_battery_cur(short *cur)
{
	int ret = 0;
	char cmd = 0;
	char buffer[3] = {0};

	cmd = CMD_I2C_GET_BATTERY_CUR;

	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0)
	{
		printk("[EC_I2C] I2C not connect \n");
		return -1 ;
	}


	*cur = buffer[1] << 8;
	*cur += buffer[2];
	
	return ret;
}

int i2c_check_interrupt(char *type, char *event)
{
	int ret = 0;
	char cmd = CMD_I2C_GET_INT_TYPE;
	char temp[2]={0};

	if(ec_i2c_data)
	{
		printk("[EC_I2C] ec_i2c_data has init\n");
		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,2);

		*type = temp[1];
	}
	else
	{
		printk("[EC_I2C] ec_i2c_data does not init\n");
		ret = -1;
		return ret;
	}

	printk("[EC_I2C] i2c_check_interrupt type is 0x%x\n",*type);

	(*event) = 0;

	if((*type) & 0x1)
	{
		printk("[EC_I2C] Detect SDP\n");
	}

	if((*type) & 0x2)
	{
		printk("[EC_I2C] Detect Thermal alert.\n");
		
		// Get thermal alert
		cmd = CMD_I2C_GET_THERMAL_ALERT;
		temp[0]=0;
		temp[1]=0;

		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,2);

		(*event) = temp[1];
		printk("[EC_I2C] Thermal alert event is 0x%x.\n",*event);
	}

	if((*type) & 0x4)
	{
		printk("[EC_I2C] Ultra Power Mode Wake Up\n");
	}

	if((*type) & 0x40)
	{
		printk("[EC_I2C] Station EC Zero shutdown!!!!\n");
		station_shutdown = true;
	} else
		station_shutdown = false;
	
	return ret;
}

static ssize_t pd_fw_ver_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	u8 fw_ver[3] = {0};
	char cmd = 0;

	printk("[EC_I2C] pd_fw_ver_show\n");

	cmd = CMD_I2C_GET_PD_FW;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,fw_ver,3);
	if (ret < 0)
		return sprintf(buf, "FF\n");

	printk("[EC_I2C] pd_fw_ver_show : %c%c\n",fw_ver[1],fw_ver[2]);
	
	return snprintf(buf, PAGE_SIZE,"%c%c\n", fw_ver[1], fw_ver[2]);
}

static ssize_t set_color_temp_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};

	printk("[EC_I2C] : Enter set_color_temp_store \n");
	
	ret = kstrtou32(data, 10, &val);
	if (ret)
	{
		printk("[EC_I2C] : kstrtou32 ERROR !!\n");
		return count;
	}

	cmd[0] =  CMD_I2C_SET_COLOR_TEMP;
	cmd[1] =  val;


	printk("[EC_I2C] : cmd[1] is %d!\n",cmd[1]);
		
	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t reconnect_dp_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	printk("[EC_I2C] : Enter reconnect_dp_store \n");

	asus_dp_change_state(0,4);
	asus_dp_change_state(1,4);

	return count;
}

static ssize_t restore_display_config_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int bl_level = 1023;
	char bl_ch[2];

	printk("[EC_I2C] : Enter restore_display_config_store \n");

	if (lastFps >= 60 && lastFps < 90)
		i2c_set_display_fps(2);
	else if (lastFps >= 90 && lastFps < 120)
		i2c_set_display_fps(1);
	else if (lastFps == 120)
		i2c_set_display_fps(0);

	bl_level = get_last_backlight_value();
	bl_ch[0]=(char)(bl_level & 0xFF);
	bl_ch[1]=(char)((bl_level >> 8) & 0xFF);
	i2c_set_display_bl((char*)bl_ch);

	if (g_station_hbm_mode == 0) {
		i2c_set_hbm(0);
	} else if (g_station_hbm_mode == 1) {
		i2c_set_hbm(1);
	}

	return count;
}

static ssize_t display_id_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[2] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_PANEL_ID;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);
	if (ret < 0) {
		buffer[1] = 255;
		return sprintf(buf, "%d\n",buffer[1]);
	}

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer[1]);
}

static ssize_t latch_key_state_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int state = 255;
	
	ec_i2c_get_porta_cc_state(&state);

	return snprintf(buf, PAGE_SIZE,"%d\n", state);
}

static ssize_t set_cover_state_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};
	
	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	cmd[0] =  CMD_I2C_SET_COVER_STATE;
	
	if (val > 0) {
		cmd[1] = 1;
	}else {
		cmd[1] = 0;
	}
	
	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t disable_charger_suspend_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};
	
	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	cmd[0] =  CMD_I2C_DISABLE_CHARGER_SUSPEND;
	
	if (val > 0) {
		cmd[1] = 1;
	}else {
		cmd[1] = 0;
	}
	
	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t fps_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[2] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_FPS;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	if (buffer[1])
		printk("[EC_I2C] FPS 90\n");
	else
		printk("[EC_I2C] FPS 60\n");

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer[1]);
}

static ssize_t model_name_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[12] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_MODEL_NAME;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,12);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%s\n", &buffer[1]);
}

static int i2c_get_station_hwid(void)
{
	int ret = 0;
	char buffer[3] = {0};
	char model_name[12] = {0};
	char cmd = 0;

	if (EC_FW_VER < 535) {
		printk("[EC_I2C] EC FW %d is too old, set Station HWID as ROG_Station2\n", EC_FW_VER);
		Station_HWID = ROG_Station2;
		return 0;
	}

	cmd = CMD_I2C_GET_HW_ID;
	
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0) {
		printk("[EC_I2C] I2C not connect, get HWID fail\n");
		Station_HWID = 255;
		return -1;
	}

	if ( (buffer[1] == 1) && (buffer[2] == 1) )
		Station_HWID = ROG_Station2;
	else if ( (buffer[1] == 1) && (buffer[2] == 0) )
		Station_HWID = ROG_Station3;
	else if ( (buffer[1] == 0) && (buffer[2] == 1) )
	{
		printk("[EC_I2C] use model name to check station hwid\n");

		cmd = CMD_I2C_GET_MODEL_NAME;
		ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,model_name,12);
		if (ret < 0)
			printk("[EC_I2C] I2C not connect\n");

		printk("[EC_I2C] model name = %s\n", &model_name[1]);

		if (strncmp(&model_name[1], "ZS660KLS", 8) != 0)
			Station_HWID = ROG_Station3;
		else
			Station_HWID = ROG_Station2;
	}
	else
		Station_HWID = ROG_Station_other;

	printk("[EC_I2C] GPIO status : %#x, %#x, Station HWID = %#x, model name = %s\n", buffer[1], buffer[2], Station_HWID, &model_name[1]);
	return 0;
}

static ssize_t hw_id_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	if (i2c_get_station_hwid() < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%#x\n", Station_HWID);
}

static ssize_t ec_ssn_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[23] = {0};
	char temp[12] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_EC_SSN_L;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,12);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	strncpy(buffer,&temp[1],11);

	memset(temp,0,12);

	cmd = CMD_I2C_GET_EC_SSN_H;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,temp,12);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");
	
	strncpy(&buffer[11],&temp[1],11);

	return snprintf(buf, PAGE_SIZE,"%s\n", buffer);
}

#ifdef ASUS_FACTORY_BUILD
static ssize_t ec_ssn_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	char cmd[13] = {0};
	char temp[22] = {0};

	if(count > 16)
	{
		printk("[EC_I2C] ec_ssn_store count is %d too large,Please check data\n",count);
		ret = -1;
		return ret;
	}

	strcpy(temp,data);
	
	printk("[EC_I2C] ec_ssn_store data is %s,count is%d\n",data,count);
	printk("[EC_I2C] ec_ssn_store temp is %s\n",temp);

	cmd[0] =  CMD_I2C_SET_EC_SSN;
	cmd[1] =  1;

	strncpy(&cmd[2],temp,11);
	
	printk("[EC_I2C] ASUS COMMAND ID is 0x%x,count is %d\n",cmd[0],count);

	printk("[EC_I2C] cmd[1] is %d \n",cmd[1]);

	printk("[EC_I2C] str_data is %c \n",cmd[12]);

	ret = ec_i2c_write(ec_i2c_data->client,cmd,13);

	memset(cmd,0,13);
	
	cmd[0] =  CMD_I2C_SET_EC_SSN;
	cmd[1] =  2;

	strncpy(&cmd[2],&temp[11],11);

	printk("[EC_I2C] ASUS COMMAND ID is 0x%x,count is %d\n",cmd[0],count);

	printk("[EC_I2C] cmd[1] is %d \n",cmd[1]);

	printk("[EC_I2C] str_data is %s \n",&cmd[2]);

	ret = ec_i2c_write(ec_i2c_data->client,cmd,13);
	
	return count;
}

static ssize_t model_name_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	char cmd[12] = {0};

	if(count > 11)
	{
		printk("[EC_I2C] model_name_store count is %d too large,Please check data\n",count);
		ret = -1;
		return ret;
	}

	cmd[0] =  CMD_I2C_WRITE_MODEL_NAME;
	
	strncpy(&cmd[1],data,count);

	printk("[EC_I2C] model_name_store cmd[0] is 0x%x\n",cmd[0]);
	printk("[EC_I2C] model_name_store cmd is %s\n",&cmd[1]);

	ret = ec_i2c_write(ec_i2c_data->client,cmd,12);

	return count;
}

#endif
static ssize_t battery_48_hours_state_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};
	
	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	cmd[0] =  CMD_I2C_SET_BATTERY_48H_STATE;
	
	if (val > 0) {
		cmd[1] = 1;
	}else {
		cmd[1] = 0;
	}
	
	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t battery_48_hours_state_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[2] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_BATTERY_48H_STATE;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);

	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer[1]);
}

static ssize_t pwm_freq_duty_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char cmd = 0;
	char buffer[3] = {0};

	cmd = CMD_I2C_GET_PWM_FRQ_DUTY;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C_not_connect");
	
	return snprintf(buf, PAGE_SIZE,"freq : %d KHz, duty : %d \n", buffer[1],buffer[2]);
}

static ssize_t init_state_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{	
	i2c_get_ec_ready_state();

	return snprintf(buf, PAGE_SIZE,"%x\n", gEC_init);
}

static ssize_t init_state_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		printk("[EC_I2C] Send EC init cmd\n");
		
		gEC_init = 0;
	
		i2c_notify_ec_enumerate_message(0x01);
		
		printk("[EC_I2C] gEC_init is %d\n",gEC_init);
		
	}else {
		printk("[EC_I2C] No Send EC init cmd\n");
		return count;
	}

	return count;
}

static ssize_t uart_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	char buffer = 0;
	int ret = 0;


	ret = i2c_get_gpio_data(&buffer,EC_UART_GPIO);

	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	if(buffer)
		return snprintf(buf, PAGE_SIZE,"0x%x\n", 0);
	else
		return snprintf(buf, PAGE_SIZE,"0x%x\n", 1);
}

static ssize_t uart_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		printk("[EC_I2C] Enable EC Uart.\n");
		ret = i2c_to_gpio_set(EC_UART_GPIO, 0);
	}else {
		printk("[EC_I2C] Disable EC Uart\n");
		ret = i2c_to_gpio_set(EC_UART_GPIO, 1);
	}

	return count;
}

static ssize_t dp_fw_ver_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	u8 fw_ver[3] = {0};
	char cmd = 0;

	printk("[EC_I2C] dp_fw_ver_show\n");

	cmd = CMD_I2C_GET_DP_FW;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,fw_ver,3);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C_not_connect");

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", fw_ver[1], fw_ver[2]);
}

static ssize_t INT_check_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char type = 0;
	char event = 0;

	ret = i2c_check_interrupt(&type, &event);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"type:%d, event:%d\n", type, event);
}

static ssize_t sd_power_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	u8 gpio = 0x1;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		printk("[EC_I2C] Enable SD power.\n");
		ret = i2c_to_gpio_set(gpio, 0);
	}else {
		printk("[EC_I2C] Disable SD power\n");
		ret = i2c_to_gpio_set(gpio, 1);
	}

	return count;
}

static ssize_t sd_power_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	char buffer =0;
	int ret = 0;
	u8 gpio = 0x1;


	ret = i2c_get_gpio_data(&buffer, gpio);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	if(buffer > 0)
		return snprintf(buf, PAGE_SIZE,"0x%x\n", 0);
	else
		return snprintf(buf, PAGE_SIZE,"0x%x\n", 1);
}

static ssize_t PDO_current_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	int type;
	short vol, cur;

	ret = i2c_get_charger_type(&type, &vol, &cur);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", cur);
}

static ssize_t PDO_voltage_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	int type;
	short vol, cur;

	ret = i2c_get_charger_type(&type, &vol, &cur);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", vol);
}

static ssize_t factory_mode_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};
	
	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	cmd[0] =  CMD_I2C_SET_FACTORY_MODE;
	
	if (val > 0) {
		cmd[1] = 1;
	}else {
		cmd[1] = 0;
	}
	
	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t factory_mode_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer[2] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_FACTORY_MODE;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);

	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer[1]);
}

static ssize_t thermal_alert_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char cmd = 0;
	char buffer[2] = {0};
	int state;

	cmd = CMD_I2C_GET_THERMAL_ALERT;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	state = (int)buffer[1];
	
	return snprintf(buf, PAGE_SIZE,"%d\n", state);
}

static ssize_t battery_cur_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	short cur;
	char cmd = 0;
	char buffer[3] = {0};

	cmd = CMD_I2C_GET_BATTERY_CUR;

	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	cur = buffer[1] << 8;
	cur += buffer[2];
	
	return snprintf(buf, PAGE_SIZE,"%d\n", cur);
}

static ssize_t battery_vol_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char cmd = 0;
	char buffer[3] = {0};
	int vol = 0;
	
	cmd = CMD_I2C_GET_BATTERY_VOL;

	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	vol = buffer[1] << 8;
	vol += buffer[2];

	return snprintf(buf, PAGE_SIZE,"%d\n", vol);
}

static ssize_t battery_cap_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char cmd  = 0;
	char buffer[2] = {0};

	cmd = CMD_I2C_GET_BATTERY_CAP;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer[1]);
}

static ssize_t charger_type_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	int type;
	short vol, cur;

	ret = i2c_get_charger_type(&type, &vol, &cur);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", type);
}

static ssize_t enable_mipi_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;
	char cmd[2] = {0};

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	cmd[0] = CMD_I2C_ENABLE_MIPI;

	if (val > 0) {
		printk("[EC_I2C] Send enable MIPI cmd\n");
		cmd[1] = 1;
	}else {
		printk("[EC_I2C]  disable MIP cmd\n");
		cmd[1] = 0;
	}

	ret=ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t duty_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int duty = 0;
	char cmd[3] = {0};
	int ret = 0;

	sscanf(data, "%d", &duty);

	cmd[0] = CMD_I2C_SET_DUTY;
	cmd[1] = duty;
	
	ret=ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t freq_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int freq = 0 ;
	char cmd[3] = {0};
	int ret = 0;

	sscanf(data, "%d", &freq);
	
	cmd[0] = CMD_I2C_SET_FREQ;
	cmd[1] = freq;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}

static ssize_t rpm_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	int RPM = 0;
	char buffer[3] = {0};
	char cmd = 0;
	
	cmd = CMD_I2C_GET_RPM;
	printk("[EC_I2C] rpm_show 0x%x\n",cmd);
	
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);

	RPM = buffer[1] << 8;
	RPM += buffer[2];

	
	printk("[EC_I2C] rpm_show : %d\n", RPM);


	return snprintf(buf, PAGE_SIZE,"%d\n", RPM);
}

static ssize_t pwm_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{

	int ret = 0;
	char cmd[2] = {0};
	u32 val;

	printk("[EC_I2C] pwm_store \n");

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if(val > 0)
	{
		cmd[0] = CMD_I2C_ENABLE_PWM;
		cmd[1] = 1;
	} else {
		cmd[0] = CMD_I2C_ENABLE_PWM;
		cmd[1] = 0;
	}

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);
	
	return count;
}

int get_ec_fw_ver(void)
{
	int ret = 0;
	char buffer[4] = {0};
	char cmd = 0;

	cmd = CMD_I2C_GET_EC_FW_Version;
	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,4);
	if (ret < 0) {
		printk("[EC_I2C] CMD_I2C_GET_EC_FW_Version I2C error!!!\n");
		return ret;
	}

	sscanf((buffer+1), "%d", &EC_FW_VER);
	printk("[EC_I2C] EC_FW_VER: %d\n", EC_FW_VER);
	return ret;
}

static ssize_t fw_ver_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;

	ret = get_ec_fw_ver();
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", EC_FW_VER);
}

static ssize_t get_gpio_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{

	printk("[EC_I2C] get_gpio_store\n");
	
	sscanf(data, "%x", &asus_gpio);

	printk("[EC_I2C] asus_gpio is 0x%x\n",asus_gpio);

	return count;
}

static ssize_t get_gpio_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	char buffer = 0;

	printk("[EC_I2C] get_gpio_show\n");

	ret = i2c_get_gpio_data(&buffer,asus_gpio);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return sprintf(buf, "0x%x\n", buffer);
}

static ssize_t set_gpio_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret=0;
	int gpio = 0, gpio_value = 0;

	sscanf(data, "%x %x", &gpio, &gpio_value);

	ret = i2c_to_gpio_set((u8)gpio, (u8)gpio_value);

	printk("[EC_I2C] set_gpio_store : ret %d\n", ret);
	return count;
}

int i2c_get_state(char *state)
{
	int ret = 0;
	char buffer[3] = {0};
	char cmd = 0;

	cmd = CMD_I2C_STATE;
	printk("[EC_I2C] i2c_get_state 0x%x\n",cmd);

	ret = ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,3);

	state = buffer;
	
	printk("[EC_I2C] i2c_get_state : 0x%x 0x%x\n",state[1],state[2]);

	return ret;
}

int hid_to_get_u0504_state(int *state)
{
	int ret = 0;
	char buffer[2] = {0};
	char cmd = 0;

	cmd = CMD_I2C_U0504_GET_STATE;
	printk("[EC_I2C] hid_to_get_u0504_state 0x%x\n",cmd);

	ec_i2c_read(ec_i2c_data->client,&cmd,1,buffer,2);

	*state = (int)buffer[1];
	
	printk("[EC_I2C] u0504 state : %d\n", (*state));

	return ret;
}
EXPORT_SYMBOL(hid_to_get_u0504_state);


static ssize_t u0504_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	int ret = 0;
	int state;

	ret = hid_to_get_u0504_state(&state);
	if (ret < 0)
		return sprintf(buf, "%s\n", "I2C not connect");

	return snprintf(buf, PAGE_SIZE,"%d\n", state);
}


static ssize_t disconnect_porta_cc_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	char cmd[2] = {0};
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return count;

	cmd[0] = CMD_I2C_DISCONNECT_PORTA_CC;
	cmd[1] = 1;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}


static ssize_t connect_porta_cc_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	char cmd[2] = {0};
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return count;

	cmd[0] = CMD_I2C_CONNECT_PORTA_CC;
	cmd[1] = 1;

	ret = ec_i2c_write(ec_i2c_data->client,cmd,2);

	return count;
}


static DEVICE_ATTR(u0504, S_IRUGO | S_IWUSR, u0504_show, NULL);
static DEVICE_ATTR(set_gpio, S_IRUGO | S_IWUSR, NULL, set_gpio_store);
static DEVICE_ATTR(get_gpio, S_IRUGO | S_IWUSR, get_gpio_show, get_gpio_store);
static DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, fw_ver_show, NULL);
static DEVICE_ATTR(pwm, S_IRUGO | S_IWUSR, NULL, pwm_store);
static DEVICE_ATTR(rpm, S_IRUGO | S_IWUSR, rpm_show, NULL);
static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, NULL, freq_store);
static DEVICE_ATTR(duty, S_IRUGO | S_IWUSR, NULL, duty_store);
static DEVICE_ATTR(enable_mipi, S_IRUGO | S_IWUSR, NULL, enable_mipi_store);
static DEVICE_ATTR(charger_type, S_IRUGO | S_IWUSR, charger_type_show, NULL);
static DEVICE_ATTR(battery_cap, S_IRUGO | S_IWUSR, battery_cap_show, NULL);
static DEVICE_ATTR(battery_vol, S_IRUGO | S_IWUSR, battery_vol_show, NULL);
static DEVICE_ATTR(battery_cur, S_IRUGO | S_IWUSR, battery_cur_show, NULL);
static DEVICE_ATTR(thermal_alert, S_IRUGO | S_IWUSR, thermal_alert_show, NULL);
static DEVICE_ATTR(factory_mode, S_IRUGO | S_IWUSR, factory_mode_show, factory_mode_store);
static DEVICE_ATTR(StationPDO_voltage_max, S_IRUGO | S_IWUSR, PDO_voltage_show, NULL);
static DEVICE_ATTR(StationPDO_current_max, S_IRUGO | S_IWUSR, PDO_current_show, NULL);
static DEVICE_ATTR(sd_power, S_IRUGO | S_IWUSR, sd_power_show, sd_power_store);
static DEVICE_ATTR(INT_check, S_IRUGO | S_IWUSR, INT_check_show, NULL);
static DEVICE_ATTR(DP_FW, S_IRUGO | S_IWUSR, dp_fw_ver_show, NULL);
static DEVICE_ATTR(uart, S_IRUGO | S_IWUSR, uart_show, uart_store);
static DEVICE_ATTR(init_state, S_IRUGO | S_IWUSR, init_state_show, init_state_store);
static DEVICE_ATTR(pwm_freq_duty, S_IRUGO | S_IWUSR, pwm_freq_duty_show, NULL);
static DEVICE_ATTR(battery_48_hours_state, S_IRUGO | S_IWUSR, battery_48_hours_state_show, battery_48_hours_state_store);
#ifdef ASUS_FACTORY_BUILD
static DEVICE_ATTR(ec_ssn, S_IRUGO | S_IWUSR, ec_ssn_show, ec_ssn_store);
static DEVICE_ATTR(model_name, S_IRUGO | S_IWUSR, model_name_show, model_name_store);
#else
static DEVICE_ATTR(ec_ssn, S_IRUGO | S_IWUSR, ec_ssn_show, NULL);
static DEVICE_ATTR(model_name, S_IRUGO | S_IWUSR, model_name_show, NULL);
#endif
static DEVICE_ATTR(hw_id, S_IRUGO | S_IWUSR, hw_id_show, NULL);
static DEVICE_ATTR(fps, S_IRUGO | S_IWUSR, fps_show, NULL);
static DEVICE_ATTR(disable_charger_suspend, S_IRUGO | S_IWUSR, NULL, disable_charger_suspend_store);
static DEVICE_ATTR(set_cover_state, S_IRUGO | S_IWUSR, NULL, set_cover_state_store);
static DEVICE_ATTR(latch_key_state, S_IRUGO | S_IWUSR, latch_key_state_show, NULL);
static DEVICE_ATTR(display_id, S_IRUGO | S_IWUSR, display_id_show, NULL);
static DEVICE_ATTR(reconnect_dp, S_IRUGO | S_IWUSR, NULL, reconnect_dp_store);
static DEVICE_ATTR(disconnect_porta_cc, S_IRUGO | S_IWUSR, NULL, disconnect_porta_cc_store);
static DEVICE_ATTR(connect_porta_cc, S_IRUGO | S_IWUSR, NULL, connect_porta_cc_store);
static DEVICE_ATTR(set_color_temp, S_IRUGO | S_IWUSR, NULL, set_color_temp_store);
static DEVICE_ATTR(restore_display_config, S_IRUGO | S_IWUSR, NULL, restore_display_config_store);
static DEVICE_ATTR(PD_FW, S_IRUGO | S_IWUSR, pd_fw_ver_show, NULL);


static struct attribute *ec_i2c_attrs[] = {
	&dev_attr_set_gpio.attr,
	&dev_attr_get_gpio.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_pwm.attr,
	&dev_attr_rpm.attr,
	&dev_attr_freq.attr,
	&dev_attr_duty.attr,
	&dev_attr_enable_mipi.attr,
	&dev_attr_charger_type.attr,
	&dev_attr_battery_cap.attr,
	&dev_attr_battery_vol.attr,
	&dev_attr_battery_cur.attr,
	&dev_attr_thermal_alert.attr,
	&dev_attr_u0504.attr,
	&dev_attr_factory_mode.attr,
	&dev_attr_StationPDO_voltage_max.attr,
	&dev_attr_StationPDO_current_max.attr,
	&dev_attr_sd_power.attr,
	&dev_attr_INT_check.attr,
	&dev_attr_DP_FW.attr,
	&dev_attr_uart.attr,
	&dev_attr_init_state.attr,
	&dev_attr_pwm_freq_duty.attr,
	&dev_attr_battery_48_hours_state.attr,
	&dev_attr_ec_ssn.attr,
	&dev_attr_hw_id.attr,
	&dev_attr_model_name.attr,
	&dev_attr_fps.attr,
	&dev_attr_disable_charger_suspend.attr,
	&dev_attr_set_cover_state.attr,
	&dev_attr_latch_key_state.attr,
	&dev_attr_display_id.attr,
	&dev_attr_reconnect_dp.attr,
	&dev_attr_disconnect_porta_cc.attr,
	&dev_attr_connect_porta_cc.attr,
	&dev_attr_set_color_temp.attr,
	&dev_attr_restore_display_config.attr,
	&dev_attr_PD_FW.attr,
	NULL
};

const struct attribute_group ec_i2c_group = {
	.attrs = ec_i2c_attrs,
};

/************************************************************************
* Name: ec_i2c_create_sysfs
* Brief: create sysfs interface
* Input:
* Output:
* Return: return 0 if success
***********************************************************************/
int ec_i2c_create_sysfs(struct i2c_client *client)
{
    int ret = 0;
	
	ec_i2c_class = class_create(THIS_MODULE, I2C_CLASS_NAME);
	if (IS_ERR(ec_i2c_class)) {
		printk("[EC_I2C] ec_i2c_create_sysfsis failed - unregister chrdev.\n");
	}

	device_create(ec_i2c_class, &client->dev,
			    ec_i2c_data->devt, ec_i2c_data, "dongle");

    ret = sysfs_create_group(&client->dev.kobj, &ec_i2c_group);
    if (ret) {
        printk("[EC_I2C]: sysfs_create_group() failed!!");
        sysfs_remove_group(&client->dev.kobj, &ec_i2c_group);
        return -ENOMEM;
    } else {
        printk("[EC_I2C]: sysfs_create_group() succeeded!!");
    }

    return ret;
}

static int ec_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	printk("[EC_I2C] ec_i2c_remove !\n");
	
	gEC_init = 0;
	station_shutdown = false;

	Station_HWID = ROG_Station_none;
	printk("[EC_I2C] Clean Station HWID as %#x !\n", ROG_Station_none);

	complete_all(&hid_state);
	msleep(50);
	reinit_completion(&hid_state);

	sysfs_remove_group(&client->dev.kobj, &ec_i2c_group);

	device_destroy(ec_i2c_class,ec_i2c_data->devt);
	 
	class_destroy(ec_i2c_class);

	mutex_lock(&ec_i2c_func_mutex);

	ec_check_int.i2c_check_interrupt = NULL;

	ec_set_gpio.i2c_to_gpio_set = NULL;

	ec_get_gpio.i2c_get_gpio_data = NULL;

	ec_battery_func.i2c_get_battery_cap = NULL;
	ec_battery_func.i2c_get_battery_cur = NULL;
	ec_battery_func.i2c_get_battery_vol = NULL;
	ec_battery_func.i2c_get_charger_type = NULL;
	ec_battery_func.i2c_set_ultra_power_mode = NULL;

	ec_set_dp_display.i2c_set_dp_display_id = NULL;
	ec_set_dp_display.i2c_set_display_bl = NULL;
	ec_set_dp_display.i2c_control_display = NULL;
	ec_set_dp_display.i2c_set_display_fps = NULL;
	ec_set_dp_display.i2c_set_hbm = NULL;

	ec_porta_cc.ec_i2c_get_porta_cc_state = NULL;

	ec_fw_ver.i2c_get_ec_fw_ver = NULL;
	
	mutex_unlock(&ec_i2c_func_mutex);

	is_porta_cc_locked = 0;
	pogo_sync_key = 0;
	is_ec_has_removed = 255;

	ec_i2c_data = NULL;

	ec_i2c_driver_state = 0;

	ec_i2c_is_suspend = 0;
	ec_i2c_ultra_mode = 0;

	printk("[EC_I2C] ec_i2c_remove is %d !\n",ec_i2c_driver_state);
	
	return err;
}


static int ec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	u8 retry=0;
	int state = 255;
	struct ec_i2c_platform_data *platform_data;
	// BSP SZ +++ Lydia_Wu add for Station display
    int bl_level = 1023;
    char bl_ch[2];
    // BSP SZ --- Lydia_Wu add for Station display

	printk("[EC_I2C] ec_i2c_probe !\n");

	station_shutdown = false;
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
       		printk("[EC_I2C] i2c_check_functionality error !\n");
       		return -ENODEV;
    }

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ec_i2c_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	
	ec_i2c_data = platform_data;
	platform_data->client = client;


    err = ec_i2c_create_sysfs(client);
    if (err) {
        printk("[EC_I2C]create sysfs node fail");
    }

	if (station_shutdown) {
		printk("[EC_I2C] station_shutdown %d\n", station_shutdown);
		return 0;
	}
	
	do {
		i2c_notify_ec_enumerate_message(0x01);
		i2c_get_ec_ready_state();
		printk("[EC_I2C] EC init state %d\n", gEC_init);
		msleep(100);
		retry++;
	} while(!gEC_init && retry < 10);

	if (gEC_init) {
		i2c_to_gpio_set(EC_UART_GPIO, 1); // Disable EC uart
		complete_all(&hid_state);
	}else
		printk("[EC_I2C] EC init fail!\n");

	mutex_lock(&ec_i2c_func_mutex);

	ec_check_int.i2c_check_interrupt = i2c_check_interrupt;

	ec_set_gpio.i2c_to_gpio_set = i2c_to_gpio_set;

	ec_get_gpio.i2c_get_gpio_data = i2c_get_gpio_data;
	
	ec_battery_func.i2c_get_battery_cap = i2c_get_battery_cap;
	ec_battery_func.i2c_get_battery_cur = i2c_get_battery_cur;
	ec_battery_func.i2c_get_battery_vol = i2c_get_battery_vol;
	ec_battery_func.i2c_get_charger_type = i2c_get_charger_type;
	ec_battery_func.i2c_set_ultra_power_mode = i2c_set_ultra_power_mode;

	ec_set_dp_display.i2c_set_dp_display_id = i2c_set_dp_display_id;
	ec_set_dp_display.i2c_set_display_bl = i2c_set_display_bl;
	ec_set_dp_display.i2c_control_display = i2c_control_display;
	ec_set_dp_display.i2c_set_display_fps = i2c_set_display_fps;
	ec_set_dp_display.i2c_set_hbm = i2c_set_hbm;


	ec_porta_cc.ec_i2c_get_porta_cc_state = ec_i2c_get_porta_cc_state;

	ec_fw_ver.i2c_get_ec_fw_ver = i2c_get_ec_fw_ver;

	mutex_unlock(&ec_i2c_func_mutex);

	get_ec_fw_ver();
	i2c_get_station_hwid();

	asus_hid_is_connected();

    // BSP SZ +++ Lydia_Wu add for Station display
	if (!g_Charger_mode) {
		if (lastFps >= 60 && lastFps < 90)
			i2c_set_display_fps(2);
		else if (lastFps >= 90 && lastFps < 120)
			i2c_set_display_fps(1);
		else if (lastFps == 120)
			i2c_set_display_fps(0);
	}else {
		printk("[EC_I2C] Is in charger mode.\n");
	}

    bl_level = get_last_backlight_value();
    bl_ch[0]=(char)(bl_level & 0xFF);
    bl_ch[1]=(char)((bl_level >> 8) & 0xFF);
    i2c_set_display_bl((char*)bl_ch);

    if (g_station_hbm_mode == 0) {
        i2c_set_hbm(0);
    } else if (g_station_hbm_mode == 1) {
        i2c_set_hbm(1);
    }

    i2c_control_display(1);
    // BSP SZ --- Lydia_Wu add for Station display

	ec_i2c_get_porta_cc_state(&state);
	
	printk("[EC_I2C] : state is %d,is_porta_cc_locked is %d\n",state,is_porta_cc_locked);
	
	if(1 == state){
		if(is_porta_cc_locked == 0){
			printk("[EC_I2C] ec_i2c_probe PortA CC is connect\n");
			is_porta_cc_locked = 1 ;
			gDongleType = 2;
			blocking_notifier_call_chain(&ec_hid_event_header,gDongleType,NULL);
			ec_hid_uevent();
		}
	} else{
		is_porta_cc_locked = 0;
		printk("[EC_I2C] ec_i2c_probe PortA CC is disconnect\n");
	}

	ec_i2c_driver_state = 1 ;
	
	printk("[EC_I2C] ec_i2c_probe is :%d\n",ec_i2c_driver_state);

	
	return err;
	
}

int ec_i2c_suspend(struct device *dev)
{
	int err = 0;

	ec_i2c_is_suspend = 1;
	printk("[EC_I2C] ec_i2c_suspend : %d!\n",ec_i2c_is_suspend);

	return err;
}

int ec_i2c_resume(struct device *dev)
{
	int err = 0;

	ec_i2c_is_suspend = 0;
	printk("[EC_I2C] ec_i2c_resume : %d!\n",ec_i2c_is_suspend);

	if (ec_i2c_ultra_mode == 1)
	{
		printk("[EC_I2C] do ultra mode here\n");
		i2c_set_ultra_power_mode(0);
	}

	ec_i2c_ultra_mode = 0;
	
	return err;
}

static const struct i2c_device_id ec_i2c_id[] = {
	{ "ec_i2c", 0},
	{},
};

static const struct dev_pm_ops ec_i2c_pm_ops = {
	.suspend	= ec_i2c_suspend,
	.resume		= ec_i2c_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ec_i2c_match_table[] = {
	{ .compatible = "ec_i2c",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ec_i2c_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name		= "ec_i2c",
		.owner = THIS_MODULE,
		.pm	= &ec_i2c_pm_ops,
		.of_match_table	= ec_i2c_match_table,
	},
	.probe		= ec_i2c_probe,
	.remove		= ec_i2c_remove,
	.id_table 	= ec_i2c_id,
};


static int __init ec_i2c_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ec_i2c_driver);
	if (ret)
		printk("[EC_I2C] EC_I2C driver init failed.\n");
	else
		printk("[EC_I2C] EC_I2C driver init success.\n");
	
	return ret;
}
module_init(ec_i2c_bus_init);

static void __exit ec_i2c_bus_exit(void)
{
	i2c_del_driver(&ec_i2c_driver);
}
module_exit(ec_i2c_bus_exit);

MODULE_AUTHOR("ASUS Lotta Lu");
MODULE_DESCRIPTION("EC I2C Interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ec i2c");

