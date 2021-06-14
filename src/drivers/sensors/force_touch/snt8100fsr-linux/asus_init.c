#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "asus_init.h"
struct delayed_work RST_WORK;

int SNT_SUSPEND_FLAG = 1;
int write_fail_count = 0;
int write_fail_reset_trigger = 1;

//Reset Function
void check_i2c_error(void){
	//ASUSEvtlog("[Grip] read/write fail, count=%d\n", write_fail_count);
	write_fail_count++;
	if(write_fail_count >= write_fail_reset_trigger){
#ifdef FACTORY_FLAG
#else
	  ASUSEvtlog("[Grip] read/write fail, count=%d, call reset function\n", write_fail_count);
    		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(0));
#endif
		/* reset write_fail_count in Reset_func */
		//write_fail_count = 0;
	}
}

void set_pinctrl(struct device *dev, char *str)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	PRINT_INFO("Set_pinctrl start!");
	key_pinctrl = devm_pinctrl_get(dev);
	if(key_pinctrl!=NULL){
		set_state = pinctrl_lookup_state(key_pinctrl, str);
		if(set_state!=NULL){
			//dev_info("pinctrl_lookup_state: set_state=%s", set_state->name));
			ret = pinctrl_select_state(key_pinctrl, set_state);
			if(ret < 0){
				PRINT_ERR("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
			}
			PRINT_INFO("Set_pinctrl done!");
		}
	} else {
		PRINT_ERR("pinctrl_lookup_state: key_pinctrl=NULL!");
	}
}

void set_1V2_2V8_pin_func(struct work_struct *work_orig) {
    	uint16_t reg_chidlsb, reg_chidmsb;
	PRINT_INFO("Set pinctl: SOC GPIO21 pull-up");
	set_pinctrl(snt8100fsr_g->dev, GRIP_SOC_GPIO21_ON_LOOKUP_STATE);
	msleep(500);
	PRINT_INFO("Set pinctl: PM8150B GPIO12 pull-up");
	set_pinctrl(snt8100fsr_g->dev, GRIP_PM8150B_GPIO12_LOOKUP_STATE);
	msleep(500);
	PRINT_INFO("Test====ap lock for load fw====");
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_LSB, &reg_chidlsb);
	PRINT_INFO("REGISTER_CHIP_ID_LSB = 0x%x", reg_chidlsb);
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_MSB, &reg_chidmsb);
	PRINT_INFO("REGISTER_CHIP_ID_MSB = 0x%x", reg_chidmsb);
	//MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	//PRINT_INFO("GPIO133: %d",gpio_get_value(RST_GPIO));
    return;
}

void set_1V2_2V8_pin_func_not_wq(void) {
	PRINT_INFO("Set pinctl: SOC GPIO21 pull-up");
	set_pinctrl(snt8100fsr_g->dev, GRIP_SOC_GPIO21_ON_LOOKUP_STATE);
	msleep(500);
	PRINT_INFO("Set pinctl: PM8150B GPIO12 pull-up");
	set_pinctrl(snt8100fsr_g->dev, GRIP_PM8150B_GPIO12_LOOKUP_STATE);
	msleep(500);
	//PRINT_INFO("GPIO133: %d",gpio_get_value(RST_GPIO));
    return;
}

void asus_init_probe(void){
	int ret;
    //struct device_node *np = dev->of_node;
	struct DPC_status *DPC_status_t;
	struct grip_status *grip_state_t;

    //Initialization for main i2c device only, should be put after this line
    DPC_status_t = memory_allocate(sizeof(*DPC_status_t),
                                 GFP_KERNEL);
    grip_state_t = memory_allocate(sizeof(*grip_state_t),
                                 GFP_KERNEL);
    DPC_status_t->Condition = 0xa710;
    DPC_status_t->High = 0x14;
    DPC_status_t->Low = 0x5;
    DPC_status_g = DPC_status_t;
    memset(grip_state_t, -1, sizeof(*grip_state_t));
    grip_status_g = grip_state_t;
	
    asus_wq = create_workqueue(asus_grip_queue);
    if (!asus_wq) {
        PRINT_CRIT("Unable to create_workqueue(%s)", asus_grip_queue);
        return;
    }
    INIT_DELAYED_WORK(&check_resume, check_gesture_after_resume);
    INIT_DELAYED_WORK(&check_stuck_wake, check_stuck_semaphore);
    INIT_DELAYED_WORK(&rst_recovery_wk, grip_dump_status_func);
    INIT_DELAYED_WORK(&rst_gpio_wk, Reset_Func);
    INIT_DELAYED_WORK(&check_onoff_wk, Check_fw_onoff);
    wake_lock_init(&(snt8100fsr_g->snt_wakelock), WAKE_LOCK_SUSPEND, "snt_wakelock"); 


    /* Feed 1v2 and 2v8 to the chip */
    //if (of_property_read_bool(np, "grip_gpio12")){
	PRINT_INFO("Set lock to make sure firmware loading down");
	if(&snt8100fsr_g->ap_lock!=NULL){
		//PRINT_INFO("ap_lock has initialed");
		MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	}
	INIT_DELAYED_WORK(&RST_WORK, set_1V2_2V8_pin_func);
	workqueue_queue_work(&RST_WORK, 0);
	//set_1V2_2V8_pin_func_not_wq();
	PRINT_INFO("WQ: call set_rst_pin_func");
    //}

    /* Clay ioctl +++*/
    ret = sntSensor_miscRegister();
    if (ret < 0) {
		//return ret;
		PRINT_INFO("creat misc fail");
		//mutex_unlock(&snt8100fsr_g->ap_lock);
    }
	

    create_Grip_en_proc_file();
    create_Grip_frame_proc_file();
    create_Grip_raw_en_proc_file();
	
    /* Tap proc */
    create_GripTap1En_proc_file();
    create_GripTap2En_proc_file();
    create_GripTap_Sense_En_proc_file();
    create_GripTap1Force_proc_file();
    create_GripTap2Force_proc_file();
    create_GripTap1_Vibtator_enable_proc_file();
    create_GripTap2_Vibtator_enable_proc_file();
    create_GripTap1_Rest_enable_proc_file();
    create_GripTap2_Rest_enable_proc_file();
    create_Grip_Tap1_MIN_Position_proc_file();
    create_Grip_Tap2_MIN_Position_proc_file();
    create_Grip_Tap1_MAX_Position_proc_file();
    create_Grip_Tap2_MAX_Position_proc_file();
    create_Grip_Tap1_slope_window_proc_file();
    create_Grip_Tap2_slope_window_proc_file();
    create_Grip_Tap1_slope_tap_force_proc_file();
    create_Grip_Tap2_slope_tap_force_proc_file();
    create_Grip_Tap1_slope_release_force_proc_file();
    create_Grip_Tap2_slope_release_force_proc_file();
    create_Grip_Tap1_delta_tap_force_proc_file();
    create_Grip_Tap2_delta_tap_force_proc_file();
    create_Grip_Tap1_delta_release_force_proc_file();
    create_Grip_Tap2_delta_release_force_proc_file();
	
    create_GripTap1_TouchUpForce_proc_file();
    create_GripTap2_TouchUpForce_proc_file();

    
    /* Squeeze proc */
    create_GripSqueezeEn_proc_file();
    create_GripSqueezeForce_proc_file();
    create_Grip_Squeeze_short_limit_proc_file();
    create_Grip_Squeeze_short_dur_proc_file();
    create_Grip_Squeeze_long_dur_proc_file();
    create_GripSqueeze_up_rate_proc_file();
    create_GripSqueeze_up_total_proc_file();
    create_GripSqueeze_drop_rate_proc_file();
    create_GripSqueeze_drop_total_proc_file();

    create_GripSqueezeEn1_proc_file();
    create_GripSqueezeForce1_proc_file();
    create_Grip_Squeeze1_short_dur_proc_file();
    create_Grip_Squeeze1_long_dur_proc_file();
    create_GripSqueeze1_up_rate_proc_file();
    create_GripSqueeze1_up_total_proc_file();
    create_GripSqueeze1_drop_rate_proc_file();
    create_GripSqueeze1_drop_total_proc_file();
	
    create_GripSqueezeEn2_proc_file();
    create_GripSqueezeForce2_proc_file();
    create_Grip_Squeeze2_short_dur_proc_file();
    create_Grip_Squeeze2_long_dur_proc_file();
    create_GripSqueeze2_up_rate_proc_file();
    create_GripSqueeze2_up_total_proc_file();
    create_GripSqueeze2_drop_rate_proc_file();
    create_GripSqueeze2_drop_total_proc_file();
	
    /* Swipe proc */
    create_Grip_Swipe1_En_proc_file();
    create_Grip_Swipe2_En_proc_file();
    create_Grip_Swipe1_Velocity_proc_file();
    create_Grip_Swipe2_Velocity_proc_file();
    create_Grip_Swipe1_Len_proc_file();
    create_Grip_Swipe2_Len_proc_file();
	
    /* Slide proc */
    create_Grip_Slide1_En_proc_file();
    create_Grip_Slide2_En_proc_file();
    create_Grip_Slide1_Distance_proc_file();
    create_Grip_Slide2_Distance_proc_file();
    create_Grip_Slide1_force_proc_file();
    create_Grip_Slide2_force_proc_file();
    create_Grip_Slide1_Vibtator_enable_proc_file();
    create_Grip_Slide2_Vibtator_enable_proc_file();

    /******* Dynamic Loading FW ********/
    create_Grip_FW_VER_proc_file();
    create_Grip_FW_RESULT_proc_file();
    create_Grip_set_power_proc_file();

    /* Squeeze Factor */
    create_GripSQ_Bar0_factory_proc_file();
    create_GripSQ_Bar1_factory_proc_file();
    create_GripSQ_Bar2_factory_proc_file();

    /* Factory requirment */
    create_asusGripDebug_proc_file();
    create_GripI2cCheck_proc_file();
    create_GripFPCCheck_proc_file();
    create_Calibration_raw_data_proc_file();
    create_GripDisable_WakeLock_proc_file();
    create_Grip_Apply_GoldenK_proc_file();
    create_Grip_ReadK_proc_file();

}

