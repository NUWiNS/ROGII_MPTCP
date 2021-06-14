#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
//#include <linux/wakelock.h>
#include "grip_Wakelock.h"

extern int SntSensor_miscOpen(struct inode *inode, struct file *file);
extern int SntSensor_miscRelease(struct inode *inode, struct file *file);
extern long SntSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg);

extern void create_GripI2cCheck_proc_file(void);
extern void create_GripFPCCheck_proc_file(void);
extern void create_Calibration_raw_data_proc_file(void);
extern void create_asusGripDebug_proc_file(void);
extern void create_GripDisable_WakeLock_proc_file(void);

//Enable interface
extern void create_Grip_frame_proc_file(void);
extern void create_Grip_raw_en_proc_file(void);
extern void create_Grip_en_proc_file(void);
	
//Gesture proc
extern void create_GripTap1En_proc_file(void);
extern void create_GripTap2En_proc_file(void);
extern void create_GripTap3En_proc_file(void);
extern void create_GripTap_Sense_En_proc_file(void);
extern void create_GripTap1Force_proc_file(void);
extern void create_GripTap2Force_proc_file(void);
extern void create_GripTap3Force_proc_file(void);
extern void create_GripTap1_Vibtator_enable_proc_file(void);
extern void create_GripTap2_Vibtator_enable_proc_file(void);
extern void create_GripTap1_Rest_enable_proc_file(void);
extern void create_GripTap2_Rest_enable_proc_file(void);
extern void create_Grip_Tap1_MIN_Position_proc_file(void);
extern void create_Grip_Tap2_MIN_Position_proc_file(void);
extern void create_Grip_Tap1_MAX_Position_proc_file(void);
extern void create_Grip_Tap2_MAX_Position_proc_file(void);
extern void create_Grip_Tap1_slope_window_proc_file(void);
extern void create_Grip_Tap2_slope_window_proc_file(void);
extern void create_Grip_Tap1_slope_tap_force_proc_file(void);
extern void create_Grip_Tap2_slope_tap_force_proc_file(void);
extern void create_Grip_Tap1_slope_release_force_proc_file(void);
extern void create_Grip_Tap2_slope_release_force_proc_file(void);
extern void create_Grip_Tap1_delta_tap_force_proc_file(void);
extern void create_Grip_Tap2_delta_tap_force_proc_file(void);
extern void create_Grip_Tap1_delta_release_force_proc_file(void);
extern void create_Grip_Tap2_delta_release_force_proc_file(void);

extern void create_GripTap1_TouchUpForce_proc_file(void);
extern void create_GripTap2_TouchUpForce_proc_file(void);
extern void create_GripTap3_TouchUpForce_proc_file(void);

extern void create_GripSqueezeEn_proc_file(void);
extern void create_GripSqueezeForce_proc_file(void);
extern void create_Grip_Squeeze_short_dur_proc_file(void);
extern void create_Grip_Squeeze_long_dur_proc_file(void);
extern void create_Grip_Squeeze_short_limit_proc_file(void);
extern void create_GripSqueeze_up_rate_proc_file(void);
extern void create_GripSqueeze_up_total_proc_file(void);
extern void create_GripSqueeze_drop_rate_proc_file(void);
extern void create_GripSqueeze_drop_total_proc_file(void);
extern void create_GripSqueezeEn1_proc_file(void);
extern void create_GripSqueezeForce1_proc_file(void);
extern void create_Grip_Squeeze1_short_dur_proc_file(void);
extern void create_Grip_Squeeze1_long_dur_proc_file(void);
extern void create_Grip_Squeeze1_short_limit_proc_file(void);
extern void create_GripSqueeze1_up_rate_proc_file(void);
extern void create_GripSqueeze1_up_total_proc_file(void);
extern void create_GripSqueeze1_drop_rate_proc_file(void);
extern void create_GripSqueeze1_drop_total_proc_file(void);
extern void create_GripSqueezeEn2_proc_file(void);
extern void create_GripSqueezeForce2_proc_file(void);
extern void create_Grip_Squeeze2_short_dur_proc_file(void);
extern void create_Grip_Squeeze2_long_dur_proc_file(void);
extern void create_Grip_Squeeze2_short_limit_proc_file(void);
extern void create_GripSqueeze2_up_rate_proc_file(void);
extern void create_GripSqueeze2_up_total_proc_file(void);
extern void create_GripSqueeze2_drop_rate_proc_file(void);
extern void create_GripSqueeze2_drop_total_proc_file(void);

extern void create_Grip_Slide1_En_proc_file(void);
extern void create_Grip_Slide2_En_proc_file(void);
extern void create_Grip_Slide1_Distance_proc_file(void);
extern void create_Grip_Slide2_Distance_proc_file(void);
extern void create_Grip_Slide1_force_proc_file(void);
extern void create_Grip_Slide2_force_proc_file(void);
extern void create_Grip_Slide1_Vibtator_enable_proc_file(void);
extern void create_Grip_Slide2_Vibtator_enable_proc_file(void);
extern void create_Grip_Swipe1_En_proc_file(void);
extern void create_Grip_Swipe2_En_proc_file(void);
extern void create_Grip_Swipe1_Len_proc_file(void);
extern void create_Grip_Swipe2_Len_proc_file(void);
extern void create_Grip_Swipe1_Velocity_proc_file(void);
extern void create_Grip_Swipe2_Velocity_proc_file(void);



//Function: DPC wake from low power mode
extern void Wait_Wake_For_RegW(void);
extern void DPC_write_func(int flag);

// Gesture enable func
extern void grip_raw_enable_func(int val);
extern void grip_enable_func_noLock(int val);
extern void grip_tap1_enable_func(int val);
extern void grip_tap2_enable_func(int val);
extern void grip_tap3_enable_func(int val);
extern void grip_tap1_force_func(int val);
extern void grip_tap2_force_func(int val);
extern void grip_tap3_force_func(int val);
extern void grip_tap_sense_enable_func(int val);
extern void grip_tap1_vibrator_enable_func(int val);
extern void grip_tap2_vibrator_enable_func(int val);
extern void grip_tap1_finger_reseting_enable_func(int val);
extern void grip_tap2_finger_reseting_enable_func(int val);
extern void grip_tap1_min_position_func(int val);
extern void grip_tap2_min_position_func(int val);
extern void grip_tap3_min_position_func(int val);
extern void grip_tap1_max_position_func(int val);
extern void grip_tap2_max_position_func(int val);
extern void grip_tap3_max_position_func(int val);

//Gesture Threshold func
extern void grip_squeeze1_enable_func(int val);
extern void grip_squeeze1_force_func(int val);
extern void grip_squeeze1_long_dur_func(int val);
extern void grip_squeeze1_short_dur_func(int val);
extern void grip_squeeze1_up_rate_func(int val);
extern void grip_squeeze1_up_total_func(int val);
extern void grip_squeeze1_drop_rate_func(int val);
extern void grip_squeeze1_drop_total_func(int val);
extern void grip_squeeze2_enable_func(int val);
extern void grip_squeeze2_force_func(int val);
extern void grip_squeeze2_long_dur_func(int val);
extern void grip_squeeze2_short_dur_func(int val);
extern void grip_squeeze2_up_rate_func(int val);
extern void grip_squeeze2_up_total_func(int val);
extern void grip_squeeze2_drop_rate_func(int val);
extern void grip_squeeze2_drop_total_func(int val);

extern void grip_slide1_enable_func(int val);
extern void grip_slide2_enable_func(int val);
extern void grip_slide1_dist_func(int val);
extern void grip_slide2_dist_func(int val);
extern void grip_slide1_force_func(int val);
extern void grip_slide2_force_func(int val);

extern void grip_swipe1_enable_func(int val);
extern void grip_swipe2_enable_func(int val);
extern void grip_swipe1_velocity_func(int val);
extern void grip_swipe2_velocity_func(int val);
extern void grip_swipe1_len_func(int val);
extern void grip_swipe2_len_func(int val);



extern int g_info[16];
extern struct file_operations sentons_snt_fops;
extern struct miscdevice sentons_snt_misc;
extern int sntSensor_miscRegister(void);

extern void check_gesture_before_suspend(void);
extern void check_gesture_after_resume(struct work_struct *work);
extern struct delayed_work check_resume;

#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif


extern int Health_Check_Enable(int en);
extern void Into_DeepSleep_fun(void);
/******* Dynamic Loading FW ********/
extern int fw_version;
extern void create_Grip_FW_RESULT_proc_file(void);
extern void create_Grip_FW_VER_proc_file(void);
extern void create_Grip_set_power_proc_file(void);
extern void create_GripSQ_Bar0_factory_proc_file(void);
extern void create_GripSQ_Bar1_factory_proc_file(void);
extern void create_GripSQ_Bar2_factory_proc_file(void);
extern void create_Grip_Apply_GoldenK_proc_file(void);
extern void create_Grip_ReadK_proc_file(void);

extern uint16_t Grip_B0_F_value;
extern uint16_t Grip_B1_F_value;
extern uint16_t Grip_B2_F_value;
extern int fw_loading_status;
extern bool G_Skip_Sq1_Long;
extern bool G_Skip_Sq2_Long;
extern char *g_product_string;
extern enum DEVICE_HWID g_ASUS_hwID;


/* Workaround for stucked semaphore */
extern void check_stuck_semaphore(struct work_struct *work);
extern struct delayed_work check_stuck_wake;
/* Workaround for stucked semaphore */

/* reset func for write fail */
extern struct delayed_work rst_recovery_wk;
extern struct delayed_work rst_gpio_wk;
extern void Reset_Func(struct work_struct *work);
extern void grip_dump_status_func(struct work_struct *work);
extern struct workqueue_struct *asus_wq;

extern struct delayed_work check_onoff_wk;
extern void Check_fw_onoff(struct work_struct *work);
extern void Enable_tap_sensitive(const char *buf, size_t count);


extern void set_sq_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_tap_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_slide_gesture(uint16_t slide_id, uint16_t *reg_val, int index);
extern void set_swipe_gesture(uint16_t slide_id, uint16_t *reg_val, int index);

extern void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len);
/* Enable/disable Grip Sensor Power 1V2_2V8 */
extern void Power_Control(int en); 

extern void Grip_Driver_IRQ_EN(bool flag);
extern void Grip_Chip_IRQ_EN(bool flag);

