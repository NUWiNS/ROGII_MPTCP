/*****************************************************************************
* File: customize.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include <linux/input.h>
#include "config.h"
#include <linux/input/mt.h>
#include "track_report.h"
#include "input_device.h"
#include "debug.h"
#include "device.h"
#include "file_control.h"
#include <linux/unistd.h>
#include <linux/delay.h>
#include<linux/ktime.h>
#include "utils.h"


int cust_write_registers(void *dev, int reg, int num, void *value);
int cust_read_registers(void *dev, int reg, int num, void *value);

#define USE_SHIM_DEMO
#ifdef  USE_SHIM_DEMO
#define MAX_TRACK_REPORTS 12
#define MT_MAX_BARID       2
#define MT_MAX_TOUCHBAR    5
#define MT_MAX_SLOTS      (MT_MAX_BARID*MT_MAX_TOUCHBAR)
#define PANEL_SIZE 2559
/*
 *  State machine of parsing inbound Track Report:
 *  <frame_no trackId barId force_level center bottom top>
 */

#define SNT_SYSFS_TR_STATE_FRAME        0
#define SNT_SYSFS_TR_STATE_BAR          1
#define SNT_SYSFS_TR_STATE_TRACK        2
#define SNT_SYSFS_TR_STATE_FORCE        3
#define SNT_SYSFS_TR_STATE_POS          4
#define SNT_SYSFS_TR_STATE_MINPOS       5
#define SNT_SYSFS_TR_STATE_MAXPOS       6
#define SNT_SYSFS_TR_STATE_MAX          7

uint16_t TAP0_BIT0 = 0x0100;
uint16_t TAP0_BIT1 = 0x0202;
uint16_t TAP0_BIT2 = 0x0000;
uint16_t TAP0_BIT3 = 0xFFFF;
uint16_t TAP0_BIT4 = 0xFFFF;
uint16_t TAP0_BIT5 = 0xFFFF;
uint16_t TAP0_BIT6 = 0;
uint16_t TAP0_BIT7 = 0;

uint16_t TAP1_BIT0 = 0x0120;
uint16_t TAP1_BIT1 = 0x0402;
uint16_t TAP1_BIT2 = 0x0000; //bar1 range: 76~356   0x92~0x164
uint16_t TAP1_BIT3 = 0xFFFF;
uint16_t TAP1_BIT4 = 0xFFFF;
uint16_t TAP1_BIT5 = 0xFFFF;
uint16_t TAP1_BIT6 = 0;
uint16_t TAP1_BIT7 = 0;

uint16_t TAP2_BIT0 = 0x0120;
uint16_t TAP2_BIT1 = 0x0A00; //use for trigger5 for HW >= ER2
uint16_t TAP2_BIT2 = 0x0000;
uint16_t TAP2_BIT3 = 0xFFFF;
uint16_t TAP2_BIT4 = 0xFFFF;
uint16_t TAP2_BIT5 = 0xFFFF;
uint16_t TAP2_BIT6 = 0;
uint16_t TAP2_BIT7 = 0;

uint16_t SQ1_BIT0 = 0x0985;
uint16_t SQ1_BIT1 = 0;
uint16_t SQ1_BIT2 = 0xFFFF;
uint16_t SQ1_BIT3 = 0;
uint16_t SQ1_BIT4 = 0xFFFF;
uint16_t SQ1_BIT5 = 0x1532;
uint16_t SQ1_BIT6 = 0;
uint16_t SQ1_BIT7 = 0xFFFF;
uint16_t SQ1_BIT8 = 0x0001;

uint16_t SQ2_BIT0 = 0x0801;
uint16_t SQ2_BIT1 = 0;
uint16_t SQ2_BIT2 = 0xFFFF;
uint16_t SQ2_BIT3 = 0;
uint16_t SQ2_BIT4 = 0xFFFF;
uint16_t SQ2_BIT5 = 0x1532;
uint16_t SQ2_BIT6 = 0;
uint16_t SQ2_BIT7 = 0xFFFF;
uint16_t SQ2_BIT8 = 0x0001;

uint16_t SLIDE0_BIT0 = 0x0001; //0x1 GRE Bit Enable
uint16_t SLIDE0_BIT1 = 0;
uint16_t SLIDE0_BIT2 = 0xFFFF;
uint16_t SLIDE0_BIT3 = 0x1000;
uint16_t SLIDE0_BIT4 = 0x0001; //Second slide use 1 * 0.16cm^2
uint16_t SLIDE0_BIT5 = 0;

uint16_t SLIDE1_BIT0 = 0x0101; //0x1 GRE Bit Enable
uint16_t SLIDE1_BIT1 = 0;
uint16_t SLIDE1_BIT2 = 0xFFFF;
uint16_t SLIDE1_BIT3 = 0x1000;
uint16_t SLIDE1_BIT4 = 0x0001; //Second slide use 1 * 0.16cm^2
uint16_t SLIDE1_BIT5 = 0;

uint16_t SWIPE1_BIT0 = 0x0001;
uint16_t SWIPE1_BIT1 = 0;
uint16_t SWIPE1_BIT2 = 0xFFFF;
uint16_t SWIPE1_BIT3 = 0x0030;
uint16_t SWIPE1_BIT4 = 0x0001;

uint16_t SWIPE2_BIT0 = 0x0101;
uint16_t SWIPE2_BIT1 = 0;
uint16_t SWIPE2_BIT2 = 0xFFFF;
uint16_t SWIPE2_BIT3 = 0x0030;
uint16_t SWIPE2_BIT4 = 0x0001;

/*==========================================================================*/
/* register_input_events()                                                  */
/* Customize to register which input events we'll be sending                */
/*==========================================================================*/
void register_input_events(struct input_dev *input_dev) {
    //Clay: simulate sensor hal
    	/* Set Light Sensor input device */
	input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_dev->evbit);
	
	//Gesture Type
	//input_set_capability(input_dev, EV_ABS, ABS_MT_DISTANCE);
	//__set_bit(ABS_MT_DISTANCE, input_dev->absbit);
	//input_set_abs_params(input_dev, ABS_MT_DISTANCE, 0, 12, 0, 0);
	
	//length
	//input_set_capability(input_dev, EV_ABS, ABS_MT_ORIENTATION);
	//__set_bit(ABS_MT_ORIENTATION, input_dev->absbit);
	//input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 5, 0, 0);
	
	//Trk_id
	input_set_capability(input_dev, EV_ABS, ABS_MT_TRACKING_ID);
	__set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 8, 0, 0);

	//Bar_id
	input_set_capability(input_dev, EV_ABS, ABS_MT_WIDTH_MAJOR);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, -2, -1, 0, 0);

	//PRESSUR
	input_set_capability(input_dev, EV_ABS, ABS_MT_WIDTH_MINOR);
	__set_bit(ABS_MT_WIDTH_MINOR, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 255, 0, 0);

	//Fr_nr
	input_set_capability(input_dev, EV_ABS, ABS_MT_BLOB_ID);
	__set_bit(ABS_MT_BLOB_ID, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_BLOB_ID, 0, 65535, 0, 0);

	//Top
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_X);
	__set_bit(ABS_MT_TOOL_X, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_X, 0, PANEL_SIZE, 0, 0);
	
	//Bot
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_Y);
	__set_bit(ABS_MT_TOOL_Y, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_Y, 0, PANEL_SIZE, 0, 0);

	//Center
	input_set_capability(input_dev, EV_ABS, ABS_MT_TOOL_TYPE);
	__set_bit(ABS_MT_TOOL_TYPE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, PANEL_SIZE, 0, 0);
	
    PRINT_DEBUG("done");
}
void input_event_report(int g_id, int len, int trk_id, int bar_id, int force, int fr_nr, int center){
	
	//Gesture Type
	input_report_abs(get_input_device(), ABS_MT_TOOL_X, g_id);
	//Length
	input_report_abs(get_input_device(), ABS_MT_TOOL_Y, len);
	//Trk_id
	input_report_abs(get_input_device(), ABS_MT_TRACKING_ID, trk_id);
	//Bar_id
	input_report_abs(get_input_device(), ABS_MT_WIDTH_MAJOR, bar_id);
	//PRESSUR
	input_report_abs(get_input_device(), ABS_MT_WIDTH_MINOR, force);
	//Fr_nr
	input_report_abs(get_input_device(), ABS_MT_BLOB_ID, fr_nr);
	//center
	input_report_abs(get_input_device(), ABS_MT_TOOL_TYPE, center);
	input_event(get_input_device(), EV_SYN, SYN_REPORT, 7);
	input_sync(get_input_device());
}


/*
 * Support functions to set custom operational profiles
 *
 * cust_write_registers(dev, StartRegId, NumReg, *RegValues)
 *     write 1 or more registers to SNT8100FSR starting at specified register.
 *
*/
static void set_touch_enable(void *dev, uint16_t reg_val) {
  cust_write_registers(dev, 1, 1, &reg_val);
}

static void set_frame_rate(void *dev, uint16_t reg_val) {
  cust_write_registers(dev, 2, 1, &reg_val);
}

static void set_triggers(void *dev, uint16_t *reg_val) {
  cust_write_registers(dev, 0x10, 7, reg_val);
}

static void L_set_tap_gesture(void *dev, uint16_t tap_id, uint16_t *reg_val) {
  uint16_t cfg_bank_write[3] = { tap_id*16, 16, 0x0802};
  uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
  cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
  cust_write_registers(dev, 0x200, 8, reg_val);
  cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

static void L_set_squeeze_gesture(void *dev, uint16_t sq_id, uint16_t *reg_val) {
  uint16_t cfg_bank_write[3] = { sq_id*18, 18, 0x0a02};
  uint16_t cfg_bank_commit[3] = { 0, 0, 0x0a03};
  cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
  cust_write_registers(dev, 0x200, 9, reg_val);
  cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

static void L_set_slider_gesture(void *dev, uint16_t slider_id, uint16_t *reg_val) {
  uint16_t cfg_bank_write[3] = { slider_id*12, 12, 0x0b02};
  uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
  cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
  cust_write_registers(dev, 0x200, 6, reg_val);
  cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

static void set_dpc(void *dev, uint16_t hi_hz, uint16_t lo_hz, uint16_t ctl) {
  uint16_t reg_val[3] = { hi_hz, lo_hz, ctl};
  cust_write_registers(dev, 0x39, 3, reg_val);
}

/*
 * Customization function for the sysfs attribute file "profile". Use this
 * function to set up specific settings of registers for the various operating
 * modes desired for the SNT8100FSR.
 *
 * The profiles in this function are examples and not meant to be a
 * comprehensive set of profiles. Users should provide their on content
 * based on their product needs.
 *
*/
void set_operational_profile(void *dev, int profile)
{
  PRINT_FUNC("profile = %d", profile);
  
  switch (profile) {
    case    0 : { /* 50 Hz, No DPC                                      */
      set_touch_enable(dev, 0);               // disable touch
      set_dpc(dev, 0, 0, 0);                  // disable dpc
      set_frame_rate(dev, 50);                // frame rate = 50
      set_touch_enable(dev, 1);               // enable touch
      break;
    } 
    case    101: {  /* 50Hz, No DPC, Enable Tap 0, Bar1, Trigger 1            */
      uint16_t snt_profile_100_tap[8] = { 0x1001, 0x0200, 0x0000, 0xffff, 0x1008, 0x1008, 0x0000, 0x0000 };
      uint16_t snt_profile_100_trigger[7] = {0, 0, 0, 0, 0, 0, 0};
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      set_triggers(dev, snt_profile_100_trigger);     // set Trig1 to level
      L_set_tap_gesture(dev, 0, snt_profile_100_tap);   // enable tap 0
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
    case    102: {  /* 50Hz, No DPC, Enable Tap 1, Bar2, Trigger 2            */
      uint16_t snt_profile_100_tap[8] = { 0x1021, 0x0400, 0x0000, 0xffff, 0x1008, 0x1008, 0x0000, 0x0000 };
      uint16_t snt_profile_100_trigger[7] = {0, 0, 0, 0, 0, 0, 0};
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      set_triggers(dev, snt_profile_100_trigger);     // set Trig1 to level
      L_set_tap_gesture(dev, 1, snt_profile_100_tap);   // enable tap 1
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
    // Set Tap0
    // 0x2c 3 0 16 0x0802 # Tap0 is offset 0, size=16 bytes
    // 0x200 8
    // 0x1001         # init_force=10, bar_id=0, en=1
    // 0x0304         # trig_id = 1, gre=1, slope_window=4 (10ms increments)
    // 0 0xffff       # set entire bar as active region
    // 0x0404         # delta_rel_force=4, delta_tap_force=4
    // 0x0202         # slope_rel_force=2, slope_tap_force=2
    // 0 0            # palm_reject_thresh=0, drag_reject_thresh=0
    // 0x2c 3 0 0 0x0803

    // # Set Tap1
    // 0x2c 3 16 16 0x0802 # Tap1 is offset 16, size=16 bytes
    // 0x200 8
    // 0x1021         # init_force=10, bar_id=1, en=1
    // 0x0504         # trig_id = 2, gre=1, slope_window=4 (10ms increments)
    // 0 0xffff       # set entire bar as active region
    // 0x0404         # delta_rel_force=4, delta_tap_force=4
    // 0x0202         # slope_rel_force=2, slope_tap_force=2
    // 0 0            # palm_reject_thresh=0, drag_reject_thresh=0
    // 0x2c 3 0 0 0x0803
    case    103: {  /* 100Hz, No DPC, Set Tap 0 and Tap 1            */
      uint16_t snt_profile_100_tap0[8] = { 0x1001, 0x0304, 0x0000, 0xffff, 0x0404, 0x0202, 0x0000, 0x0000 };
      uint16_t snt_profile_100_tap1[8] = { 0x1021, 0x0504, 0x0000, 0xffff, 0x0404, 0x0202, 0x0000, 0x0000 };
      uint16_t snt_profile_100_trigger[7] = {0, 0, 0, 0, 0, 0, 0};
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 100);                       // frame rate = 100
      set_triggers(dev, snt_profile_100_trigger);     // set Trig to level
      L_set_tap_gesture(dev, 0, snt_profile_100_tap0);  // enable and configure tap 0
      L_set_tap_gesture(dev, 1, snt_profile_100_tap1);  // enable and configure tap 1
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
    case    201: {  /* 50Hz, No DPC, squeeze0: Bar1(bar_id=0) only, SqMinForce = 10, SqShortDur = 1 sec, SqLongDur = 2 sec, trig_id = 0 (interface v3.9) */
      uint16_t snt_profile_100_squeeze[9] = { 0x800a, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x3264, 0x0000, 0xFFFF, 0x0001 };
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      L_set_squeeze_gesture(dev, 0, snt_profile_100_squeeze);   // enable seueeze 0
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
   case    202: {  /* 50Hz, No DPC, squeeze0: Bar2(bar_id=1) only, SqMinForce = 10, SqShortDur = 1 sec, SqLongDur = 2 sec, trig_id = 0 (interface v3.9) */
      uint16_t snt_profile_100_squeeze[9] = { 0x090a, 0x0000, 0xFFFF, 0x0000, 0xFFFF, 0x3264, 0x0000, 0xFFFF, 0x0001 };
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      L_set_squeeze_gesture(dev, 0, snt_profile_100_squeeze);   // enable seueeze 0
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
    // Slider 0
    // 0x2c 3 0 12 0x0b02
    // 0x200 6
    // 0x8000            # en=1, bar_id=0
    // 0 0xffff          # entire bar
    // 0x1000            # move=1mm, fthresh=0
    // 0 0               # reserved
    // 0x02c 3 0 0 0x0b03
    case    301: {  /* 50Hz, No DPC, slider0, Bar1(bar_id=0), entire bar, move=1mm, fthresh=0 */
      uint16_t snt_profile_100_slider[9] = { 0x8000, 0x0000, 0xFFFF, 0x1000, 0x0000, 0x0000 };
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      L_set_slider_gesture(dev, 0, snt_profile_100_slider);   // slider 0
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }

    // Slider 1
    // 0x2c 3 12 12 0x0b02
    // 0x200 6
    // 0x8100            # en=1, bar_id=1
    // 0 0xffff          # entire bar
    // 0x1000            # move=1mm, fthresh=0
    // 0 0               # reserved
    // 0x02c 3 0 0 0x0b03
    case    302: {  /* 50Hz, No DPC, slider1, Bar2(bar_id=1), entire bar, move=1mm, fthresh=0 */
      uint16_t snt_profile_100_slider[9] = { 0x8100, 0x0000, 0xFFFF, 0x1000, 0x0000, 0x0000 };
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 0, 0, 0);                          // disable dpc
      set_frame_rate(dev, 50);                        // frame rate = 50
      L_set_slider_gesture(dev, 1, snt_profile_100_slider);   // slider 1
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
#ifdef DYNAMIC_PWR_CTL
    case    10 : {   /* 5/50 DPC, timer = 2 seconds                     */
      set_touch_enable(dev, 0);                       // disable touch
      set_dpc(dev, 50, 5, 0x8000 | 2000);             // dpc 50Hz/5Hz/2sec
      set_touch_enable(dev, 1);                       // enable touch
      break;
    }
#endif
    default:
      PRINT_CRIT("Unknown profile %d", profile);
      break;
  }  
}



/*==========================================================================*/
/* process_track_report()                                                   */
/* Customize to process track reports from the device                       */
/*==========================================================================*/
int track_report_count = 0;
void process_track_reports(uint16_t frame,
                           struct track_report *tr,
                           size_t count) 
{
    int i;
    int gs_found = 0;
    int slot_id;
    int gesture_id = 0;
    //float trans = 0;
    long trans_result = 0;
	//time_start=ktime_get();
	//elapsed_ns = ktime_to_ns(ktime_sub(ktime_get(), time_start));

    //Grip type
    if(snt8100fsr_g->en_sensor_evt){
	for(i = 0; i < count; i++) {
		// diagnostic section 
		if (tr[i].bar_id==0 && tr[i].trk_id==0 && tr[i].force_lvl <= TR_DIAG_REC_VERS_MAX) {
			break;
	        }
		slot_id = (tr[i].bar_id*MT_MAX_TOUCHBAR) + tr[i].trk_id%MT_MAX_TOUCHBAR;
	        // gesture section
	        if (gs_found ||(tr[i].bar_id==0 && tr[i].trk_id==0 && tr[i].force_lvl > TR_DIAG_REC_VERS_MAX)) {
	            if (gs_found == 0) {
	                // process gesture header data here
	                p_gs_hdr_rec_t p = (p_gs_hdr_rec_t) &tr[i];
	                // ...
			if (p->swipe0_velocity != 0){
				gesture_id = 5;
			} else if (p->swipe1_velocity != 0){
				gesture_id = 6;
			}
			
			if(gesture_id!=0)
				PRINT_INFO("SW[%d,%d,%d], SQ0[%02x], TAP[%02x], G_ID=%d",			
						p->swipe0_velocity,p->swipe1_velocity,p->swipe2_velocity,p->squeeze,p->tap, gesture_id);
			
	                gs_found = 1;
			if(gesture_id == 5){
				if(p->swipe0_velocity > 0) //bottom to top
					input_event_report(gesture_id, 0, 0, 0 ,0, 0, 0);
				else//top to bottom
					input_event_report(gesture_id, 0, 1, 0 ,0, 0, 0);
			}else if(gesture_id == 6){
				if(p->swipe1_velocity > 0) //bottom to top
					input_event_report(gesture_id, 0, 0, 0 ,0, 0, 0);
				else//top to bottom
					input_event_report(gesture_id, 0, 1, 0 ,0, 0, 0);
			}else if(gesture_id > 0){
				input_event_report(gesture_id, count, tr[i].trk_id, tr[i].bar_id, tr[i].force_lvl, frame, tr[i].center);
			}else{
			}
		   } else {
	                // process slider records here
	                p_gs_slider_rec_t p = (p_gs_slider_rec_t) &tr[i];
			if (p->escape0 == GS_RPT_TYPE_SLIDE) {
				uint8_t id0 = GS_GET_SLIDER_ID0(p->slider_finger_id);
	                	uint8_t id1 = GS_GET_SLIDER_ID1(p->slider_finger_id);
	                	uint8_t fid0 = GS_GET_SLIDER_FID0(p->slider_finger_id);
	                	uint8_t fid1 = GS_GET_SLIDER_FID1(p->slider_finger_id);
	                        PRINT_INFO("id0=%u, id1=%u, fid0=%u, fid1=%u", id0, id1, fid0, fid1);
	                        if(fid0 == 1){
	                                if (id0 == 0) {
	                                        gesture_id = 12;
	                                        PRINT_INFO("1. Slider[%u,%u]: F%u, P%u, g_id=%d", id0, fid0,
	                                                    p->slider_force0, p->slider_pos0,
	                                                    gesture_id);
	                                        input_event_report(gesture_id, count, fid0, id0, p->slider_force0, frame, p->slider_pos0);
	                                }else if (id0==1){
	                                        gesture_id = 13;
	                                        PRINT_INFO("2. Slider[%u,%u]: F%u, P%u, g_id=%d", id1, fid0,
	                                                    p->slider_force0, p->slider_pos0,
	                                                    gesture_id);
	                                        input_event_report(gesture_id, count, fid0, id0, p->slider_force0, frame, p->slider_pos0);
	                                }
	                        }
	                        if(fid1 == 1){
	                                if(id1==1){
	                                        gesture_id = 13;
	                                        PRINT_INFO("3. Slider[%u,%u]: F%u, P%u, g_id=%d", id1, fid1,
	                                                    p->slider_force1, p->slider_pos1,
	                                                    gesture_id);
	                                        input_event_report(gesture_id, count, fid1, id1, p->slider_force1, frame, p->slider_pos1);
	                                }
	                        }
              		} else if (p->escape0 == GS_RPT_TYPE_SQUEEZE) {
                    		p_gs_squeeze_rec_t p_sq = (p_gs_squeeze_rec_t) p;
				int sq2 = 0;
				if(p_sq->squeeze[0] > 0){
					if(p_sq->squeeze[0] == 0x10) gesture_id = 9;
					else if(p_sq->squeeze[0] == 0x48) gesture_id = 7;
					else if(p_sq->squeeze[0] == 0x80) gesture_id = 8;
					else if(p_sq->squeeze[0] == 0x28) gesture_id = 10;
					else if(p_sq->squeeze[0] == 0x08) gesture_id = 11;

					if(gesture_id == 8 && G_Skip_Sq1_Long)
							gesture_id = 10;
					
					input_event_report(gesture_id, count,  sq2, tr[i].bar_id, tr[i].force_lvl, frame, tr[i].center);	
					gesture_id = 0;
				}
				if(p_sq->squeeze[1] > 0){
					sq2 = 1;
					if(p_sq->squeeze[1] == 0x10) gesture_id = 9; 
					else if(p_sq->squeeze[1] == 0x48) gesture_id = 7;
					else if(p_sq->squeeze[1] == 0x80) gesture_id = 8;
					else if(p_sq->squeeze[1] == 0x28) gesture_id = 10;
					else if(p_sq->squeeze[1] == 0x08) gesture_id = 11;
					
					if(gesture_id == 8 && G_Skip_Sq2_Long)
							gesture_id = 10;
					
					input_event_report(gesture_id, count,  sq2, tr[i].bar_id, tr[i].force_lvl, frame, tr[i].center);
				}
		                PRINT_INFO("SQ0[%02x], SQ1[%02x], g_id=%d", p_sq->squeeze[0], p_sq->squeeze[1], gesture_id);
	               }
		   }
	        }else {
	        	if(grip_status_g->G_TAP1_EN == 1 || grip_status_g->G_TAP2_EN == 1 || grip_status_g->G_TAP3_EN == 1){
	        		//Normal Data
		        	input_event_report(0, count, tr[i].trk_id, tr[i].bar_id, tr[i].force_lvl, frame, tr[i].center);
	        		if(track_report_count % 100 == 0){
					track_report_count = 0;
					PRINT_INFO("trk_id1=%d, bar_id=%d, force=%d, top=%d, center=%d, bot=%d", 
						tr[i].trk_id, tr[i].bar_id, tr[i].force_lvl, tr[i].top, tr[i].center, tr[i].bottom);
	        		}
	        	}else{
		        	//Squeeze Case apply K data
		        	if(tr[i].bar_id == 1){
					//trans = (float)B1_F_value / 256;
					//trans_result = (int)(trans * tr[i].force_lvl);
					trans_result = (255 * tr[i].force_lvl) /Grip_B1_F_value;
					if(trans_result > 255) trans_result = 255;
		        		input_event_report(0, count, tr[i].trk_id, tr[i].bar_id, trans_result, frame, tr[i].center);
		        	}else if(tr[i].bar_id == 2){
					//trans = (float)B2_F_value / 256;
					//trans_result = (int)(trans * tr[i].force_lvl);
					trans_result = (255 * tr[i].force_lvl) /Grip_B2_F_value;
					if(trans_result > 255) trans_result = 255;
		        		input_event_report(0, count, tr[i].trk_id, tr[i].bar_id, trans_result, frame, tr[i].center);
		        	}else{
					trans_result = (255 * tr[i].force_lvl) /Grip_B0_F_value;
					if(trans_result > 255) trans_result = 255;
			        	input_event_report(0, count, tr[i].trk_id, tr[i].bar_id, trans_result, frame, tr[i].center);
		        	}	
					
	        		if(track_report_count % 20 == 0){
					track_report_count = 0;
					PRINT_INFO("trk_id0=%d, bar_id=%d, force=%d, top=%d, center=%d, bot=%d", 
						tr[i].trk_id, tr[i].bar_id, trans_result, tr[i].top, tr[i].center, tr[i].bottom);
	        		}
	        	}
			track_report_count++;
			/*
			    PRINT_DEBUG("Track Report (slot: %u) ->", slot_id);
			    PRINT_DEBUG("   Frame Nr: (%u)->", frame);
			    PRINT_DEBUG("     Bar ID: %u", tr[i].bar_id);
			    PRINT_DEBUG("   Track ID: %u", tr[i].trk_id);
			    PRINT_DEBUG("  Force Lvl: %u", tr[i].force_lvl);
			    PRINT_DEBUG("        Top: %u", tr[i].bottom);
			    PRINT_DEBUG("     Center: %u", tr[i].center);
			    PRINT_DEBUG("     Bottom: %u", tr[i].top);
			*/
		}
	}
    }
    PRINT_DEBUG("done");
}
#else
#define MAX_TRACK_REPORTS 12
#define MT_MAX_BARID       2
#define MT_MAX_TOUCHBAR    5
#define MT_MAX_SLOTS      (MT_MAX_BARID*MT_MAX_TOUCHBAR)
#define PANEL_SIZE 2559
#define UINT32_MAX 0xffffffff
/*==========================================================================*/
/* register_input_events()                                                  */
/* Customize to register which input events we'll be sending                */
/*==========================================================================*/
void register_input_events(struct input_dev *input_dev) {
    PRINT_FUNC();
	input_dev->name = "snt8100fsr";
	input_dev->id.bustype = BUS_I2C;


	//Trk_id
	/*
	input_set_capability(input_dev, EV_ABS, ABS_X);
	__set_bit(ABS_X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Y);
	__set_bit(ABS_Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Z);
	__set_bit(ABS_Z, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_Z, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RX);
	__set_bit(ABS_RX, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RX, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RY);
	__set_bit(ABS_RY, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RY, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RZ);
	__set_bit(ABS_RZ, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RZ, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT0X);
	__set_bit(ABS_HAT0X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT0X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT0Y);
	__set_bit(ABS_HAT0Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT0Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT1X);
	__set_bit(ABS_HAT1X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT1X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT1Y);
	__set_bit(ABS_HAT1Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT1Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT2X);
	__set_bit(ABS_HAT2X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT2X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT2Y);
	__set_bit(ABS_HAT2Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT2Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT3X);
	__set_bit(ABS_HAT3X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT3X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT3Y);
	__set_bit(ABS_HAT3Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT3Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_THROTTLE);
	__set_bit(ABS_THROTTLE, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_THROTTLE, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RUDDER);
	__set_bit(ABS_RUDDER, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RUDDER, 0, UINT32_MAX, 0, 0);
	*/

/*

    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_BLOB_ID, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_X, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_Y, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT0Y, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT1X, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT1Y, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT2X, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT2Y, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT3X, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_HAT3Y, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_THROTTLE, 0, UINT32_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_RUDDER, 0, UINT32_MAX, 0, 0);

    PRINT_DEBUG("done");
}

#define TR_TOT_IN_PACKET(idx)     (((idx) == 2) ?  2 : 7)
#define TR_TOT_REMAIN(count, idx) ((count)-(idx)*7)
#define MIN(a, b) ((a<b) ? a : b)
uint8_t INPUT_ID[16] = {
    ABS_MT_WIDTH_MAJOR, ABS_MT_WIDTH_MAJOR, ABS_MT_PRESSURE,
    ABS_MT_BLOB_ID, ABS_MT_TOOL_X, ABS_MT_TOOL_Y,
    ABS_MT_TOUCH_MAJOR, ABS_HAT0Y, ABS_HAT1X,
    ABS_HAT1Y, ABS_HAT2X, ABS_HAT2Y,
    ABS_HAT3X, ABS_HAT3Y, ABS_THROTTLE, ABS_RUDDER,
};
*/
    //PART 1
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);

    input_set_abs_params(input_dev, ABS_MT_POSITION_X, -2, -1, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 2559, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	
    input_mt_destroy_slots(input_dev);
    input_mt_init_slots(input_dev, MT_MAX_SLOTS, 0);
	
    //PART 2
    //set_bit(EV_SYN, input_dev->evbit);
    //set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_ABS, ABS_X);
	__set_bit(ABS_X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Y);
	__set_bit(ABS_Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Z);
	__set_bit(ABS_Z, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_Z, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RX);
	__set_bit(ABS_RX, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RX, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RY);
	__set_bit(ABS_RY, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RY, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RZ);
	__set_bit(ABS_RZ, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RZ, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT0X);
	__set_bit(ABS_HAT0X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT0X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT0Y);
	__set_bit(ABS_HAT0Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT0Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT1X);
	__set_bit(ABS_HAT1X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT1X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT1Y);
	__set_bit(ABS_HAT1Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT1Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT2X);
	__set_bit(ABS_HAT2X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT2X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT2Y);
	__set_bit(ABS_HAT2Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT2Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT3X);
	__set_bit(ABS_HAT3X, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT3X, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_HAT3Y);
	__set_bit(ABS_HAT3Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_HAT3Y, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_THROTTLE);
	__set_bit(ABS_THROTTLE, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_THROTTLE, 0, UINT32_MAX, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RUDDER);
	__set_bit(ABS_RUDDER, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_RUDDER, 0, UINT32_MAX, 0, 0);

    PRINT_DEBUG("done");
}

/*==========================================================================*/
/* process_track_report()                                                   */
/* Customize to process track reports from the device                       */
/*==========================================================================*/
#define TR_TOT_IN_PACKET(idx)     (((idx) == 2) ?  2 : 7)
#define TR_TOT_REMAIN(count, idx) ((count)-(idx)*7)
#define MIN(a, b) ((a<b) ? a : b)
uint8_t INPUT_ID[16] = {
    ABS_X, ABS_Y, ABS_Z,
    ABS_RX, ABS_RY, ABS_RZ,
    ABS_HAT0X, ABS_HAT0Y, ABS_HAT1X,
    ABS_HAT1Y, ABS_HAT2X, ABS_HAT2Y,
    ABS_HAT3X, ABS_HAT3Y, ABS_THROTTLE, ABS_RUDDER,
};
void process_track_reports(uint16_t frame,
                           struct track_report *tr,
                           size_t count) {
    //Part 1
    int i;
    int remap_center = 0;
    int R_side_cont = 80;
    int L_side_cont = 77;

    //int i;//, j;
    int32_t trbuf[16];
    uint16_t packet_header = 0x5A96;
    uint8_t  packet_tot = (count / 7) + 1;
    uint8_t  packet_idx, tr_idx, cnt;
    PRINT_FUNC();

    /* process a track report */
    /* ...code... */

    /*
    PRINT_INFO("[austin][KERN] count: %zu, packet_tot: %d", count, packet_tot);
    for(i = 0; i < count; i++) {
        PRINT_INFO("   Frame Nr: (%u)->", frame);
        PRINT_INFO("     Bar ID: %u", tr[i].bar_id);
        PRINT_INFO("   Track ID: %u", tr[i].trk_id);
        PRINT_INFO("  Force Lvl: %u", tr[i].force_lvl);
        PRINT_INFO("        Top: %u", tr[i].top);
        PRINT_INFO("     Center: %u", tr[i].center);
        PRINT_INFO("     Bottom: %u", tr[i].bottom);
    }
    */
    //Part 1
    if(snt8100fsr_g->en_demo) {
	    for(i = 0; i < count; i++) {
		    int slot_id = (tr[i].bar_id*MT_MAX_TOUCHBAR) + tr[i].trk_id%MT_MAX_TOUCHBAR;

		    input_mt_slot(get_input_device(), slot_id);

		    if (tr[i].force_lvl > 0) {

		    //input_report_key(get_input_device(), BTN_TOOL_FINGER, true);
		    /* ===Remap Touch Event===
		    * R-side: 80~1696 to 80~2559  => 0~1616 to 0~2479
		    * L-side: 77~233 to 2185~2559 => 0~156 to 2108~2482
		    */
			if(tr[i].bar_id == 0){
				remap_center = R_side_cont + ((tr[i].center-R_side_cont)*1534)/1000;
    			}else if (tr[i].bar_id == 1){
			  	remap_center = L_side_cont + ((tr[i].center-L_side_cont)*241)/100;
	    		}
			input_mt_report_slot_state(get_input_device(), MT_TOOL_FINGER, true);
			//input_report_key(get_input_device(), BTN_TOUCH, 1);
			//input_report_key(get_input_device(), BTN_TOOL_FINGER, true);
			input_report_abs(get_input_device(), ABS_MT_POSITION_X, -1 - tr[i].bar_id);
			//input_report_abs(get_input_device(), ABS_MT_POSITION_Y, 2559-tr[i].center);
			input_report_abs(get_input_device(), ABS_MT_POSITION_Y, 2559 - remap_center);
			input_report_abs(get_input_device(), ABS_MT_PRESSURE, tr[i].force_lvl);
		    } else {
			    input_mt_report_slot_state(get_input_device(), MT_TOOL_FINGER, false);
			    //input_report_key(get_input_device(), BTN_TOUCH, 0);
			    //input_report_key(get_input_device(), BTN_TOOL_FINGER, 0);
			    input_report_abs(get_input_device(), ABS_MT_PRESSURE, tr[i].force_lvl);
		    }
	    }
	    input_mt_sync_frame(get_input_device());
	    input_sync(get_input_device());
    }else{
	    for (packet_idx = 0; packet_idx < packet_tot; packet_idx++) {

	        memset(trbuf, 0, sizeof(trbuf));
	        PRINT_DEBUG("[austin][KERN] %d(%d) out of %d", packet_idx, TR_TOT_IN_PACKET(packet_idx), packet_tot);

	        trbuf[0]  = (packet_header << 16) | (packet_tot << 8) | (packet_idx);
	        trbuf[15] = 0;

	        PRINT_DEBUG("[austin][KERN] cnt: %d (min(%d, %d))",
	                MIN(TR_TOT_IN_PACKET(packet_idx), (int)TR_TOT_REMAIN(count, packet_idx)),
	                TR_TOT_IN_PACKET(packet_idx), (int)TR_TOT_REMAIN(count, packet_idx));

	        input_report_abs(get_input_device(), INPUT_ID[0], trbuf[0]);
	        PRINT_DEBUG("[austin][KERN] trbuf[0]: %08X", trbuf[0]);

	        //for (cnt = 0; cnt < TR_TOT_IN_PACKET(packet_idx); cnt++) {
		for (cnt = 0; cnt < MIN(TR_TOT_IN_PACKET(packet_idx),
					TR_TOT_REMAIN(count, packet_idx)); cnt++) {

	            tr_idx = packet_idx * 7 + cnt;
	            trbuf[cnt*2+1] = (int32_t)((tr[tr_idx].center << 16) | (tr[tr_idx].force_lvl << 8) |
	                                        (tr[tr_idx].bar_id << 5)  | (tr[tr_idx].trk_id));
	            trbuf[cnt*2+2] = (int32_t)((tr[tr_idx].bottom << 16) | tr[tr_idx].top);
	            input_report_abs(get_input_device(), INPUT_ID[cnt*2+1], trbuf[cnt*2+1]);
	            input_report_abs(get_input_device(), INPUT_ID[cnt*2+2], trbuf[cnt*2+2]);
	            PRINT_DEBUG("[austin][KERN] trbuf[%d](%d): %08X", cnt*2+1, INPUT_ID[cnt*2+1], trbuf[cnt*2+1]);
	            PRINT_DEBUG("[austin][KERN] trbuf[%d](%d): %08X", cnt*2+2, INPUT_ID[cnt*2+2], trbuf[cnt*2+2]);
	        }

	        input_sync(get_input_device());

	        input_report_abs(get_input_device(), INPUT_ID[0], -1);
	        input_sync(get_input_device());
	    }
    }
    PRINT_INFO("done");
}
#endif

#if USE_TRIG_IRQ
/*==========================================================================*/
/* process_trigger()                                                        */
/* Customize to process trigger interrupts from the device                  */
/* trig_id values: 0, 1, 2 correlate to TRIG0, TRIG1, TRIG2 in config.h     */
/*==========================================================================*/
void process_trigger(int trig_id, int gpio_value)
{
    PRINT_FUNC();
    if(trig_id == 1){
	if(gpio_value == 1){
      		PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "down");
		input_event_report(trig_id, 0, 0, 0 ,0, 0, 0);
	}else{
      		PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "up");
		input_event_report(trig_id, 0, 1, 0 ,0, 0, 0);
	}
    }else if (trig_id == 2){
		if(gpio_value == 1){
      			PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "down");
			input_event_report(trig_id, 0, 0, 0 ,0, 0, 0);
		}else{
      			PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "up");
			input_event_report(trig_id, 0, 1, 0 ,0, 0, 0);
		}
    }else{
		if(gpio_value == 1){
      			PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "down");
			input_event_report(trig_id, 0, 0, 0 ,0, 0, 0);
		}else{
      			PRINT_INFO("Gesture: Tap%d, finger %s!", trig_id, "up");
			input_event_report(trig_id, 0, 1, 0 ,0, 0, 0);
		}
    }
}
#endif
