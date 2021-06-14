/*****************************************************************************
* File: i2c-bus.c
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "workqueue.h"

#include "config.h"
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "debug.h"

#include "sonacomm.h"

#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
//#include <linux/wakelock.h>
#include "grip_Wakelock.h"
#include "file_control.h"
#include "workqueue.h"
#include "locking.h"
#include "asus_init.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MAX_CMD_FIFO        256
#define CMD_HDR_SIZE        8
#define MAX_CMD_BYTES       (MAX_CMD_FIFO + 2*CMD_HDR_SIZE)

#define DATA_SIZE           (17 * 1024)

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
int snt_i2c_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in);
int snt_i2c_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out);

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t *request_out;
static uint8_t  *status_in;
static uint8_t  *data_in;
static uint8_t  *data_out;
extern int SNT_SUSPEND_FLAG;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int snt_i2c_open(struct i2c_client *i2c) {
    // Allocate our data buffers in contiguous memory for DMA support
    status_in = memory_allocate(I2C_FIFO_SIZE, GFP_DMA);
    if (status_in == NULL) {
        PRINT_CRIT("status_in = memory_allocate(%d) failed", I2C_FIFO_SIZE);
        return -1;
    }

    request_out = memory_allocate(I2C_FIFO_SIZE, GFP_DMA);
    if (request_out == NULL) {
        PRINT_CRIT("request_out = memory_allocate(%d) failed", I2C_FIFO_SIZE);
        return -1;
    }

    data_in = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    data_out = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    PRINT_INFO("I2C Address: 0x%02X", i2c->addr);

    PRINT_DEBUG("done");
    return 0;
}

void snt_i2c_close(struct snt8100fsr *snt8100fsr) {
    PRINT_FUNC();

    if (request_out) {
        memory_free(request_out);
        request_out = NULL;
    }

    if (status_in) {
        memory_free(status_in);
        status_in = NULL;
    }

    if (data_in) {
        memory_free(data_in);
        data_in = NULL;
    }

    if (data_out) {
        memory_free(data_out);
        data_out = NULL;
    }

    PRINT_DEBUG("done");
}

/*
 * snt_i2c_get_boot_status() can only be called during firmware loading
 * prior to the last interrupt saying the system is ready.
 */
uint8_t snt_i2c_get_boot_status(struct snt8100fsr *snt8100fsr) {
    PRINT_CRIT("Not Implemented\n");
    return 0xFF;
}


/*
 * Wake the device up over the I2C bus by issuing a write of 4 bytes
 * to the I2C Wake Device which operates on a different I2C address.
 */
int snt_i2c_wake_device(struct snt8100fsr *snt8100fsr) {
    int ret;
    uint8_t buf[4] = {0, 0, 0, 0};

    PRINT_FUNC();
    //PRINT_INFO("DPC: Wake device");
    // We will use the I2C Wake Device for this command
    if (snt8100fsr_wake_i2c_g == NULL) {
        PRINT_CRIT("Unable to wake device due to missing wake I2C device");
        return -1;
    }

    ret = snt_i2c_write(snt8100fsr_wake_i2c_g, 4, buf);

    if (ret) {
        PRINT_ERR("snt_i2c_write(snt8100fsr_wake_i2c, 4, buf) failed!");
        return ret;
    }

    PRINT_DEBUG("done");
    return 0;
}

int snt_i2c_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in) {

    int count;
    PRINT_FUNC("%d bytes", num_read);

    count = i2c_master_recv(snt8100fsr->i2c, data_in, num_read);
    if (count < 0) {
	check_i2c_error();
        PRINT_CRIT("I2C write failed, error = %d\n", count);
        return -1;
    } else if (count != num_read) {
        PRINT_CRIT("I2C read of %d bytes only read %d bytes",
                   num_read, count);
        return -1;
    }

    PRINT_DEBUG("%d bytes read", num_read);
    return 0;
}

extern int write_fail_count;
int snt_i2c_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out) {
    int count;
#ifdef SNT_I2C_WRITE_DEBUG_VERBOSE  
    PRINT_FUNC("%d bytes", num_write);
#endif
    /*ASUS BSP Clay +++: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/
    if(write_fail_count>0){
	PRINT_CRIT("Skip wakeup i2c write since write_fail_count = %d", write_fail_count);
	return -1;
    }
    /*ASUS BSP Clay ---: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/
    count = i2c_master_send(snt8100fsr->i2c, data_out, num_write);
    if (count < 0) {
	check_i2c_error();
        PRINT_CRIT("I2C write failed, error = %d\n", count);
        return -1;
    } else if (count != num_write) {
        PRINT_CRIT("I2C write of %d bytes only wrote %d bytes",
                   num_write, count);
        return -1;
    }
#ifdef SNT_I2C_WRITE_DEBUG_VERBOSE
    PRINT_DEBUG("%d bytes written", num_write);
#endif  
    return 0;
}

int snt_i2c_read_fifo_pkt(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    uint8_t addr_phase[2];
    int count;
#ifdef SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
    PRINT_FUNC("len %d", len);
#endif
    addr_phase[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
    if (len > I2C_MAX_PKT_SIZE_BYTES) {
        PRINT_CRIT("Warning. Max I2C read is %d bytes, truncating.\n", I2C_MAX_PKT_SIZE_BYTES);
        len = I2C_MAX_PKT_SIZE_BYTES;
    }

    if (len == 0) {
        PRINT_CRIT("ERROR: Must read at least 1 word\n");
        return -1;
    }

    addr_phase[1] = (len / 2) - 1;
    count = i2c_master_send(snt8100fsr->i2c, addr_phase, 2);
    if (count != 2) {
	check_i2c_error();
        PRINT_CRIT("I2C header write failed len = %d\n", count);
        PRINT_CRIT("len = 0x%04x, addr_phase[0]=0x%02x, addr_phase[1]=0x%02x\n", 
			len, addr_phase[0], addr_phase[1]);
        return -1;
    }

    count = i2c_master_recv(snt8100fsr->i2c, in_val, len);
    if (count < 0) {
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != len) {
        PRINT_CRIT("I2C Read failed len=%d (expected %d)\n",
                   len, count);
        return -1;
    }
#ifdef SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
    PRINT_DEBUG("done");
#endif  
    return 0;
}


int snt_i2c_read_fifo(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    int ret = 0;

   //PRINT_FUNC("reg 0x%X", reg);

    while (len != 0 && ret == 0) {
        uint16_t pkt_len = (len > I2C_MAX_PKT_SIZE_BYTES) ? I2C_MAX_PKT_SIZE_BYTES : len;
        len -= pkt_len;
        ret = snt_i2c_read_fifo_pkt(snt8100fsr, reg, pkt_len, in_val);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt read failed at len %d, reg 0x%02X", len, reg);
        }
        in_val += pkt_len;
    }
    //PRINT_DEBUG("done");
    return ret;
}
int snt_i2c_write_fifo_pkt(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int write_len;
    int count;

    PRINT_FUNC("len %d", len);
	/*ASUS BSP Clay +++: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/
	if(write_fail_count>0){
		PRINT_CRIT("Skip i2c write since write_fail_count = %d", write_fail_count);
		return -1;
	}
	/*ASUS BSP Clay ---: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/
	
    data_out[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
    if (len > I2C_MAX_PKT_SIZE_BYTES) {
      PRINT_CRIT("Warning. Max i2c register write is %d bytes, truncating.\n", I2C_MAX_PKT_SIZE_BYTES);
      len = I2C_MAX_PKT_SIZE_BYTES;
    }
    if (len == 0) {
      PRINT_CRIT("ERROR: Must write at least 1 word\n");
      return -1;
    }

    data_out[1] = (len / 2) - 1;
    memcpy(&data_out[2], out_val, len);
    write_len = len + 2;

    /* Debug output of the hex values of the data
    for(count = 0; count < write_len; count++) {
        PRINT_DEBUG("%02X (%d)", data_out[count], data_out[count]);
    }*/
    count = i2c_master_send(snt8100fsr->i2c, data_out, write_len);
    if (count < 0) {
      check_i2c_error();
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != write_len) {
      PRINT_CRIT("ERROR: I2C Write failed len=%d (expected %d)\n", count, 
write_len);
      return -1;
    }
    PRINT_DEBUG("done");
    return 0;
}

int snt_i2c_write_fifo(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int ret = 0;
    PRINT_FUNC("reg 0x%X", reg);
    while (len != 0 && ret == 0) {
        uint16_t pkt_len = (len > I2C_MAX_PKT_SIZE_BYTES) ? I2C_MAX_PKT_SIZE_BYTES : len;
        len -= pkt_len;
        ret = snt_i2c_write_fifo_pkt(snt8100fsr, reg, pkt_len, out_val);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt write failed at len %d, reg 0x%02X", len, reg);
        }
        out_val += pkt_len;
    }
    PRINT_DEBUG("done");
    return ret;
}


/*==========================================================================*/
/* Device Probe/Remove/Resume/Suspend                                       */
/*==========================================================================*/
#ifdef USE_I2C_BUS
static int snt_i2c_probe(struct i2c_client *i2c,
                         const struct i2c_device_id *id)
{
    int ret;
    struct device *dev = &i2c->dev;
    struct device_node *np = dev->of_node;
    struct snt8100fsr *snt8100fsr;
    PRINT_FUNC();
    PRINT_INFO("snt_i2c_probe Enter\n");
	PRINT_INFO("Host irq GPIO%d: %d", IRQ_GPIO, gpio_get_value(IRQ_GPIO));
	PRINT_INFO("Tap IRQ GPIO%d: %d", TRIG0_GPIO, gpio_get_value(TRIG0_GPIO));
	PRINT_INFO("Tap IRQ GPIO%d: %d", TRIG1_GPIO, gpio_get_value(TRIG1_GPIO));
	PRINT_INFO("Tap IRQ GPIO%d: %d", TRIG2_GPIO, gpio_get_value(TRIG2_GPIO));
	PRINT_INFO("Rst GPIO%d: %d", RST_GPIO, gpio_get_value(RST_GPIO));
	
	//fp_device = kzalloc(sizeof(*fp_device), GFP_KERNEL);
    snt8100fsr = kzalloc(sizeof(*snt8100fsr),
                                 GFP_KERNEL);

    if(!snt8100fsr) {
        PRINT_CRIT("failed to allocate memory for struct snt8100fsr");
        return -ENOMEM;
    }

    memset(snt8100fsr, 0, sizeof(*snt8100fsr));

    snt8100fsr->bus_type = BUS_TYPE_I2C;
    snt8100fsr->dev = dev;
    i2c_set_clientdata(i2c, snt8100fsr);
    snt8100fsr->i2c = i2c;

    //gpio_reset = of_get_named_gpio(dev->of_node, "snt,gpio_reset", 0);
    //PRINT_INFO("gpio_test = %d", gpio_reset);
    //PRINT_INFO("GPIO133: %d",gpio_get_value(RST_GPIO));
    //if(gpio_request(gpio_reset, "snt_gpio_reset")){
    //    gpio_direction_output(gpio_reset, 1);
    //}
    
    /* We use two I2C devices to communicate with the snt8100fsr chip.
     * The main address is used for general communication, the secondary
     * address is used only to awaken the device when it is in sleep mode.
     * We must detect which I2C device is being initialized, the main one,
     * or the secondary address.
     */	
    if (of_property_read_bool(np, "wake-device")) {
	// Initialization for wake i2c device only, should be put after this line
        PRINT_INFO("Wake I2C device discovered");
        snt8100fsr->wake_i2c_device = true;
        snt8100fsr_wake_i2c_g = snt8100fsr;
	//pull-up GPIO4
	if (of_property_read_bool(np, "grip_gpio4")){
		PRINT_INFO("get_GPIO4");
		set_pinctrl(dev, GRIP_PM8150_GPIO4_LOOKUP_STATE);
	}

        // We don't do any more init for this device
        PRINT_DEBUG("done");
        return 0;
    }

    PRINT_INFO("Main I2C device discovered");
    snt8100fsr->wake_i2c_device = false;
    snt8100fsr->en_demo = 0;
    snt8100fsr->en_sensor_evt = 1;
    snt8100fsr->snt_state = GRIP_SHUTDOWN;
    snt8100fsr->sleep_ms_time = 10;

    // Save this as our main device and to be used for sysFS access
    snt8100fsr_g = snt8100fsr;

#ifdef DYNAMIC_PWR_CTL
    // create wake semaphore
    sema_init(&snt8100fsr_g->wake_rsp, 0);
    sema_init(&snt8100fsr_g->wake_req, 0);
    snt8100fsr_g->enable_dpc_flag = DYNAMIC_PWR_CTL;
#endif

    // Set up semaphores to control waiting for sonacomm response
    sema_init(&snt8100fsr_g->sc_wf_rsp_req, 0);
    sema_init(&snt8100fsr_g->sc_wf_rsp, 0);

    ret = main_init();
    if (ret) {
        PRINT_CRIT("main_init() failed");
        return ret;
    }

    ret = snt_i2c_device_init(i2c, snt8100fsr);
    if(ret) {
        PRINT_CRIT("snt_i2c_device_init() failed");
        return ret;
    }

    ret = snt_i2c_open(snt8100fsr->i2c);
        if (ret) {
        PRINT_CRIT("snt_i2c_open() failed");
        return ret;
    }
	asus_init_probe();

    /*
     * Upload the firmware asynchronously. When finished,
     * it will call start_event_processing() in event.c
     */     
#ifdef UPLOAD_FIRMWARE
    upload_firmware(snt8100fsr, FIRMWARE_LOCATION);
#else
    start_event_processing(snt8100fsr);
#endif

    // Start our sysfs interface
    snt_sysfs_init(snt8100fsr_g, true);

    PRINT_DEBUG("done");
    return 0;
}

static int snt_i2c_remove(struct i2c_client *i2c)
{
    struct snt8100fsr *snt8100fsr;
    PRINT_FUNC();

    /* Check if this is the i2c_wake_device used to awaken a sleeping
     * snt8100fsr chip.
     */
    snt8100fsr = i2c_get_clientdata(i2c);
    if (snt8100fsr && snt8100fsr->wake_i2c_device) {
        PRINT_INFO("removing wake_i2c_device");
        return 0;
    }

    // Else we are on the main i2c device, uninit the sysfs interface
    if (snt8100fsr_g) {
        snt_sysfs_init(snt8100fsr_g, false);
    }

    main_exit();
    return 0;
}

static int snt_i2c_suspend(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    PRINT_INFO("snt_i2c_suspend+++");
    ret = snt_suspend(dev);
	SNT_SUSPEND_FLAG = 0;
    PRINT_DEBUG("done");
    PRINT_INFO("snt_i2c_suspend---");
    return ret;
}

static int snt_i2c_resume(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    PRINT_INFO("snt_i2c_resume+++");
     ret = snt_resume(dev);
	SNT_SUSPEND_FLAG = 1;
     PRINT_DEBUG("done");
    PRINT_INFO("snt_i2c_resume---");
    return ret;
}

/* Clay +++ */
//#ifdef CLAY_WANG_FLAG
static const struct of_device_id snt8100fsr_i2c_dt_ids[] = {
    { .compatible = "sentons, snt8100fsr-i2c" },
//{ .compatible = "qcom,grip",},
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, snt8100fsr_i2c_dt_ids);
//#endif
/* Clay --- */

static struct i2c_device_id snt_i2c_id[] = {
    {SENTONS_DRIVER_NAME, 0},
//{"grip_sensor", 0},
    {},
};

static const struct dev_pm_ops snt_i2c_pm_ops = {
    .suspend = snt_i2c_suspend,
    .resume = snt_i2c_resume,
};

static struct i2c_driver snt_i2c_driver = {
    .driver = {
           .name = SENTONS_DRIVER_NAME,
           .owner = THIS_MODULE,
          .of_match_table = of_match_ptr(snt8100fsr_i2c_dt_ids),
           .pm = &snt_i2c_pm_ops,
           },
    .probe = snt_i2c_probe,
    .remove = snt_i2c_remove,
    .id_table = snt_i2c_id,
};

MODULE_DEVICE_TABLE(i2c, snt_i2c_id);

module_i2c_driver(snt_i2c_driver);
#endif
