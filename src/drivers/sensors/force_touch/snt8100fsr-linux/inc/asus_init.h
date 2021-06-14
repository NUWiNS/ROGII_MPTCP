#include "sonacomm.h"

#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
//#include <linux/wakelock.h>
#include "grip_Wakelock.h"
#include "locking.h"
#include "file_control.h"
#include <linux/of_gpio.h>

extern struct delayed_work RST_WORK;
#define asus_grip_queue "snt8100fsr-asus_queue"

#define GRIP_PM8150_GPIO4_LOOKUP_STATE	"grip_clk32"
//#define GRIP_GPIO1_ON_LOOKUP_STATE		"gpio1_pm845"
#define GRIP_SOC_GPIO21_ON_LOOKUP_STATE		"pinctrl_1v2_2v8_init"
#define GRIP_GPIO1_OFF_LOOKUP_STATE		"gpio1_pm845_off"
#define GRIP_PM8150B_GPIO12_LOOKUP_STATE	"gpio12_rst_on"
#define GRIP_PM8150B_GPIO12_OFF	"gpio12_rst_off"
extern void set_pinctrl(struct device *dev, char *str);
extern void set_1V2_2V8_pin_func(struct work_struct *work_orig);
extern void asus_init_probe(void);
extern void check_i2c_error(void);

