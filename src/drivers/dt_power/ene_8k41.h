#include <linux/leds.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>

struct ene_8k41_platform_data {
	u8 fw_version;

	bool FW_update_done;
	struct mutex ene_mutex;
	struct led_classdev led;	/* LED control */
};
