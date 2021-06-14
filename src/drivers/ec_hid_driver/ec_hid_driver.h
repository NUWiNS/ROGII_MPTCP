#include <linux/fcntl.h> 
//#include <stdio.h>
//#include <stdlib.h>
#include <linux/unistd.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/hid.h>
#include <linux/semaphore.h>

//Include extcon register
#include <../extcon/extcon.h>

//For HID wait for completion
#include <linux/completion.h>

#define	CLASS_NAME		    "ec_hid"
#define	TEST_STRING		    "JEDI_DONGLE"
//#define HID_PATCH			"/dev/hidraw0"

enum asus_dongle_type
{
	Dongle_NO_INSERT = 0,
	Dongle_INBOX2,
	Dongle_Station2,
	Dongle_DT1,
	Dongle_PCIE,
	Dongle_ERROR,
	Dongle_Others,
	Dongle_default_status = 255,
};

/*
 * 	gDongleEvent : only for Station ( gDongleType == 2 )
 *
 * 	0 	: Normal mode
 * 	1 	: Upgrade mode
 * 	2 	: Low Battery mode
 * 	3 	: ShutDown & Virtual remove mode
 */
enum asus_DongleEvent_type
{
	DongleEvent_Normal_mode = 0,
	DongleEvent_Upgrade_mode,
	DongleEvent_LowBattery_mode,
	DongleEvent_shutdown_remove_mode,
};

extern struct hidraw *g_hidraw;
static struct class *ec_hid_class;

struct ec_hid_data *g_hid_data;
EXPORT_SYMBOL(g_hid_data);

bool station_shutdown = false;
EXPORT_SYMBOL(station_shutdown);

extern void asus_station_enable_host(int);
extern bool Stataion_sd_transfer;
int ec_hid_event_register(struct notifier_block *nb);
int ec_hid_event_unregister(struct notifier_block *nb);

// Block HID input report
bool block_hid_input = false;
EXPORT_SYMBOL(block_hid_input);

bool is_ec_adc_tm5_run = false;

// Record HID used status
bool hid_used = false;
EXPORT_SYMBOL(hid_used);

// Register Hall sensor
bool suspend_by_hall = false;
EXPORT_SYMBOL(suspend_by_hall);

#define MAX_MEMBERS 10
struct vote_member {
	int id;
	char *name;
	bool vote;
};

struct ec_hid_data {
	dev_t devt;
	struct device *dev;
	
	uint8_t previous_event;

	u8 fw_version;

	int pogo_det;
	int pogo_sleep;
	int pogo_aux_oe;
	int pogo_id_adc;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;

	bool lock;
	struct mutex report_mutex;
	struct mutex pogo_mutex;
	struct semaphore pogo_sema;
	struct mutex pogo_id_mutex;

	struct notifier_block notifier;
};

void hid_switch_usb_autosuspend(bool flag){
	struct hid_device *hdev;
	struct usb_interface *intf;

	if (g_hidraw == NULL || g_hid_data->lock) {
		printk("[EC_HID] g_hidraw is NULL or lock %d\n", g_hid_data->lock);
		return;
	}

	hdev = g_hidraw->hid;
	intf = to_usb_interface(hdev->dev.parent);

	printk("[EC_HID] hid_swithc_usb_autosuspend %d\n", flag);
	if(flag) {
		usb_enable_autosuspend(interface_to_usbdev(intf));
	}else {
		usb_disable_autosuspend(interface_to_usbdev(intf));
	}

	return;
}
EXPORT_SYMBOL_GPL(hid_switch_usb_autosuspend);

// For suspend vote
struct vote_member hid_member[MAX_MEMBERS];

#if 0
int hid_check_vote(struct vote_member *hid_member){
	int len=0, result=1;

	printk("[EC_HID] hid_check_vote\n");
	for(len=0 ; len<MAX_MEMBERS ; len++)
	{
		if (hid_member[len].id == -1){
			continue;
		}

		if (hid_member[len].vote == false){
			printk("[EC_HID] Find vote false!!!!\n");
			printk("[EC_HID] ID : %d, NAME : %s, VOTE : %d\n", hid_member[len].id, hid_member[len].name, hid_member[len].vote);
			result = 0;
			return result;
		}
	}
	printk("[EC_HID] Every one votes true!!\n");
	return result;
}

int hid_vote_register(char *name){
	int len;

    mutex_lock(&g_hid_data->report_mutex);
	printk("[EC_HID] hid_vote_register : %s\n", name);

	for(len=0 ; len<MAX_MEMBERS ; len++ )
	{
		if(hid_member[len].id == -1){
			hid_member[len].id = len;
			hid_member[len].name = name;
			hid_member[len].vote = false;
			printk("[EC_HID] ID : %d, NAME : %s, VOTE : %d\n", hid_member[len].id, hid_member[len].name, hid_member[len].vote);
			break;
		}

		if(len == 9){
			printk("[EC_HID] register fail...\n");
			mutex_unlock(&g_hid_data->report_mutex);
			return -1;
		}
	}

    mutex_unlock(&g_hid_data->report_mutex);
	return hid_member[len].id;
}
EXPORT_SYMBOL_GPL(hid_vote_register);

int hid_vote_unregister(int id, char *name){
    mutex_lock(&g_hid_data->report_mutex);
	printk("[EC_HID] hid_vote_unregister : %d : %s\n", id, name);

	if(hid_member[id].id == id && hid_member[id].name != NULL){
		if (!strcmp(hid_member[id].name, name)){
			hid_member[id].id = -1;
			hid_member[id].name = NULL;
			hid_member[id].vote = false;
			printk("[EC_HID] Clear ID : %d\n", id);
		} else
			printk("[EC_HID] Name mismatch. %s\n", hid_member[id].name);
	} else
		printk("[EC_HID] Id or Name mismatch. %d\n", hid_member[id].id);

    mutex_unlock(&g_hid_data->report_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(hid_vote_unregister);

int hid_suspend_vote(int id){
	int result=0;

	if(id<0){
		printk("[EC_HID] error ID %d\n", id);
		return -1;
	}

    mutex_lock(&g_hid_data->report_mutex);

	printk("[EC_HID] hid_suspend_vote : %d\n", id);
	if (hid_member[id].id == id){
		hid_member[id].vote = true;
		printk("[EC_HID] ID : %d, NAME : %s, VOTE : %d\n", hid_member[id].id, hid_member[id].name, hid_member[id].vote);
	} else {
		printk("[EC_HID] Not register this ID. %d\n", id);
		mutex_unlock(&g_hid_data->report_mutex);
		return 0;
	}

	result = hid_check_vote(hid_member);
	//if(result && suspend_by_hall && !Stataion_sd_transfer){
	if(0){
		printk("[EC_HID] trigger USB turn off HOST, %d %d\n", result, suspend_by_hall);
		asus_station_enable_host(0);
	}

    mutex_unlock(&g_hid_data->report_mutex);
    return 1;
}
EXPORT_SYMBOL_GPL(hid_suspend_vote);

void hid_init_vote(struct vote_member *hid_member){
	int len=0;

	printk("[EC_HID] hid_init_vote\n");
	for(len=0 ; len<MAX_MEMBERS ; len++)
	{
		hid_member[len].id = -1;
		hid_member[len].name = NULL;
		hid_member[len].vote = false;
	}
}

void hid_reset_vote(struct vote_member *hid_member){
	int len=0;

	printk("[EC_HID] hid_reset_vote\n");
	for(len=0 ; len<MAX_MEMBERS ; len++)
	{
		hid_member[len].vote = false;
	}
}

void ec_hid_reset_vote(void){
	hid_reset_vote(hid_member);
}
EXPORT_SYMBOL_GPL(ec_hid_reset_vote);
#else
int hid_check_vote(struct vote_member *hid_member){
	return 0;
}

int hid_vote_register(char *name){
	return 0;
}
EXPORT_SYMBOL_GPL(hid_vote_register);

int hid_vote_unregister(int id, char *name){
	return 0;
}
EXPORT_SYMBOL_GPL(hid_vote_unregister);

int hid_suspend_vote(int id){
	return 0;
}
EXPORT_SYMBOL_GPL(hid_suspend_vote);

void hid_init_vote(struct vote_member *hid_member){
}

void hid_reset_vote(struct vote_member *hid_member){
}

void ec_hid_reset_vote(void){
}
EXPORT_SYMBOL_GPL(ec_hid_reset_vote);
#endif
