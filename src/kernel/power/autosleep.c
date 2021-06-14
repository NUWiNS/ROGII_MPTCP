// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
//[+++]Debug for active wakelock before entering suspend

#include <linux/workqueue.h>
#include <linux/module.h>
//[---]Debug for active wakelock before entering suspend
#include "power.h"

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
//[+++]Debug for active wakelock before entering suspend

struct work_struct pms_printer;

struct work_struct pm_cpuinfo_printer;

void pmsp_print(void);
extern bool g_resume_status;
extern void pmsp_print_gpio_keys(void);
extern void print_pm_cpuinfo_gpio_keys(void);
//[---]Debug for active wakelock before entering suspend
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count) ||
		system_state != SYSTEM_RUNNING) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(autosleep_state);

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
		schedule_timeout_uninterruptible(HZ / 2);

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

extern struct timer_list unattended_timer;//Debug for active wakelock before entering suspend

int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
/*[+++]Debug for active wakelock before entering suspend */
        g_resume_status = false; //in sleep mode

        printk("[PM]unattended_timer: mod_timer (due to auto_sleep)\n");
        mod_timer(&unattended_timer, jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));

/*[---]Debug for active wakelock before entering suspend */
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
/*[+++]Debug for active wakelock before entering suspend */

        printk("[PM]unattended_timer: del_timer (due to late_resume)\n");
        del_timer(&unattended_timer);
/*[---]Debug for active wakelock before entering suspend */
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}
/*[+++]Debug for active wakelock before entering suspend*/
void pmsp_print(void){
    schedule_work(&pms_printer);
    return;
}

EXPORT_SYMBOL(pmsp_print);
void print_pm_cpuinfo(void){
    schedule_work(&pm_cpuinfo_printer);
    return;
}
EXPORT_SYMBOL(print_pm_cpuinfo);
void pms_printer_func(struct work_struct *work){

	

}
void pm_cpuinfo_func(struct work_struct *work){

	

}
/*[---]Debug for active wakelock before entering suspend*/
int __init pm_autosleep_init(void)
{
/*[+++]Debug for active wakelock before entering suspend*/
    int ret = 0;

    INIT_WORK(&pms_printer, pms_printer_func);
    //ret = switch_dev_register(&pmsp_dev);
    if (ret < 0)
        printk("[PM]%s:fail to register switch power_manager_printer \n",__func__);
    else
        printk("[PM]%s:success to register pmsp switch \n",__func__);
 
    INIT_WORK(&pm_cpuinfo_printer, pm_cpuinfo_func);
    //ret = switch_dev_register(&pm_dumpthread_dev);
    if (ret < 0)
        printk("[PM]%s:fail to register switch device pm_dumpthread_dev\n",__func__);
    else
        printk("[PM]%s:success to register switch device pm_dumpthread_dev\n",__func__);
/*[---]Debug for active wakelock before entering suspend*/
	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
