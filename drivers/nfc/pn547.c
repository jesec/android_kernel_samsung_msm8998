/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "pn547.h"
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
#include <mach/msm_xo.h>
#include <linux/workqueue.h>
#endif
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>

#define SIG_NFC 44
#define MAX_BUFFER_SIZE		512
#define NFC_DEBUG 0

#define FEATURE_NFC_TEST
#define FEATURE_PN80T

struct pn547_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice pn547_device;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
	unsigned int irq_gpio;
	int pvdd;
	const char *nfc_pvdd;

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	unsigned int ese_pwr_req;
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
#endif
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */

	atomic_t irq_enabled;
	atomic_t read_flag;
	bool cancel_read;
	struct wake_lock nfc_wake_lock;
	struct clk *nfc_clock;
	unsigned int clk_req_gpio;
	int i2c_probe;

#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	unsigned int clk_req_gpio;
	unsigned int clk_req_irq;
	struct msm_xo_voter *nfc_clock;
	struct work_struct work_nfc_clock;
	struct workqueue_struct *wq_clock;
	bool clock_state;
#endif
	long	nfc_service_pid; /*used to signal the nfc the nfc service */
};

static struct pn547_dev *pn547_dev;

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
static struct semaphore ese_access_sema;
static void release_ese_lock(p61_access_state_t  p61_current_state);
static struct semaphore svdd_sync_onoff_sema;
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout);
static unsigned char svdd_sync_wait;
#endif

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
#if NFC_DEBUG
		pr_err("%s, irq_gpio = %d\n", __func__,
			gpio_get_value(pn547_dev->irq_gpio));
#endif
		return IRQ_HANDLED;
	}

	/* Wake up waiting readers */
	atomic_set(&pn547_dev->read_flag, 1);
	wake_up(&pn547_dev->read_wq);


#if NFC_DEBUG
	pr_info("pn547 : call\n");
#endif
	wake_lock_timeout(&pn547_dev->nfc_wake_lock, 2*HZ);
	return IRQ_HANDLED;
}

#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
static void nfc_work_func_clock(struct work_struct *work)
{
	struct pn547_dev *pn547_dev = container_of(work, struct pn547_dev,
					      work_nfc_clock);
	int ret = 0;

	if (gpio_get_value(pn547_dev->clk_req_gpio)) {
		if (pn547_dev->clock_state == false) {
			ret = msm_xo_mode_vote(pn547_dev->nfc_clock,
					MSM_XO_MODE_ON);
			if (ret < 0)
				pr_err("%s: Failed to vote TCX0_A1 ON (%d)\n",
						__func__, ret);
			pn547_dev->clock_state = true;
		}
	} else {
		if (pn547_dev->clock_state == true) {
			ret = msm_xo_mode_vote(pn547_dev->nfc_clock,
					MSM_XO_MODE_OFF);
			if (ret < 0)
				pr_err("%s: Failed to vote TCX0_A1 OFF (%d)\n",
						__func__, ret);
			pn547_dev->clock_state = false;
		}
	}
}

static irqreturn_t pn547_dev_clk_req_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;
	queue_work(pn547_dev->wq_clock, &pn547_dev->work_nfc_clock);
	return IRQ_HANDLED;
}
#endif

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE] = { 0, };
	int ret = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes. irq=%s\n", __func__, count,
			gpio_get_value(pn547_dev->irq_gpio) ? "1" : "0");

#if NFC_DEBUG
	pr_info("pn547 : + r\n");
#endif

	mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		atomic_set(&pn547_dev->read_flag, 0);
		if (filp->f_flags & O_NONBLOCK) {
			pr_info("%s : O_NONBLOCK\n", __func__);
			ret = -EAGAIN;
			goto fail;
		}

#if NFC_DEBUG
		pr_info("pn547: wait_event_interruptible : in\n");
#endif
		if (!gpio_get_value(pn547_dev->irq_gpio))
			ret = wait_event_interruptible(pn547_dev->read_wq,
					atomic_read(&pn547_dev->read_flag));

#if NFC_DEBUG
		pr_info("pn547 :   h\n");
#endif

		if (pn547_dev->cancel_read) {
			pn547_dev->cancel_read = false;
			ret = -1;
			goto fail;
		}

		if (ret)
			goto fail;

	}

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);

#if NFC_DEBUG
	pr_info("pn547: i2c_master_recv\n");
#endif
	mutex_unlock(&pn547_dev->read_mutex);
	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned (%d,%d)\n", __func__,
				ret, pn547_dev->i2c_probe);
		return ret;
	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
				__func__, ret);
		return -EIO;
	}

	if (copy_to_user(buf, tmp, ret)) {
		pr_err("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev;
	char tmp[MAX_BUFFER_SIZE] = { 0,};
	int ret = 0, retry = 2;

	pn547_dev = filp->private_data;

#if NFC_DEBUG
	pr_info("pn547 : + w\n");
#endif

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	do {
		retry--;
		ret = i2c_master_send(pn547_dev->client, tmp, count);
		if (ret == count)
			break;
		usleep_range(6000, 10000); /* Retry, chip was in standby */
#if NFC_DEBUG
		pr_debug("%s : retry = %d\n", __func__, retry);
#endif
	} while (retry);

#if NFC_DEBUG
	pr_info("pn547 : - w\n");
#endif

	if (ret != count) {
		pr_err("%s : i2c_master_send returned (%d,%d)\n", __func__,
				ret, pn547_dev->i2c_probe);
		ret = -EIO;
	}

	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						   struct pn547_dev,
						   pn547_device);
	filp->private_data = pn547_dev;
	pr_info("%s : %d,%d (%d)\n", __func__, imajor(inode), iminor(inode),
			pn547_dev->i2c_probe);

	return 0;
}

static int pn547_dev_release(struct inode *inode, struct file *filp)
{
	pr_info("%s\n", __func__);
	return 0;
}

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
static void p61_update_access_state(struct pn547_dev *pn547_dev,
		p61_access_state_t current_state, bool set)
{
    /*pr_info("%s: Enter current_state = 0x%x\n", __func__,
			pn547_dev->p61_current_state);*/
    if (current_state) {
        if(set){
            if(pn547_dev->p61_current_state == P61_STATE_IDLE)
                pn547_dev->p61_current_state = P61_STATE_INVALID;
            pn547_dev->p61_current_state |= current_state;
        }
        else{
            pn547_dev->p61_current_state ^= current_state;
            if(!pn547_dev->p61_current_state)
                pn547_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    pr_info("%s pn547: Exit current_state = 0x%x\n", __func__,
			pn547_dev->p61_current_state);
}

static void p61_get_access_state(struct pn547_dev *pn547_dev,
		p61_access_state_t *current_state)
{
    if (current_state == NULL)
        pr_err("%s : invalid state of p61_access_state_t\n", __func__);
    else
        *current_state = pn547_dev->p61_current_state;
}

static void p61_access_lock(struct pn547_dev *pn547_dev)
{
	pr_info("%s\n", __func__);
	mutex_lock(&pn547_dev->p61_state_mutex);
}

static void p61_access_unlock(struct pn547_dev *pn547_dev)
{
	pr_info("%s\n", __func__);
	mutex_unlock(&pn547_dev->p61_state_mutex);
}

static int signal_handler(p61_access_state_t state, long nfc_pid)
{
	struct siginfo sinfo;
	pid_t pid;
	struct task_struct *task;
	int sigret = 0;
	int ret = 0;

	pr_info("pn547 %s pid:%ld\n", __func__, nfc_pid);
    if(nfc_pid == 0)
    {
        pr_info("nfc_pid is clear don't call signal_handler.\n");
		return -1;
    }

	memset(&sinfo, 0, sizeof(struct siginfo));
	sinfo.si_signo = SIG_NFC;
	sinfo.si_code = SI_QUEUE;
	sinfo.si_int = state;
	pid = nfc_pid;

    task = pid_task(find_vpid(pid), PIDTYPE_PID);
    if (task) {
        pr_info("pn547 %s.\n", task->comm);
        sigret = send_sig_info(SIG_NFC, &sinfo, task);
        if (sigret < 0) {
			pr_info("pn547 send_sig_info failed.. %d.\n", sigret);
			ret = -1;
        }
    }
	else {
		pr_info("pn547 finding task from PID failed\n");
		ret = -1;
    }

	return ret;
}

static void svdd_sync_onoff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; /*100 ms timeout*/
    unsigned long tempJ = msecs_to_jiffies(timeout);
    pr_info("%s: Enter nfc_service_pid: %ld\n", __func__, nfc_service_pid);
    if(nfc_service_pid) {
        if (0 == signal_handler(origin, nfc_service_pid)) {
            sema_init(&svdd_sync_onoff_sema, 0);
			svdd_sync_wait = 1;
            pr_info("Waiting for svdd protection response");
            if (down_timeout(&svdd_sync_onoff_sema, tempJ) != 0)
            {
                pr_info("svdd wait protection: Timeout");
            }
            pr_info("svdd wait protection : released");
			svdd_sync_wait = 0;
        }
    }
    pr_info("%s: Exit\n", __func__);
}

static int release_svdd_wait(void)
{
    unsigned char i = 0;
    pr_info("%s: Enter \n", __func__);
    
    for(i=0;i<9;i++) {
        if(svdd_sync_wait) {
            up(&svdd_sync_onoff_sema);
            svdd_sync_wait = 0;
            break;
        } else msleep(10);
    }
    pr_info("%s: Exit\n", __func__);
    return 0;
}

static int pn547_set_pwr(struct pn547_dev *pdev, unsigned long arg)
{
	int ret = 0;
	p61_access_state_t current_state;
	
	p61_get_access_state(pdev, &current_state);
	switch (arg) {
	case 0: /* power off */
		if (atomic_read(&pdev->irq_enabled) == 1) {
			atomic_set(&pdev->irq_enabled, 0);
			disable_irq_wake(pdev->client->irq);
			disable_irq_nosync(pdev->client->irq);
		}
		if(current_state & P61_STATE_DWNLD) 
			p61_update_access_state(pdev, P61_STATE_DWNLD, false);

		if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
			p61_update_access_state(pdev, P61_STATE_IDLE, true);

		pr_info("%s power off, irq=%d\n", __func__,
			atomic_read(&pdev->irq_enabled));
		gpio_set_value(pdev->firm_gpio, 0);

		pdev->nfc_ven_enabled = false;
		/* Don't change Ven state if spi made it high */
		if (pdev->spi_ven_enabled == false) {
			gpio_set_value_cansleep(pdev->ven_gpio, 0);
		}
		usleep_range(4900, 5000);
		break;
	case 1: /* power on */
		if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0)
				p61_update_access_state(pdev, P61_STATE_IDLE, true);

		if(current_state & P61_STATE_DWNLD) 
			p61_update_access_state(pdev, P61_STATE_DWNLD, false);

		gpio_set_value(pdev->firm_gpio, 0);

		pdev->nfc_ven_enabled = true;
		if (pdev->spi_ven_enabled == false) {
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
		}
		usleep_range(4900, 5000);
		if (atomic_read(&pdev->irq_enabled) == 0) {
			atomic_set(&pdev->irq_enabled, 1);
			enable_irq(pdev->client->irq);
			enable_irq_wake(pdev->client->irq);
		}

		svdd_sync_wait = 0;

		pr_info("%s power on, irq=%d\n", __func__,
			atomic_read(&pdev->irq_enabled));
		break;
	case 2:
		if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
		/* NFCC fw/download should not be allowed if p61 is used by SPI */
			pr_info("%s not be allowed to reset/FW download \n",
					__func__);
			/*p61_access_unlock(pn547_dev); redundant */
			return -EBUSY; /* Device or resource busy */
		}
		pdev->nfc_ven_enabled = true;
		if (pdev->spi_ven_enabled == false) {
			/* power on with firmware download (requires hw reset) */
			p61_update_access_state(pdev, P61_STATE_DWNLD, true);
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
			gpio_set_value(pdev->firm_gpio, 1);
			usleep_range(4900, 5000);
			gpio_set_value_cansleep(pdev->ven_gpio, 0);
			usleep_range(4900, 5000);
			gpio_set_value_cansleep(pdev->ven_gpio, 1);
			usleep_range(4900, 5000);
			if (atomic_read(&pdev->irq_enabled) == 0) {
				atomic_set(&pdev->irq_enabled, 1);
				enable_irq(pdev->client->irq);
				enable_irq_wake(pdev->client->irq);
			}
			pr_info("%s power on with firmware, irq=%d\n", __func__,
					atomic_read(&pdev->irq_enabled));
			pr_info("%s: VEN=%d FIRM=%d\n", __func__,
					gpio_get_value(pdev->ven_gpio),
					gpio_get_value(pdev->firm_gpio));
		}
		break;
	case 3:
		pr_info("%s Read Cancel\n", __func__);
		pdev->cancel_read = true;
		atomic_set(&pdev->read_flag, 1);
		wake_up(&pdev->read_wq);
		break;
	default:
		pr_err("%s bad arg %lu\n", __func__, arg);
		/* changed the p61 state to idle*/
		/*p61_access_unlock(pn547_dev); redundant*/
		ret = -EINVAL;
	}
	return ret;
}

static int pn547_p61_set_spi_pwr(struct pn547_dev *pdev, unsigned long arg)
{
	int ret = 0;
	p61_access_state_t current_state;

	p61_get_access_state(pn547_dev, &current_state);
	pr_info("%s: PN61_SET_SPI_PWR cur=0x%x\n", __func__, current_state);
	switch (arg) {
	case 0: /*else if (arg == 0)*/
#ifdef FEATURE_PN80T
		pr_info("%s : power off ese PN80T\n", __func__);
		if(current_state & P61_STATE_SPI_PRIO) {
			p61_update_access_state(pn547_dev, P61_STATE_SPI_PRIO, false);
			if (!(current_state & P61_STATE_JCP_DWNLD)) {
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid %ld\n", __func__,
							pn547_dev->nfc_service_pid);
                    if (!(current_state & P61_STATE_WIRED)) {
                        svdd_sync_onoff(pn547_dev->nfc_service_pid, 
								P61_STATE_SPI_SVDD_SYNC_START |
								P61_STATE_SPI_PRIO_END);
                    } else
                        signal_handler(P61_STATE_SPI_PRIO_END,
								pn547_dev->nfc_service_pid);
				} 
				else {
					pr_info("%s invalid nfc service pid.. %ld\n", __func__,
							pn547_dev->nfc_service_pid);
				}
			} else if (!(current_state & P61_STATE_WIRED)) {
				svdd_sync_onoff(pn547_dev->nfc_service_pid, 
						P61_STATE_SPI_SVDD_SYNC_START);
			}
			pn547_dev->spi_ven_enabled = false;
        	if (!(current_state & P61_STATE_WIRED)) {
                gpio_set_value(pn547_dev->ese_pwr_req, 0);
                msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_END);
            }
			if (pn547_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn547_dev->ven_gpio, 0);
				msleep(10);
			}
		}
		else if (current_state & P61_STATE_SPI) {
			p61_update_access_state(pn547_dev, P61_STATE_SPI, false);
			if (!(current_state & P61_STATE_WIRED) &&
					!(current_state & P61_STATE_JCP_DWNLD)) {
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid ---- %ld\n", __func__,
							pn547_dev->nfc_service_pid);
					svdd_sync_onoff(pn547_dev->nfc_service_pid, 
						P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
				}
				else {
					pr_info("%s invalid nfc service pid..--- %ld\n",
							__func__, pn547_dev->nfc_service_pid);
				}
				gpio_set_value(pn547_dev->ese_pwr_req, 0);
				msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid, 
						P61_STATE_SPI_SVDD_SYNC_END);
			}
			/*If JCOP3.2 or 3.3 for handling triple mode protection signal NFC service */
			else {
				if (!(current_state & P61_STATE_JCP_DWNLD)) {
					if (pn547_dev->nfc_service_pid) {
						pr_info("%s nfc service pid %ld\n", __func__,
								pn547_dev->nfc_service_pid);
						svdd_sync_onoff(pn547_dev->nfc_service_pid, 
							P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
					}
					else {
						pr_info("%s: invalid nfc service pid.. %ld\n",
								__func__, pn547_dev->nfc_service_pid);
					}
				}
                else {
					svdd_sync_onoff(pn547_dev->nfc_service_pid,
							P61_STATE_SPI_SVDD_SYNC_START);
                }

                gpio_set_value(pn547_dev->ese_pwr_req, 0);
                msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_END);
                pr_info("PN80T legacy ese_pwr_gpio off %s", __func__);
			}
			pn547_dev->spi_ven_enabled = false;
			if (pn547_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn547_dev->ven_gpio, 0);
				msleep(10);
			}
		}
		else {
			pr_err("%s: power off ese failed, current_state = 0x%x\n",
					__func__, pn547_dev->p61_current_state);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EPERM; /* Operation not permitted */
		}
#else
		pr_info("%s : power off ese\n", __func__);
		if(current_state & P61_STATE_SPI_PRIO){
			p61_update_access_state(pn547_dev, P61_STATE_SPI_PRIO, false);
			if (!(current_state & P61_STATE_WIRED)) {
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_PRIO_END);
				gpio_set_value(pn547_dev->ese_pwr_req, 0);
				msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_END);
			}
			else {
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid %ld\n", __func__,
							pn547_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO_END,
							pn547_dev->nfc_service_pid);
				}
				else {
					pr_info("%s invalid nfc service pid.. %ld\n", __func__,
							pn547_dev->nfc_service_pid);
				}
			}	
			pn547_dev->spi_ven_enabled = false;
			if (pn547_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn547_dev->ven_gpio, 0);
				msleep(10);
			}
		}
		else if (current_state & P61_STATE_SPI) {
			p61_update_access_state(pn547_dev, P61_STATE_SPI, false);
			if (!(current_state & P61_STATE_WIRED)) {
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
				gpio_set_value(pn547_dev->ese_pwr_req, 0);
				msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_SPI_SVDD_SYNC_END);
			}
			/*If JCOP3.2 or 3.3 for handling triple mode protection signal NFC service */
			else
			{
				/* if (current_state & P61_STATE_WIRED) */
				{
					if (pn547_dev->nfc_service_pid){
						pr_info("%s nfc service pid %ld\n", __func__,
								pn547_dev->nfc_service_pid);
						signal_handler(P61_STATE_SPI_END,
								pn547_dev->nfc_service_pid);
					} else {
						pr_info("%s: invalid nfc service pid.. %ld\n",
								__func__, pn547_dev->nfc_service_pid);
					}
				}
			}
			pn547_dev->spi_ven_enabled = false;
			if (pn547_dev->nfc_ven_enabled == false) {
				gpio_set_value(pn547_dev->ven_gpio, 0);
				msleep(10);
			}
		}
		else {
			pr_err("%s: power off ese failed, current_state = 0x%x\n",
					__func__, pn547_dev->p61_current_state);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EPERM; /* Operation not permitted */
		}
#endif
		break;
	case 1: /*if (arg == 1) */
		pr_info("%s : power on ese\n", __func__);
		if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO|P61_STATE_DWNLD))==0)
		{
			p61_update_access_state(pn547_dev, P61_STATE_SPI, true);
			/*To handle triple mode protection signal
			  NFC service when SPI session started*/ 
			if (!(current_state & P61_STATE_JCP_DWNLD)) {/*FEATURE_PN80T*/
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid ---- %ld\n", __func__,
							pn547_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI, pn547_dev->nfc_service_pid);
				}
				else {
					pr_info("%s invalid nfc service pid %ld\n", __func__,
						pn547_dev->nfc_service_pid);
				}
			}
			pn547_dev->spi_ven_enabled = true;
			if (pn547_dev->nfc_ven_enabled == false) {
				/* provide power to NFCC if, NFC service not provided */
				gpio_set_value(pn547_dev->ven_gpio, 1);
				msleep(10);
			}
			/* pull the gpio to high once NFCC is power on*/
			gpio_set_value(pn547_dev->ese_pwr_req, 1);
			msleep(10);
		} else {
			pr_info("%s : PN61_SET_SPI_PWR -  power on ese failed\n", __func__);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 2: /*	else if (arg == 2) */
		pr_info("%s : reset\n", __func__);
		if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
			if (pn547_dev->spi_ven_enabled == false) {
				pn547_dev->spi_ven_enabled = true;
				if (pn547_dev->nfc_ven_enabled == false) {
					/* provide power to NFCC if, NFC service not provided */
					gpio_set_value(pn547_dev->ven_gpio, 1);
					msleep(10);
				}
			}
			svdd_sync_onoff(pn547_dev->nfc_service_pid,
					P61_STATE_SPI_SVDD_SYNC_START);
			gpio_set_value(pn547_dev->ese_pwr_req, 0);
			msleep(60);
			svdd_sync_onoff(pn547_dev->nfc_service_pid,
					P61_STATE_SPI_SVDD_SYNC_END);
			gpio_set_value(pn547_dev->ese_pwr_req, 1);
			msleep(10);
		} else {
			pr_info("%s : reset failed \n", __func__);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 3: /*else if (arg == 3) */
		pr_info("%s : Prio Session Start power on ese\n", __func__);
		if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO|P61_STATE_DWNLD))==0){
			p61_update_access_state(pn547_dev, P61_STATE_SPI_PRIO, true);
#ifdef FEATURE_PN80T
			if (current_state & P61_STATE_WIRED)
#endif
			{
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid %ld\n", __func__,
							pn547_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO,
							pn547_dev->nfc_service_pid);
				} else {
					pr_info("%s invalid nfc service pid.. %ld\n",
							__func__, pn547_dev->nfc_service_pid);
				}
			}
			pn547_dev->spi_ven_enabled = true;
			if (pn547_dev->nfc_ven_enabled == false) {
				/* provide power to NFCC if, NFC service not provided */
				gpio_set_value(pn547_dev->ven_gpio, 1);
				msleep(10);
			}
			/* pull the gpio to high once NFCC is power on*/
			gpio_set_value(pn547_dev->ese_pwr_req, 1);
			msleep(10);
		}
		else {
			pr_info("%s: Prio Session Start power on ese failed 0x%x\n",
					__func__, current_state);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 4: /*else if (arg == 4)*/
		if (current_state & P61_STATE_SPI_PRIO) {
			pr_info("%s : Prio Session Ending...\n", __func__);
			p61_update_access_state(pn547_dev, P61_STATE_SPI_PRIO, false);
			/*after SPI prio timeout, the state is changing from SPI prio to SPI */
			p61_update_access_state(pn547_dev, P61_STATE_SPI, true);
#ifdef FEATURE_PN80T
			if (current_state & P61_STATE_WIRED)
#endif
			{
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid  %ld", __func__,
							pn547_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO_END,
							pn547_dev->nfc_service_pid);
				} else
					pr_info("%s invalid nfc service pid.. %ld", __func__,
							pn547_dev->nfc_service_pid);
			}
		}
		else {
			pr_info("%s : Prio Session End failed 0x%x\n", __func__,
					current_state);
			/*p61_access_unlock(pn547_dev); redundant */
			ret = -EBADRQC; /* Device or resource busy */
		}
		break;
	case 5: /*else if(arg == 5)*/
		release_ese_lock(P61_STATE_SPI);
		break;
	default: /*	else*/
		pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
		/*p61_access_unlock(pn547_dev); redundant */
		ret = -EBADRQC; /* Invalid request code */
	}

	return ret;
}

static int pn547_p61_set_wired_access(struct pn547_dev *pdev, unsigned long arg)
{
	p61_access_state_t current_state;
	int ret = 0;

	p61_get_access_state(pn547_dev, &current_state);
	pr_info("%s:  cur=0x%x\n", __func__, current_state);
	switch (arg) {
	case 0: /*else if (arg == 0)*/
		pr_info("%s : disabling \n", __func__);
		if (current_state & P61_STATE_WIRED) {
			p61_update_access_state(pn547_dev, P61_STATE_WIRED, false);
#ifndef FEATURE_PN80T
			if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_DWP_SVDD_SYNC_START);
				gpio_set_value(pn547_dev->ese_pwr_req, 0);
				msleep(60);
				svdd_sync_onoff(pn547_dev->nfc_service_pid,
						P61_STATE_DWP_SVDD_SYNC_END);
			}
#endif
		} else {
			pr_err("%s : failed, current_state = %x \n",
					__func__, pn547_dev->p61_current_state);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EPERM; /* Operation not permitted */
		}
		break;
	case 1: /*	if (arg == 1)*/
		if (current_state) {
			pr_info("%s : enabling\n", __func__);
			p61_update_access_state(pn547_dev, P61_STATE_WIRED, true);
			if (current_state & P61_STATE_SPI_PRIO) {
				if (pn547_dev->nfc_service_pid) {
					pr_info("%s nfc service pid  %ld",
							__func__, pn547_dev->nfc_service_pid);
					signal_handler(P61_STATE_SPI_PRIO,
							pn547_dev->nfc_service_pid);
				} else {
					pr_info("%s invalid nfc service pid.. %ld",
							__func__, pn547_dev->nfc_service_pid);
				}
			}
#ifndef FEATURE_PN80T
			if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
				gpio_set_value(pn547_dev->ese_pwr_req, 1);
				msleep(10);
			}
#endif
		}
		else {
			pr_info("%s : enabling failed\n", __func__);
			/*p61_access_unlock(pn547_dev); redundant*/
			ret = -EBUSY; /* Device or resource busy */
		}
		break;
	case 2: /*	else if(arg == 2)*/
		pr_info("%s : P61 ESE POWER REQ LOW\n", __func__);
#ifndef FEATURE_PN80T
		svdd_sync_onoff(pn547_dev->nfc_service_pid,
				P61_STATE_DWP_SVDD_SYNC_START);
		gpio_set_value(pn547_dev->ese_pwr_req, 0);
		msleep(60);
		svdd_sync_onoff(pn547_dev->nfc_service_pid,
				P61_STATE_DWP_SVDD_SYNC_END);
#endif
		break;
	case 3: /*	else if(arg == 3)*/
		pr_info("%s : P61 ESE POWER REQ HIGH\n", __func__);
#ifndef FEATURE_PN80T
		gpio_set_value(pn547_dev->ese_pwr_req, 1);
		msleep(10);
#endif
		break;
	case 4: /*else if(arg == 4)*/
		release_ese_lock(P61_STATE_WIRED);
		break;
	default: /*else*/

		pr_info("%s : bad arg %lu\n", __func__, arg);
		/*p61_access_unlock(pn547_dev);*/
		ret = -EBADRQC; /* Invalid request code */
	}
	
	return ret;
}

#ifdef FEATURE_PN80T
static int set_jcop_download_state(unsigned long arg)
{
        p61_access_state_t current_state = P61_STATE_INVALID;
        int ret = 0;
        p61_get_access_state(pn547_dev, &current_state);
        pr_info("%s: PN547_SET_DWNLD_STATUS:JCOP Dwnld state arg = %ld\n",
				__func__, arg);
        if (arg == JCP_DWNLD_INIT) {
            if (pn547_dev->nfc_service_pid) {
                pr_info("%s nfc service pid ---- %ld\n",
						__func__, pn547_dev->nfc_service_pid);
                signal_handler(JCP_DWNLD_INIT, pn547_dev->nfc_service_pid);
            }
            else {
                if (current_state & P61_STATE_JCP_DWNLD)
                    ret = -EINVAL;
                else
                    p61_update_access_state(pn547_dev, P61_STATE_JCP_DWNLD, true);
            }
        }
        else if (arg == JCP_DWNLD_START) {
            if (current_state & P61_STATE_JCP_DWNLD)
                ret = -EINVAL;
            else
                p61_update_access_state(pn547_dev, P61_STATE_JCP_DWNLD, true);
        }
        else if (arg == JCP_SPI_DWNLD_COMPLETE) {
            if (pn547_dev->nfc_service_pid) {
                signal_handler(JCP_DWP_DWNLD_COMPLETE, pn547_dev->nfc_service_pid);
            }
            p61_update_access_state(pn547_dev, P61_STATE_JCP_DWNLD, false);
        }
        else if (arg == JCP_DWP_DWNLD_COMPLETE) {
            p61_update_access_state(pn547_dev, P61_STATE_JCP_DWNLD, false);
        }
        else {
            pr_info("%s bad jcop download arg %lu\n", __func__, arg);
            p61_access_unlock(pn547_dev);
            return -EBADRQC; /* Invalid request code */
        }
        pr_info("%s: PN547_SET_DWNLD_STATUS = %x", __func__, current_state);

    return ret;
}
#endif
#endif

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
static unsigned char p61_trans_acc_on = 0;
#endif
long pn547_dev_ioctl(struct file *filp,
			   unsigned int cmd, unsigned long arg)
{
	/*struct pn547_dev *pn547_dev = filp->private_data;*/
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	p61_access_state_t current_state;
	int ret = 0;

	if (cmd == P547_GET_ESE_ACCESS)
	{
		return get_ese_lock(P61_STATE_WIRED, arg);
	}
	else if(cmd == P547_REL_SVDD_WAIT)
	{
		return release_svdd_wait();
	}
	p61_access_lock(pn547_dev);
	switch (cmd) {
	case PN547_SET_PWR:
		ret = pn547_set_pwr(pn547_dev, arg);
		break;

	case P61_SET_SPI_PWR:
		ret = pn547_p61_set_spi_pwr(pn547_dev, arg);
		break;

	case P61_GET_PWR_STATUS:
		current_state = P61_STATE_INVALID;
		p61_get_access_state(pn547_dev, &current_state);
		pr_info("%s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
		break;

#ifdef FEATURE_PN80T
    case PN547_SET_DWNLD_STATUS:
        ret = set_jcop_download_state(arg);
        if (ret < 0) {
            p61_access_unlock(pn547_dev);
        }
        break;
#endif

	case P61_SET_WIRED_ACCESS:
		ret = pn547_p61_set_wired_access(pn547_dev, arg);
		break;

	case P547_SET_NFC_SERVICE_PID:
		current_state = P61_STATE_INVALID;
		p61_get_access_state(pn547_dev, &current_state);
        if((p61_trans_acc_on ==  1) &&
			((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0))
            release_ese_lock(P61_STATE_WIRED);

        pr_info("%s : The NFC Service PID is %ld\n", __func__, arg);
        pn547_dev->nfc_service_pid = arg;
    break;
	
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		/*p61_access_unlock(pn547_dev);*/
		pr_info("%s :exit cmd = %u, arg = %ld\n", __func__, cmd, arg);
		ret = -EINVAL;
	}
	p61_access_unlock(pn547_dev);
	return (long)ret;
#else
	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			usleep_range(4900, 5000);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
			usleep_range(4900, 5000);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			usleep_range(4900, 5000);
			if (atomic_read(&pn547_dev->irq_enabled) == 0) {
				atomic_set(&pn547_dev->irq_enabled, 1);
				enable_irq(pn547_dev->client->irq);
				enable_irq_wake(pn547_dev->client->irq);
			}
			pr_info("%s power on with firmware, irq=%d\n", __func__,
				atomic_read(&pn547_dev->irq_enabled));
		} else if (arg == 1) {
			/* power on */
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			usleep_range(4900, 5000);
			if (atomic_read(&pn547_dev->irq_enabled) == 0) {
				atomic_set(&pn547_dev->irq_enabled, 1);
				enable_irq(pn547_dev->client->irq);
				enable_irq_wake(pn547_dev->client->irq);
			}
			pr_info("%s power on, irq=%d\n", __func__,
				atomic_read(&pn547_dev->irq_enabled));
		} else if (arg == 0) {
			/* power off */
			if (atomic_read(&pn547_dev->irq_enabled) == 1) {
				atomic_set(&pn547_dev->irq_enabled, 0);
				disable_irq_wake(pn547_dev->client->irq);
				disable_irq_nosync(pn547_dev->client->irq);
			}
			pr_info("%s power off, irq=%d\n", __func__,
				atomic_read(&pn547_dev->irq_enabled));
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
			usleep_range(4900, 5000);
		} else if (arg == 3) {
			pr_info("%s Read Cancel\n", __func__);
			pn547_dev->cancel_read = true;
			atomic_set(&pn547_dev->read_flag, 1);
			wake_up(&pn547_dev->read_wq);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
#endif
}

EXPORT_SYMBOL(pn547_dev_ioctl);

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout)
{
	unsigned long tempJ = msecs_to_jiffies(timeout);
	pr_info("get_ese_lock: enter p61_current_state =(0x%x), timeout = %d, jiffies = %lu\n"
			, p61_current_state, timeout, tempJ);
	if(down_timeout(&ese_access_sema, tempJ) != 0)
	{
		pr_err("get_ese_lock: timeout p61_current_state = %d\n", p61_current_state);
		return -EBUSY;
	}
    p61_trans_acc_on = 1;
	pr_info("get_ese_lock: exit p61_trans_acc_on =%d, timeout = %d\n"
			, p61_trans_acc_on, timeout);
	return 0;
}
EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(p61_access_state_t  p61_current_state)
{
	pr_info("%s: enter p61_current_state = (0x%x)\n", __func__, p61_current_state);
	up(&ese_access_sema);
    p61_trans_acc_on = 0;
	pr_info("%s: p61_trans_acc_on =%d exit\n", __func__, p61_trans_acc_on);
}
#endif

static const struct file_operations pn547_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = pn547_dev_read,
	.write = pn547_dev_write,
	.open = pn547_dev_open,
	.release = pn547_dev_release,
	.unlocked_ioctl = pn547_dev_ioctl,
};

static int pn547_parse_dt(struct device *dev,
	struct pn547_dev *pdev)
{
	struct device_node *np = dev->of_node;
#ifdef CONFIG_NFC_PM_BBCLK2
	u32 gpio_flags;
#endif

	pdev->irq_gpio = of_get_named_gpio(np, "pn547,irq-gpio", 0);

	pdev->ven_gpio = of_get_named_gpio(np, "pn547,ven-gpio", 0);

	pdev->firm_gpio = of_get_named_gpio(np, "pn547,firm-gpio", 0);

#ifdef CONFIG_NFC_PM_BBCLK2
	pdev->clk_req_gpio = of_get_named_gpio(np, "pn547,clk_req-gpio",0);
#endif
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	pdev->ese_pwr_req = of_get_named_gpio(np, "pn547,pwr_req", 0);
#endif

	if (of_get_property(dev->of_node, "pn547,ldo_control", NULL)) {
		if (of_property_read_string(np, "pn547,nfc_pvdd",
					&pdev->nfc_pvdd) < 0) {
			pr_err("%s - get nfc_pvdd error\n", __func__);
			pdev->nfc_pvdd = NULL;
		} else
			pr_info("%s LDO nfc_pvdd :%s\n", __func__, pdev->nfc_pvdd);
	}
	else {
		pdev->pvdd = of_get_named_gpio(np, "pn547,pvdd-gpio", 0);
		if (pdev->pvdd < 0) {
			pr_info("%s : pvdd-gpio is not set.", __func__);
			pdev->pvdd = 0;
		}
	}
	pr_info("%s: irq : %d, ven : %d, firm : %d\n",
			__func__, pdev->irq_gpio, pdev->ven_gpio,
			pdev->firm_gpio);

	return 0;
}

static int pn547_regulator_onoff(struct device *dev,
		struct	pn547_dev *pdev, int onoff)
{
	int rc = 0;
	struct regulator *regulator_nfc_pvdd;

	pr_err("%s - onoff = %d\n", __func__, onoff);
	regulator_nfc_pvdd = regulator_get(dev, pdev->nfc_pvdd);
	if (IS_ERR(regulator_nfc_pvdd) || regulator_nfc_pvdd == NULL) {
		pr_err("%s - regulator_nfc_pvdd regulator_get fail\n", __func__);
		return -ENODEV;
	}

	pr_info("%s - onoff = %d\n", __func__, onoff);

	if (onoff == NFC_I2C_LDO_ON) {
		rc = regulator_enable(regulator_nfc_pvdd);
		if (rc) {
			pr_err("%s - enable failed, rc=%d\n",
				__func__, rc);
			goto done;
		}
	} else {
		rc = regulator_disable(regulator_nfc_pvdd);
		if (rc) {
			pr_err("%s - disable failed, rc=%d\n",
				__func__, rc);
			goto done;
		}
	}

	done:
	regulator_put(regulator_nfc_pvdd);

	return rc;
}
#ifdef FEATURE_NFC_TEST
static ssize_t pn547_class_show(struct class * class, struct class_attribute *attr, char *buf) {

	int size;

	int ret = 0;
	int count = 4;
	char tmp[128] = {0x20, 0x00, 0x01, 0x00, };
	int retry;
	bool old_ven, old_irq;
	int old_read_value;

	pr_err("%s : start\n", __func__);


	old_ven = pn547_dev->nfc_ven_enabled;
	pr_info("%s : old_ven is %d\n", __func__, old_ven);
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	p61_access_lock(pn547_dev);
#endif
	retry = 20;
	if(!old_ven) {	/* if nfc status is off */
		pn547_dev->nfc_ven_enabled = true;

		if (pn547_dev->spi_ven_enabled == false) {
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
		}
		usleep_range(4900, 5000);
	}
	else {	/* if nfc status is on */
		//wake up device
		gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
		usleep_range(4900, 5000);
		gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
		usleep_range(4900, 5000);
		//intercept i2c_master_recv
		pn547_dev->cancel_read = 1;
		atomic_set(&pn547_dev->read_flag, 1);
	}
	wake_up_all(&pn547_dev->read_wq);
	while(!mutex_trylock(&pn547_dev->read_mutex) && --retry) {
		usleep_range(15, 20);
	}
	if(!retry) {
		pr_err("%s : mutex_trylock failed. check pn547_dev_read()\n", __func__);
		ret = sprintf(buf, "test failed : device in use\n");
		goto fail_lock;
	}

	pr_info("%s : read_mutex locked. retry : %d\n", __func__, retry);

	atomic_set(&pn547_dev->read_flag, 0);
	pn547_dev->cancel_read = 0;
	old_irq = atomic_read(&pn547_dev->irq_enabled);
	pr_info("%s : old_irq is %d\n", __func__, old_irq);
	if(!old_irq) {
		atomic_set(&pn547_dev->irq_enabled, 1);
		enable_irq(pn547_dev->client->irq);
		enable_irq_wake(pn547_dev->client->irq);
		pr_info("%s power on, irq=%d\n", __func__, atomic_read(&pn547_dev->irq_enabled));
	}
	retry = 2;
	do {
		ret = i2c_master_send(pn547_dev->client, tmp, count);
		if(count == ret) {
			break;
		}
		pr_info("%s : i2c_master_send error. ret: %d, retry: %d\n", __func__, ret, retry);
		usleep_range(6000, 10000); /* Retry, chip was in standby */
	} while(retry--);

	if(ret != count) {
		pr_err("%s : failed. count error. send=%d, recv=%d\n", __func__, count, ret);
		ret = 0;
		goto fail;
	}

	pr_err("%s : send success. returned: %d\n", __func__, ret);

	//wait for reply


#if NFC_DEBUG
	pr_info("pn547: wait_event_interruptible : in\n");
#endif
	ret = 0;
	old_read_value = atomic_read(&pn547_dev->read_flag);
	pr_info("%s: read_flag %d, cancel_read %d", __func__, old_read_value, pn547_dev->cancel_read);
	if(!old_read_value) {
		ret = wait_event_interruptible(pn547_dev->read_wq,
				atomic_read(&pn547_dev->read_flag));
	}

#if NFC_DEBUG
	pr_info("pn547 :   h\n");
#endif

	if (pn547_dev->cancel_read) {
		pn547_dev->cancel_read = false;
		ret = 0;
		//todo : old_ven and old_irq rollback needed
		goto fail;
	}

	if (ret) {
		ret = 0;
		goto fail;
	}

/* Read data */
	count = 6;
	ret = i2c_master_recv(pn547_dev->client, tmp, count);

#if NFC_DEBUG
	pr_info("pn547: i2c_master_recv\n");
#endif
	mutex_unlock(&pn547_dev->read_mutex);

	if(!old_ven) {	/* if nfc status is off */
		if (pn547_dev->spi_ven_enabled == false) {
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
		}
		usleep_range(4900, 5000);
		pn547_dev->nfc_ven_enabled = false;
	}
	if(!old_irq) {
		atomic_set(&pn547_dev->irq_enabled, 0);
		disable_irq_wake(pn547_dev->client->irq);
		disable_irq_nosync(pn547_dev->client->irq);
	}
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	p61_access_unlock(pn547_dev);
#endif
	if (ret < 0 || ret > count) {
		pr_err("%s: i2c_master_recv returned %d. count : %d\n", __func__, ret, count);
		return 0;
	}
	size = sprintf(buf, "test completed!! size: %d, data: %X %X %X %X %X %X\n", ret, tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
	pr_err("%s : recv success.\n", __func__);

	msleep(10);

	return size;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
fail_lock:
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	p61_access_unlock(pn547_dev);
#endif
	return ret;
}
static ssize_t pn547_class_store(struct class * class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static CLASS_ATTR(test, 0664, pn547_class_show, pn547_class_store);
#endif

static int pn547_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	int err;
	int addr;
	char tmp[4] = {0x20, 0x00, 0x01, 0x01};
	int addrcnt;
	/*struct pn547_i2c_platform_data *platform_data;*/
	/*struct pn547_dev *pn547_dev;*/
#ifdef FEATURE_NFC_TEST
	struct class * nfc_class;
#endif

	pr_info("%s entered\n", __func__);

	if (client->dev.of_node) {
		pn547_dev = devm_kzalloc(&client->dev,
			sizeof(struct pn547_dev), GFP_KERNEL);
		if (!pn547_dev) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		err = pn547_parse_dt(&client->dev, pn547_dev);
		if (err)
			return err;
	} else {
		pr_err("%s no dts\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	ret = gpio_request(pn547_dev->irq_gpio, "nfc_irq");
	if (ret)
		return -ENODEV;
	ret = gpio_request(pn547_dev->ven_gpio, "nfc_ven");
	if (ret)
		goto err_ven;
	ret = gpio_request(pn547_dev->firm_gpio, "nfc_firm");
	if (ret)
		goto err_firm;
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	ret = gpio_request(pn547_dev->ese_pwr_req, "ese_pwr");
	if (ret)
		goto err_ese;
#endif
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	ret = gpio_request(pn547_dev->clk_req_gpio, "nfc_clk_req");
	if (ret)
		goto err_clk_req;
#endif
#ifdef CONFIG_NFC_PM_BBCLK2
	ret = gpio_request(pn547_dev->clk_req_gpio, "nfc_clk_req");
	if (ret)
		goto err_clk_req;
#endif
	ret = pn547_regulator_onoff(&client->dev, pn547_dev, NFC_I2C_LDO_ON);
	if (ret < 0)
		pr_err("%s pn547 regulator_on fail err = %d\n",
				__func__, ret);
	usleep_range(1000, 1100);

#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	pn547_dev->nfc_clock = msm_xo_get(MSM_XO_TCXO_A1, "nfc");
	if (IS_ERR(pn547_dev->nfc_clock)) {
		ret = PTR_ERR(pn547_dev->nfc_clock);
		printk(KERN_ERR "%s: Couldn't get TCXO_A1 vote for NFC (%d)\n",
					__func__, ret);
		ret = -ENODEV;
		goto err_get_clock;
	}
	pn547_dev->clock_state = false;
#endif
#ifdef CONFIG_NFC_PM_BBCLK2
	pn547_dev->nfc_clock = clk_get(&client->dev, "nfc_clock");
	if (IS_ERR(pn547_dev->nfc_clock)) {
		ret = PTR_ERR(pn547_dev->nfc_clock);
		printk(KERN_ERR "%s: Couldn't get D1 (%d)\n",
					__func__, ret);
	} else {
		if (clk_prepare_enable(pn547_dev->nfc_clock))
			printk(KERN_ERR "%s: Couldn't prepare D1\n",
					__func__);
	}
#endif
#if defined(CONFIG_NFC_PN547_CLK_BBCLK2)
	pn547_dev->nfc_clk = clk_get(&client->dev, "nfc_clk");
	if (IS_ERR(pn547_dev->nfc_clk)) {
		ret = PTR_ERR(pn547_dev->nfc_clk);
		printk(KERN_ERR "%s: Couldn't get D1 (%d)\n",
					__func__, ret);
	} else {
		if (clk_prepare_enable(pn547_dev->nfc_clk))
			printk(KERN_ERR "%s: Couldn't prepare D1\n",
					__func__);
	}
#endif

	client->irq = gpio_to_irq(pn547_dev->irq_gpio);
	pr_info("%s : IRQ num %d\n", __func__, client->irq);

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	pn547_dev->p61_current_state = P61_STATE_IDLE;
	pn547_dev->nfc_ven_enabled = false;
	pn547_dev->spi_ven_enabled = false;
#endif
	pn547_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);

#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	sema_init(&ese_access_sema, 1);
	mutex_init(&pn547_dev->p61_state_mutex);
#endif

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
#ifdef CONFIG_NFC_PN547
	pn547_dev->pn547_device.name = "pn547";
#else
	pn547_dev->pn547_device.name = "pn544";
#endif
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	gpio_direction_input(pn547_dev->irq_gpio);
	gpio_direction_output(pn547_dev->ven_gpio, 0);
	gpio_direction_output(pn547_dev->firm_gpio, 0);
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	gpio_direction_output(pn547_dev->ese_pwr_req, 0);
#endif
#if defined(CONFIG_NFC_PN547_CLOCK_REQUEST) || defined(CONFIG_NFC_PM_BBCLK2)
	gpio_direction_input(pn547_dev->clk_req_gpio);
#endif

	i2c_set_clientdata(client, pn547_dev);
	wake_lock_init(&pn547_dev->nfc_wake_lock,
			WAKE_LOCK_SUSPEND, "nfc_wake_lock");
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	pn547_dev->wq_clock = create_singlethread_workqueue("nfc_wq");
	if (!pn547_dev->wq_clock) {
		ret = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	INIT_WORK(&pn547_dev->work_nfc_clock, nfc_work_func_clock);
#endif
	ret = request_irq(client->irq, pn547_dev_irq_handler,
			  IRQF_TRIGGER_RISING, "pn547", pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	disable_irq_nosync(pn547_dev->client->irq);
	atomic_set(&pn547_dev->irq_enabled, 0);

#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	ret = request_irq(pn547_dev->clk_req_irq, pn547_dev_clk_req_irq_handler,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			, "pn547_clk_req", pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq(clk_req) failed\n");
		goto err_request_irq_failed;
	}

	enable_irq_wake(pn547_dev->clk_req_irq);
#endif

	gpio_set_value(pn547_dev->ven_gpio, 1);
	gpio_set_value(pn547_dev->firm_gpio, 1); /* add firmware pin */
	usleep_range(4900, 5000);
	gpio_set_value(pn547_dev->ven_gpio, 0);
	usleep_range(4900, 5000);
	gpio_set_value(pn547_dev->ven_gpio, 1);
	usleep_range(4900, 5000);

	for (addr = 0x2B; addr > 0x27; addr--) {
		client->addr = addr;
		addrcnt = 2;
		do {
			ret = i2c_master_send(client, tmp, 4);
			if (ret > 0) {
				pr_info("%s : i2c addr(0x%X), ret(%d)\n",
					__func__, client->addr, ret);
				pn547_dev->i2c_probe = ret;
				break;
			}
		} while (addrcnt--);
		if (ret > 0)
			break;
	}

	if(ret <= 0) {
		pr_info("%s : ret(%d), i2c_probe(%d)\n",
			__func__, ret, pn547_dev->i2c_probe);
		client->addr = 0x2B;
	}
	gpio_set_value(pn547_dev->ven_gpio, 0);
	gpio_set_value(pn547_dev->firm_gpio, 0); /* add */

	if (ret < 0)
		pr_err("%s : fail to get i2c addr\n", __func__);
	else
		pr_info("%s : success, i2c_probe(%d)\n", __func__,
				pn547_dev->i2c_probe);

#ifdef FEATURE_NFC_TEST
	nfc_class = class_create(THIS_MODULE, "nfc_test");
	if (IS_ERR(&nfc_class)) {
		pr_err("%s: failed to create nfc class\n", __func__);
	}
	else {
		ret = class_create_file(nfc_class, &class_attr_test);
		if (ret)
			pr_err("%s: failed to create file\n", __func__);
	}
#endif

	return 0;

err_request_irq_failed:
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
err_create_workqueue:
#endif
	misc_deregister(&pn547_dev->pn547_device);
	wake_lock_destroy(&pn547_dev->nfc_wake_lock);
err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	mutex_destroy(&pn547_dev->p61_state_mutex);
#endif
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	msm_xo_put(pn547_dev->nfc_clock);
err_get_clock:
#endif
#if defined(CONFIG_NFC_PN547_CLOCK_REQUEST) || defined(CONFIG_NFC_PM_BBCLK2)
	gpio_free(pn547_dev->clk_req_gpio);
err_clk_req:
#endif
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	gpio_free(pn547_dev->ese_pwr_req);
err_ese:
#endif
	gpio_free(pn547_dev->firm_gpio);
err_firm:
	gpio_free(pn547_dev->ven_gpio);
err_ven:
	gpio_free(pn547_dev->irq_gpio);
	kfree(pn547_dev);
	pr_err("%s faild!\n", __func__);
	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(client);
#ifdef CONFIG_NFC_PM_BBCLK2
	if(pn547_dev->nfc_clock)
		clk_unprepare(pn547_dev->nfc_clock);
#endif
#if defined(CONFIG_NFC_PN547_CLK_BBCLK2)
	if (pn547_dev->nfc_clk)
		clk_unprepare(pn547_dev->nfc_clk);
#endif
	wake_lock_destroy(&pn547_dev->nfc_wake_lock);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	mutex_destroy(&pn547_dev->p61_state_mutex);
#endif
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
#ifdef CONFIG_NFC_PM_BBCLK2
	gpio_free(pn547_dev->clk_req_gpio);
#endif
#ifdef CONFIG_NFC_PN547_CLOCK_REQUEST
	gpio_free(pn547_dev->clk_req_gpio);
	msm_xo_put(pn547_dev->nfc_clock);
#endif
#ifdef CONFIG_NFC_PN547_ESE_SUPPORT
	pn547_dev->p61_current_state = P61_STATE_INVALID;
	pn547_dev->nfc_ven_enabled = false;
	pn547_dev->spi_ven_enabled = false;
#endif
	kfree(pn547_dev);

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{"pn547", 0},
	{}
};

static struct of_device_id nfc_match_table[] = {
	{ .compatible = "pn547",},
	{},
};

static struct i2c_driver pn547_driver = {
	.id_table = pn547_id,
	.probe = pn547_probe,
	.remove = pn547_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "pn547",
		.of_match_table = nfc_match_table,
	},
};

/*
 * module load/unload record keeping
 */
extern unsigned int lpcharge;
static int __init pn547_dev_init(void)
{
	pr_info("Loading pn547 driver\n");
	if (lpcharge) {		
		pr_info("%s LPM, Do not load nfc driver\n", __func__);
		return 0;
	}
	else
		return i2c_add_driver(&pn547_driver);
}

module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	pr_info("Unloading pn547 driver\n");
	i2c_del_driver(&pn547_driver);
}

module_exit(pn547_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
