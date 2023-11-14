/**
 * SPDX-License-Identifier: GPL-2.0+
 *
 * @file      pwm-rockchip-capture.c
 * @brief     Rockchip RK3568 PWM Capture Mode Standard Usage Flow Driver.
 * @author    tzuhao.hu <tzuhao.hu@foxmail.com>
 * @date      2023/11/14
 * 
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

/* PWM0 registers  */
#define PWM_REG_CNTR			0x00  /* Counter Register */
#define PWM_REG_HPR			0x04  /* Period Register */
#define PWM_REG_LPR			0x08  /* Duty Cycle Register */
#define PWM_REG_CTRL			0x0c  /* Control Register */
#define PWM3_REG_INTSTS			0x10  /* Interrupt Status Refister For Pwm3*/
#define PWM2_REG_INTSTS			0x20  /* Interrupt Status Refister For Pwm2*/
#define PWM1_REG_INTSTS			0x30  /* Interrupt Status Refister For Pwm1*/
#define PWM0_REG_INTSTS			0x40  /* Interrupt Status Refister For Pwm0*/
#define PWM3_REG_INT_EN			0x14  /* Interrupt Enable Refister For Pwm3*/
#define PWM2_REG_INT_EN			0x24  /* Interrupt Enable Refister For Pwm2*/
#define PWM1_REG_INT_EN			0x34  /* Interrupt Enable Refister For Pwm1*/
#define PWM0_REG_INT_EN			0x44  /* Interrupt Enable Refister For Pwm0*/

/*REG_CTRL bits definitions*/
#define PWM_ENABLE			(1 << 0)
#define PWM_DISABLE			(0 << 0)

/*operation mode*/
#define PWM_MODE_ONESHOT		(0x00 << 1)
#define PWM_MODE_CONTINUMOUS		(0x01 << 1)
#define PWM_MODE_CAPTURE		(0x02 << 1)

/*duty cycle output polarity*/
#define PWM_DUTY_POSTIVE		(0x01 << 3)
#define PWM_DUTY_NEGATIVE		(0x00 << 3)

/*incative state output polarity*/
#define PWM_INACTIVE_POSTIVE		(0x01 << 4)
#define PWM_INACTIVE_NEGATIVE		(0x00 << 4)

/*clock source select*/
#define PWM_CLK_SCALE			(1 << 9)
#define PWM_CLK_NON_SCALE		(0 << 9)

#define PWM_CH0_INT			(1 << 0)
#define PWM_CH1_INT			(1 << 1)
#define PWM_CH2_INT			(1 << 2)
#define PWM_CH3_INT			(1 << 3)
#define PWM_PWR_KEY_INT			(1 << 7)

#define PWM_CH0_POL			(1 << 8)
#define PWM_CH1_POL			(1 << 9)
#define PWM_CH2_POL			(1 << 10)
#define PWM_CH3_POL			(1 << 11)

#define PWM_CH0_INT_ENABLE		(1 << 0)
#define PWM_CH0_INT_DISABLE		(0 << 0)

#define PWM_CH1_INT_ENABLE		(1 << 1)
#define PWM_CH1_INT_DISABLE		(0 << 1)

#define PWM_CH2_INT_ENABLE		(1 << 2)
#define PWM_CH2_INT_DISABLE		(0 << 2)

#define PWM_CH3_INT_ENABLE		(1 << 3)
#define PWM_CH3_INT_DISABLE		(0 << 3)

#define PWM_INT_ENABLE			1
#define PWM_INT_DISABLE			0

/*prescale factor*/
#define PWMCR_MIN_PRESCALE			0x00
#define PWMCR_MAX_PRESCALE			0x07

#define PWMDCR_MIN_DUTY				0x0001
#define PWMDCR_MAX_DUTY				0xFFFF

#define PWMPCR_MIN_PERIOD			0x0001
#define PWMPCR_MAX_PERIOD			0xFFFF

#define PWMPCR_MIN_PERIOD			0x0001
#define PWMPCR_MAX_PERIOD			0xFFFF

#define PWM_REG_INTSTS(n)		((3 - (n)) * 0x10 + 0x10)
#define PWM_REG_INT_EN(n)		((3 - (n)) * 0x10 + 0x14)
#define RK_PWM_VERSION_ID(n)		((3 - (n)) * 0x10 + 0x2c)
#define PWM_REG_PWRMATCH_CTRL(n)	((3 - (n)) * 0x10 + 0x50)
#define PWM_REG_PWRMATCH_LPRE(n)	((3 - (n)) * 0x10 + 0x54)
#define PWM_REG_PWRMATCH_HPRE(n)	((3 - (n)) * 0x10 + 0x58)
#define PWM_REG_PWRMATCH_LD(n)		((3 - (n)) * 0x10 + 0x5C)
#define PWM_REG_PWRMATCH_HD_ZERO(n)	((3 - (n)) * 0x10 + 0x60)
#define PWM_REG_PWRMATCH_HD_ONE(n)	((3 - (n)) * 0x10 + 0x64)
#define PWM_PWRMATCH_VALUE(n)		((3 - (n)) * 0x10 + 0x68)
#define PWM_PWRCAPTURE_VALUE(n)		((3 - (n)) * 0x10 + 0x9c)

#define PWM_CH_INT(n)			BIT(n)
#define PWM_CH_POL(n)			BIT(n+8)

#define PWM_CH_INT_ENABLE(n)		BIT(n)
#define PWM_PWR_INT_ENABLE		BIT(7)
#define CH3_PWRKEY_ENABLE		BIT(3)

struct rk_pwm_capture_data_t {
	u_int64_t period_ns;
	u_int64_t duty_ns;
    u_int8_t capture_ms;
};

enum pwm_div {
	PWM_DIV1	= (0x0 << 12),
	PWM_DIV2	= (0x1 << 12),
	PWM_DIV4	= (0x2 << 12),
	PWM_DIV8	= (0x3 << 12),
	PWM_DIV16	= (0x4 << 12),
	PWM_DIV32	= (0x5 << 12),
	PWM_DIV64	= (0x6 << 12),
	PWM_DIV128	= (0x7 << 12),
};

static DEFINE_MUTEX(device_mutex);

typedef enum _CAP_STATE {
	CAP_IDLE,
	CAP_JUNK_DATA_1, // lpr/hpr junk data
	CAP_JUNK_DATA_2, // hpr/lpr junk data
	CAP_DATA,
	CAP_DONE,
} eCAP_STATE;

struct rk_pwm_capture_sysfs_data {
	dev_t devno;
	char dev_name[32];
	struct cdev _cdev;
	struct class *_class;

	struct rk_pwm_capture_data_t user_data;
};

struct rk_pwm_capture_drvdata {
    void __iomem *base;
    int irq;
	int irq_cpu_id;
	struct device dev;
    int pwm_freq_nstime; // Cycle of capture clock
    int pwm_channel; // RK3568 4-built-in PWM channels
    int hpr; // Number of capture clock cycles at high levels in a PWM cycle to be tested
	int lpr; // Number of capture clock cycles at low levels in a PWM cycle to be tested
	eCAP_STATE state;
	struct clk *clk;
    struct clk *p_clk;
	struct rk_pwm_capture_sysfs_data sdata;
};

static void rk_pwm_capture_hw_disabled(void __iomem *pwm_base, uint pwm_id)
{
	int val;

	val = readl_relaxed(pwm_base + PWM_REG_CTRL);
	val = (val & 0xFFFFFFFE) | PWM_DISABLE;
	writel_relaxed(val, pwm_base + PWM_REG_CTRL);
}

static void rk_pwm_capture_hw_enabled(void __iomem *pwm_base, uint pwm_id)
{
	int val;

	val = readl_relaxed(pwm_base + PWM_REG_CTRL);
	val = (val & 0xFFFFFFFE) | PWM_ENABLE;
	writel_relaxed(val, pwm_base + PWM_REG_CTRL);
}

static void rk_pwm_capture_start(struct rk_pwm_capture_drvdata *ddata)
{
	ddata->hpr = 0;
	ddata->lpr = 0;
	ddata->state = CAP_JUNK_DATA_1;
	
	rk_pwm_capture_hw_enabled(ddata->base, ddata->pwm_channel);
}

static void rk_pwm_capture_stop(struct rk_pwm_capture_drvdata *ddata)
{
	//spin_lock(&ddata->lock);
	ddata->state = CAP_IDLE;
	//spin_unlock(&ddata->lock);

	rk_pwm_capture_hw_disabled(ddata->base, ddata->pwm_channel);
}

static bool rk_pwm_capture_data_available(struct rk_pwm_capture_drvdata *ddata)
{
	return (ddata->state == CAP_DONE && ddata->hpr && ddata->lpr);
}

static int rk_pwm_capture_sysfs_open(struct inode * node, struct file *f)
{
    struct rk_pwm_capture_sysfs_data *sysfs_data;
	struct rk_pwm_capture_drvdata *ddata;

    sysfs_data = container_of(node->i_cdev, struct rk_pwm_capture_sysfs_data, _cdev);
	ddata = container_of(sysfs_data, struct rk_pwm_capture_drvdata, sdata);

	if (!mutex_trylock(&device_mutex)) {
        dev_err(&ddata->dev, "Device already open by another application\n");
        return -EBUSY;
    }
    
    f->private_data = ddata;

    return 0;
}

static int rk_pwm_capture_sysfs_release(struct inode * node, struct file *fp)
{
	struct rk_pwm_capture_drvdata *ddata;

	mutex_unlock(&device_mutex);

	ddata = fp->private_data;
	ddata->sdata.user_data.period_ns = 0;
	ddata->sdata.user_data.duty_ns = 0;
	ddata->state = CAP_IDLE;
	rk_pwm_capture_hw_disabled(ddata->base, ddata->pwm_channel);

	return 0;
}

static ssize_t rk_pwm_capture_sysfs_read(struct file *fp, char __user * buf, size_t size, loff_t *lf)
{
	int ret;
	int timeout = 0;
	struct rk_pwm_capture_drvdata *ddata;

	ddata = fp->private_data;
	if (size <= 0 || size > 128) {
		size = 128;
	}

	rk_pwm_capture_start(ddata);

	do {
		if (timeout >= 50) {
			rk_pwm_capture_stop(ddata);
			ret = -EIO;
			dev_err(&ddata->dev, "Failed to capture data: %d.\n", ret);
			return ret;
		}
		msleep(1);
		timeout ++;
	} while (!rk_pwm_capture_data_available(ddata));

	rk_pwm_capture_stop(ddata);

	ddata->sdata.user_data.period_ns = ddata->pwm_freq_nstime * (ddata->hpr + ddata->lpr);
	ddata->sdata.user_data.duty_ns   = ddata->pwm_freq_nstime * ddata->hpr;
	ddata->sdata.user_data.capture_ms = timeout;

	ret = copy_to_user(buf, &ddata->sdata.user_data, size);
	if (ret) {
		dev_err(&ddata->dev, "failed to copy to user: %d\n", ret);
		return ret;
	}

	return size;
}

struct file_operations rk_pwm_capture_fops = 
{
    .owner = THIS_MODULE,
    .open  = rk_pwm_capture_sysfs_open,
    .release = rk_pwm_capture_sysfs_release,
    .read = rk_pwm_capture_sysfs_read
};

static void rk_pwm_capture_int_ctrl(void __iomem *pwm_base, uint pwm_id, int ctrl)
{
	int val;

	if (pwm_id > 3) {
		return;
	}
	val = readl_relaxed(pwm_base + PWM_REG_INT_EN(pwm_id));
	if (ctrl) {
		val |= PWM_CH_INT_ENABLE(pwm_id);
		//"pwm int enabled, value is 0x%x\n", val);
		writel_relaxed(val, pwm_base + PWM_REG_INT_EN(pwm_id));
	} else {
		val &= ~PWM_CH_INT_ENABLE(pwm_id);
		//"pwm int disabled, value is 0x%x\n", val);
	}
	writel_relaxed(val, pwm_base + PWM_REG_INT_EN(pwm_id));
}


static irqreturn_t rk_pwm_capture_hw_irq(int irq, void *dev_id)
{
	struct rk_pwm_capture_drvdata *ddata = dev_id;
	unsigned int channel = ddata->pwm_channel;
	int val;
	int lpr;
	int hpr;

	val = readl_relaxed(ddata->base + PWM_REG_INTSTS(channel));

	if ((val & PWM_CH_INT(channel)) == 0) {
		return IRQ_HANDLED;
	}
	
	if ((val & PWM_CH_POL(channel)) == 0) {
		lpr = readl_relaxed(ddata->base + PWM_REG_LPR);
		dev_dbg(&ddata->dev, "lpr: %d\n", lpr);
		if (ddata->state != CAP_DONE) {
			ddata->lpr = lpr;
		}
	} else {
		hpr = readl_relaxed(ddata->base + PWM_REG_HPR);
		dev_dbg(&ddata->dev, "hpr: %d\n", hpr);
		if (ddata->state != CAP_DONE) {
			ddata->hpr = hpr;
		}
	}

	//spin_lock(&ddata->lock);
	switch (ddata->state)
	{
	case CAP_JUNK_DATA_1:
		ddata->state = CAP_JUNK_DATA_2;
		break;
	case CAP_JUNK_DATA_2:
		ddata->lpr = ddata->hpr = 0;
		ddata->state = CAP_DATA;
		break;
	case CAP_DATA:
		if (ddata->lpr && ddata->hpr) {
			ddata->state = CAP_DONE;
		}
		break;
	default:
		break;
	}
	//spin_unlock(&ddata->lock);

	writel_relaxed(PWM_CH_INT(channel), ddata->base + PWM_REG_INTSTS(channel));

	return IRQ_HANDLED;
}

static int rk_pwm_capture_hw_init(void __iomem *pwm_base, uint pwm_id)
{
	int val;

	if (pwm_id > 3) {
		return -EINVAL;
	}
	//1. disabled pwm PWM_PWMx_CTRL.pwm_en
	rk_pwm_capture_hw_disabled(pwm_base, pwm_id);
	//2. capture mode PWM_PWMx_CTRL.pwm_mode
	val = readl_relaxed(pwm_base + PWM_REG_CTRL);
	val = (val & 0xFFFFFFF9) | PWM_MODE_CAPTURE;
	writel_relaxed(val, pwm_base + PWM_REG_CTRL);
	//set clk div, clk div to 64 PWM_PWMx_CTRL.scale/*prescale/clk_src_sel/clk_sel
	val = readl_relaxed(pwm_base + PWM_REG_CTRL);
	val = (val & 0xFF0001FF) | PWM_DIV64;
	writel_relaxed(val, pwm_base + PWM_REG_CTRL);
	//4. enabled pwm int PWM_INT_EN.chx_int_en
	rk_pwm_capture_int_ctrl(pwm_base, pwm_id, PWM_INT_ENABLE);
	//5. enabled pwm PWM_PWMx_CTRL.pwm_en
	// rk_pwm_capture_hw_enabled(pwm_base, pwm_id); // Not starting immediately to save resources

	return 0;
}

static int rk_pwm_capture_hw_deinit(void __iomem *pwm_base, uint pwm_id)
{
	int val;

	if (pwm_id > 3) {
		return -EINVAL;
	}

	// diabled pwm PWM_PWMx_CTRL.pwm_en
	val = readl_relaxed(pwm_base + PWM_REG_CTRL);
	val = (val & 0xFFFFFFFE) | PWM_DISABLE;
	writel_relaxed(val, pwm_base + PWM_REG_CTRL);
	
	return 0;
}

static int rk_pwm_capture_irq_init(struct platform_device *pdev)
{	
	struct rk_pwm_capture_drvdata *ddata;
	struct cpumask cpumask;
	int irq;
	int ret;
	int cpu_id;

	ddata = platform_get_drvdata(pdev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
        return -EINVAL;
	}
    ddata->irq = irq;

	cpumask_clear(&cpumask);
	cpumask_set_cpu(cpu_id, &cpumask);
	irq_set_affinity(irq, &cpumask);
    ret = devm_request_irq(&pdev->dev, irq, rk_pwm_capture_hw_irq,
	 		       IRQF_NO_SUSPEND, "rk_pwm_capture_irq", ddata);
    if (ret) {
		dev_err(&pdev->dev, "cannot claim PWR_IRQ: %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static int rk_pwm_capture_clk_init(struct platform_device *pdev)
{
	int ret;
	int pwm_freq;
	struct rk_pwm_capture_drvdata *ddata;

	ddata = platform_get_drvdata(pdev);

	ret = clk_prepare_enable(ddata->clk);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable bus clk: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(ddata->p_clk);
	if (ret) {
		dev_err(&pdev->dev, "Can't enable bus periph clk: %d\n", ret);
		return ret;
	}
	pwm_freq = clk_get_rate(ddata->clk) / 64;
	//dev_info(&pdev->dev,"pwm_freq: %d\n", pwm_freq);
	ddata->pwm_freq_nstime = 1000000000 / pwm_freq;
	//dev_info(&pdev->dev,"pwm_freq_nstime: %d\n", ddata->pwm_freq_nstime);

	return 0;
}

static int rk_pwm_capture_clk_deinit(struct platform_device *pdev)
{
	struct rk_pwm_capture_drvdata *ddata;

	ddata = platform_get_drvdata(pdev);	

	if (ddata->p_clk) {
		clk_unprepare(ddata->p_clk);
	} 
	if (ddata->clk) {
		clk_unprepare(ddata->clk);
	}

	return 0;
}

static int rk_pwm_capture_sysfs_init(struct platform_device *pdev)
{
	int ret;
	struct rk_pwm_capture_drvdata *ddata;
	struct rk_pwm_capture_sysfs_data *sdata;

	ddata = platform_get_drvdata(pdev);
	sdata = &ddata->sdata;

	ret = alloc_chrdev_region(&(sdata->devno), 1, 1, sdata->dev_name);
	if (ret) {
        dev_err(&pdev->dev, "failed to alloc_chrdev_region\n");
		return -EAGAIN;
	}
	sdata->_cdev.owner = THIS_MODULE;
	ret = cdev_add(&(sdata->_cdev), sdata->devno, 1);
	if (ret) {
        dev_err(&pdev->dev, "failed to add cdev\n");
		return -EAGAIN;
	}
	cdev_init(&(sdata->_cdev), &rk_pwm_capture_fops);

	sdata->_class = class_create(THIS_MODULE, sdata->dev_name);
    device_create(sdata->_class, NULL, 
					sdata->devno, NULL, "%s", sdata->dev_name);

	return 0;
}

static int rk_pwm_capture_sysfs_deinit(struct platform_device *pdev)
{
	struct rk_pwm_capture_drvdata *ddata;
	struct rk_pwm_capture_sysfs_data *sdata;

	ddata = platform_get_drvdata(pdev);
	sdata = &(ddata->sdata);

	device_destroy(sdata->_class, sdata->devno);
	cdev_del(&(sdata->_cdev));
	unregister_chrdev_region(sdata->devno, 1);
	class_destroy(sdata->_class);

	return 0;
}

static int rk_pwm_capture_get_dts_property(struct platform_device *pdev)
{
	struct resource *r;
	struct rk_pwm_capture_drvdata *ddata;
	struct device_node *np = pdev->dev.of_node;
	struct clk *clk;
    struct clk *p_clk;
	int pwm_channel;
	int ret;

	ddata = platform_get_drvdata(pdev);

	ret = of_device_is_available(np);
	if (!ret) {
		dev_info(&pdev->dev, "node is not available: %d\n", ret);
		return -ENODEV;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!r) {
		dev_err(&pdev->dev, "no memory resources defined\n");
		return -ENODEV;
	}

	sprintf(ddata->sdata.dev_name, "%s@capture", r->name);
	
	ddata->base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(ddata->base)) {
		return PTR_ERR(ddata->base);
	}

	ddata->dev = pdev->dev;

	ddata->irq_cpu_id = 1;

    clk = devm_clk_get(&pdev->dev, "pwm");
    if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		if (ret != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "Can't get bus clk: %d\n", ret);
		}
		return ret;
	}
	ddata->clk = clk;
	p_clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(p_clk)) {
		ret = PTR_ERR(p_clk);
		if (ret != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "Can't get pclk: %d\n", ret);
		}
		return ret;
	}
	ddata->p_clk = p_clk;

	of_property_read_u32(np, "pwm_channel", &pwm_channel);
	pwm_channel %= 4;
	ddata->pwm_channel = pwm_channel;
	if (pwm_channel > 3) {
		dev_err(&pdev->dev, "pwm id error\n");
		return -EINVAL;
	}

	return 0;
}

static int rk_pwm_capture_probe(struct platform_device *pdev)
{
    struct rk_pwm_capture_drvdata *ddata;
    int ret;

    ddata = devm_kzalloc(&pdev->dev, sizeof(struct rk_pwm_capture_drvdata),
			     GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	ddata->base = ddata->clk = ddata->p_clk = NULL;
	ddata->state = CAP_IDLE;
	platform_set_drvdata(pdev, ddata);

	if (0 != (ret = rk_pwm_capture_get_dts_property(pdev))) {
		return ret;
	}

	if (0 != (rk_pwm_capture_irq_init(pdev))) {
		return ret;
	}

	if (0 != (rk_pwm_capture_clk_init(pdev))) {
		return ret;
	}

    rk_pwm_capture_hw_init(ddata->base, ddata->pwm_channel); 

	if (0 != (rk_pwm_capture_sysfs_init(pdev))) {
		rk_pwm_capture_clk_deinit(pdev);
		return ret;
	}

	dev_info(&pdev->dev, "probe successful!");

    return 0;
}

static int rk_pwm_capture_remove(struct platform_device *pdev)
{
	struct rk_pwm_capture_drvdata *ddata = platform_get_drvdata(pdev);

	rk_pwm_capture_sysfs_deinit(pdev);
	rk_pwm_capture_hw_deinit(ddata->base, ddata->pwm_channel);
	rk_pwm_capture_clk_deinit(pdev);

	dev_info(&pdev->dev, "remove!");

	return 0;
}

static const struct of_device_id rk_pwm_capture_of_match[] = {
	{ .compatible =  "rockchip,pwm-capture"},
	{ }
};

MODULE_DEVICE_TABLE(of, rk_pwm_capture_of_match);

static struct platform_driver rk_pwm_capture_driver = {
	.driver = {
		.name = "pwm-capture",
		.of_match_table = rk_pwm_capture_of_match,
	},
	.remove = rk_pwm_capture_remove,
	.probe  = rk_pwm_capture_probe
};
module_platform_driver(rk_pwm_capture_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("tzuhao.hu");
MODULE_DESCRIPTION("Rockchip RK3568 PWM Capture Mode Standard Usage Flow Driver.");