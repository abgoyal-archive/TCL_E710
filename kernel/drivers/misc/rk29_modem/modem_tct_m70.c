#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/stat.h>	 /* permission constants */
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <mach/board.h>

#include <linux/platform_device.h>

#include "rk29_modem.h"


#undef GPIO_HIGH
#undef GPIO_LOW

#define GPIO_HIGH 0
#define GPIO_LOW  1

// 确保不出现重复处理wakeup
static int do_wakeup_handle = 0;
static irqreturn_t m70_irq_handler(int irq, void *dev_id);
static int __devinit m70_resume(struct platform_device *pdev);

static struct rk29_io_t m70_io_ap_ready = {     //AP WAKEUP BB //3G_WAKE_IN
    .io_addr    = RK30_PIN2_PB7,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
};

static struct rk29_io_t m70_io_power = {      //AP POWER //3G_PWR
    .io_addr    = RK30_PIN2_PB6,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
};

static struct rk29_irq_t m70_irq_bp_wakeup_ap= {  //BB WAKEUP AP  //3G_WAKE_OUT
    .irq_addr   = RK30_PIN2_PB5,
    .irq_trigger = IRQF_TRIGGER_FALLING, // 下降沿触发
};


static struct rk29_io_t m70_io_w_disable = {      //AP WLAN_DISABLE //3G_W_DISABLE
    .io_addr    = RK30_PIN2_PC0,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
};

static struct rk29_io_t m70_io_onoff = {      //AP ON/OFF //3G_CONTROL
    .io_addr    = RK30_PIN2_PA1,
    .enable     = GPIO_LOW,
    .disable    = GPIO_HIGH,
};

static struct rk29_io_t m70_io_rst = {      //AP ON/OFF //3G_RST
    .io_addr    = RK30_PIN2_PA2,
    .enable     = GPIO_HIGH,
    .disable    = GPIO_LOW,
};

static struct platform_driver m70_platform_driver = {
	.driver		= {
		.name		= "tct_m70",
	},
	.suspend    = rk29_modem_suspend,
	.resume     = rk29_modem_resume,
};

static struct rk29_modem_t m70_driver = {
    .driver         = &m70_platform_driver,
    .modem_power    = &m70_io_power,
    .ap_ready       = &m70_io_ap_ready,
    .bp_wakeup_ap   = &m70_irq_bp_wakeup_ap,
    .modem_rst      = &m70_io_rst,
    .modem_onoff    = &m70_io_onoff,
    .modem_w_disable = &m70_io_w_disable,
//    .status         = MODEM_ENABLE,
    .status         = MODEM_DISABLE,  //MODIFY for wfy ,for input 3G pin namber when reboot
    .dev_init       = NULL,
    .dev_uninit     = NULL,
    .irq_handler    = m70_irq_handler,
    .suspend        = NULL,
    .resume         = NULL,//m70_resume,
    
    .enable         = NULL,
    .disable        = NULL,
    .sleep          = NULL,
    .wakeup         = NULL,
};

static void do_test1(struct work_struct *work)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
    // 标志AP已就绪，BB可以上报数据给AP
    gpio_direction_output(m70_driver.ap_ready->io_addr, m70_driver.ap_ready->enable);
}

static DECLARE_DELAYED_WORK(test1, do_test1);

static int __devinit m70_resume(struct platform_device *pdev)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);

/* cmy: 目前在系统被唤醒后，在这边设置AP_RDY，但由于通信依赖于其它的设备驱动(比如USB或者串口)
        需要延时设置AP_RDY信号
        更好的做法是在它所依赖的设备就绪后(唤醒)，再设置AP_RDY。做法两种:
            1 将设置AP_RDY的函数注入到目标设备的resume函数中
            2 在rk29_modem_resume中，等待目标设备resume之后，再设置AP_RDY
 */
    schedule_delayed_work(&test1, 2*HZ);

    return 0;
}

/*
    m70 模组的 IRQ 处理函数，该函数由rk29_modem中的IRQ处理函数调用
 */
static irqreturn_t m70_irq_handler(int irq, void *dev_id)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);

    if( irq == gpio_to_irq(m70_driver.bp_wakeup_ap->irq_addr) )
    {
        if( !do_wakeup_handle )
        {
            do_wakeup_handle = 1;
            // 当接收到 bb wakeup ap 的IRQ后，申请一个8秒的suspend锁，时间到后自动释放
            // 释放时如果没有其它的锁，就将再次挂起.
            wake_lock_timeout(&m70_driver.wakelock_bbwakeupap, 8 * HZ);
        } else
            printk("%s: already wakeup\n", __FUNCTION__);

        return IRQ_HANDLED;
    }
    
    return IRQ_NONE;
}


#define M800_DETECT_GPIO RK30_PIN1_PA4
static irqreturn_t m800_detect_handler(int irq, void *dev_id)
{
	int val = gpio_get_value(M800_DETECT_GPIO);

	if (val == GPIO_HIGH)
		pr_info("m800 out!!!!!!!!!!!!!!!\n");
	else if (val == GPIO_LOW)
		pr_info("m800 in!!!!!!!!!!!!!!!!\n");

	irq_set_irq_type(irq, 
			val ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);

    return IRQ_HANDLED;
}

int install_m800_detect_irq(void)
{
	int ret;
    int irq;
    

	irq = gpio_to_irq(M800_DETECT_GPIO);

	ret = gpio_request(RK30_PIN1_PA4, "m800_detect_gpio");
	if (ret < 0) {
		pr_err("%s: gpio_request(%ld) failed\n", __func__, M800_DETECT_GPIO);
		return ret;
	}

	gpio_direction_input(M800_DETECT_GPIO);
//    gpio_pull_updown(M800_DETECT_GPIO, 0);

	ret = request_irq(irq, m800_detect_handler, 
			gpio_get_value(M800_DETECT_GPIO) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING, 
			"m800_detect_irq", NULL);
	if (ret < 0) {
		pr_err("%s: request_irq(%d) failed\n", __func__, irq);
		gpio_free(M800_DETECT_GPIO);
		return ret;
	}

	enable_irq_wake(irq);

	return 0;
}

static int __init m70_init(void)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);

	install_m800_detect_irq();

//   rk29_mux_api_set(GPIO6C76_CPUTRACEDATA76_NAME,GPIO4H_GPIO6C76); //RST
    return rk29_modem_init(&m70_driver);
}

static void __exit m70_exit(void)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
    rk29_modem_exit();
}

module_init(m70_init);
module_exit(m70_exit);

MODULE_AUTHOR("huanghuabin");
MODULE_DESCRIPTION("ROCKCHIP modem driver");
MODULE_LICENSE("GPL");

#if 0
int test(void)
{
    printk(">>>>>> test \n ");
    int ret = gpio_request(IRQ_BB_WAKEUP_AP, NULL);
    if(ret != 0)
    {
        printk(">>>>>> gpio_request failed! \n ");
        gpio_free(IRQ_BB_WAKEUP_AP);
        return ret;
    }

//    printk(">>>>>> set GPIOPullUp \n ");
//    gpio_pull_updown(IRQ_BB_WAKEUP_AP, GPIOPullUp);
//    printk(">>>>>> set GPIO_HIGH \n ");
//    gpio_direction_output(IRQ_BB_WAKEUP_AP, GPIO_HIGH);

//    printk(">>>>>> set GPIO_LOW \n ");
//    gpio_direction_output(IRQ_BB_WAKEUP_AP, GPIO_LOW);
//    msleep(1000);
    
    gpio_free(IRQ_BB_WAKEUP_AP);

    printk(">>>>>> END \n ");
}
#endif

