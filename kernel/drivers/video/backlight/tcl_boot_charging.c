#include <linux/linux_logo.h>
#include <linux/reboot.h>
#include <linux/adc.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/iomux.h>

#define PWM_MUX_NAME       GPIO0D6_PWM2_NAME 
#define PWM_MUX_MODE     GPIO0D_PWM2      
#define PWM_MUX_MODE_GPIO  GPIO0D_GPIO0D6
#define PWM_GPIO 	   RK30_PIN0_PD6

extern const struct linux_logo high_temp_clut224;
extern const struct linux_logo low_temp_clut224;
extern void logo_display(const struct linux_logo *logo);

static unsigned char backlight_driver_registered;
static int boot_charge_ok;

extern struct tcl_common_data *tcl_common_data;

void turn_on_backlight(void)
{
	if (tcl_common_data->bat_low)
		return;
	tcl_common_data->bat_low = 0;

	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
	
	rk29_bl->props.brightness = 180;
	rk29_bl_update_status_zero(rk29_bl);
	
	gpio_set_value(LCD_EN_PIN,GPIO_HIGH);	//turn off LCD
}

void turn_off_backlight(void)
{
/*
	rk29_bl->props.brightness = 0;
	rk29_bl_update_status_zero(rk29_bl);

	gpio_set_value(LCD_EN_PIN,GPIO_LOW);	//turn off LCD
*/
	rk29_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (gpio_request(PWM_GPIO, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(PWM_GPIO, GPIO_LOW);
}

void early_kernel_power_off(void)
{
	kernel_power_off();
}
/*
static int battery_present(void)
{
	return 1;
}
*/
int in_charging(void)
{
	return !gpio_get_value(DC_DET_PIN);
}

extern int charge_full(void);

static int get_battery_voltage(struct adc_client *client)
{

//#if 0
//	return (adc_sync_read(client) * 2500 * 2) / 1024;
//#else
//	return 3700;
//#endif
	
	int voltage = (adc_sync_read(client) * 2500 * 2) / 1024;
	return voltage;
}

static int power_key_pressed(void)
{
	return !gpio_get_value(PLAY_ON_PIN);
}

int boot_charge_full(void)
{
	return boot_charge_ok;
}

static const struct linux_logo *image_tbl[] = 
{
	&bat_charge1_clut224, 
	&bat_charge2_clut224,
	&bat_charge3_clut224,
	&bat_charge4_clut224,
	&bat_charge5_clut224,
	&bat_charge6_clut224,
	&bat_charge7_clut224,
	&bat_charge8_clut224,
	&bat_charge9_clut224,
};
static int image_num = sizeof(image_tbl) / sizeof(image_tbl[0]);

static const struct linux_logo *temp_image_tbl[] =
{
	&high_temp_clut224,
	&low_temp_clut224,
};


#define MSLEEP_TIME 500
#define BL_AUTO_OFF_CNT (10000 / MSLEEP_TIME)
#define LONG_PRESS_CNT (1000 / MSLEEP_TIME)
#define FULL_POWER_CNT (20000 / MSLEEP_TIME)
struct boot_charging 
{
	struct delayed_work play_on_work;
	int bl_off;
	int bl_auto_off_cnt;
	int long_press_cnt;
	int full_power_cnt;
	unsigned char low_power;
	unsigned char full_power;
};

static void __play_on_work(struct work_struct *work)
{
	struct delayed_work *d_w = container_of(work, struct delayed_work, work);
	struct boot_charging *b_c = container_of(d_w, struct boot_charging, play_on_work);

	if (b_c->bl_off) 
	{
		b_c->bl_auto_off_cnt = 0;
		b_c->bl_off = 0;
		turn_on_backlight();
	} 
	else 
	{
		b_c->bl_off = 1;
		turn_off_backlight();
	}

	enable_irq(gpio_to_irq(PLAY_ON_PIN));
}

extern int get_temp_value();
static void stop(struct boot_charging *b_c, struct adc_client *client)
{
	int i;
	struct linux_logo tmp;

	tmp.type = LINUX_LOGO_CLUT224;

	pr_info("image_num=%d!!!!!!!\n", image_num);

	pr_info("temp_value = %d\n",get_temp_value());
	
	if (get_battery_voltage(client) < CONTINUE_VOLTAGE_IN_CHARGING)
		b_c->low_power = 1;

	if (gpio_get_value(PLAY_ON_PIN) == GPIO_LOW)
	{
		if (!b_c->low_power)
			return;
	}

	if (!in_charging() && !power_key_pressed())
		early_kernel_power_off();

	/* set_ts_function(1); */

	i = 0;
	tmp.clut = image_tbl[i]->clut;
	tmp.data = image_tbl[i]->data;
	logo_display(&tmp);
	msleep(100);
	platform_driver_register(&rk29_backlight_driver);
	backlight_driver_registered = 1;

	while (1) 
	{		
		if (b_c->long_press_cnt == LONG_PRESS_CNT) 
		{
			if (!b_c->low_power)
				return;
			else
				b_c->long_press_cnt = 0;
		}
		

		if (b_c->bl_auto_off_cnt == BL_AUTO_OFF_CNT && !b_c->bl_off) 
		{
			b_c->bl_off = 1;
			turn_off_backlight();
		}		
	
		if (gpio_get_value(PLAY_ON_PIN) == GPIO_LOW)
			b_c->long_press_cnt++;
		else
			b_c->long_press_cnt = 0;


		if (!b_c->bl_off)
			b_c->bl_auto_off_cnt++;
		
		if (!in_charging())
			early_kernel_power_off();


	if((3 == get_temp_value()) || (4 == get_temp_value()))	//high_temp
	{
		tmp.clut = temp_image_tbl[0]->clut;
		tmp.data = temp_image_tbl[0]->data;
		logo_display(&tmp);
		
		msleep(MSLEEP_TIME);
//		turn_off_backlight();
//		msleep(MSLEEP_TIME);
		turn_on_backlight();

//		pr_info("high temp\n");		
		continue;
	}
	
	if((1 == get_temp_value()) || (2 == get_temp_value()))
	{
		tmp.clut = temp_image_tbl[1]->clut;
                tmp.data = temp_image_tbl[1]->data;
                logo_display(&tmp);		
		
		msleep(MSLEEP_TIME);
//		turn_off_backlight();
//		msleep(MSLEEP_TIME);
		turn_on_backlight();
		
//		pr_info("low temp\n");
		continue;		
	}



		if (b_c->low_power)
			i = 0;
		else if (b_c->full_power)
			i = image_num - 1;
		else if (i == image_num)
			i = 0;

		tmp.clut = image_tbl[i]->clut;
		tmp.data = image_tbl[i]->data;

		logo_display(&tmp);

		if (b_c->low_power) 
		{
			if (get_battery_voltage(client) >= CONTINUE_VOLTAGE_IN_CHARGING)
				b_c->low_power = 0;
		}


		if (!b_c->full_power) 
		{
			if (charge_full()) 
			{
				pr_info("battery full!!!!!!!!!!!!!!!!!!!!\n");
				b_c->full_power = 1;
				boot_charge_ok = 1;
			}
		}

		i++;
		
		msleep(MSLEEP_TIME);
	}
}

static irqreturn_t play_on_interrupt(int irq, void *dev_id)
{
	struct boot_charging *b_c = (struct boot_charging *)dev_id;
	
	disable_irq_nosync(irq);
	schedule_delayed_work(&b_c->play_on_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

static void tcl_boot_charging(void)
{
	int ret;
	struct boot_charging *b_c;
	struct adc_client *client;

	b_c = kzalloc(sizeof(struct boot_charging), GFP_KERNEL);
	if (!b_c)
		goto EXIT_0;

	#ifdef CONFIG_SKIP_CHARGE_OFF
		goto EXIT_0;
	#endif	

//	if (!battery_present())
//		goto EXIT_0;

	if ((board_boot_mode() == BOOT_MODE_REBOOT) || (board_boot_mode() == BOOT_MODE_RECOVERY))
		goto EXIT_0;
		
	client = adc_register(0, NULL, NULL);
	if (!client)
	{
		printk("adc_register failed!!!!!!!!!!\n");
		goto EXIT_0;
	}

	if (!in_charging()) 
	{
		if (power_key_pressed() || board_boot_mode() != BOOT_MODE_NORMAL) 
		{
			if (get_battery_voltage(client) >= CONTINUE_VOLTAGE_NOT_IN_CHARGING)
				goto EXIT_0;
			else 
			{
				logo_display(image_tbl[0]);
				msleep(100);
				platform_driver_register(&rk29_backlight_driver);

				msleep(10*1000);
				early_kernel_power_off();
			}
		} 
		else 
			early_kernel_power_off();
	}
	

	INIT_DELAYED_WORK(&b_c->play_on_work, __play_on_work);
	
	ret = gpio_request(PLAY_ON_PIN, "play_on");
	if (ret)  
	{
		printk("failed to request PLAY_ON_PIN\n");
		goto EXIT_0;
	}
	ret = gpio_direction_input(PLAY_ON_PIN);
	if (ret) 
	{
		printk("failed to set gpio PLAY_ON_PIN input\n");
		goto EXIT_1;
	}
	ret = request_irq(gpio_to_irq(PLAY_ON_PIN), play_on_interrupt, IRQF_TRIGGER_RISING, "play_on_irq", b_c);
	if (ret) 
	{
		printk("failed to request_irq PLAY_ON_PIN\n");
		goto EXIT_2;
	}
	
	stop(b_c, client);

EXIT_2:
	free_irq(gpio_to_irq(PLAY_ON_PIN), b_c);
EXIT_1:
	gpio_free(PLAY_ON_PIN);

EXIT_0:	
	pr_info("enter EXIT_0\n");
	logo_display(&logo_linux_clut224);
	if (!backlight_driver_registered) 
	{
		msleep(100);
		platform_driver_register(&rk29_backlight_driver);
	}

	turn_on_backlight();

	if (b_c)
		kfree(b_c);
}
