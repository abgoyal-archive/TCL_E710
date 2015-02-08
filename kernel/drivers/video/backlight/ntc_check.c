#include <linux/linux_logo.h>
#include <linux/reboot.h>

#include <linux/adc.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/iomux.h>
#include <mach/tcl_common.h>

#define DISABLE 0
#define ENABLE 1
/*
struct ntc_data {
	struct workqueue_struct     *wq;
	struct delayed_work         delay_work;
	struct adc_client       *client;
};
*/
struct ntc_data *gNtcData;

enum temp_type {
	NORMAL,
	LOWER_THAN_MINUS_20,
	LOWER_THAN_0,
	HIGHER_THAN_45,
	HIGHER_THAN_60
};

//int ADC_for_MIC_DET = 0;
int get_ntc_value(struct adc_client *client)
{
	int value = adc_sync_read(client);

	if(value < 0)
        {
                printk("%s:get adc level err!\n",__FUNCTION__);
                return 500;	//read value when adc is suspended, return a normal value
        }
//	if (ADC_for_MIC_DET == 0)	//MIC检测值
	
	return value;
//	else 
//		return 500;		//默认normal状态
}

int	high_temp;
int 	low_temp;
int	temp_value;

int 	get_temp_value()
{
#if 0
	int temp_value;
	
	if (high_temp == 1)
		temp_value = 1;
	else if(low_temp == 1)
		temp_value = 2;
#endif
	return temp_value;
}

int NTC_power_off = 0;

int check_ntc_power_off()
{
	return NTC_power_off;
}
//int skip_ntc_value = 0;
extern int bqSetCHGCONFIG();
void ntc_adc_to_temp()
{
	int ntc_value;
	ntc_value = get_ntc_value(gNtcData->client);
	pr_info(">>>>>>>>>>>>>>>ntc_value = %d\n",ntc_value);

	if(ntc_value >= 303 && ntc_value <= 790)
	{
		temp_value = NORMAL;
		bqSetCHGCONFIG(ENABLE);
	
	}
	else if(ntc_value >= 205 && ntc_value < 303)	//45-60,show high temp logo
	{
		temp_value = HIGHER_THAN_45;
		bqSetCHGCONFIG(DISABLE);
	}
	else if(ntc_value >= 790 && ntc_value <= 1006)	//-20~0,show low temp logo
	{
		temp_value = LOWER_THAN_0;
		bqSetCHGCONFIG(DISABLE);
	}
	else if(ntc_value >1006)			//<-20,turn off
	{		
		pr_info("lower than -20\n");
		temp_value = LOWER_THAN_MINUS_20;
		bqSetCHGCONFIG(DISABLE);

		if (android_booted())
		{
			if (gpio_get_value(DC_DET_PIN) == 1)	//not in charge,turn off
			//	kernel_power_off();
				NTC_power_off = 1;
		}
		else
			pr_info(">>>>>>>>>not booted yet\n");
	}
	else if(ntc_value < 205)			//>60,turn off
	{
		if (ntc_value >= 0 && ntc_value <= 100)
		{
 			cancel_delayed_work(&gNtcData->delay_work);
			bqSetCHGCONFIG(ENABLE);
//			skip_ntc_value = 1;	//for PAD designed before PIO3
		}
		else
		{	
			pr_info("higher than 60\n");
			temp_value = HIGHER_THAN_60;
			bqSetCHGCONFIG(DISABLE);
		
			if (android_booted())
			{
				if (gpio_get_value(DC_DET_PIN) == 1)	//not in charge,turn off
				//	kernel_power_off();
					NTC_power_off = 1;
			}
			else
				pr_info(">>>>>>>>>not booted yet\n");
			}
	}
	pr_info("temp_value = %d\n",temp_value);

}

void ntc_adc_timer_work(struct work_struct *work)
{
	queue_delayed_work(gNtcData->wq, &gNtcData->delay_work, msecs_to_jiffies(1000*20));
	ntc_adc_to_temp();
}

void ntc_check()
{
	int ret = 0;
	int ntc_value;
	struct adc_client *client;
	struct ntc_data	    *data;

        data = kzalloc(sizeof(*data), GFP_KERNEL);
        if (data == NULL) {
                ret = -ENOMEM;
                goto err_data_alloc_failed;
        }

#ifdef CONFIG_SKIP_NTC
	goto EXIT_0;
#endif
	
	gNtcData = data;
	client = adc_register(2, NULL, NULL);
        if (!client)
        {
                printk("adc_register failed!!!!!!!!!!\n");
                goto EXIT_0;
        }
	data->client = client;	
	ntc_value = get_ntc_value(client);

	data->wq = create_singlethread_workqueue("ntc_check");
	INIT_DELAYED_WORK(&data->delay_work, ntc_adc_timer_work);
        queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(1));

	pr_info("ntc_value = %d\n",ntc_value); 


	return 0;

EXIT_0:
        pr_info("enter EXIT_0\n");
err_data_alloc_failed:
        kfree(data);

        printk("ntc_check: error!\n");

        return ret;
}
