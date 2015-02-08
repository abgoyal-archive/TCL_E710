/*
 * bq2416x battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>

#include "bq24195_charger.h"

static struct i2c_client *bq24195_client = NULL;

static struct mutex sys_chg_control_mutex;
int sys_chg_control;

static int bq24195_read(const char reg)
{
	int ret;
	char val;

	if (!bq24195_client)
		return -ENODEV;
	
	ret = i2c_master_reg8_recv(bq24195_client, reg, &val, 1, 100 * 1000);

	if (ret < 0)
		return ret;
	else
		return val; 
}

static int bq24195_write(const char reg, const char val)
{
	char __val = val;
	int ret;
	if (!bq24195_client)
		return -ENODEV;

	ret = i2c_master_reg8_send(bq24195_client, reg, &__val, 1, 100 * 1000);
}

int bqSetVINDPM(int vdpm)
{
/*************************************************************
* bqSetVINDPM:											     *
*															 *
* Accepted Inputs: VINDPM_MIN >= vreg <= VINDPM_MAX  		 *
*															 *
* Returns:												     *
*		-1: Invalid Setting						             *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
* NOTE: Accepted values are determine by VINDPM_MIN,		 *
* VINDPM_MAX variables defined in .h file. If invalid voltage*
* is detected regulation voltage will be kept as it is.		 *
**************************************************************/

	int code = 0;
	int vregbits = 0;
	int success;
	
	if((vdpm < VINDPM_MIN) | (vdpm > VINDPM_MAX))
		//Invalid vreg value
		return -1;
	else
	{
		code = ((vdpm - VINDPM_MIN)/VINDPM_STEP);
		vregbits = code << VINDPM_LSHIFT; 
		Reg00Val = Reg00Val & VINDPM_MASK;
		Reg00Val = vregbits | Reg00Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg00Add, Reg00Val);

		return success;
	}
}

int bqSetChgVoltage(int vreg)
{
/*************************************************************
* bqSetChgVoltage: Send battery regulation voltage in mV and *
* the function will calculate the closest value without      *
* going above the desired value. Function will calculate     *
* the I2C code and store it in vregbits. Reg03Val keeps track*
* of the overall register value as there are other features  *
* that can be programmed on this register.				     *
* Accepted Inputs: VREGMIN >= vreg <= VREG_MAX   	         *
*															 *
* Returns:												     *
*		-1: Invalid Regulation Voltage			             *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
* NOTE: Accepted values are determine by VREG_MAX, VREG_MIN *
*       variables defined in .h file. If invalid voltage is  *
*		detected regulation voltage will be kept as it is.   *
**************************************************************/

	int code = 0;
	int vregbits = 0;
	int success;
	
	if((vreg < VREG_MIN) | (vreg > VREG_MAX))
		//Invalid vreg value
		return -1;
	else
	{
		code = ((vreg - VREG_MIN)/VREG_STEP);
		vregbits = code << VREG_LSHIFT; 
		Reg04Val = Reg04Val & VREG_MASK;
		Reg04Val = vregbits | Reg04Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg04Add, Reg04Val);

		return success;
	}
}

int bqSetIINDPM(int code)
{

/*************************************************************
* bqSetIINDPM: Changes input current limit, actual current	 *
*				is the lesser of the I2C and ILIM settings	 *
*														     *
* Accepted Inputs: IINLIM_100MA, IINLIM_150MA, IINLIM_500MA  *
* IINLIM_900MA, IINLIM_1200MA, IINLIM_1500MA, IINLIM_2000MA, *
* IINLIM_3000MA												 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	
	int success;
	int RegVal;

	if((code != IINLIM_100MA) & (code != IINLIM_150MA)& (code != IINLIM_500MA)& (code != IINLIM_900MA)
		& (code != IINLIM_1200MA)& (code != IINLIM_1500MA)& (code != IINLIM_2000MA)& (code != IINLIM_3000MA))
		return -1;
	else
	{
		RegVal = code << IINDPM_LSHIFT;
		Reg00Val = Reg00Val & IINDPM_MASK;
		Reg00Val = RegVal | Reg00Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg00Add, Reg00Val);	

		return success;
	}

}

int bqSetFASTCHRG(int ichg)
{
/*************************************************************
* bqSetFASTCHRG:											     *
*															 *
* Accepted Inputs: ICHG_MIN >= ichg <= ICHG_MAX  			 *
*															 *
* Returns:												     *
*		-1: Invalid Setting						             *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
* NOTE: Accepted values are determine by ICHG_MIN,			 *
* ICHG_MAX variables defined in .h file. If invalid voltage	 *
* is detected regulation voltage will be kept as it is.		 *
**************************************************************/

	int code = 0;
	int regbits = 0;
	int success;
	
	if((ichg < ICHG_MIN) | (ichg > ICHG_MAX))
		//Invalid vreg value
		return -1;
	else
	{
		code = ((ichg - ICHG_MIN)/ICHG_STEP);
		regbits = code << ICHG_LSHIFT; 
		Reg02Val = Reg02Val & ICHG_MASK;
		Reg02Val = regbits | Reg02Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg02Add, Reg02Val);

		return success;
	}
}

/*
static int set_chg_current(void)
{
	int chg_cur;
	
	return bqSetFASTCHRG(chg_cur);
}
*/

int bqEnTIMER(int enable)
{
/*************************************************************
* bqEnTIMER:Disable or enable Safety Timer Setting	 		 *
* Accepted Inputs: ENABLE, DISABLE							 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	int success;
	int RegVal;

	if((enable != ENABLE) & (enable != DISABLE))
		return -1;
	else
	{
		RegVal = enable << ENTIMER_LSHIFT;
		Reg05Val = Reg05Val & ENTIMER_MASK;
		Reg05Val = RegVal | Reg05Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg05Add, Reg05Val);

		return success;
	}

}

int bqSetWatchDog(int code)
{

/*************************************************************
* bqSetWatchDog:											 *
*														     *
* Accepted Inputs: DISABLE, WatchDog_40s, WatchDog_80s,		 *
*					WatchDog_160s							 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	
	int success;
	int RegVal;

	if((code != DISABLE) & (code != WatchDog_40s)& (code != WatchDog_80s)& (code != WatchDog_160s))
		return -1;
	else
	{
		RegVal = code << WatchDog_LSHIFT;
		Reg05Val = Reg05Val & WatchDog_MASK;
		Reg05Val = RegVal | Reg05Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg05Add, Reg05Val);

		return success;
	}

}

int bqSetCHGCONFIG(int code)
{

/*************************************************************
* bqSetCHGCONFIG: Charger Configuration: Disable, Charge	 *
*					Battery, or OTG							 *
*														     *
* Accepted Inputs: DISABLE, CHARGE_BATTERY, OTG				 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	
	int success;
	int RegVal;

	if((code != DISABLE) & (code != CHARGE_BATTERY)& (code != OTG))
		return -1;
	else
	{
		RegVal = code << CHGCONFIG_LSHIFT;
		Reg01Val = Reg01Val & CHGCONFIG_MASK;
		Reg01Val = RegVal | Reg01Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg01Add, Reg01Val);

		return success;
	}

}

int bqSetSYSMIN(int vlimit)
{
/*************************************************************
* bqSetVINDPM:											     *
*															 *
* Accepted Inputs: SYSMIN_MIN >= vlimit <= SYSMIN_MAX  		 *
*															 *
* Returns:												     *
*		-1: Invalid Setting						             *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
* NOTE: Accepted values are determine by SYSMIN_MIN,		 *
* SYSMIN_MAX variables defined in .h file. If invalid voltage*
* is detected regulation voltage will be kept as it is.		 *
**************************************************************/

	int code = 0;
	int regbits = 0;
	int success;
	
	if((vlimit < SYSMIN_MIN) | (vlimit > SYSMIN_MAX))
		//Invalid vreg value
		return -1;
	else
	{
		code = ((vlimit - SYSMIN_MIN)/SYSMIN_STEP);
		regbits = code << SYSMIN_LSHIFT; 
		Reg01Val = Reg01Val & SYSMIN_MASK;
		Reg01Val = regbits | Reg01Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg01Add, Reg01Val);

		return success;
	}
}

int bqEnDpDmDet(int enable)
{
/*************************************************************
* bqEnDpDmDet:											 *
* Accepted Inputs: ENABLE, DISABLE							 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	int success;
	int RegVal;

	if((enable != ENABLE) & (enable != DISABLE))
		return -1;
	else
	{
		RegVal = enable << ENDPDM_LSHIFT;
		Reg07Val = Reg07Val & ENDPDM_MASK;
		Reg07Val = RegVal | Reg07Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg07Add, Reg07Val);

		return success;
	}

}

int bqEnTERM(int enable)
{
/*************************************************************
* bqEnTERM:Disable or enable Charge Termination		 		 *
* Accepted Inputs: ENABLE, DISABLE							 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	int success;
	int RegVal;

	if((enable != ENABLE) & (enable != DISABLE))
		return -1;
	else
	{
		RegVal = enable << ENTERM_LSHIFT;
		Reg05Val = Reg05Val & ENTERM_MASK;
		Reg05Val = RegVal | Reg05Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg05Add, Reg05Val);

		return success;
	}

}

int bqEnDPDM(int enable)
{
/*************************************************************
* bqEnDPDM:Disable or enable D+/D- Detection		 		 *
* Accepted Inputs: ENABLE, DISABLE							 *
*															 *
* Returns:												     *
*		-1: Invalid Input									 *
*		 0: I2C Write Fail			                         *
*		 1: I2C Write Success						         *
*														     *
* Can be modified to send Ack bit as the success code        *
*															 *
**************************************************************/
	int success;
	int RegVal;

	if((enable != ENABLE) & (enable != DISABLE))
		return -1;
	else
	{
		RegVal = enable << ENDPDM_LSHIFT;
		Reg07Val = Reg07Val & ENDPDM_MASK;
		Reg07Val = RegVal | Reg07Val;

		//Execute I2C Write Function
		success = bq24195_write(Reg07Add, Reg07Val);
		
		return success;
	}
}

static int ntc_test_flag, ntc_test_value;
static ssize_t bq24195_ntc_test_flag_store(struct device *dev, 
				    struct device_attribute *attr,
				    const char *buf, ssize_t count)
{
	ntc_test_flag = simple_strtol(buf, NULL, 10);
	return count;
}

static ssize_t bq24195_ntc_test_value_store(struct device *dev, 
				    struct device_attribute *attr,
				    const char *buf, ssize_t count)
{
	ntc_test_value = simple_strtol(buf, NULL, 10);
	return count;
}


static struct workqueue_struct *bq24195_reset_wq;
static struct work_struct bq24195_reset_work;
static struct timer_list bq24195_reset_timer;

/*----------------------NTC test--------------------------*/
struct bq24195 {
	struct timer_list ntc_watch_timer;
	struct work_struct ntc_watch_work;
};

static struct bq24195 *bq24195;
extern int get_temp_value();
int get_ntc_state(void)
{
	if (ntc_test_flag == 1) {
		return ntc_test_value;
	} else {
/*		int reg09val = bq24195_read(Reg09Add);
		if (reg09val < 0)
			return reg09val;

		reg09val &= 0x07;

		return reg09val;
*/
		return get_temp_value();
	}
}

static void __ntc_watch_work(struct work_struct *work)
{
	pr_info("NTC state: %d\n", get_ntc_state());
}

static void __ntc_watch_timer(unsigned long data)
{
	queue_work(bq24195_reset_wq, &bq24195->ntc_watch_work);
	mod_timer(&bq24195->ntc_watch_timer, jiffies + msecs_to_jiffies(1000));
}
/*-------------------------END----------------------------*/

static void handle_ntc(void)
{
	int chg_set;

	if (get_ntc_state() == 5) {
		pr_info("ntc: TEMP too low!!!!!!!!!!!!\n");
		chg_set = DISABLE;
	} else if (get_ntc_state() == 6) {
		pr_info("ntc: TEMP too high!!!!!!!!!!!!!!!!\n");
		chg_set = DISABLE;
	} else {
		chg_set = CHARGE_BATTERY;
		//pr_info("ntc: TEMP normal!!!!!!!!!!!!!!!!\n");
	}

	bqSetCHGCONFIG(chg_set);
}

int bqGetStat(void)
{
        int reg08val = bq24195_read(Reg08Add);
        if (reg08val < 0)
                return reg08val;

//pr_info("state = %d\n",reg08val);
        reg08val = reg08val >> 4;
        reg08val &= 0x3;
//pr_info("state = %d\n",reg08val);
        return reg08val;
}

int charge_full(void)
{
        // charge ok
        if (bqGetStat() == 3)
                return 1;
        else
                return 0;
}

int usb_connected(void)
{
	int reg08val = bq24195_read(Reg08Add);
	if(reg08val < 0)
		return reg08val;
	
	reg08val = reg08val >> 6;
	reg08val &= 0x3;

	if(reg08val == 1)
		return 1;
	else
		return 0;
}

int bq24195_init(struct i2c_client *client)
{	
	int ret;	
	
	pr_info(" -----------%s\n",__FUNCTION__);
	ret = bqSetWatchDog(DISABLE);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set in watchdog\n");
		goto EXIT;
	}
	msleep(10);
	
	ret = bqSetVINDPM(4440);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set in vdpm\n");
		goto EXIT;
	}
	
	ret = bqSetChgVoltage(4208);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set chg volt\n");
		goto EXIT;
	}
	
	ret = bqSetIINDPM(IINLIM_2000MA);	
	if (ret < 0) {
		dev_err(&client->dev, "failed to set cur limit\n");
		goto EXIT;
	}
	
	ret = bqSetFASTCHRG(2048);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set chg cur\n");
		goto EXIT;
	}

	ret = bqSetSYSMIN(3600);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set in vsys\n");
		goto EXIT;
	}
/*	ret = bqSetCHGCONFIG(CHARGE_BATTERY);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set in chgconfig\n");
		goto EXIT;
	}
*/	
	bqEnTERM(ENABLE);	//charge OK INT
	bqEnTIMER(ENABLE);
	bqEnDPDM(ENABLE);	
	
	mutex_lock(&sys_chg_control_mutex);
//	if (!sys_chg_control)
//		handle_ntc();
	mutex_unlock(&sys_chg_control_mutex);
	
	ret = 0;
EXIT:
    return ret;
}
EXPORT_SYMBOL(bq24195_init);


static ssize_t bq24195_all_regs_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "reg0=%x\nreg1=%x\nreg2=%x\nreg3=%x\nreg4=%x\nreg5=%x\nreg6=%x\nreg7=%x\nreg8=%x\nreg9=%x\nreg10=%x\n", bq24195_read(Reg00Add), \
		bq24195_read(Reg01Add), bq24195_read(Reg02Add), bq24195_read(Reg03Add), \
		bq24195_read(Reg04Add), bq24195_read(Reg05Add), bq24195_read(Reg06Add), \
		bq24195_read(Reg07Add),bq24195_read(Reg08Add),bq24195_read(Reg09Add),bq24195_read(Reg10Add));
}


static ssize_t bq24195_chg_enable_store(struct device *dev, 
				    struct device_attribute *attr,
				    const char *buf, ssize_t count)
{
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&sys_chg_control_mutex);
	if (enable == 1) {
		sys_chg_control = 0;
		bqSetCHGCONFIG(CHARGE_BATTERY);
	} else if (enable == 0) {
		sys_chg_control = 1;
		bqSetCHGCONFIG(DISABLE);
	}
	mutex_unlock(&sys_chg_control_mutex);

	return count;
}

static ssize_t bq24195_chg_enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", !sys_chg_control);
}

static ssize_t bq24195_stat_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int stat = bqGetStat();
	if (stat != 2)
		return sprintf(buf, "ERROR");   // for factory test
	else
		return sprintf(buf, "%d\n", stat);
}



static DEVICE_ATTR(regs, 0644, bq24195_all_regs_show, NULL);
//static DEVICE_ATTR(fault, 0644, bq24195_fault_show, NULL);
static DEVICE_ATTR(stat, 0644, bq24195_stat_show, NULL);
static DEVICE_ATTR(chg_enable, 0644, bq24195_chg_enable_show, bq24195_chg_enable_store);
static DEVICE_ATTR(ntc_test_flag, 0644, NULL, bq24195_ntc_test_flag_store);
static DEVICE_ATTR(ntc_test_value, 0644, NULL, bq24195_ntc_test_value_store);





static void __bq24195_reset_work(struct work_struct *work)
{
	bqSetWatchDog(DISABLE);
	
	msleep(10);
	
	bq24195_init(bq24195_client);
}

static void __bq24195_reset_timer(unsigned long data)
{
	queue_work(bq24195_reset_wq, &bq24195_reset_work);
	bq24195_reset_timer.expires  = jiffies + msecs_to_jiffies(5000);
	add_timer(&bq24195_reset_timer);
}

static int bq24195_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	mutex_init(&sys_chg_control_mutex);
	
	bq24195_client = client;
	
	if (bq24195_init(client)) {
		dev_err(&client->dev, "device not found!\n");
		return -ENODEV;
	}

	device_create_file(&client->dev, &dev_attr_regs);
//	device_create_file(&client->dev, &dev_attr_fault);
	device_create_file(&client->dev, &dev_attr_stat);
	device_create_file(&client->dev, &dev_attr_chg_enable);	
	device_create_file(&client->dev, &dev_attr_ntc_test_flag);
	device_create_file(&client->dev, &dev_attr_ntc_test_value);
/*
	INIT_WORK(&bq24195_reset_work, __bq24195_reset_work);
	bq24195_reset_wq = create_singlethread_workqueue("bq24195_reset_wq");


	setup_timer(&bq24195_reset_timer, __bq24195_reset_timer, NULL);
	bq24195_reset_timer.expires  = jiffies + msecs_to_jiffies(1000);
	add_timer(&bq24195_reset_timer);
*/	
	return 0;
}

static int bq24195_charger_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id bq24195_id[] = {
	{ "bq24195", 0 },
};

static struct i2c_driver bq24195_charger_driver = {
	.driver = {
		.name = "bq24195",
	},
	.probe = bq24195_charger_probe,
	.remove = bq24195_charger_remove,
	.id_table = bq24195_id,
};

static int __init bq24195_charger_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&bq24195_charger_driver);
	if (ret)
		printk(KERN_ERR "Unable to register bq24195 driver\n");
	
	return ret;
}

static void __exit bq24195_charger_exit(void)
{
	i2c_del_driver(&bq24195_charger_driver);
}

subsys_initcall(bq24195_charger_init);
module_exit(bq24195_charger_exit);

MODULE_DESCRIPTION("bq24195 charger driver");
MODULE_LICENSE("GPL");
