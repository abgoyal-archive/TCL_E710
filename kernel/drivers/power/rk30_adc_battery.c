/* drivers/power/rk30_adc_battery.c
 *
 * battery detect driver for the rk30 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/wakelock.h>

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif

static int rk30_battery_dbg_level = 0;
module_param_named(dbg_level, rk30_battery_dbg_level, int, 0644);
#define pr_bat( args...) \
	do { \
		if (rk30_battery_dbg_level) { \
			pr_info(args); \
		} \
	} while (0)


/*******************以下参数可以修改******************************/
#define	TIMER_MS_COUNTS		 50	//定时器的长度ms 1000 ------modified by wh,2013-2-19
//以下参数需要根据实际测试调整
#define	SLOPE_SECOND_COUNTS	               15	//统计电压斜率的时间间隔s
#define	DISCHARGE_MIN_SECOND	               30	//最快放电电1%时间  45   ------modified by wh,2013-2-19
#define	CHARGE_MIN_SECOND	               45	//最快充电电1%时间
#define	CHARGE_MID_SECOND	               90	//普通充电电1%时间
#define	CHARGE_MAX_SECOND	               250	//最长充电电1%时间
#define   CHARGE_FULL_DELAY_TIMES          10          //充电满检测防抖时间
#define   USBCHARGE_IDENTIFY_TIMES        5           //插入USB混流，pc识别检测时间

#define	NUM_VOLTAGE_SAMPLE	                       ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_DISCHARGE_MIN_SAMPLE	         ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_CHARGE_MIN_SAMPLE	         ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	         ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	     
#define	NUM_CHARGE_MAX_SAMPLE	         ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	  
#define   NUM_CHARGE_FULL_DELAY_TIMES         ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define   NUM_USBCHARGE_IDENTIFY_TIMES      ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度

#define BAT_2V5_VALUE	                                     2500

#define BATT_FILENAME "/data/bat_last_capacity.dat"
#ifndef CONFIG_AC_SUSPEND
static struct wake_lock batt_wake_lock;
#endif
static struct i2c_client *bq24195_client;

struct batt_vol_cal{
	u32 disp_cal;
	u32 dis_charge_vol;
	u32 charge_vol;
};

struct temp_ad_table{
	u32 temp;
	u32 ad_value;
};
#ifdef CONFIG_BATTERY_RK30_VOL3V8

#define BATT_MAX_VOL_VALUE                             4120               	//满电时的电池电压	 
#define BATT_ZERO_VOL_VALUE                            3400              	//关机时的电池电压
#define BATT_NOMAL_VOL_VALUE                         3800               
//divider resistance 
#define BAT_PULL_UP_R                                         200
#define BAT_PULL_DOWN_R                                    200

/*
static struct batt_vol_cal  batt_table1[] = {
	{0,3450,3654},{5,3556,3810},{10,3580,3833},{15,3602,3868},{20,3626,3900},{25,3641,3906},
	{30,3651,3916},{35,3662,3926},{40,3675,3938},{45,3691,3956},{50,3709,3975},
	{55,3731,3997},{60,3755,4021},{65,3784,4048},{70,3814,4080},{75,3849,4113},
	{80,3887,4152},{85,3932,4193},{90,3976,4198},{95,4026,4198},{100,4105,4198},
};
*/
static struct batt_vol_cal  batt_table1[] = {
	{0,3450,3654},{10,3573,3818},{20,3632,3869},
	{30,3654,3904},{40,3677,3917},{50,3711,3938},
	{60,3756,3968},{70,3820,4003},
	{80,3886,4043},{90,3963,4100},{100,4050,4170},
};


#else
#define BATT_MAX_VOL_VALUE                              8284              	//Full charge voltage
#define BATT_ZERO_VOL_VALUE                             6800            	// power down voltage 
#define BATT_NOMAL_VOL_VALUE                          7600                

//定义ADC采样分压电阻，以实际值为准，单位K

#define BAT_PULL_UP_R                                         300 
#define BAT_PULL_DOWN_R                                    100

static struct batt_vol_cal  batt_table1[] = {
	{0,6800,7400},    {1,6840,7440},    {2,6880,7480},     {3,6950,7450},       {5,7010,7510},    {7,7050,7550},
	{9,7080,7580},    {11,7104,7604},   {13,7140,7640},   {15,7160,7660},      {17,7220,7720},
	{19,7260,7760},  {21,7280,7780},   {23,7304,7802},   {25,7324,7824},      {27,7344,7844},
	{29,7360,7860},  {31,7374,7874},   {33,7386,7886},   {35,7398,7898},      {37,7410,7910},//500
	{39,7420,7920},  {41,7424,7928},   {43,7436,7947},   {45,7444,7944},      {47,7450,7958}, //508
	{49,7460,7965},  {51,7468,7975},   {53, 7476,7990},  {55,7482,8000},      {57,7492,8005}, // 5 14
	{59,7500,8011},  {61,7510,8033},   {63,7528,8044},   {65,7548,8055},      {67,7560,8066},//506
	{69,7600,8070},  {71,7618,8075},   {73,7634,8080},   {75,7654,8085},      {77,7690,8100}, //400
	{79,7900,8180},  {81,7920,8210},   {83,7964,8211},   {85,8000,8214},      {87,8002,8218},//290
	{89,8012, 8220}, {91,8022,8235},   {93,8110,8260},   {95,8140,8290},       {97,8170,8300},  {100,8200 ,8310},//110

};
#endif


#define BATT_NUM  ARRAY_SIZE(batt_table1)
#define adc_to_voltage(adc_val)                           ((adc_val * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R))

struct batt_id_table {
	int 	bat_id_min;
	int 	bat_id_max;
	struct batt_vol_cal *batt_vol_table;
};

struct batt_id_table batid_table[] = {
	{0,800,batt_table1},
};
static struct batt_vol_cal  batt_table_def[] = {
	{0,3450,3654},{10,3580,3833},{20,3626,3900},
	{30,3651,3916},{40,3675,3938},{50,3709,3975},
	{60,3755,4021},{70,3814,4080},
	{80,3887,4152},{90,3976,4193},{100,4105,4198},
};

static struct temp_ad_table  temp_table[] = {
	{-30,1083},{-29,1076},{-28,1069},{-27,1062},{-26,1054},
	{-25,1047},{-24,1040},{-23,1032},{-22,1023},{-21,1015},
	{-20,1006},{-19,997}, {-18,988}, {-17,979}, {-16,969},
	{-15,960}, {-14,950}, {-13,940}, {-12,929}, {-11,919},
	{-10,908}, {-9,897},  {-8,886},  {-7,875},  {-6,864},
	{-5,852},  {-4,840},  {-3,829},  {-2,817},  {-1,805},
	{0,793}, {1,781}, {2,769}, {3,756}, {4,744},
	{5,732}, {6,719}, {7,707}, {8,694}, {9,682},
	{10,670},{11,657},{12,645},{13,633},{14,620},
	{15,608},{16,596},{17,584},{18,572},{19,560},
	{20,548},{21,537},{22,525},{23,514},{24,503},
	{25,492},{26,481},{27,469},{28,459},{29,449},
	{30,438},{31,428},{32,418},{33,408},{34,398},
	{35,389},{36,379},{37,370},{38,361},{39,352},
	{40,344},{41,335},{42,327},{43,319},{44,311},
	{45,303},{46,296},{47,288},{48,281},{49,274},
	{50,267},{51,260},{52,253},{53,247},{54,241},
	{55,234},{56,228},{57,223},{58,217},{59,211},
	{60,206},{61,201},{62,195},{63,190},{64,186},
	{65,181},{66,176},{67,172},{68,167},{69,163},
};

static struct batt_vol_cal *batt_table;
//static struct batt_id_table *batid_bat_table;

//#define BATID_TABLE_NUM (sizeof(batid_table)/sizeof(batt_id_table))
#define BATID_TABLE_NUM ARRAY_SIZE(batid_table)
#define TEMP_TABLE_NUM ARRAY_SIZE(temp_table)

/********************************************************************************/

extern int get_ntc_state(void);
extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);
extern int charge_full(void);


struct rk30_adc_battery_data {
	int irq;
	
	struct timer_list       timer;
	struct workqueue_struct *wq;
	struct delayed_work 	    delay_work;
	struct work_struct 	    dcwakeup_work;
	struct work_struct                   lowerpower_work;
	bool                    resume;
	
	struct rk30_adc_battery_platform_data *pdata;

	int                     full_times;
	
	struct adc_client       *client; 
	int                     adc_val;
	int			bat_id;
	int                     adc_samples[NUM_VOLTAGE_SAMPLE+2];
	
	int                     bat_status;
	int                     bat_status_cnt;
	int                     bat_health;
	int                     bat_present;
	int                     bat_voltage;
	int                     bat_capacity;
	int                     bat_change;
	
	int                     old_charge_level;
	int                    *pSamples;
	int                     gBatCapacityDisChargeCnt;
	int                     gBatCapacityChargeCnt;
	int 	          capacitytmp;
	int                     poweron_check;
	int                     suspend_capacity;

	int                     status_lock;
	
	int						low_power;
	//----------lgw@tcl---------------------
	//ntc?ì2aμ????è1y???ò1yμíí￡?13?μ?μ?±ê??
	struct delayed_work	charger_detect_work;
	int			charger;
	unsigned char ntc_not_charging;
};
struct rk30_adc_battery_data *gBatteryData;

enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

void switch_to_batdet(void)
{	
	gpio_set_value(BATDET_SW, GPIO_HIGH);
}
void switch_to_batid(void)
{	
	gpio_set_value(BATDET_SW, GPIO_LOW);
}

static int print_bat_i=1;
static int print_bat_num=1;
static int print_bat_flag = 1;
static void rk30_adc_battery_scan_timer(unsigned long data)
{
	if(print_bat_flag)
	{
		print_bat_i++;
		if(print_bat_i==20*60)	//print every minute
		{
			print_bat_i=1;
			printk(KERN_INFO ">>>>>>>>>>>>>>>>%d:voltage = %d:capacity_show = %d%%\n",print_bat_num++,gBatteryData->bat_voltage,gBatteryData->bat_capacity);
		}
	}
	mod_timer(&gBatteryData->timer, jiffies + msecs_to_jiffies(50));
}

static ssize_t print_bat_flag_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int n=simple_strtoul(buf,NULL,0);
	printk(KERN_INFO ">>>>>>>>>>>>n=%d\n",n);
	print_bat_flag=n;
	print_bat_i=1;
	print_bat_num=1;

	return count;
}

static DEVICE_ATTR(print_bat_flag, 0644, NULL, print_bat_flag_store);
// lgw 2012-06-14
static ssize_t rk30_adc_battery_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "bat_id = %d\n",gBatteryData->bat_id);
}
static DEVICE_ATTR(batid_show, 0644, rk30_adc_battery_id_show, NULL);

static int rk30_adc_battery_load_capacity(void)
{
	char value[4];
	int* p = (int *)value;
	long fd = sys_open(BATT_FILENAME,O_RDONLY,0);

	if(fd < 0){
		pr_bat("rk30_adc_battery_load_capacity: open file /data/bat_last_capacity.dat failed\n");
		return -1;
	}

	sys_read(fd,(char __user *)value,4);
	sys_close(fd);

	return (*p);
}

static void rk30_adc_battery_put_capacity(int loadcapacity)
{
	char value[4];
	int* p = (int *)value;
	long fd = sys_open(BATT_FILENAME,O_CREAT | O_RDWR,0);

	if(fd < 0){
		pr_bat("rk30_adc_battery_put_capacity: open file /data/bat_last_capacity.dat failed\n");
		return;
	}
	
	*p = loadcapacity;
	sys_write(fd, (const char __user *)value, 4);

	sys_close(fd);
}

static void rk30_adc_battery_charge_enable(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
	}
}

static void rk30_adc_battery_charge_disable(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
	}
}

extern int usb_connected();
//extern int suspend_flag;
static int rk30_adc_battery_get_charge_level(struct rk30_adc_battery_data *bat)
{
	int charge_on = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
#if 0
#if defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	if (pdata->dc_det_pin != INVALID_GPIO){
		if (gpio_get_value (pdata->dc_det_pin) == pdata->dc_det_level){
			if (usb_connected())
				charge_on = CHARGER_USB;
			else
				charge_on = CHARGER_AC;
		}
	}
#endif

#if defined  (CONFIG_BATTERY_RK30_USB_CHARGE)
	if (charge_on == 0){
		if (suspend_flag)
			return;
		if (1 == dwc_vbus_status()) {          //检测到USB插入，但是无法识别是否是充电器
		                                 //通过延时检测PC识别标志，如果超时检测不到，说明是充电
			if (0 == get_msc_connect_flag()){                               //插入充电器时间大于一定时间之后，开始进入充电状态
				if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES){
					gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
					charge_on = 1;
				}
			}                               //否则，不进入充电模式
		}                   
		else{
			gBatUsbChargeCnt = 0;
			if (2 == dwc_vbus_status()) {
				charge_on = 1;
			}
		}
	}
#endif
#endif
	charge_on = bat->charger;
/*
        if (charge_on == CHARGER_AC || charge_on == CHARGER_USB)
                keep_wake__wake_lock();
        else
                keep_wake__wake_unlock();
*/
	return charge_on;
}

extern int get_temp_value();

static struct power_supply rk30_ac_supply;
static int ntc_state; //ntc?ì2aμ?μ?×′ì??μ
static int rk30_adc_battery_status_samples(struct rk30_adc_battery_data *bat)
{
	static int old_state = 0;
	int new_state;

	//----------lgw@tcl-----------------------
	int ntc_exception; //ntc?ì2aμ????èòì3￡ê±μ?±ê??
	int update_ac_property; //?üD?êê???÷×′ì?
	//----------END---------------------------

	int charge_level;
	
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	charge_level = rk30_adc_battery_get_charge_level(bat);

	//检测充电状态变化情况
	if (charge_level != bat->old_charge_level){
		bat->old_charge_level = charge_level;
		bat->bat_change  = 1;
		
		if(charge_level == CHARGER_AC) {            
			rk30_adc_battery_charge_enable(bat);
		}
		else{
			rk30_adc_battery_charge_disable(bat);
		}
		bat->bat_status_cnt = 0;        //状态变化开始计数
	}

	//---------------lgw@tcl----------------
	update_ac_property = 0;
	ntc_exception = 0;

	ntc_state = get_ntc_state();
	new_state = ntc_state;

	if (ntc_state)			//温度异常
	{
		ntc_exception = 1;
		goto NTC_EXCEPTION;
	}
	else				//温度正常
	{
		if (bat->ntc_not_charging)		//未充电
		{
			bat->ntc_not_charging = 0;	//使能充电
			bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			bat->bat_change = 1;	
			update_ac_property = 1;
		}
		old_state = new_state;
	}
	//----------------END-------------------
	
	if(charge_level == 0){   
	//discharge
		bat->full_times = 0;
		bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else{
	//CHARGE	    
		#if 0
		if (pdata->charge_ok_pin == INVALID_GPIO){  //no charge_ok_pin

			if (bat->bat_capacity == 100){
				if (bat->bat_status != POWER_SUPPLY_STATUS_FULL){
					bat->bat_status = POWER_SUPPLY_STATUS_FULL;
					bat->bat_change  = 1;
				}
			}
			else{
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		}
		else{  // pin of charge_ok_pin
		#endif
			if (!charge_full()){

				bat->full_times = 0;
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			}
			else{
	//检测到充电满电平标志
				bat->full_times++;

				if (bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) {
					bat->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
				}

				if ((bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) && (bat->bat_capacity >= 90)){
					if (bat->bat_status != POWER_SUPPLY_STATUS_FULL){
						bat->bat_status = POWER_SUPPLY_STATUS_FULL;
						bat->bat_capacity = 100;
						bat->bat_change  = 1;
					}
				}
				else{
					bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
				}
			}
		#if 0
		}
		#endif
	}
        if (charge_level == CHARGER_USB)
                bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

//----------------lgw@tcl----------------------------------
NTC_EXCEPTION:
	if (ntc_exception)			//温度异常
	{
		bat->ntc_not_charging = 1;	//停止充电

		if (old_state != new_state)	//温度状态变化
		{
			pr_info("old_state = %d,new_state = %d\n",old_state,new_state);

			bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			bat->bat_change = 1;
			update_ac_property = 1;
		}
		old_state = new_state;
	}

//	if (update_ac_property)
//		power_supply_changed(&rk30_ac_supply);
//-----------------END--------------------------------------

	
	return charge_level;
}

unsigned char startup_charger_detected;
static int *pSamples;
static void rk30_adc_battery_voltage_samples(struct rk30_adc_battery_data *bat)
{
	int value;
	int i,*pStart = bat->adc_samples, num = 0;
	int level = rk30_adc_battery_get_charge_level(bat);


	value = bat->adc_val;
	adc_async_read(bat->client);

	*pSamples++ = adc_to_voltage(value);

	bat->bat_status_cnt++;
	if (bat->bat_status_cnt > NUM_VOLTAGE_SAMPLE)  bat->bat_status_cnt = NUM_VOLTAGE_SAMPLE + 1;

	num = pSamples - pStart;
	
	if (num >= NUM_VOLTAGE_SAMPLE){
		pSamples = pStart;
		num = NUM_VOLTAGE_SAMPLE;
		
	}

	value = 0;
	for (i = 0; i < num; i++){
		value += bat->adc_samples[i];
	}
	bat->bat_voltage = value / num;

	/*消除毛刺电压*/
	if(level){
		if(bat->bat_voltage >= batt_table[BATT_NUM-1].charge_vol+ 10)
			bat->bat_voltage = batt_table[BATT_NUM-1].charge_vol  + 10;
		else if(bat->bat_voltage <= batt_table[0].charge_vol  - 10)
			bat->bat_voltage =  batt_table[0].charge_vol - 10;
	}
	else{
		if(bat->bat_voltage >= batt_table[BATT_NUM-1].dis_charge_vol+ 10)
			bat->bat_voltage = batt_table[BATT_NUM-1].dis_charge_vol  + 10;
		else if(bat->bat_voltage <= batt_table[0].dis_charge_vol  - 10)
			bat->bat_voltage =  batt_table[0].dis_charge_vol - 10;

	}
}

int timer_count = 0;
static int rk30_adc_battery_voltage_to_capacity(struct rk30_adc_battery_data *bat, int BatVoltage)
{
	int i = 0;
	int capacity = 0;

	struct batt_vol_cal *p;
	p = batt_table;

	if (rk30_adc_battery_get_charge_level(bat)){  //charge
		if(BatVoltage >= (p[BATT_NUM - 1].charge_vol)){
			capacity = 100;
		}	
		else{
			if(BatVoltage <= (p[0].charge_vol)){
				capacity = 0;
			}
			else{
				for(i = 0; i < BATT_NUM - 1; i++){

					if(((p[i].charge_vol) <= BatVoltage) && (BatVoltage < (p[i+1].charge_vol))){
						capacity = p[i].disp_cal + ((BatVoltage - p[i].charge_vol) *  (p[i+1].disp_cal -p[i].disp_cal ))/ (p[i+1].charge_vol- p[i].charge_vol);
						break;
					}
				}
			}  
		}

	}
	else{  //discharge
		if(BatVoltage >= (p[BATT_NUM - 1].dis_charge_vol)){
			capacity = 100;
		}	
		else{
			if(BatVoltage <= (p[0].dis_charge_vol)){
				capacity = 0;
			}
			else{
				for(i = 0; i < BATT_NUM - 1; i++){
					if(((p[i].dis_charge_vol) <= BatVoltage) && (BatVoltage < (p[i+1].dis_charge_vol))){
						capacity =  p[i].disp_cal+ ((BatVoltage - p[i].dis_charge_vol) * (p[i+1].disp_cal -p[i].disp_cal ) )/ (p[i+1].dis_charge_vol- p[i].dis_charge_vol) ;
						break;
					}
				}
			}  

		}


	}
	
	timer_count ++;
	if (20*60 == timer_count)
	{
		timer_count = 0; 
	   	pr_info(">>>>>>>>>>>>>>>BatVoltage_real = %d, capacity_real = %d\n",BatVoltage,capacity);	
	}
    return capacity;
}

extern int wifi_probed;
static void rk30_adc_battery_capacity_samples(struct rk30_adc_battery_data *bat)
{
	int capacity = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	int charge_level = rk30_adc_battery_get_charge_level(bat);
	capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
        //系统启动阶段电量管控不受规则约束
//        if ((startup_charger_detected == 0) && (wifi_probed == 0))
        if (startup_charger_detected == 0)
        {
		if (wifi_probed == 0)
	        {
		        bat->bat_capacity = capacity;
                	bat->bat_change = 1;
                	goto END;
	        }
	}

	//充放电状态变化后，Buffer填满之前，不更新
	if (bat->bat_status_cnt < NUM_VOLTAGE_SAMPLE)  {
		bat->gBatCapacityDisChargeCnt = 0;
		bat->gBatCapacityChargeCnt    = 0;
		return;
	}
	
 
	if (rk30_adc_battery_get_charge_level(bat)){
		if (capacity > bat->bat_capacity){
			//实际采样到的容量比显示的容量大，逐级上升
			if (++(bat->gBatCapacityDisChargeCnt) >= NUM_CHARGE_MIN_SAMPLE){
				bat->gBatCapacityDisChargeCnt  = 0;
				if (bat->bat_capacity < 99){
					bat->bat_capacity++;
					bat->bat_change  = 1;
				}
			}
			bat->gBatCapacityChargeCnt = 0;
		}
		else{  //   实际的容量比采样比 显示的容量小
					bat->gBatCapacityDisChargeCnt = 0;
		            (bat->gBatCapacityChargeCnt)++;
            
			//if (pdata->charge_ok_pin != INVALID_GPIO)
			{
				if (charge_full()){
				//检测到电池充满标志，同时长时间内充电电压无变化，开始启动计时充电，快速上升容量
					if (bat->gBatCapacityChargeCnt >= NUM_CHARGE_MIN_SAMPLE){
						bat->gBatCapacityChargeCnt = 0;
						if (bat->bat_capacity < 99){
							bat->bat_capacity++;
							bat->bat_change  = 1;
						}
					}
				}
				else{
#if 0					
					if (capacity > capacitytmp){
					//过程中如果电压有增长，定时器复位，防止定时器模拟充电比实际充电快
						gBatCapacityChargeCnt = 0;
					}
					else if (/*bat->bat_capacity >= 85) &&*/ (gBatCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE)){
						gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

						if (bat->bat_capacity < 99){
						bat->bat_capacity++;
						bat->bat_change  = 1;
						}
					}
				}
#else			//  防止电池老化后出现冲不满的情况，
					if (capacity > bat->capacitytmp){
					//过程中如果电压有增长，定时器复位，防止定时器模拟充电比实际充电快
						bat->gBatCapacityChargeCnt = 0;
					}
					else{

						if ((bat->bat_capacity >= 85) &&((bat->gBatCapacityChargeCnt) > NUM_CHARGE_MAX_SAMPLE)){
							bat->gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

							if (bat->bat_capacity < 99){
								bat->bat_capacity++;
								bat->bat_change  = 1;
							}
						}
					}
				}
#endif

			}
		#if 0
			else{
			//没有充电满检测脚，长时间内电压无变化，定时器模拟充电
				if (capacity > bat->capacitytmp){
				//过程中如果电压有增长，定时器复位，防止定时器模拟充电比实际充电快
					bat->gBatCapacityChargeCnt = 0;
				}
				else{

					if ((bat->bat_capacity >= 85) &&(bat->gBatCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE)){
						bat->gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);

						if (bat->bat_capacity < 99){
							bat->bat_capacity++;
							bat->bat_change  = 1;
						}
					}
				}
				

			}        
		#endif			
		}
	}    
	else{   
	//放电时,只允许电压下降
		if (capacity < bat->bat_capacity){
			if (++(bat->gBatCapacityDisChargeCnt) >= NUM_DISCHARGE_MIN_SAMPLE){
				bat->gBatCapacityDisChargeCnt = 0;
				if (bat->bat_capacity > 0){
					bat->bat_capacity-- ;
					bat->bat_change  = 1;
				}
			}
		}
		else{
			bat->gBatCapacityDisChargeCnt = 0;
		}
		bat->gBatCapacityChargeCnt = 0;
	}

END:
		bat->capacitytmp = capacity;
}

//extern int boot_charge_full(void);

//static int poweron_check = 0;
static void rk30_adc_battery_poweron_capacity_check(void)
{

	int new_capacity, old_capacity;

	new_capacity = gBatteryData->bat_capacity;
	old_capacity = rk30_adc_battery_load_capacity();
	if ((old_capacity <= 0) || (old_capacity >= 100)){
		old_capacity = new_capacity;
	}    

	if (gBatteryData->bat_status == POWER_SUPPLY_STATUS_FULL){
		if (new_capacity > 80){
			gBatteryData->bat_capacity = 100;
		}
	}
	else if (gBatteryData->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	//chargeing state
	//问题：
//	//1）长时间关机放置后，开机后读取的容量远远大于实际容量怎么办？
//	//2）如果不这样做，短时间关机再开机，前后容量不一致又该怎么办？
//	//3）一下那种方式合适？
	//gBatteryData->bat_capacity = new_capacity;
		gBatteryData->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}else{
#if 0
		if(new_capacity > old_capacity + 50 )
			gBatteryData->bat_capacity = new_capacity;
		else
			gBatteryData->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;  //avoid the value of capacity increase 
#else
                if (new_capacity - old_capacity >= 10)
                        gBatteryData->bat_capacity = new_capacity;
                else if (old_capacity - new_capacity >= 10)
                        gBatteryData->bat_capacity = new_capacity;
                else
                        gBatteryData->bat_capacity = old_capacity;		
#endif
	}

 if (rk30_adc_battery_get_charge_level(gBatteryData) == CHARGER_AC)
    {
                gBatteryData->bat_status = POWER_SUPPLY_STATUS_CHARGING;
                if (charge_full())
                {
                        gBatteryData->bat_status = POWER_SUPPLY_STATUS_FULL;
                        gBatteryData->bat_capacity = 100;
                }
    }

#if 0   //charge off 
        if (boot_charge_full())
                gBatteryData->bat_capacity = 100;
#endif
	//printk("capacity = %d, new_capacity = %d, old_capacity = %d\n",gBatteryData->bat_capacity, new_capacity, old_capacity);

	gBatteryData->bat_change = 1;
}

#if defined(CONFIG_BATTERY_RK30_USB_CHARGE)
static int rk30_adc_battery_get_usb_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}

static enum power_supply_property rk30_adc_battery_usb_props[] = {
    
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_usb_supply = 
{
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,

	.get_property   = rk30_adc_battery_get_usb_property,

	.properties     = rk30_adc_battery_usb_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_usb_props),
};
#endif

#if defined(CONFIG_BATTERY_RK30_AC_CHARGE)
static irqreturn_t rk30_adc_battery_dc_wakeup(int irq, void *dev_id)
{   
	pr_info("enter %s!!!!!!!!!!!!!!\n", __func__);

	queue_work(gBatteryData->wq, &gBatteryData->dcwakeup_work);
	return IRQ_HANDLED;
}


static int rk30_adc_battery_get_ac_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			if (CHARGER_AC == rk30_adc_battery_get_charge_level(gBatteryData))
			{
				int s = ntc_state;//get_ntc_state();
				if (s)
					val->intval = 0;
				else
					val->intval = 1;
				}
			else
				{
				val->intval = 0;	
				}
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk30_adc_battery_ac_props[] = 
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_ac_supply = 
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,

	.get_property   = rk30_adc_battery_get_ac_property,

	.properties     = rk30_adc_battery_ac_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_ac_props),
};

extern int bq24195_init(struct i2c_client *client);
static void rk30_adc_battery_dcdet_delaywork(struct work_struct *work)
{
	int ret;
	struct rk30_adc_battery_platform_data *pdata;
	int irq;
	int irq_flag;
	
	pdata    = gBatteryData->pdata;
	irq        = gpio_to_irq(pdata->dc_det_pin);
	irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	rk28_send_wakeup_key(); // wake up the system

	free_irq(irq, NULL);
	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);// reinitialize the DC irq 
	if (ret) {
		free_irq(irq, NULL);
	}

//	power_supply_changed(&rk30_ac_supply);

	gBatteryData->bat_status_cnt = 0;        //the state of battery is change
        cancel_delayed_work_sync(&gBatteryData->charger_detect_work);
        schedule_delayed_work(&gBatteryData->charger_detect_work, msecs_to_jiffies(2000));

	bq24195_init(bq24195_client);
}


#endif

static int rk30_adc_battery_get_status(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_status);
}

extern struct ntc_data *gNtcData;
extern int get_ntc_value(struct adc_client *client);
static int rk30_adc_battery_get_temp(struct rk30_adc_battery_data *bat)
{
	int i,temp;
	int adc_value = get_ntc_value(gNtcData->client);
	
	for (i = 0; i < TEMP_TABLE_NUM; i++)
	{
        	if((adc_value <= temp_table[i].ad_value) && (adc_value >= temp_table[i+1].ad_value))
       	 	{
                	temp = temp_table[i+1].temp;
                	break;
        	}
	}

	return temp;
}

static int rk30_adc_battery_get_health(struct rk30_adc_battery_data *bat)
{
	int s;
	s = ntc_state;	

	if (1 == s) 
		return POWER_SUPPLY_HEALTH_FROZEN;	//<-20
	else if (2 == s)
		return POWER_SUPPLY_HEALTH_COLD;	//-20~0
	else if (3 == s)				
		return POWER_SUPPLY_HEALTH_OVERHEAT;	//45~60
	else if (4 == s)
		return POWER_SUPPLY_HEALTH_HOT;		//>60
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int rk30_adc_battery_get_present(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage < BATT_MAX_VOL_VALUE) ? 0 : 1;
}

static int rk30_adc_battery_get_voltage(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage );
}

static int rk30_adc_battery_get_capacity(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_capacity);
}

static int rk30_adc_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{		
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = rk30_adc_battery_get_status(gBatteryData);
			DBG("gBatStatus=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = rk30_adc_battery_get_health(gBatteryData);
			DBG("gBatHealth=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_TEMP:
                        val->intval = rk30_adc_battery_get_temp(gBatteryData);
                        DBG("gBatTemp=%d\n",val->intval);
                        break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = rk30_adc_battery_get_present(gBatteryData);
			DBG("gBatPresent=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val ->intval = rk30_adc_battery_get_voltage(gBatteryData);
			DBG("gBatVoltage=%d\n",val->intval);
			break;
		//	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//		val->intval = 1100;
		//		break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = rk30_adc_battery_get_capacity(gBatteryData);
			DBG("gBatCapacity=%d%%\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = BATT_MAX_VOL_VALUE;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = BATT_ZERO_VOL_VALUE;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static enum power_supply_property rk30_adc_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static struct power_supply rk30_battery_supply = 
{
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,

	.get_property   = rk30_adc_battery_get_property,

	.properties     = rk30_adc_battery_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_props),
};

#ifdef CONFIG_PM
static void rk30_adc_battery_resume_check(void)
{
	int i;
	int level,oldlevel;
	int new_capacity, old_capacity;
	struct rk30_adc_battery_data *bat = gBatteryData;

	bat->old_charge_level = -1;
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample
	level = oldlevel = rk30_adc_battery_status_samples(bat);//init charge status

	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++) {               //0.3 s   
	
		mdelay(1);
		rk30_adc_battery_voltage_samples(bat);              //get voltage
		level = rk30_adc_battery_status_samples(bat);       //check charge status
		if (oldlevel != level){		
		    oldlevel = level;                               //if charge status changed, reset sample
		    i = 0;
		}        
	}
	new_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	old_capacity =gBatteryData-> suspend_capacity;

	if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	//chargeing state
		bat->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}
	else{
		bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;  // aviod the value of capacity increase    dicharge
	}

}

//extern struct ntc_data *gNtcData;
static int rk30_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	int irq;
	gBatteryData->suspend_capacity = gBatteryData->bat_capacity;
	cancel_delayed_work(&gBatteryData->delay_work);
	
	cancel_delayed_work(&gNtcData->delay_work);	//wh@tcl 6-18	
//modify by wfy,for disble low power irq(2013-07-31)
#if 0
	if( gBatteryData->pdata->batt_low_pin != INVALID_GPIO){
		
		irq = gpio_to_irq(gBatteryData->pdata->batt_low_pin);
		enable_irq(irq);
	    	enable_irq_wake(irq);
    	}
#endif
	return 0;
}

static int rk30_adc_battery_resume(struct platform_device *dev)
{
	int irq;
	gBatteryData->resume = true;
	queue_delayed_work(gBatteryData->wq, &gBatteryData->delay_work, msecs_to_jiffies(100));
	queue_delayed_work(gNtcData->wq, &gNtcData->delay_work, msecs_to_jiffies(500));	//唤醒添加NTC 查询 wh@tcl 6-18

	if( gBatteryData->pdata->batt_low_pin != INVALID_GPIO){
		
		irq = gpio_to_irq(gBatteryData->pdata->batt_low_pin);
	    	disable_irq_wake(irq);
		disable_irq(irq);
    	}
	return 0;
}
#else
#define rk30_adc_battery_suspend NULL
#define rk30_adc_battery_resume NULL
#endif


unsigned long AdcTestCnt = 0;
static void rk30_adc_battery_timer_work(struct work_struct *work)
{
#ifdef CONFIG_PM
	if (gBatteryData->resume) {
		rk30_adc_battery_resume_check();
		gBatteryData->resume = false;
	}
#endif
	
	rk30_adc_battery_status_samples(gBatteryData);

	if (gBatteryData->poweron_check && startup_charger_detected){   
		gBatteryData->poweron_check = 0;
		rk30_adc_battery_poweron_capacity_check();
	}

	rk30_adc_battery_voltage_samples(gBatteryData);
	rk30_adc_battery_capacity_samples(gBatteryData);

	if( CHARGER_AC == rk30_adc_battery_get_charge_level(gBatteryData)){  // charge
		if(0 == gBatteryData->status_lock ){	
		#ifndef CONFIG_AC_SUSPEND
			wake_lock(&batt_wake_lock);  //lock
		#endif
			gBatteryData->status_lock = 1; 
		}
	}
	else{
		if(1 == gBatteryData->status_lock ){	
		#ifndef CONFIG_AC_SUSPEND
			wake_unlock(&batt_wake_lock);  //unlock
		#endif
			gBatteryData->status_lock = 0; 
		}

	}
	
	/*update battery parameter after adc and capacity has been changed*/
	if(gBatteryData->bat_change){
		gBatteryData->bat_change = 0;
		if (startup_charger_detected)
			rk30_adc_battery_put_capacity(gBatteryData->bat_capacity);
		power_supply_changed(&rk30_battery_supply);
	}

	if (rk30_battery_dbg_level){
		if (++AdcTestCnt >= 2)
			{
			AdcTestCnt = 0;

			printk("Status = %d, RealAdcVal = %d, RealVol = %d,gBatVol = %d, gBatCap = %d, RealCapacity = %d, dischargecnt = %d, chargecnt = %d\n", 
			gBatteryData->bat_status, gBatteryData->adc_val, adc_to_voltage(gBatteryData->adc_val), 
			gBatteryData->bat_voltage, gBatteryData->bat_capacity, gBatteryData->capacitytmp, gBatteryData->gBatCapacityDisChargeCnt,gBatteryData-> gBatCapacityChargeCnt);

		}
	}
	queue_delayed_work(gBatteryData->wq, &gBatteryData->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));

}

void check_bat_vol(struct rk30_adc_battery_data *bat)
{
        struct batt_vol_cal *p;
        p = batt_table;


	rk30_adc_battery_voltage_samples(gBatteryData);              //get voltage
	pr_info("==============%s,%d,vol = %d\n",__func__,__LINE__,bat->bat_voltage);

	if (0 == rk30_adc_battery_get_charge_level(bat)){  			//discharge
		if(bat->bat_voltage <= (p[0].dis_charge_vol))
                {       bat->bat_capacity = 0;
			pr_info("==============%s,%d,vol = %d\n",__func__,__LINE__,bat->bat_voltage);
		}	
	}
	bat->bat_change  = 1;	
}

static int rk30_adc_battery_io_init(struct rk30_adc_battery_platform_data *pdata)
{
	int ret = 0;
	
	if (pdata->io_init) {
		pdata->io_init();
	}
	
	//charge control pin
	if (pdata->charge_set_pin != INVALID_GPIO){
	    	ret = gpio_request(pdata->charge_set_pin, NULL);
	    	if (ret) {
	    		printk("failed to request dc_det gpio\n");
	    		goto error;
		    	}
	    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
	}
	
	//dc charge detect pin
	if (pdata->dc_det_pin != INVALID_GPIO){
	    	ret = gpio_request(pdata->dc_det_pin, NULL);
	    	if (ret) {
	    		printk("failed to request dc_det gpio\n");
	    		goto error;
	    	}
	
	    	gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
	    	ret = gpio_direction_input(pdata->dc_det_pin);
	    	if (ret) {
	    		printk("failed to set gpio dc_det input\n");
	    		goto error;
	    	}
	}
	
	//charge ok detect
	if (pdata->charge_ok_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->charge_ok_pin, NULL);
	    	if (ret) {
	    		printk("failed to request charge_ok gpio\n");
	    		goto error;
	    	}
	
	    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
	    	ret = gpio_direction_input(pdata->charge_ok_pin);
	    	if (ret) {
	    		printk("failed to set gpio charge_ok input\n");
	    		goto error;
	    	}
	}
	//batt low pin
	if( pdata->batt_low_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->batt_low_pin, NULL);
	    	if (ret) {
	    		printk("failed to request batt_low_pin gpio\n");
	    		goto error;
	    	}
	
	    	gpio_pull_updown(pdata->batt_low_pin, GPIOPullUp); 
	    	ret = gpio_direction_input(pdata->batt_low_pin);
	    	if (ret) {
	    		printk("failed to set gpio batt_low_pin input\n");
	    		goto error;
	    	}
	}
    
	return 0;
error:
	return -1;
}

//extern void kernel_power_off(void);
static void rk30_adc_battery_check(struct rk30_adc_battery_data *bat)
{
	int i;
	int level,oldlevel;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	printk("%s--%d:\n",__FUNCTION__,__LINE__);

	bat->old_charge_level = -1;
	bat->capacitytmp = 0;
	bat->suspend_capacity = 0;
	
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample
	level = oldlevel = rk30_adc_battery_status_samples(bat);//init charge status

	bat->full_times = 0;
	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++){                //0.3 s
		mdelay(1);
		rk30_adc_battery_voltage_samples(bat);              //get voltage
		//level = rk30_adc_battery_status_samples(bat);       //check charge status
		level = rk30_adc_battery_get_charge_level(bat);

		if (oldlevel != level){
			oldlevel = level;                               //if charge status changed, reset sample
			i = 0;
		}        
	}

	bat->bat_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);  //init bat_capacity
	
	bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (rk30_adc_battery_get_charge_level(bat)){
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;

		if (charge_full()){
			//if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level)
			{
				bat->bat_status = POWER_SUPPLY_STATUS_FULL;
				bat->bat_capacity = 100;
			}
		}
	}



#if 0
	rk30_adc_battery_poweron_capacity_check();
#else
	gBatteryData->poweron_check = 1;
#endif
//	gBatteryData->poweron_check = 0;

/*******************************************
//开机采样到的电压和上次关机保存电压相差较大，怎么处理？
if (bat->bat_capacity > old_capacity)
{
if ((bat->bat_capacity - old_capacity) > 20)
{

}
}
else if (bat->bat_capacity < old_capacity)
{
if ((old_capacity > bat->bat_capacity) > 20)
{

}
}
*********************************************/
	if (bat->bat_capacity == 0) bat->bat_capacity = 1;


#if 0
	if ((bat->bat_voltage <= batt_table[0].dis_charge_vol+ 50)&&(bat->bat_status != POWER_SUPPLY_STATUS_CHARGING)){
		kernel_power_off();
	}
#endif
}

static void rk30_adc_battery_callback(struct adc_client *client, void *param, int result)
{
#if 0
	struct rk30_adc_battery_data  *info = container_of(client, struct rk30_adc_battery_data,
		client);
	info->adc_val = result;
#endif
	if (result < 0){
		pr_bat("adc_battery_callback    resule < 0 , the value ");
		return;
	}
	else{
		gBatteryData->adc_val = result;
		pr_bat("result = %d, gBatteryData->adc_val = %d\n", result, gBatteryData->adc_val );
	}
	return;
}

/*-------------lgw@tcl---------------*/
int battery_power_low(void)
{
	return gBatteryData->low_power;
}
/*--------------END-------------------*/

#if 1
static void rk30_adc_battery_lowerpower_delaywork(struct work_struct *work)
{
	int irq;
#ifndef CONFIG_AC_SUSPEND
	wake_lock(&batt_wake_lock);
#endif
	
//	if( gBatteryData->pdata->batt_low_pin != INVALID_GPIO){
//		irq = gpio_to_irq(gBatteryData->pdata->batt_low_pin);
//		disable_irq(irq);
//	}

	msleep(2000);
	
	if (!gpio_get_value(gBatteryData->pdata->batt_low_pin))
	{
		printk("lowerpower\n");
		
		gBatteryData->low_power = 1;
		
		rk28_send_wakeup_key(); // wake up the system
		
		power_supply_changed(&rk30_battery_supply);	
	}
	else
	{
	#ifndef CONFIG_AC_SUSPEND
		wake_unlock(&batt_wake_lock);
	#else
		;
	#endif
	}
	
	return;
}


static irqreturn_t rk30_adc_battery_low_wakeup(int irq,void *dev_id)
{
	pr_info("enter %s!!!!!!!!!!!!!\n", __func__);

	queue_work(gBatteryData->wq, &gBatteryData->lowerpower_work);
	return IRQ_HANDLED;
}

#endif

extern int usb_connected();
static void rk30_adc_battery_charger_detect_work(struct work_struct *work)
{
        struct rk30_adc_battery_platform_data *pdata = gBatteryData->pdata;

        if (gpio_get_value(pdata->dc_det_pin) == GPIO_HIGH)
                gBatteryData->charger = CHARGER_BATTERY;
        else {
                if (usb_connected())
                        gBatteryData->charger = CHARGER_USB;
                else
                        gBatteryData->charger = CHARGER_AC;
        }

         //更新适配器状态
        power_supply_changed(&rk30_ac_supply);

        pr_info("%s: charger=%d\n", __func__, gBatteryData->charger);
}


void startup_charger_detect(void)
{
        schedule_delayed_work(&gBatteryData->charger_detect_work, msecs_to_jiffies(1));
}
static int rk30_adc_battery_probe(struct platform_device *pdev)
{
	int    ret;
	int    irq;
	int    irq_flag;
	int    i = 0;
	struct adc_client                   *client;
	struct rk30_adc_battery_data          *data;
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gBatteryData = data;

	platform_set_drvdata(pdev, data);

   	data->pdata = pdata;
	data->status_lock = 0; 	
	ret = rk30_adc_battery_io_init(pdata);
	 if (ret) {
	 	goto err_io_init;
	}
    
	memset(data->adc_samples, 0, sizeof(int)*(NUM_VOLTAGE_SAMPLE + 2));

	 //register adc for battery sample
	client = adc_register(0, rk30_adc_battery_callback, NULL);  //pdata->adc_channel = ani0
	if(!client)
		goto err_adc_register_failed;
	    
	 //variable init
	data->client  = client;

	/*-----------wh---------*/	
	switch_to_batid();		//read batid
	data->bat_id = adc_sync_read(client);

	for(i = 0;i < BATID_TABLE_NUM;i++)
	{
		if((data->bat_id >= batid_table[i].bat_id_min) && (data->bat_id <= batid_table[i].bat_id_max))
		{
			batt_table = batid_table[i].batt_vol_table;
			break;
		}
	}
	if(i == BATID_TABLE_NUM)
	{
		batt_table = batt_table_def;
	}

	switch_to_batdet();		//read bat_value
	/*-----------wh---------*/
	data->adc_val = adc_sync_read(client);	
	
	ret = power_supply_register(&pdev->dev, &rk30_battery_supply);
	if (ret){
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
		

#if defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk30_usb_supply);
	if (ret){
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
#ifndef CONFIG_AC_SUSPEND
 	wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");	
#endif

	data->wq = create_singlethread_workqueue("adc_battd");
	INIT_DELAYED_WORK(&data->delay_work, rk30_adc_battery_timer_work);
	queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));
	
	INIT_DELAYED_WORK(&data->charger_detect_work, rk30_adc_battery_charger_detect_work);	

#if  defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk30_ac_supply);
	if (ret) {
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
pr_info("------------%s,%d\n",__func__,__LINE__);
	//init dc dectet irq & delay work
	if (pdata->dc_det_pin != INVALID_GPIO){
		INIT_WORK(&data->dcwakeup_work, rk30_adc_battery_dcdet_delaywork);
		
		irq = gpio_to_irq(pdata->dc_det_pin);	        
		irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	    	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);
	    	if (ret) {
	    		printk("failed to request dc det irq\n");
	    		goto err_dcirq_failed;
	    	}
	    	enable_irq_wake(irq);  
    	
	}
#endif

pr_info("------------%s,%d\n",__func__,__LINE__);
#if 0
	// batt low irq lowerpower_work
	if( pdata->batt_low_pin != INVALID_GPIO){
		INIT_WORK(&data->lowerpower_work, rk30_adc_battery_lowerpower_delaywork);
		
		irq = gpio_to_irq(pdata->batt_low_pin);
	    	ret = request_irq(irq, rk30_adc_battery_low_wakeup, IRQF_TRIGGER_FALLING/*IRQF_TRIGGER_LOW*/, "batt_low_irq", NULL);

	    	if (ret) {
	    		printk("failed to request batt_low_irq irq\n");
	    		goto err_lowpowerirq_failed;
	    	}
//		disable_irq(irq);
//		disable_irq_wake(irq);  
//		disable_irq(irq);
    	
    	}
#endif
pr_info("------------%s,%d\n",__func__,__LINE__);
	setup_timer(&data->timer, rk30_adc_battery_scan_timer, (unsigned long)data);
	data->timer.expires  = jiffies + 2000;
	add_timer(&data->timer);

	if (gpio_get_value(pdata->dc_det_pin) == GPIO_LOW)
		data->charger = CHARGER_AC;
	else
		data->charger = CHARGER_BATTERY;

	//Power on Battery detect
	rk30_adc_battery_check(data);
	// lgw 2012-06-14
	device_create_file(&pdev->dev, &dev_attr_batid_show);
	device_create_file(&pdev->dev, &dev_attr_print_bat_flag);
	//lgw 2012-07-04
//	keep_wake__wake_lock_init();

	printk(KERN_INFO "rk30_adc_battery: driver initialized\n");
	
	return 0;
	
#if defined (CONFIG_BATTERY_RK30_USB_CHARGE)
err_usb_failed:
	power_supply_unregister(&rk30_usb_supply);
#endif

err_ac_failed:
#if defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	power_supply_unregister(&rk30_ac_supply);
#endif

err_battery_failed:
	power_supply_unregister(&rk30_battery_supply);
    
err_dcirq_failed:
	free_irq(gpio_to_irq(pdata->dc_det_pin), data);
#if 1
 err_lowpowerirq_failed:
	free_irq(gpio_to_irq(pdata->batt_low_pin), data);
#endif
err_adc_register_failed:
err_io_init:    
err_data_alloc_failed:
	kfree(data);

	printk("rk30_adc_battery: error!\n");
    
	return ret;
}

static int rk30_adc_battery_remove(struct platform_device *pdev)
{
	struct rk30_adc_battery_data *data = platform_get_drvdata(pdev);
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;

	cancel_delayed_work(&gBatteryData->delay_work);	
#if defined(CONFIG_BATTERY_RK30_USB_CHARGE)
	power_supply_unregister(&rk30_usb_supply);
#endif
#if defined(CONFIG_BATTERY_RK30_AC_CHARGE)
	power_supply_unregister(&rk30_ac_supply);
#endif
	power_supply_unregister(&rk30_battery_supply);

	free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	kfree(data);
	//lgw 2012-07-04
//	keep_wake__wake_lock_destroy();
	
	return 0;
}

static struct platform_driver rk30_adc_battery_driver = {
	.probe		= rk30_adc_battery_probe,
	.remove		= rk30_adc_battery_remove,
	.suspend		= rk30_adc_battery_suspend,
	.resume		= rk30_adc_battery_resume,
	.driver = {
		.name = "rk30-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk30_adc_battery_init(void)
{
	return platform_driver_register(&rk30_adc_battery_driver);
}

static void __exit rk30_adc_battery_exit(void)
{
	platform_driver_unregister(&rk30_adc_battery_driver);
}

/* subsys_initcall(rk30_adc_battery_init); */
module_init(rk30_adc_battery_init);
module_exit(rk30_adc_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk30");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");
