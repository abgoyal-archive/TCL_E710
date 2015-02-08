#include <linux/adc.h>

#define TRS_TRRS_CONNECTOR_CHECK

#define OFF 0
#define ON 1
static inline void speaker_switch(int x)
{
	if (x == OFF) {
		pr_info("Speaker Close\n");
		turn_off_speaker();
	} else {
		pr_info("Speaker Open\n");
		turn_on_speaker();
	}
}

#ifdef TRS_TRRS_CONNECTOR_CHECK
#define AD2VOLTAGE(ad) (((ad) * 2500 * (220 + 220)) / (1024 * 220))
enum mic_type get_mic_type(int headset_status)
{
	struct adc_client *client;
	int ad_value;

	client = adc_register(1, NULL, NULL);
	if (!client)
		return LOCAL_MIC;
	
	ad_value = adc_sync_read(client);
	pr_info("Headset type value: AD(%d), VOLTAGE(%d)\n", ad_value, AD2VOLTAGE(ad_value));

	adc_unregister(client);

	if (AD2VOLTAGE(ad_value) > 2000 && headset_status == HEADSET_IN)
		return HP_MIC;
	else
		return LOCAL_MIC;
}
#else
enum mic_type get_mic_type(int headset_status)
{
	if (headset_status == HEADSET_IN)
		return HP_MIC;
	else
		return LOCAL_MIC;
}
#endif

static int prev_headset_gpio_level;

static void headset_state_init(void)
{
	if (gpio_get_value(headset_info->pdata->Headset_gpio) == GPIO_HIGH) {
		headset_info->headset_status = HEADSET_OUT;
		headset_info->cur_headset_status = ~(BIT_HEADSET|BIT_HEADSET_NO_MIC);
	} else {
		headset_info->headset_status = HEADSET_IN;
		headset_info->cur_headset_status = BIT_HEADSET_NO_MIC;
	}

	prev_headset_gpio_level = gpio_get_value(headset_info->pdata->Headset_gpio);
	
	switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);
	
//	change_codec_mic_path(get_mic_type(headset_info->headset_status));
}

#define IN_LOOP_CNT (2000 / 50)
#define OUT_LOOP_CNT (1000 / 50)
static struct timer_list headset_scan;
static struct work_struct headset_work;
static struct workqueue_struct *headset_wq;
static int loop;
static int set_state;

static void headset_scan_timer(unsigned long data)
{
	queue_work(headset_wq, &headset_work);
	mod_timer(&headset_scan, jiffies + msecs_to_jiffies(50));
}

static void headset_work_work(struct work_struct *w)
{
	int headset_gpio_level = gpio_get_value(headset_info->pdata->Headset_gpio);

	if (prev_headset_gpio_level == GPIO_HIGH) {
		/* 如果耳机检测口电平从高到低，则说明有插耳机的意图，马上关闭功放并上报一次耳机插入事件，让android系统能及时做出反应 */
		if (headset_gpio_level == GPIO_LOW && set_state == 0) {
			speaker_switch(OFF);
                        if(android_booted())
                        	{
                                gpio_set_value(HP_CON, GPIO_HIGH);
			printk("set HP_CON GPIO_HIGH\n");
                              }

			headset_info->headset_status = HEADSET_IN;
			headset_info->cur_headset_status = BIT_HEADSET_NO_MIC;
			switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);

			set_state = 1;
		}

		if (set_state == 1)
			loop++;

		/* 去抖动处理 */
		if (loop == IN_LOOP_CNT) {
			/* 强行认为耳机已经完全插入了，就算此时耳机被拔出来了，也会立刻在下面的else分支里纠正过来 */
			prev_headset_gpio_level = GPIO_LOW;
//			change_codec_mic_path(get_mic_type(HEADSET_IN));

			set_state = 0;
			loop = 0;
		}
	} else {
		if (headset_gpio_level == GPIO_HIGH && set_state == 0) {
			headset_info->headset_status = HEADSET_OUT;
			headset_info->cur_headset_status = ~(BIT_HEADSET|BIT_HEADSET_NO_MIC);
			switch_set_state(&headset_info->sdev, headset_info->cur_headset_status);
//miaozh modify			
//			gpio_set_value(HP_CON, GPIO_HIGH);
			gpio_set_value(HP_CON, GPIO_LOW);
			printk("set HP_CON GPIO_LOW\n");

			set_state = 1;
		}

		if (set_state == 1)
			loop++;
	
		/* 这里主要是防止拔耳机时的漏音问题 */
		if (loop == OUT_LOOP_CNT) {
			prev_headset_gpio_level = GPIO_HIGH;
//			change_codec_mic_path(get_mic_type(HEADSET_OUT));

			/* speaker_switch(ON); */

			set_state = 0;
			loop = 0;
		}
	}
}

static void headset_timer_init(void)
{
	INIT_WORK(&headset_work, headset_work_work);
	headset_wq = create_singlethread_workqueue("headset_wq");

	init_timer(&headset_scan);
	headset_scan.function = headset_scan_timer;
	headset_scan.data = 0;
	headset_scan.expires = jiffies + msecs_to_jiffies(500);
	add_timer(&headset_scan);
}
/*
int headset_in(void)
{
	return headset_info->headset_status;
}
*/
