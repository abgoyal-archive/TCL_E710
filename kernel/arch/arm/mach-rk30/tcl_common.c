#include <linux/proc_fs.h>
#include <linux/irq.h>

struct tcl_common_data *tcl_common_data;

#define TPS65910_REGULATOR_DISABLE(X) do { \
	struct regulator *tps65910_##X; \
	char r_name[10] = {0}; \
	strcpy(r_name, #X); \
	tps65910_##X = regulator_get(NULL, #X); \
	if (IS_ERR(tps65910_##X)) \
		pr_err("Failed to get regulator %s: %ld\n", r_name, PTR_ERR(tps65910_##X)); \
	else { \
		pr_info("Disable regulator %s\n", r_name); \
		regulator_disable(tps65910_##X); \
		regulator_put(tps65910_##X); \
	} \
} while (0)

#define TPS65910_REGULATOR_ENABLE(X, Y, Z) do { \
	struct regulator *tps65910_##X; \
	char r_name[10] = {0}; \
	strcpy(r_name, #X); \
	tps65910_##X = regulator_get(NULL, #X); \
	if (IS_ERR(tps65910_##X)) \
		pr_err("Failed to get regulator %s: %ld\n", r_name, PTR_ERR(tps65910_##X)); \
	else { \
		pr_info("Enable regulator %s\n", r_name); \
		regulator_enable(tps65910_##X); \
		regulator_set_voltage(tps65910_##X, Y, Z); \
		regulator_put(tps65910_##X); \
	} \
} while (0)

void turn_on_tp(void)
{
	TPS65910_REGULATOR_ENABLE(vaux2, 2900000, 2900000);
}

void turn_off_tp(void)
{
	TPS65910_REGULATOR_DISABLE(vaux2);
}

void turn_on_camera(void)
{
	TPS65910_REGULATOR_ENABLE(vaux1, 2800000, 2800000);
}

void turn_off_camera(void)
{
	TPS65910_REGULATOR_DISABLE(vaux1);
}

void turn_on_VCC_VIP(void)
{
	TPS65910_REGULATOR_ENABLE(vdig1, 2700000, 2700000);
}

void turn_off_VCC_VIP(void)
{
	TPS65910_REGULATOR_DISABLE(vdig1);
}

void turn_on_VCCA33(void)
{
	TPS65910_REGULATOR_ENABLE(vaux33, 3300000, 3300000);
}

void turn_off_VCCA33(void)
{
	TPS65910_REGULATOR_DISABLE(vaux33);
}

inline unsigned char battery_low(void)
{
	return tcl_common_data->bat_low;
}

void turn_on_GPS(void)
{
	gpio_direction_output(MT6620_LNA_EN, GPIO_HIGH);
}

void turn_off_GPS(void)
{
	gpio_direction_output(MT6620_LNA_EN, GPIO_LOW);
}

void turn_on_speaker(void)
{
//	turn_on_camera();
	gpio_set_value(SPK_CON, GPIO_HIGH);
}


void turn_off_speaker(void)
{
//	turn_off_camera();
	gpio_set_value(SPK_CON, GPIO_LOW);
}

void turn_on_headset(void)
{
	gpio_set_value(HP_CON, GPIO_HIGH);
}

void turn_off_headset(void)
{
	gpio_set_value(HP_CON, GPIO_LOW);
}


extern void rk29_send_power_key(int state);
extern suspend_state_t get_suspend_state(void);
static void inline send_power_key(void)
{
	rk29_send_power_key(1);
	rk29_send_power_key(0);
}

static void hall_correct(unsigned long data)
{
	int hall_gpio_stat = gpio_get_value(HALL_INT);

	if (hall_gpio_stat == 0) 
		if (get_suspend_state() == 0)
			send_power_key();
}

extern void kernel_power_off(void);
static void do_ntc1_work(struct work_struct *work)
{
	pr_info("DC_DET = %d\n", gpio_get_value(DC_DET_PIN));
	if (gpio_get_value(DC_DET_PIN) == 1)	//not in charge
		kernel_power_off();
}


static void do_ntc2_work(struct work_struct *work)
{
	pr_info("DC_DET = %d\n", gpio_get_value(DC_DET_PIN));
	if (gpio_get_value(DC_DET_PIN) == 1)	//not in charge
		kernel_power_off();
}

static void do_hall_work(struct work_struct *work)
{
	int hall_gpio_stat;
	
	hall_gpio_stat = gpio_get_value(HALL_INT);
	if (hall_gpio_stat) {
		rk28_send_wakeup_key();
		tcl_common_data->hall_closed = 0;
	} else {
		del_timer(&tcl_common_data->hall_correct_timer);

		if (get_suspend_state() == 0)
			send_power_key();
		
		tcl_common_data->hall_correct_timer.expires = jiffies + msecs_to_jiffies(2000);
		add_timer(&tcl_common_data->hall_correct_timer);
		
		tcl_common_data->hall_closed = 1;
	}
}

#if 0
static void do_bat_low_work(struct work_struct *work)
{
	wake_lock(&tcl_common_data->bat_low_lock);

	msleep(2 * 60 * 1000);

	if (gpio_get_value(BAT_LOW) == GPIO_LOW) {
		pr_info("%s:%d\n", __func__, __LINE__);

		tcl_common_data->bat_low = 1;

		rk28_send_wakeup_key();
	}

	wake_unlock(&tcl_common_data->bat_low_lock);
}
#endif

unsigned char hall_closed(void)
{
	return tcl_common_data->hall_closed;
}

static irqreturn_t hall_int_handler(int irq, void *dev_id)
{
	pr_info("%s!!!!!!!!!\n", __func__);
	irq_set_irq_type(gpio_to_irq(HALL_INT), gpio_get_value(HALL_INT) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	queue_work(tcl_common_data->hall_wq, &tcl_common_data->hall_work);
	return IRQ_HANDLED;
}

static irqreturn_t ntc1_int_handler(int irq, void *dev_id)
{
	pr_info("%s!!!!!!!!!\n", __func__);
	irq_set_irq_type(gpio_to_irq(NTC1_INT), IRQF_TRIGGER_FALLING);
	queue_work(tcl_common_data->ntc1_wq, &tcl_common_data->ntc1_work);

	return IRQ_HANDLED;
}



static irqreturn_t ntc2_int_handler(int irq, void *dev_id)
{
	pr_info("%s!!!!!!!!!\n", __func__);
	irq_set_irq_type(gpio_to_irq(NTC2_INT), IRQF_TRIGGER_FALLING);
	queue_work(tcl_common_data->ntc2_wq, &tcl_common_data->ntc2_work);
	
	return IRQ_HANDLED;
}

#if 0
static irqreturn_t bat_low_handler(int irq, void *dev_id)
{
	pr_info("bat_low\n");
	disable_irq_nosync(irq);
	schedule_work(&tcl_common_data->bat_low_work);
	return IRQ_HANDLED;
}
#endif

int android_booted(void)
{
	return tcl_common_data->android_booted;
}

static int android_booted__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
    
	len = sprintf(page, "%d\n", tcl_common_data->android_booted);
	return len;
}


extern void startup_charger_detect(void);
extern unsigned char startup_charger_detected;
extern void reinit_battery_voltage_samples(void);
//extern struct mutex mutex; /* from rk29_adc_battery.c */

extern int headset_in(void);
static void do_some_job(void)
{

	pr_info("%s\n",__FUNCTION__);
	startup_charger_detect();
//	reinit_battery_voltage_samples();
	msleep(500);
//	mutex_lock(&mutex);
	startup_charger_detected = 1;
//	mutex_unlock(&mutex);
	msleep(6000);
	tcl_common_data->android_booted = 1;
	
	if(headset_in()) 
		turn_on_headset();
}

static void do_android_booted_work(struct work_struct *work)
{
	do_some_job();
}
/*
 * android启动完的时候，会设置属性sys.boot_completed=1，在init.rc配置，当sys.boot_completed=1
 * 的时候，往/proc/tcl/android_booted写1.
 */
static int android_booted__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int tmp = simple_strtol(buffer, NULL, 10);
//	tcl_common_data->android_booted = simple_strtol(buffer, NULL, 10);

//	schedule_work(&tcl_common_data->android_booted_work);
	queue_work(tcl_common_data->hall_wq, &tcl_common_data->android_booted_work);
	
	return count;
}

extern int get_adc2_value(void);
static int adc2__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	len = sprintf(page, "%d\n", get_adc2_value());
	return len;
}

static int adc2__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	return count;
}
static int mic_sw_value;
static int mic_sw__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
    
	len = sprintf(page, "%d\n", mic_sw_value);
	return len;
}

static int mic_sw__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int tmp = simple_strtol(buffer, NULL, 10);

	mic_sw_value = tmp;

	gpio_set_value(MIC_SW, tmp);

	return count;
}

static int batdet_sw_value;
static int batdet_sw__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
    
	len = sprintf(page, "%d\n", batdet_sw_value);
	return len;
}

static int batdet_sw__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int tmp = simple_strtol(buffer, NULL, 10);

	batdet_sw_value = tmp;

	gpio_set_value(BATDET_SW, tmp);

	return count;
}

//add by wfy-----for all module debug-----------------------------------
#ifdef CONFIG_PROC_TCL_DEBUG
#define ADD_DEBUG_ITAM(name,help) static int name##_tcl_debug_flag=0;\
	int name##_tcl_debug_flag_get(void){return name##_tcl_debug_flag;}\
	static char name##_tcl_debug_name[]=#name; \
	static char name##_tcl_debug_help[]=help

#define TCL_DEBUG_ITAM(name) { name##_tcl_debug_name,&( name##_tcl_debug_flag), name##_tcl_debug_help}
struct tcl_debug_support_s {
	char *name;
	int *flag;
	char *help;
};

/*------------------------------------------------------
  you can add a debug item here
    for example:
	ADD_DEBUG_ITAM(item_name,"item help info");
-------------------------------------------------------*/
ADD_DEBUG_ITAM(usb_serial,"open usb serial read and write info.");

/*------------------------------------------------------
  when you add a debug item,you must add the item in this
  table
    example:
	TCL_DEBUG_ITAM(item_name),
-------------------------------------------------------*/
static struct tcl_debug_support_s tcl_debug_support_table[]={
	TCL_DEBUG_ITAM(usb_serial),
	{NULL,NULL,NULL},
};

static int tcldbg__read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len=0,i;
	char *buf=page;
	
	len+=sprintf(buf,"tcl_debug support:\n");
	
	for(i=0;tcl_debug_support_table[i].name!=NULL;i++)
		len+=sprintf(buf+len,"  %d.  %s(%d):%s\n",i+1,  
		tcl_debug_support_table[i].name,
		*(tcl_debug_support_table[i].flag),
		tcl_debug_support_table[i].help);

	return len;
}

static int tcldbg__write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int i,len,val=-1;
	struct tcl_debug_support_s *tp=tcl_debug_support_table;

	for(i=0;tp[i].name!=NULL;i++){
		len=strlen(tp[i].name);
		if(!strncmp(buffer,tp[i].name,len))
			break;
	}

	if(tp[i].name==NULL){
		printk("<2>""no item as %s\n",buffer);
		return count;
	}

	while(len<count){
		if(buffer[len]==' '){
			len++;
			continue;
		}

		if((buffer[len]>'0') && (buffer[len]<='9'))
			val=1;
		else
			val=0;
		break;
	}

	*(tp[i].flag)=val;
	printk("<2>""%s debug %s!\n",tp[i].name,val?"open":"close");
	return count;
}
#endif
//--------------------------------------------------------

static void create_tcl_proc_entry(struct tcl_common_data *tcd)
{
	tcd->tcl = proc_mkdir("tcl", NULL);
	if (tcd->tcl) {
		struct proc_dir_entry *android_booted;
		struct proc_dir_entry *mic_sw;
		struct proc_dir_entry *adc2;
		struct proc_dir_entry *batdet_sw;
//add by wfy-----for all module debug-----------------------------------
#ifdef CONFIG_PROC_TCL_DEBUG
		struct proc_dir_entry *tcldbg; 
#endif
//----------------------------------------------------------------
		CREATE_PROC_ENTRY(android_booted, tcd->tcl);
		CREATE_PROC_ENTRY(mic_sw, tcd->tcl);
		CREATE_PROC_ENTRY(adc2, tcd->tcl);
		CREATE_PROC_ENTRY(batdet_sw, tcd->tcl);
//add by wfy-----for all module debug-----------------------------------
#ifdef CONFIG_PROC_TCL_DEBUG
		CREATE_PROC_ENTRY(tcldbg, tcd->tcl);
#endif
//-------------------------------------------------------------
	}
}

#define TCL_KERNEL_VERSION "0.1v"
void tcl_common_init(void)
{
	int ret;

	pr_info("TCL_KERNEL_VERSION:%s!!!!!!!!!!!!!!!!\n", TCL_KERNEL_VERSION);

	tcl_common_data = kzalloc(sizeof(struct tcl_common_data), GFP_KERNEL);
	if (!tcl_common_data) {
		pr_err("kzalloc tcl_common_data failed!\n");
		return;
	}

	tcl_common_data->hall_wq = create_singlethread_workqueue("hall_wq");
	if (!tcl_common_data->hall_wq) {
		pr_err("create hall_wq failed!\n");
		return;
	}
#if 0
	tcl_common_data->ntc1_wq = create_singlethread_workqueue("ntc1_wq");
	if (!tcl_common_data->ntc1_wq) {
		pr_err("create ntc1_wq failed!\n");
		return;
	}

	tcl_common_data->ntc2_wq = create_singlethread_workqueue("ntc2_wq");
	if (!tcl_common_data->ntc2_wq) {
		pr_err("create ntc2_wq failed!\n");
		return;
	}
#endif
	/* speaker control */
	ret = gpio_request(SPK_CON, NULL);
	if (ret != 0) {
		gpio_free(SPK_CON);
		pr_err("SPK_CON gpio_request err \n ");
	} else
		gpio_direction_output(SPK_CON, GPIO_LOW);
		//gpio_direction_output(SPK_CON, GPIO_HIGH);

	/* hp control */
	ret = gpio_request(HP_CON, NULL);
	if (ret != 0) {
		gpio_free(HP_CON);
		pr_err("HP_CON gpio_request err \n ");
	} else
		gpio_direction_output(HP_CON, GPIO_LOW);

	/* uart_en */
	rk30_mux_api_set(GPIO1A6_UART1CTSN_SPI0RXD_NAME,GPIO1A_GPIO1A6);
	ret = gpio_request(UART_EN, NULL);
	if (ret != 0) {
		gpio_free(UART_EN);
		pr_err("UART_EN gpio_request err \n ");
	} else
		gpio_direction_output(UART_EN, GPIO_HIGH);

	/* batdet control */
	ret = gpio_request(BATDET_SW, NULL);
	if (ret != 0)
	{
		gpio_free(BATDET_SW);
		pr_err("BATDET_SW gpio_request err \n ");
	} else
		gpio_direction_output(BATDET_SW, GPIO_HIGH);	
	

	/* 6620 LEA control */
	ret = gpio_request(MT6620_LNA_EN, NULL);
	if (ret != 0) {
		gpio_free(MT6620_LNA_EN);
		pr_err("MT6620_LNA_EN gpio_request err \n ");
	} else
		gpio_direction_output(MT6620_LNA_EN, GPIO_HIGH);

#if 0
	/* backlight control */
	ret = gpio_request(VIB_CON, NULL);
	if (ret != 0) {
		gpio_free(VIB_CON);
		pr_err("VIB_CON gpio_request err \n ");
	} else
		gpio_direction_output(VIB_CON, GPIO_LOW);
#endif
	/* MIC_SW */
	ret = gpio_request(MIC_SW, NULL);
	if (ret != 0) {
		gpio_free(MIC_SW);
		pr_err("MIC_SW gpio_request err \n ");
	} else
		gpio_direction_output(MIC_SW, GPIO_HIGH);

	mic_sw_value = 1;

        /* NTC_SW */
	rk30_mux_api_set(GPIO3D3_UART3SIN_NAME,GPIO3D_GPIO3D3);
        ret = gpio_request(NTC_SW, NULL);
        if (ret != 0) {
                gpio_free(NTC_SW);
                pr_err("NTC_SW gpio_request err \n ");
        } else
                gpio_direction_output(NTC_SW, GPIO_HIGH);
#if 0
	/* NTC1 */

        rk30_mux_api_set(GPIO1A5_UART1SOUT_SPI0CLK_NAME,GPIO1A_GPIO1A5);
	ret = gpio_request(NTC1_INT, NULL);
        if (ret != 0) {
                gpio_free(NTC1_INT);
                pr_err("NTC1_INT gpio_request err\n");
        } else {
                int irq;
                INIT_WORK(&tcl_common_data->ntc1_work, do_ntc1_work);
                gpio_direction_input(NTC1_INT);
                irq = gpio_to_irq(NTC1_INT);
                ret = request_irq(irq, ntc1_int_handler, IRQF_TRIGGER_FALLING, "ntc1_int", NULL);	//gpio_get_value(NTC1_INT) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING
                if(ret < 0) {
                        gpio_free(NTC1_INT);
                        pr_err("NTC1_INT request_irq err\n");
                }

                enable_irq_wake(irq);
        }

	/* NTC2 */
	rk30_mux_api_set(GPIO1A7_UART1RTSN_SPI0TXD_NAME,GPIO1A_GPIO1A7);
        ret = gpio_request(NTC2_INT, NULL);
        if (ret != 0) {
                gpio_free(NTC2_INT);
                pr_err("NTC2_INT gpio_request err\n");
        } else {
                int irq;
                INIT_WORK(&tcl_common_data->ntc2_work, do_ntc2_work);
                gpio_direction_input(NTC2_INT);
                irq = gpio_to_irq(NTC2_INT);
                ret = request_irq(irq, ntc2_int_handler, IRQF_TRIGGER_FALLING, "ntc2_int", NULL);
                if(ret < 0) {
                        gpio_free(NTC2_INT);
                        pr_err("NTC2_INT request_irq err\n");
                }

                enable_irq_wake(irq);
        }
#endif
	/* HALL_INT */

	ret = gpio_request(HALL_INT, NULL);
	if (ret != 0) {
		gpio_free(HALL_INT);
		pr_err("HALL_INT gpio_request err\n");
	} else {
		int irq;
		gpio_direction_input(HALL_INT);
		irq = gpio_to_irq(HALL_INT);
		ret = request_irq(irq, hall_int_handler, gpio_get_value(HALL_INT) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING, "hall_int", NULL);
		if(ret < 0) {
			gpio_free(HALL_INT);
			pr_err("HALL_INT request_irq err\n");
		}
		
		enable_irq_wake(irq);
		INIT_WORK(&tcl_common_data->hall_work, do_hall_work);
	}

	init_timer(&tcl_common_data->hall_correct_timer);
	setup_timer(&tcl_common_data->hall_correct_timer, hall_correct, 0);

#if 0
	/* BAT_LOW */
	ret = gpio_request(BAT_LOW, NULL);
	if (ret != 0) {
		gpio_free(BAT_LOW);
		pr_err("BAT_LOW gpio_request err\n");
	} else {
		int irq;
		gpio_direction_input(BAT_LOW);
		irq = gpio_to_irq(BAT_LOW);
		ret = request_irq(irq, bat_low_handler, IRQF_TRIGGER_FALLING, "bat_low", NULL);
		if (ret < 0) {
			gpio_free(BAT_LOW);
			pr_err("BAT_LOW request_irq err\n");
		}
		
		enable_irq_wake(irq);
		INIT_WORK(&tcl_common_data->bat_low_work, do_bat_low_work);
	}
#endif

	wake_lock_init(&tcl_common_data->hall_lock, WAKE_LOCK_SUSPEND, "hall_lock");
//	wake_lock_init(&tcl_common_data->bat_low_lock, WAKE_LOCK_SUSPEND, "bat_low_lock");

	create_tcl_proc_entry(tcl_common_data);

	INIT_WORK(&tcl_common_data->android_booted_work, do_android_booted_work);
}

