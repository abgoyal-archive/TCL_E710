#ifndef __TCL_BOARD_CONFIG__
#define __TCL_BOARD_CONFIG__
#include <linux/wakelock.h>

#define SPK_CON			RK30_PIN6_PA1
#define HALL_INT		RK30_PIN6_PA7
#define BAT_LOW			RK30_PIN6_PA0
#define HP_CON			RK30_PIN0_PC7
#define MT6620_LNA_EN		RK30_PIN4_PB7
#define MIC_SW			RK30_PIN6_PA6
#define NTC_SW			RK30_PIN3_PD3
#define NTC1_INT		RK30_PIN1_PA5  // >60
#define NTC2_INT		RK30_PIN1_PA7  // <-20
#define BATDET_SW		RK30_PIN6_PB3
#define LCD_EN_PIN         	RK30_PIN6_PB0
#define UART_EN			RK30_PIN1_PA6


#define BACKLIGHT_MIN			85
#define BACKLIGHT_MAX 			235

#define CONTINUE_VOLTAGE_IN_CHARGING		3650
#define CONTINUE_VOLTAGE_NOT_IN_CHARGING	3450
#define SHUTDOWN_VOLTAGE_NOT_IN_CHARGING	CONTINUE_VOLTAGE_NOT_IN_CHARGING


#define DC_DET_PIN 		RK30_PIN6_PA5
#define PLAY_ON_PIN		RK30_PIN6_PA2

struct tcl_common_data {
	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	struct workqueue_struct *ntc1_wq;
	struct work_struct ntc1_work;
	struct workqueue_struct *ntc2_wq;
	struct work_struct ntc2_work;
	struct timer_list hall_correct_timer;

	struct work_struct bat_low_work;

	struct wake_lock hall_lock;
	struct wake_lock bat_low_lock;

	/* 表示电池低电的标记 */
	unsigned char bat_low;

	/* 表示cover是否盖上的标记 */
	unsigned char hall_closed;

	struct proc_dir_entry *tcl;
	
	/* 表示android启动完了的标记 */
	unsigned char android_booted;

	struct work_struct android_booted_work;	
};

struct ntc_data {
        struct workqueue_struct     *wq;
        struct delayed_work         delay_work;
        struct adc_client       *client;
};

enum mic_type {
	LOCAL_MIC = 0,
	HP_MIC,
};
void mute_codec(void);
void adjust_codec_max_output_gain(int headset_in);
void change_codec_mic_path(enum mic_type type);

void turn_on_tp(void);
void turn_off_tp(void);
void turn_on_camera(void);
void turn_off_camera(void);
void turn_on_backlight(void);
void turn_off_backlight(void);
void turn_on_speaker(void);
void turn_off_speaker(void);
void turn_on_headset(void);
void turn_off_headset(void);


//int battery_present(void);
//int get_battery_voltage(void);
int get_battery_capacity(void);
inline unsigned char battery_low(void);


unsigned char hall_closed(void);

#define CREATE_PROC_ENTRY_PASSWORD(entry, parent) do { \
        entry = create_proc_entry(#entry, 0666, parent); \
        if (entry) { \
                entry->read_proc = entry##__read_proc; \
                entry->write_proc = entry##__write_proc; \
        } \
} while (0)


#define CREATE_PROC_ENTRY(entry, parent) do { \
	entry = create_proc_entry(#entry, 0644, parent); \
	if (entry) { \
		entry->read_proc = entry##__read_proc; \
		entry->write_proc = entry##__write_proc; \
	} \
} while (0)

#define REMOVE_PROC_ENTRY(name, parent) do { \
    remove_proc_entry(#name, parent); \
} while (0)

int android_booted(void);

//add by wfy----for all module debug----------------------------
#ifdef CONFIG_PROC_TCL_DEBUG
#define DECLARE_FLAG(v) int v##_tcl_debug_flag_get(void)
#define TCL_DEBUG_DEFINE(v,args...) if( v##_tcl_debug_flag_get()) printk("<2>"args)

/*--------------------------------------------------------
  you must declare debug your module debug flag here when you 
  add a module debug.
	declare as this:
		DECLARE_FLAG(new_module_name);
---------------------------------------------------------*/
DECLARE_FLAG(usb_serial); 

/*---------------------------------------------------------
  this define is used print log you want,you can define in the 
  file where you use
-----------------------------------------------------------*/
#define usb_serial_debug(args...)  TCL_DEBUG_DEFINE(usb_serial,args)

#endif
//-------------------------------------------------------------

#endif

