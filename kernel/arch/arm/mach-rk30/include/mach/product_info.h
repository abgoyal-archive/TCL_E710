#ifndef __PRODUCT_INFO__
#define __PRODUCT_INFO__

typedef enum {
	JN_BATTERY,
	BYD_BATTERY,
} battery_type;

battery_type get_battery_type(void);

#endif
