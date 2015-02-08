#ifndef __BQ24195_CHARGER__
#define __BQ24195_CHARGER__

//extern int bq24195_init(struct i2c_client *client);
/*
#define Reg0Add	0x00
#define Reg1Add	0x01
#define Reg2Add	0x02
#define Reg3Add	0x03
#define Reg4Add	0x04
#define Reg5Add	0x05
#define Reg6Add	0x06
#define Reg7Add	0x07


//#define DevID 0x6B

// Variables used on bqSetChgVolt Function
#define VOREG_LSHIFT 2
#define VOREG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged
#define VOREG_STEP 20 // Value in mV
#define VOREG_MAX 4440 // Value in mV
#define VOREG_MIN 3500 // Value in mV

// Variables used on bqSetInCurLimit Function
#define IN_ILIM_1 1500 //value in mA
#define IN_ILIM_2 2500 //value in mA
#define IN_ILIM_MASK 0xFD //Bits Set to 1 on mask will remain unchanged
#define IN_ILIM_LSHIFT 1

// Variables used on bqDpDmDetect Function
#define DP_DM_DETECT_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
#define DP_DM_DETECT_ENABLE_BIT 1
#define DP_DM_DETECT_LSHIFT 0

// Variables used on bqSetSupplyPriority Function
#define NO_SUPPLY -1
#define IN_SUPPLY 0
#define USB_SUPPLY 1
#define SUPPLY_PRIORITY_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged
#define SUPPLY_PRIORITY_BIT 0x08

// Variables used on  bqWatchDogRst Function
#define WATCH_DOG_RST 1
#define WATCH_DOG_RST_LSHIFT 7
#define WATCH_DOG_RST_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on  bqUsbLockout Function
#define USB_LOCK 1
#define USB_UNLOCK 0
#define USB_LOCKOUT_LSHIFT 3
#define USB_LOCKOUT_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged

// Variables used on  bqNoBatOp Function
#define NO_BAT_OP_EN 1
#define NO_BAT_OP_DIS 0
#define NO_BAT_OP_LSHIFT 0
#define NO_BAT_OP_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

// Variables used on bqHiZMode Function
#define HI_Z_EN 1
#define HI_Z_DIS 0 
#define HI_Z_LSHIFT 0
#define HI_Z_MASK 0x7E //Bits Set to 1 on mask will remain unchanged

// Variables used on bqChgEnable Function
#define CHG_EN 0
#define CHG_DIS 1
#define CHG_EN_LSHIFT 1
#define CHG_EN_MASK 0x7D //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnableChgCurTerm Function
#define CHG_CUR_TERM_EN 1
#define CHG_CUR_TERM_DIS 0
#define CHG_CUR_TERM_EN_LSHIFT 2
#define CHG_CUR_TERM_EN_MASK 0x7B //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnableStat Function
#define STAT_EN 1
#define STAT_DIS 0
#define STAT_EN_LSHIFT 3
#define STAT_EN_MASK 0x77 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetUsbCurLimit Function 
#define USB2_H_100MA 0 //000-USB2.0 host with 100mA current limit
#define USB3_H_150MA 1 //001-USB3.0 host with 150mA current limit
#define USB2_H_500MA 2 //010 – USB2.0 host with 500mA current limit
#define USB_HC_800MA 3 //011 – USB host/charger with 800mA current limit
#define USB3_H_900MA 4 //100 – USB3.0 host with 900mA current limit
#define USB_HC_1500MA 5 //101 – USB host/charger with 1500mA current limit
#define USB_CUR_LIMIT_LSHIFT 4
#define USB_CUR_LIMIT_MASK 0x0F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqAllRegReset Function
#define RESET 1
#define RESET_LSHIFT 7
#define RESET_MASK 0x7F //Bits Set to 1 on mask will remain unchanged


// Variables used on bqSetChgCur Function
#define ICHG_MIN 550 //value in mA
#define ICHG_MAX 2500 //value in mA
#define ICHG_STEP 75 //value in mA
#define ICHG_LSHIFT 3
#define ICHG_MASK 0x07 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetTermCur Function
#define ITERM_MIN 50 //value in mA
#define ITERM_MAX 400 //value in mA
#define ITERM_STEP 50 //value in mA
#define ITERM_LSHIFT 0
#define ITERM_MASK 0xF8 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetUsbVdpm Function
#define USB_VDPM_MIN 4200 //value in mA
#define USB_VDPM_MAX 4760 //value in mA
#define USB_VDPM_STEP 80 //value in mA
#define USB_VDPM_LSHIFT 3
#define USB_VDPM_MASK 0xC7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetInVdpm Function
#define IN_VDPM_MIN 4200 //value in mA
#define IN_VDPM_MAX 4760 //value in mA
#define IN_VDPM_STEP 80 //value in mA
#define IN_VDPM_LSHIFT 0
#define IN_VDPM_MASK 0xF8 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnable2xSlowTimer Function
#define SLOW_TIMER_EN 1
#define SLOW_TIMER_DIS 0
#define SLOW_TIMER_LSHIFT 7
#define SLOW_TIMER_MASK 0x7F //Bits Set to 1 on mask will remain unchanged


// Variables used on bqSetSafteyTimer Function
#define FAST_CHG_27_MIN 0
#define FAST_CHG_6_HR 1
#define FAST_CHG_9_HR 2
#define DISABLE 3
#define SAFETY_TIMER_LSHIFT 5
#define SAFETY_TIMER_MASK 0x9F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqThermalShutdown Function
#define TS_EN 1
#define TS_DIS 0
#define TS_LSHIFT 3
#define TS_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqLowChg Function
#define LOW_CHG_EN 1
#define LOW_CHG_DIS 0
#define LOW_CHG_LSHIFT 0
#define LOW_CHG_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
*/
	
	//Parameter Settings
	int bqSetVINDPM(int vdpm);			//REG00[6:3]
	int bqSetIINDPM(int idpm);			//REG00[2:0]
	int bqSetCHGCONFIG(int code);		//REG01[5:4]
	int bqSetSYSMIN(int vlimit);		//REG01[3:1]
	//int bqSetOTGILIM(int code);		//REG01[0]			//Does not exist in 195/195L Spin
	int bqSetFASTCHRG(int ichg);		//REG02[7:2]
	int bqSetICHRG20PCNT(int code); 	//REG02[0]
	int bqSetPRECHRG(int iprechg);		//REG03[7:4]
	int bqSetTERMCHRG(int iterm);		//REG03[3:0]
	int bqSetChgVoltage(int vreg);		//REG04[7:2]
	int bqSetBATLOWV(int setting);		//REG04[1]
	int bqSetRECHRG(int setting);		//REG04[0]
	int bqSetWatchDog(int code);		//REG05[5:4]
	int bqSetFastChgTimer(int code);	//REG05[2:1]
	//int bqSetJeitaISET(int setting);	//REG05[0]			//Does not exist in 195/195L Spin
	//int bqSetBATCOMP(int resistor);	//REG06[7:5]		//Does not exist in 195/195L Spin
	//int bqSetVCLAMP(int vclamp);		//REG06[4:2]		//Does not exist in 195/195L Spin
	int bqSetTREG(int code);			//REG06[1:0]
	//int bqSetJeitaVSET(int setting);	//REG07[4]
	
	
	//Control Settings
	int bqEnHIZ(int enable);			//REG00[7]
	int bqRstREG(); 					//REG01[7]
	int bqRstWatchDog();				//REG01[6]
	int bqEnTERM(int enable);			//REG05[7]
	int bqTERMSTAT(int enable); 		//REG05[6]
	int bqEnTIMER(int enable);			//REG05[3]
	int bqEnDPDM(int enable);			//REG07[7]
	int bqEnTMR2X(int enable);			//REG07[6]
	int bqOffBATFET(int enable);		//REG07[5]
	int bqEnINTCHRGFAULT(int enable);	//REG07[1]
	int bqEnINTBATFAULT(int enable);	//REG07[0]
	
	unsigned int Reg00Val;
	unsigned int Reg01Val;
	unsigned int Reg02Val;
	unsigned int Reg03Val;
	unsigned int Reg04Val;
	unsigned int Reg05Val;
	unsigned int Reg06Val;
	unsigned int Reg07Val;
	unsigned int Reg08Val;	//Read Only
	unsigned int Reg09Val;	//Read Only
	unsigned int Reg10Val;
	
#define Reg00Add	0x00
#define Reg01Add	0x01
#define Reg02Add	0x02
#define Reg03Add	0x03
#define Reg04Add	0x04
#define Reg05Add	0x05
#define Reg06Add	0x06
#define Reg07Add	0x07
#define Reg08Add	0x08
#define Reg09Add	0x09
#define Reg10Add	0x0A
	
#define DevID 0x6B //bq24190/192/192I/195
	//#define DevID 0x6A //bq24191
	
#define DISABLE 0
#define ENABLE 1
#define RESET 1
	
	// Variables used on bqSetVINDPM Function
	#define VINDPM_MIN 3880		//value in mV
	#define VINDPM_MAX 5080		//value in mV
	#define VINDPM_STEP 80		//value in mV
	#define VINDPM_OFFSET 3880	//value in mV
	#define VINDPM_LSHIFT 3
	#define VINDPM_MASK 0x87 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetIINDPM Function
	#define IINLIM_100MA 0		//000
	#define IINLIM_150MA 1		//001
	#define IINLIM_500MA 2		//010
	#define IINLIM_900MA 3		//011
	#define IINLIM_1200MA 4		//100
	#define IINLIM_1500MA 5		//101
	#define IINLIM_2000MA 6		//110
	#define IINLIM_3000MA 7		//111
	#define IINDPM_LSHIFT 0	
	#define IINDPM_MASK 0xF8 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetCHGCONFIG Function
		//#define DISABLE 0
	#define CHARGE_BATTERY 1
	#define OTG 2
		//#define OTG 3
	#define CHGCONFIG_LSHIFT 4//5
	#define CHGCONFIG_MASK 0xCF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetSYSMIN Function
	#define SYSMIN_MIN 3000		//value in mV
	#define SYSMIN_MAX 3700		//value in mV
	#define SYSMIN_STEP 100		//value in mV
	#define SYSMIN_OFFSET 3000	//value in mV
	#define SYSMIN_LSHIFT 1
	#define SYSMIN_MASK 0xF1 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetOTGILIM Function
	#define BOOSTLIM_500mA 0
	#define BOOSTLIM_1300mA 1
	#define BOOSTLIM_LSHIFT 0
	#define BOOSTLIM_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetFASTCHRG Function
	#define ICHG_MIN 500		//value in mA
	#define ICHG_MAX 4532		//value in mA
	#define ICHG_STEP 64		//value in mA
	#define ICHG_LSHIFT 2
	#define ICHG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetICHRG20PCNT Function
	#define ICHRGFORCE_NOM 0		//*default value
	#define ICHRGFORCE_20PCNT 1
	#define ICHRGFORCE_LSHIFT 0
	#define ICHRGFORCE_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetPRECHRG Function
	#define PRECHG_MIN 128		//value in mA
	#define PRECHG_MAX 2048		//value in mA
	#define PRECHG_STEP 128		//value in mA
	#define PRECHG_LSHIFT 4
	#define PRECHG_MASK 0x0F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetTERMCHRG Function
	#define ITERM_MIN 128		//value in mA
	#define ITERM_MAX 2048		//value in mA
	#define ITERM_STEP 128		//value in mA
	#define ITERM_LSHIFT 0
	#define ITERM_MASK 0xF0 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetChgVoltage Function
	#define VREG_MIN 3504		//value in mV
	#define VREG_MAX 4512		//value in mV
	#define VREG_STEP 16		//value in mV
	#define VREG_LSHIFT 2
	#define VREG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetBATLOWV Function
	#define BATLOWV_2800mV 0
	#define BATLOWV_3000mV 1		//*default value
	#define BATLOWV_LSHIFT 1
	#define BATLOWV_MASK 0xFD //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetRECHRG Function
	#define VRECHG_100mV 0		//*default value
	#define VRECHG_300mV 1
	#define VRECHG_LSHIFT 0
	#define VRECHG_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetWatchDog Function
		//#define DISABLE 0
	#define WatchDog_40s 1		//*default value
	#define WatchDog_80s 2
	#define WatchDog_160s 3
	#define WatchDog_LSHIFT 4
	#define WatchDog_MASK 0xCF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetFastChgTimer Function
	#define CHGTIMER_5h  0
	#define CHGTIMER_8h  1		//*default value
	#define CHGTIMER_12h 2
	#define CHGTIMER_20h 3
	#define CHGTIMER_LSHIFT 1
	#define CHGTIMER_MASK 0xF9 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetJeitaISET Function
	#define JEITAISET_50PCNT  0		//*default value
	#define JEITAISET_20PCNT  1
	#define JEITAISET_LSHIFT 0
	#define JEITAISET_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetBATCOMP Function
	#define BATCOMP_MIN  0		//value in mOhm
	#define BATCOMP_MAX	 70		//value in mOhm
	#define BATCOMP_STEP 10		//value in mOhm
	#define BATCOMP_LSHIFT 5
	#define BATCOMP_MASK 0x1F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetVCLAMP Function
	#define VCLAMP_MIN  0		//value in mV
	#define VCLAMP_MAX	112		//value in mV
	#define VCLAMP_STEP 16		//value in mV
	#define VCLAMP_LSHIFT 2
	#define VCLAMP_MASK 0xE3 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqSetTREG Function
	#define TREG_60C 0
	#define TREG_80C 1
	#define TREG_100C 2
	#define TREG_120C 3		//*default value
	#define TREG_LSHIFT 0
	#define TREG_MASK 0xFC //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on SetJeitaVSET Function
	#define JEITAVSET_4050 0		//*default value
	#define JEITAVSET_4200 1
	#define JEITAVSET_LSHIFT 4
	#define JEITAVSET_MASK 0xEF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnHIZ Function
	#define ENHIZ_LSHIFT 7
	#define ENHIZ_MASK 0x7F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqRstREG Function
	#define RESETREG_LSHIFT 7
	#define RESETREG_MASK 0x7F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqRstWatchDog Function
	#define RESETWATCHDOG_LSHIFT 6
	#define RESETWATCHDOG_MASK 0xBF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnTERM Function
	#define ENTERM_LSHIFT 7
	#define ENTERM_MASK 0x7F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqTERMSTAT Function
	#define TERMSTAT_ITERM 0		//*default value
	#define TERMSTAT_EARLY 1
	#define TERMSTAT_LSHIFT 6
	#define TERMSTAT_MASK 0xBF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnTIMER Function
	#define ENTIMER_LSHIFT 3
	#define ENTIMER_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnDPDM Function
	#define ENDPDM_LSHIFT 7
	#define ENDPDM_MASK 0x7F //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnTMR2X Function
	#define EN2XTIMER_LSHIFT 6
	#define EN2XTIMER_MASK 0xBF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqOffBATFET Function
	#define OFFBATFET_LSHIFT 5
	#define OFFBATFET_MASK 0xDF //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnINTCHRGFAULT Function
	#define INTCHRGFAULT_LSHIFT 1
	#define INTCHRGFAULT_MASK 0xFD //Bits Set to 1 on mask will remain unchanged
	
	// Variables used on bqEnINTBATFAULT Function
	#define INTBATFAULT_LSHIFT 0
	#define INTBATFAULT_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
	
	


#endif

