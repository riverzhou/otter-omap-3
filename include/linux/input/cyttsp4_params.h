#ifndef _CYTTSP4_PARAMS_H_
#define _CYTTSP4_PARAMS_H_

#if defined (CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_TATE)
#define CY_MAXX 800//1368 //1296 //480
#define CY_MAXY 1280//782 //828 //256
#else
#define CY_MAXX 1296 //1944//1368 //1296 //480
#define CY_MAXY 805 //1219//782 //828 //256
#endif

/* Touchscreen Parameters */
static const uint8_t cyttsp4_param_regs[] = {
/*	H, L		  Enum	Name				*/
	0x00,  /* 00	DEBUGDATA0 */
	0x00,  /* 01	DEBUGDATA1 */
	0x15,  /* 02	NUM_ROWS */
	0x22,  /* 03	NUM_COLS */
	0x05, 0x0C,  /* 04	TOUCH_RES_X */
	0x03, 0x33,  /* 05	TOUCH_RES_Y */
	0x3B, 0xD0,  /* 06	SCREEN_SIZE_X */
	0x25, 0xBC,  /* 07	SCREEN_SIZE_Y */
	0x00, 0x06,  /* 08	CENTROID_THRESHOLD */
	0x00, 0x11,  /* 09	THRESHOLD_NOISE */
	0x00, 0x3C,  /* 0A	THRESHOLD_NEG_NOISE */
	0x00,  /* 0B	Reserved */
	0x00, 0x00,  /* 0C	Reserved */
	0x00, 0x12,  /* 0D	THRESHOLD_FINGER */
	0x01, 0x2C,  /* 0E	VEL_FINGER_X */
	0x01, 0x04,  /* 0F	VEL_FINGER_Y */
	0x01,  /* 10	FINGER_DEBOUNCE */
	0x00, 0x08,  /* 11	NUM_TX_CLOCKS */
	0x00,  /* 12	IDAC_RANGE */
	0x7B,  /* 13	IDAC_VALUE */
	0x73,  /* 14	TX_DIV_VALUE_0 */
	0x7A,  /* 15	TX_DIV_VALUE_1 */
	0x82,  /* 16	TX_DIV_VALUE_2 */
	0x00, 0x00,  /* 17	Reserved */
	0x1E,  /* 18	LARGE_OBJ_THRES */
	0x0E,  /* 19	FAT_FINGER_THRES */
	0x00,  /* 1A	Reserved */
	0x00,  /* 1B	Reserved */
	0x64,  /* 1C	LOW_POWER_INTV */
	0x0C,  /* 1D	REF_INTV */
	0x07, 0xD0,  /* 1E	ACTIVE_MODE_TIMEOUT */
	0x01,  /* 1F	WAKE_UP_SRC */
	0x00, 0x64,  /* 20	HOST_ALERT_PULSE_WIDTH */
	0x00,  /* 21	Reserved */
	0x02,  /* 22	XYFILTERMOVETHOLD */
	0x14,  /* 23	XYFILTERFASTTHOLD */
	0x08,  /* 24	XYFILTERSLOWCOEFF */
	0x10,  /* 25	XYFILTERFASTCOEFF */
	0x0A,  /* 26	FATFINGERLIMIT */
	0x0F,  /* 27	IDACHOLDOFF */
	0x00,  /* 28	FreqHoppingOperationalMode */
	0x00, 0x78,  /* 29	FreqHoppingNoiseThreshold */
	0x00, 0x00,  /* 2A	FreqHoppingBaseLevelDegradation */
	0x00, 0x00,  /* 2B	FreqHoppingRefreshCounterThreshold */
	0x00,  /* 2C	FreqHoppingRegretThreshold */
	0x00,  /* 2D	FreqHoppingTimeIncreaseMultiplier */
	0x03, 0xE8,  /* 2E	BaselineUpdateRate */
	0x01,  /* 2F	AdvancedFingerSeparation */
	0x10,  /* 30	InnerEdgeGain */
	0x00,  /* 31	OuterEdgeGain */
};

/* Touchscreen Parameters Field Sizes (Writable: 0:Readonly; 1:Writable */
static const uint8_t cyttsp4_param_size[] = {
/*   Size	  Enum	Name				*/
	1, /* 00	DEBUGDATA0 */
	1, /* 01	DEBUGDATA1 */
	1, /* 02	NUM_ROWS */
	1, /* 03	NUM_COLS */
	2, /* 04	TOUCH_RES_X */
	2, /* 05	TOUCH_RES_Y */
	2, /* 06	SCREEN_SIZE_X */
	2, /* 07	SCREEN_SIZE_Y */
	2, /* 08	CENTROID_THRESHOLD */
	2, /* 09	THRESHOLD_NOISE */
	2, /* 0A	THRESHOLD_NEG_NOISE */
	1, /* 0B	Reserved */
	2, /* 0C	Reserved */
	2, /* 0D	THRESHOLD_FINGER */
	2, /* 0E	VEL_FINGER_X */
	2, /* 0F	VEL_FINGER_Y */
	1, /* 10	FINGER_DEBOUNCE */
	2, /* 11	NUM_TX_CLOCKS */
	1, /* 12	IDAC_RANGE */
	1, /* 13	IDAC_VALUE */
	1, /* 14	TX_DIV_VALUE_0 */
	1, /* 15	TX_DIV_VALUE_1 */
	1, /* 16	TX_DIV_VALUE_2 */
	2, /* 17	Reserved */
	1, /* 18	LARGE_OBJ_THRES */
	1, /* 19	FAT_FINGER_THRES */
	1, /* 1A	Reserved */
	1, /* 1B	Reserved */
	1, /* 1C	LOW_POWER_INTV */
	1, /* 1D	REF_INTV */
	2, /* 1E	ACTIVE_MODE_TIMEOUT */
	1, /* 1F	WAKE_UP_SRC */
	2, /* 20	HOST_ALERT_PULSE_WIDTH */
	1, /* 21	Reserved */
	1, /* 22	XYFILTERMOVETHOLD */
	1, /* 23	XYFILTERFASTTHOLD */
	1, /* 24	XYFILTERSLOWCOEFF */
	1, /* 25	XYFILTERFASTCOEFF */
	1, /* 26	FATFINGERLIMIT */
	1, /* 27	IDACHOLDOFF */
	1, /* 28	FreqHoppingOperationalMode */
	2, /* 29	FreqHoppingNoiseThreshold */
	2, /* 2A	FreqHoppingBaseLevelDegradation */
	2, /* 2B	FreqHoppingRefreshCounterThreshold */
	1, /* 2C	FreqHoppingRegretThreshold */
	1, /* 2D	FreqHoppingTimeIncreaseMultiplier */
	2, /* 2E	BaselineUpdateRate */
	1, /* 2F	AdvancedFingerSeparation */
	1, /* 30	InnerEdgeGain */
	1, /* 31	OuterEdgeGain */
};

#endif 
