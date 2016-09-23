/*
 ## VIS UVC control handler
 ## ===========================
*/

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>

#include "uvc.h"
#include "sensor.h"
#include "cmdqu.h"
#include "CX3RDKOV5640.h"

#define  NOQU    //for queue debug
#define  SPEED   1// for optical zoom speed setting: 1 no support
#define  VD_FX_I2C_CMD_EVENT   (1<<6)					/* for SenAppThread. the command timer event. */

//volatile static CyBool_t hitFV = CyFalse;               /* Whether end of frame (FV) signal has been hit. */
//volatile static CyBool_t gpif_initialized = CyFalse;    /* Whether the GPIF init function has been called. */
//volatile static uint16_t prodCount = 0, consCount = 0;  /* Count of buffers received and committed during */
                                                          /* the current video frame. */
//volatile static CyBool_t stiflag = CyFalse;             /* Whether the image is still image */
//volatile static uint16_t stillcont = 0;
volatile static CyBool_t is60Hz = CyFalse;				/* Flag for frequency */
volatile static uint8_t ROIMode = 0x01;				/* for 720p has 0x04 (ROI) 0x05 and 0x06; the other Res. has 0x04 only but is not ROI.*/
//#define isWBMamu   0  // Is white balance control manual mode.

#define BLCIndex  0 // the back light compensation index
#define CamModeIndex 28 // the index of camera mode
/* the processing unit control request */
//		{ 2,    0,    0,    3,    0, 1, 0, 3, 0,   3, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},

/* the UVC control requests handler */
volatile static SensorCtrl PUCBLC =
		{BLCModeRegAct,		//Reg1: the command register address1
		 BLCModeRegGain,	//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 3,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 3,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 3,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //
//{/*1*/0x15/*BrightnessReg1*/      , 0x15/*BrightnessReg0*/       , 2,    0,    0,  255,    0, 1, 0, 3, 0, 118, 0, 118, 199, I2C_EAGLESDP_ADDR/*I2C_DevAdd_C6*/,      CyTrue,  CyTrue, 0},
//volatile static SensorCtrl PUCBright;
volatile static SensorCtrl PUCBright =
		{0x04/*BrightnessReg1*/,		//Reg1: the command register address1 for Invendo reference level.
		 0x0C/*BrightnessReg0*/,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 255,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 118,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 118,				//UVCCurVLo: the command current data value low byte
		 119,				//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //brightness, Reg1: ; Reg2: .
//		{/*2*/0x04/*ContrastReg*/         , 0x04/*ContrastReg*/          , 2,    0,    0,  255,    0, 1, 0, 3, 0, 112, 0, 112,   0, I2C_EAGLESDP_ADDR/*I2C_DevAdd_C6*/,      CyTrue,  CyTrue, 0},
//volatile static SensorCtrl PUCContrast;
volatile static SensorCtrl PUCContrast =
		{ContrastReg,		//Reg1: the command register address1
		 ContrastReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 255,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 112,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 112,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //contrast, Reg1: ; Reg2: .
//volatile static SensorCtrl PUCGain;
//		{/*4*/MainsFreqReg        , MainsFreqReg         , 2,    0,    0,    1,    0, 1, 0, 3, 0,   1, 0,   1,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // frequency 0=50Hz(PLA); 1=60Hz(NTSC).
volatile static SensorCtrl PUCPLFreq =  //in 5MP b/w it is not used.
		{MainsFreqReg,		//Reg1: the command register address1
		 MainsFreqReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 1,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 1,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 1,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //main frequency, Reg1: ; Reg2: .
//volatile static SensorCtrl PUCHueC;
//{/*5*/HuectrlRegGr        , HuectrlRegBlu        , 2,    0,    0,  255,    0, 1, 0, 3, 0, 128, 0,   0,   0, I2C_DevAdd_C6,      CyTrue,  CyTrue, 0},  //Hue control
volatile static SensorCtrl PUCHueC =
		{HuectrlRegGr,		//Reg1: the command register address1
		 HuectrlRegBlu,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 255,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 128,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //hue control, Reg1: ; Reg2: .
//		{/*6*/SaturationRegR      , SaturationRegB       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  50, 0,  50,   0, I2C_DevAdd_F2,      CyTrue,  CyTrue, 0},  //Saturation control
//volatile static SensorCtrl PUCSaturation;
volatile static SensorCtrl PUCSaturation =
		{SaturationRegR,		//Reg1: the command register address1
		 SaturationRegB,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 100,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 50,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 50,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //saturation, Reg1: ; Reg2: .
volatile static SensorCtrl PUCSharp =
		{SharpnessEnReg,		//Reg1: the command register address1
		 SharpnessGaiReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xFF,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //sharpness, Reg1: 0-disable & 1-enable; Reg2: 0x00~0xFF.
//		{/*9*/WBModeReg           , WBModeReg            , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //white balance control
//volatile static SensorCtrl PUCWBMd;
volatile static SensorCtrl PUCWBMd = //not be used in 5MP B/W
		{WBModeReg,		//Reg1: the command register address1
		 WBModeReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 5,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //write balance mode, Reg1: ; Reg2: .
//		{/*B*/ManuBWBReg          , ManuRWBReg           , 4,    0,    0,   64,    0, 1, 0, 3, 0,  32,56,  32,  56, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //white balance component: Red, Blue. Only manual mode
//volatile static SensorCtrl PUCWBLC; //?
volatile static SensorCtrl PUCWBLC =  //not be used in 5MP B/W
		{ManuBWBReg,		//Reg1: the command register address1
		 ManuRWBReg,		//Reg2: the command register address2
		 4,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 64,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 32,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 56,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //write balance components, Reg1: ; Reg2: .
//		{/*E*/DigZoomReg          , DigZoomReg           , 2,    0,    0,   27,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
//volatile static SensorCtrl PUCDZoom;
volatile static SensorCtrl PUCDZoom =
		{DigZoomReg,		//Reg1: the command register address1
		 DigZoomReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 27,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //digital zoom, Reg1: ; Reg2: .

volatile static SensorCtrl *pPUCSenCtrl[0x10] = {
	&PUCBLC,
	&PUCBright,
	&PUCContrast,
	0, //&PUCGain (AGC?)
	&PUCPLFreq,
	&PUCHueC,
	&PUCSaturation,
	&PUCSharp,
	0, //&PUCGamGain,
	&PUCWBMd,
	0, //UVCCtlID10,
	&PUCWBLC,
	0, //UVCCtlID12,
	0, //UVCCtlID13,
	&PUCDZoom,
	0 //UVCCtlID15
};

/* the Camera terminal control request */
//volatile static SensorCtrl CTCAutoExMode;
//volatile static SensorCtrl CTCExposureTAbs;
//volatile static SensorCtrl CTCFocusRel;
//volatile static SensorCtrl CTCIrisAbs;
//volatile static SensorCtrl CTCOPZoomAbs;

/* the Extentsion control request */
volatile static SensorCtrl EXTShutter =
		{ShutterReg,		//Reg1: the command register address1
		 ExFinShutReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 8,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //shutter control 0 ~ 0x12
volatile static SensorCtrl EXTSensUp =  //not be used in 5MP B/W
		{SenseUpReg,		//Reg1: the command register address1
		 SenseUpReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 9,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 1,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// sense up control 0 ~ 0x09

// 		{/*12*/MirrModeReg         , MirrModeReg          , 2,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // mirror mode control 0 ~ 0x03
volatile static SensorCtrl EXTMirror =
		{MirrModeReg,		//Reg1: the command register address1
		 MirrModeReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 3,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// mirror mode control 0 ~ 0x03
//		{/*13*/NoiRedu3DModReg     , NoiRedu3DModReg      , 2,    0,    0,    1,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // 3D noise reduce mode(data0)/level(data1). 0:off 1:on. 0 ~ 0x64
//volatile static SensorCtrl EXT3DnoiseReduceMode;
volatile static SensorCtrl EXT3DnoiseReduceMode = //2DNR mode
		{NoiRedu3DModReg,		//Reg1: the command register address1
		 NoiRedu3DModReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 1,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// 3D noise reduce mode control
//		{/*14*/NoiRedu3DLevReg     , NoiRedu3DLevReg      , 1,    0,    0,   64,    0, 1, 0, 3, 0,  32, 0,  32,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
//volatile static SensorCtrl EXT3DNoiseLev;
volatile static SensorCtrl EXT3DNoiseLev =  //2DNR gain
		{NoiRedu3DLevReg,		//Reg1: the command register address1
		 NoiRedu3DLevReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 64,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 32,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 32,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// 3D noise reduce level control

//		{/*15*/DayNightModReg      , DayNightModReg       , 2,    0,    0,    2,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // Day night mode. 0:auto 1:day mode 2:night mode
//volatile static SensorCtrl EXTDayNightMode;
volatile static SensorCtrl EXTDayNightMode =
		{DayNightModReg,		//Reg1: the command register address1
		 DayNightModReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 2,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// day night mode control
//{/*16*/DayNightDlyReg      , DayNightDlyReg       , 2,    0,    0,   63,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // day night switch delay control. 0 ~ 0x3f second
//volatile static SensorCtrl EXTDayNightdely;
volatile static SensorCtrl EXTDayNightdely =
		{DayNightDlyReg,		//Reg1: the command register address1
		 DayNightDlyReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 63,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// day night delay control
//{/*17*/DayNightLevReg      , DayNightLevReg       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  16, 0,  16,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // day to night start level. 0 ~ 0x64
//volatile static SensorCtrl EXTDayNightlev;
volatile static SensorCtrl EXTDayNightlev =
		{DayNightLevReg,		//Reg1: the command register address1
		 DayNightLevReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 100,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 16,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 16,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// day night level control
//{/*18*/NightDayLevReg      , NightDayLevReg       , 2,    0,    0,  100,    0, 1, 0, 3, 0,  16, 0,  16,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // night to day start level. 0 ~ 0x64
//volatile static SensorCtrl EXTNightDaylev;
volatile static SensorCtrl EXTNightDaylev =
		{NightDayLevReg,		//Reg1: the command register address1
		 NightDayLevReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 100,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 16,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 16,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// night day level control
//{/*19*/AExModeReg          , AExAGCReg            , 4,    0,    0,  127,    0, 1, 0, 3, 0,   0,32,   0,  32, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE mode setting & AGC level: 0:auto 1~18:manual; 0 ~ 0xff:level. read(auto), write(menu).
volatile static SensorCtrl EXTAexModGainlev =
		{AExModeReg,		//Reg1: the command register address1
		 AExAGCReg,			//Reg2: the command register address2
		 4,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 127,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 63, 				//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 63,				//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //AE mode setting & AGC level: 0:auto 1:AGC only 2:auto Shutter only 3:menual
		   //Gain level: 0 ~ 0xff
//{/*1A*/AExReferleveReg     , AExReferleveReg      , 2,    0,    0,  255,    0, 1, 0, 3, 0,  0x60, 0,  0x60,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  // AE reference level 0 ~ 0x40
//volatile static SensorCtrl EXTExpReflev;
volatile static SensorCtrl EXTExpReflev =
		{AExReferleveReg0,		//Reg1: the command register address1
		 AExReferleveReg1,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 255,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 60,				//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 60,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// AEX reference level control
//{/*1C*/SensorModeReg       , SensorModeReg        , 2,    0,    0,    6,    0, 1, 0, 3, 0,   3, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
//volatile static SensorCtrl EXTCamMode;
volatile static SensorCtrl EXTCamMode =  //not be used
		{SensorModeReg,		//Reg1: the command register address1
		 SensorModeReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 6,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 3,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 3,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// sensor mode control ???
//volatile static SensorCtrl EXTSnapshot; ???
//{/*1E*/SeveParsReg         , SeveParsReg          , 1,    0,    0,    3,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0}, //
//volatile static SensorCtrl EXTSensorPare;
volatile static SensorCtrl EXTSensorPare = //not be used
		{SeveParsReg,		//Reg1: the command register address1
		 SeveParsReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 3,					//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// sensor mode control ???
//{/*1F*/0/*I2CCtrl*/        , 0                    ,11,    0,    0,  0xff, 0xff, 1, 0, 3, 0,   0, 0,   0,   0,                0,  CyTrue, CyFalse, 0}  // index is 0x1f
//volatile static SensorCtrl EXTI2Ccmd;???
volatile static SensorCtrl EXTI2Ccmd = //not be used
		{0,/*no-fix register*/		//Reg1: the command register address1
		 0,/*no-fix register*/		//Reg2: the command register address2
		 11,				//UVCLn: the command length
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0xff,				//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0,					//UVCDefVHi: the command default data value high byte
		 0,					//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 0,/*no-fix address*/ 		//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse,		 	//AvailableF: the command available flag
		};	// sensor mode control ???
volatile static SensorCtrl EXTBLCWinPos =
		{BLCPosReg,			//Reg1: the command register address1
		 BLCSizeReg,		//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0xff,				//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x66,				//UVCDefVLo: the command default data value low byte
		 0x66, 				//UVCDefVHi: the command default data value high byte
		 0x66,				//UVCCurVLo: the command current data value low byte
		 0x66,				//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //BLC position: [7:4]:Top/bottom [3:0]:left/right; BLC size: [7:4]:height [3:0]:width.
//{/*25*/0x11/*Ext1BLCWeightCtlID5*/         , 0   , 2,    1,    0,    3,    0, 1, 0, 3, 0,   1, 0,   1,   0, I2C_EAGLESDP_ADDR,     CyTrue,  CyTrue, 0},
volatile static SensorCtrl EXTBLCWeight =
		{BLCModeRegGain,			//Reg1: the command register address1
		 BLCModeRegGain,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //
//{/*26*/BLCModeRegAct/*Ext1BLCGridCtlID6*/           , 0   , 1,    1,    0,    2,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,     CyTrue,  CyTrue, 0},
//volatile static SensorCtrl EXTBLCGrid;
volatile static SensorCtrl EXTBLCGrid =
		{BLCModeRegAct,			//Reg1: the command register address1
		 BLCModeRegAct,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 1,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 2,		  			//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0,					//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0, 				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTShutlev =
		{0x12/*ShutterFineReg*/,		//Reg1: the command register address1
		 0x12/*ShutterFineReg*/,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 255,				//UVCMaxLo: the command maximum value low byte
		 127,				//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 63,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 63,					//UVCCurVLo: the command current data value low byte
		 0,				//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //AE mode setting & AGC level: 0:auto 1:AGC only 2:auto Shutter only 3:menual
		   //Gain level: 0 ~ 0xff

volatile static SensorCtrl EXTExHyster =
		{ExHysterReg,			//Reg1: the command register address1
		 ExHysterReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTExCtrlSped =
		{ExCtrlSpdReg,			//Reg1: the command register address1
		 ExCtrlSpdReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTEnhanceMode =
		{ExEnhanceModeReg,			//Reg1: the command register address1
		 ExEnhanceModeReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTEnhanceGain =
		{ExEnhanceGainReg,			//Reg1: the command register address1
		 ExEnhanceGainReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTEnhanceSTED =
		{ExEnhanceStartReg,			//Reg1: the command register address1
		 ExEnhanceEndReg,			//Reg2: the command register address2
		 4,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXT2DNRGain =
		{Ex2DNREnableReg,		//Reg1: the command register address1
		 Ex2DNRGainReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXT2DNRSTED =
		{Ex2DNRStartReg,			//Reg1: the command register address1
		 Ex2DNREndReg,			//Reg2: the command register address2
		 4,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0xff,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x80,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x80,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTGammaCor =
		{ExGammaReg,			//Reg1: the command register address1
		 ExGammaReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0x10,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x00,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x00,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl EXTAGCMaxLimit =
		{AGCMaxLimtReg,			//Reg1: the command register address1
		 AGCMaxLimtReg,			//Reg2: the command register address2
		 2,					//UVCLn: the command length: 1th for mode and 2nd for gain level
		 0,					//UVCMinLo: the command minimum value low byte
		 0,					//UVCMinHi: the command minimum value high byte
		 0x10,				//UVCMaxLo: the command maximum value low byte
		 0,					//UVCMaxHi: the command maximum value high byte
		 1,					//UVCResLo: the command Res. value low byte
		 0,					//UVCResHi: the command Res. value high byte
		 3,					//UVCInfoLo: the command information value low byte
		 0,					//UVCInfoHi: the command information value high byte
		 0x00,				//UVCDefVLo: the command default data value low byte
		 0, 				//UVCDefVHi: the command default data value high byte
		 0x00,				//UVCCurVLo: the command current data value low byte
		 0,					//UVCCurVHi: the command current data value high byte
		 I2C_EAGLESDP_ADDR,	//DeviceAdd: the device address
		 CyTrue,			//CheckF: the command checked flag
		 CyFalse			//AvailableF: the command available flag
		}; //

volatile static SensorCtrl *pEXTSenCtrl[0x40] = {//Extension control
		&EXTShutter,
		&EXTSensUp,
		&EXTMirror,
		&EXT3DnoiseReduceMode,
		&EXT3DNoiseLev,
		&EXTDayNightMode,
		&EXTDayNightdely,
		&EXTDayNightlev,
		&EXTNightDaylev,
		&EXTAexModGainlev,
		&EXTExpReflev,
		&EXTShutlev,
		&EXTCamMode,
		0, //&EXTSnapshot,
		&EXTSensorPare,
		&EXTI2Ccmd,
		0, //&Ext1CtlID0 = 0x20,
		0, //&Ext1CtlID1,
		0, //&Ext1CtlID2,
		0, //&Ext1CtlID3,
		&EXTBLCWinPos,   		// back light compensation range
		&EXTBLCWeight,  	    // back light compensation weight (gain) factor
		&EXTBLCGrid,    	// back light compensation grid state
		&EXTExHyster,       // exposure hystereses level
		&EXTExCtrlSped,     // exposure control speed level
		&EXTEnhanceMode,    // edge enhancement mode
		&EXTEnhanceGain,    // edge enhancement gain level
		&EXTEnhanceSTED,    // edge enhancement start/end level
		&EXT2DNRGain,       // 2D noise reduction Gain level
		&EXT2DNRSTED,       // 2D noise reduction start/end level
		&EXTGammaCor,		// Gamma correction
		&EXTAGCMaxLimit, 	// AGC Maximum Gain limitation &Ext1AGCMaxLimetCtlID15,
		0
};


#ifndef CAM720
	static uint8_t CamMode = 0; //0:1080p
#else
	static uint8_t CamMode = 1; //1:720p
#endif
//	static uint8_t setRes = 0;  // 1:2592x1944; 2:1920x1080; 3:1280x720; 0:n/a
//	static uint8_t setstilRes = 0;  // 1:1920x1080; 2:2592x1944; 3:1280x720; 0:n/a

/*      RegAdd1,             RegAdd2,              length, Min1,  Min2, Max1, Max2, Res1, Res2, InfoReq1, InfoReq2, DefReq1, DefReq2,
 *            curVal1, curVal2 (index:14th), device address, checked flag, command available flag*/
#if 0 //CT controls handler
static uint8_t CTCtrlParArry[16][24]={
		{ScanMCtlID0            , 0                    , 1,    0,    0,    3,    0, 1, 0, 3, 0,   3, 0,   3,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{ShutterReg             , ShutterReg           , 1,    1,    0,   15,    0,15, 0, 3, 0,   2, 0,   2,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},
		{AutoExPCtlID2          , 0                    , 1,    0,    0,    1,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},
		{ShutterReg             , ShutterReg           , 4,    0x1,    0, 0x38, 0x01, 1, 0, 3, 0,0x4e, 0,0x4e,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},   //ExTmACtlID3
		{ExTmRCtlID4            , 0                    , 1,    0,    0,    0,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{FocACtlID5             , 0                    , 2,    0,    0,  255,    0, 1, 0, 3, 0,   1, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{FocRCtlID6             , 0                    , 2,    0,    0,    0,    0, 0, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,      CyTrue,  CyTrue, 0},  //
		{IrisAFReg              , 0                    , 2,    0,    0,   48,    0, 1, 0, 3, 0x0a,0, 0, 0xa,   0, I2C_EAGLESDP_ADDR,  CyTrue,  CyTrue, 0},  //IriACtlID7
		{IriRCtlID8             , 0                    , 1,    0,    0,  127,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{ZmOpACtlID9            , 0                    , 2,    0,    0,    5,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{OpZoomReg              , 0                    , 3,    0,    0,    0,    0, 0, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,   64,    0, 1, 0, 3, 0,  15, 17,  0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},  //
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0},
		{0                   , 0                    , 2,    0,    0,  100,    0, 1, 0, 3, 0,   0, 0,   0,   0, I2C_EAGLESDP_ADDR,  CyTrue, CyFalse, 0}  // end of the UVC CT
};
#endif

//static uint16_t ShutValueArry[8]={200, 100, 39, 20, 10, 5, 2, 1};
//static uint8_t ExTime[8][2]={{0x9c, 0x00}, {0x4e, 0x00}, {0x27, 0x00}, {0x14, 0x00}, {0x0a, 0x00}, {0x05, 0x00}, {0x02, 0x00}, {0x01, 0x00}};
static uint16_t ShutSp[16]={33333, 16667, 8333, 4000, 2000, 1000, 500, 200, 100, 10, 0}; // in microsecond.
static uint8_t curFlag[64]={0}; //the curFlag for each controls current records available. 0: unable. the data should be read from sensor and put into the records. 1: available. the data is read from records.
/*
 * WBMenuCmpArry is set for white storing balance component requests values.
 * first two bytes represent blue and last two are for red. The defaults are set to 0.
 */
static uint8_t WBMenuCmpArry[4]={
		0x20, 0x0f, 0x38, 0xf0
};
static uint8_t I2CCMDArry[12]={//the index 12 points to data available; 0: no used; 0xf: unavailable; 0xff: available.
		0
};

//static uint32_t  isFlag = 0x0; /*set current value flag*/

void I2CCmdHandler(){
	uint8_t buf[2];
	uint8_t CmdType, CmdRegLen, CmdDataLen;
	CmdType = I2CCMDArry[0];
	CmdRegLen = I2CCMDArry[1];
	CmdDataLen = I2CCMDArry[8];
	VdRingBuf *cmdQuptr = &cmdQu;
	uint8_t i;
	CyU3PDebugPrint (4, "The I2C command is 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
			I2CCMDArry[0], I2CCMDArry[1], I2CCMDArry[2], I2CCMDArry[3], I2CCMDArry[4], I2CCMDArry[5],
			I2CCMDArry[6], I2CCMDArry[7], I2CCMDArry[8], I2CCMDArry[9], I2CCMDArry[10]);
	if((I2CCMDArry[3]==0x52) && (I2CCMDArry[4]==0x30) && (I2CCMDArry[5]==0x01))
	{
		ROIMode = I2CCMDArry[9]&0x03; //set ROI mode based on the I2C data.
		if(is60Hz==CyFalse)
			{
				I2CCMDArry[9]=0x80|I2CCMDArry[9];
			}
			CyU3PDebugPrint (4, "The I2C command setting value %x %x\r\n", I2CCMDArry[9], ROIMode);

	}
	if(CmdType == 0)//I2C read
	{
		I2CCMDArry[11] = 0xf; //setting I2C data is not available.
#if 0 //for debugging
		/* test still image operation */
		if(I2CCMDArry[2] == 0xff){
			snapButFlag = 0; //press
			//CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR); //set sending interrupt status event for snap button press
		}else if(I2CCMDArry[2] == 0x0){
			snapButFlag = 0xf; //release
			//CyU3PEventSet (&glFxUVCEvent, VD_FX_INT_STA_EVENT, CYU3P_EVENT_OR); //set sending interrupt status event for snap button release
		}

		/* end of the test */
#endif
		if(1||(CmdRegLen == 4)){
				SensorRead2B(I2CCMDArry[2]|I2C_RD_MASK, I2CCMDArry[3]|I2C_RD_MASK, I2CCMDArry[4], I2CCMDArry[5], I2CCMDArry[8], buf);
				I2CCMDArry[9] = buf[0];
				if(CmdDataLen == 2){
					I2CCMDArry[10] = buf[1];
				}
			I2CCMDArry[11] = 0xff; //setting I2C data is available.
		}else{//not support currently
			CyU3PDebugPrint (4, "The I2C command length is not supported. value %d\r\n", CmdRegLen);
		}
	}else if(CmdType == 1){
		if(1||(CmdRegLen == 4)){//TODO cmdque mutual
			buf[0] = I2CCMDArry[9];
			buf[1] = I2CCMDArry[10];
			if(0 && (I2CCMDArry[3]&I2C_WR_MASK)==0x82 && (I2CCMDArry[4]==0x30) && (I2CCMDArry[5]==0x10)){
				CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);
				cmdSet(cmdQuptr, 0x23, 0x10, 0x30, STOP, 0);
				CyU3PMutexPut(cmdQuptr->ringMux);
			}
			else SensorWrite2B(I2CCMDArry[2]&I2C_WR_MASK, I2CCMDArry[3]&I2C_WR_MASK, I2CCMDArry[4], I2CCMDArry[5], I2CCMDArry[8], buf);
		}else{//not support currently
			CyU3PDebugPrint (4, "The I2C command length is not supported. value %d\r\n", CmdRegLen);
		}

	}
}

/************************************
 * set Iris mode
 * input isAuto: 0: set manual; 1: set auto
 */
inline void setIrisauto(VdRingBuf *cmdQuptr, uint8_t isAuto){
	uint8_t dataIdx = 0;
	  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
	  cmdSet(cmdQuptr, 0x20/*AFIrisMode*/, 0x27, 0x30, isAuto?0:1, dataIdx);  //set Iris Mode for AF Lens value to 0
	  cmdSet(cmdQuptr, 0x21/*noAFIrisMode*/, 0x25, 0x30, isAuto?1:2, dataIdx);  //set Iris Mode value for no-AF Lens to 0
	  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
}



inline uint8_t getShutCtrl(uint8_t Data, uint8_t* pAxMode){
	const uint16_t LnTm = 514;   // time of a line in microsecond for full Res. (2592x1944)
	uint16_t NumLn;
	uint16_t fRate, shutTm;
	uint8_t LnVal;
	switch (Data){
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		shutTm = ShutSp[Data-1];
		fRate = 30;
		NumLn = (shutTm/LnTm)*fRate;
		if(NumLn > 1944)
			NumLn =1944;
		else if(NumLn < 8)
			NumLn = 8;
		LnVal = (uint8_t)(NumLn/8);
		*pAxMode = 0x01;	// shutter menual
		CyU3PDebugPrint (4, "The shutter set value %d 0x%x 0x%x 0x%x\r\n", Data, shutTm, NumLn, LnVal); // additional debug
		break;
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
		shutTm = ShutSp[Data-1];
		fRate = 30;
		NumLn = (shutTm*fRate)/LnTm;
		if(NumLn > 1944)
			NumLn =1944;
		else if(NumLn < 8)
			NumLn = 8;
		LnVal = (uint8_t)(NumLn/8);
		*pAxMode = 0x01;	// shutter menual
		CyU3PDebugPrint (4, "The shutter set value %d 0x%x 0x%x 0x%x\r\n", Data, shutTm, NumLn, LnVal); // additional debug
		break;
	case 0: //auto
	default:
		*pAxMode = 0x00;	// auto
		LnVal = 1;
		break;
	}
	return LnVal;
}

inline void ControlHandle(uint8_t CtrlID, uint8_t  bRequest){
    CyU3PReturnStatus_t apiRetStatus = !CY_U3P_SUCCESS;
    VdRingBuf *cmdQuptr = &cmdQu;
    uint16_t readCount;
    uint8_t RegAdd0, RegAdd1, Data0, Data1, Len, idx, locCtrlID, AxMode;
    uint8_t devAdd;
    //locCtrlID = CtrlID-EXUAOFFSET+4;
    if(CtrlID >= 0x10){//the extension command over 32.
    	RegAdd0 = pEXTSenCtrl[CtrlID - 0x10]->Reg1;//ExUCtrlParArry[locCtrlID][0];
        RegAdd1 = pEXTSenCtrl[CtrlID - 0x10]->Reg2;//ExUCtrlParArry[locCtrlID][1];
        devAdd = pEXTSenCtrl[CtrlID - 0x10]->DeviceAdd;//ExUCtrlParArry[locCtrlID][15];
        Len = pEXTSenCtrl[CtrlID - 0x10]->UVCLn;//ExUCtrlParArry[locCtrlID][2];
    	//CyU3PDebugPrint (4, "The CT/EX control ctrlID: 0x%x Reg1: 0x%x Reg1: 0x%x devADD: 0x%x Len: 0x%x\r\n",
    	//		CtrlID, RegAdd0, RegAdd1, devAdd, Len);
    }else{
		RegAdd0 = pPUCSenCtrl[CtrlID]->Reg1;//CtrlParArry[CtrlID][0];
		RegAdd1 = pPUCSenCtrl[CtrlID]->Reg2;//CtrlParArry[CtrlID][1];
		devAdd = pPUCSenCtrl[CtrlID]->DeviceAdd;//CtrlParArry[CtrlID][15];
		Len = pPUCSenCtrl[CtrlID]->UVCLn;//CtrlParArry[CtrlID][2];
    	//CyU3PDebugPrint (4, "The PU control ctrlID: 0x%x Reg1: 0x%x Reg1:0x%x devADD: 0x%x Len: 0x%x\r\n",
    	//		CtrlID, RegAdd0, RegAdd1, devAdd, Len);
    }
    uint8_t dataIdx, getData=0xFF, getData1=0xff, sendData=0xff, sendData1=0xFF, reqData;
#ifdef USB_DEBUG_PRINT
    CyU3PDebugPrint (4, "The cur sensor value %d 0x%x 0x%x\r\n", CtrlID, CtrlParArry[CtrlID][13], CtrlParArry[CtrlID][14]); // additional debug
#endif
    reqData = bRequest;
    /*
     * Ext manual mode is not supported in 1080p camera
     */
    //if (0 && (CtrlID == ExtAexModCtlID9)){
    	//CyU3PDebugPrint (4, "The Aex manual mode and AGC level are not support with 1080p camera.\r\n");
    	//goto EndofSet;
    //}
    switch (bRequest)
		 {

		 case CY_FX_USB_UVC_GET_LEN_REQ: /* the length of get length request always setting to 2 */
			  glEp0Buffer[0] = Len;
			  glEp0Buffer[1] = 0;
			  CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_CUR_REQ: /* Current value. */

		 	 if(CtrlID >= 0x10/*EXUAOFFSET*/){
				 switch(CtrlID)
				 {
					 case Ext1BLCGridCtlID6:
						 if(curFlag[CtrlID]){
							 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo; //ExUCtrlParArry[CtrlID-0x20][13];//ext_control array;
							 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi; //ExUCtrlParArry[CtrlID-0x20][14];
						 }else{
							Data0 = SensorGetControl(RegAdd0, devAdd);
							if(Data0&0x80)
								glEp0Buffer[0] = 1;
							else
								glEp0Buffer[0] = 0;
							pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];
							glEp0Buffer[1] = 0; //pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi;ExUCtrlParArry[CtrlID-0x20][14];
							//curFlag[CtrlID] = CyTrue;
						 }
						 sendData = glEp0Buffer[0];
						 sendData1 = glEp0Buffer[1];
						 break;
			 	 	 case ExtAexModCtlID9:
			 	 		 if(curFlag[CtrlID]){
							 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo;//ext_control array;
							 glEp0Buffer[2] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi;
			 	 		 }else{
			 	 			glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
			 	 			glEp0Buffer[0] = glEp0Buffer[0]&0x3; // 1:0 for Aex Mode
			 	 			pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];

			 	 			glEp0Buffer[2] = SensorGetControl(RegAdd1, devAdd);
			 	 			pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = glEp0Buffer[2];
			 	 			//curFlag[CtrlID] = CyTrue;
			 	 		 }
						 glEp0Buffer[1] = 0;
						 glEp0Buffer[3] = 0;
						 sendData = glEp0Buffer[0];
						 sendData1 = glEp0Buffer[2];
						 CyU3PDebugPrint (4, "ExpM&AGC sent to host. %d %d; %d %d\r\n", glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3]);
						 break;
				 	 case ExtCamMCtlID12: //EXTCamMode
#if 0 //not be used
						 sendData = CtrlParArry[CtrlID][13];

						 if(CamMode == 1){//720p
							if(sendData >= 3){
								CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
								sendData = 0; //set back to default
								CtrlParArry[CtrlID][13] = 0;
							}
							sendData += 4;
						 }
						//CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
						 glEp0Buffer[0] = sendData;
						 glEp0Buffer[1] = 0;
#endif
						 break;
				 	 case ExtI2CCtlID15:
				 		 for(idx=0; idx<Len; idx++){
				 			glEp0Buffer[idx] = I2CCMDArry[idx];
				 		 }
				 		 sendData = glEp0Buffer[9];
				 		 sendData1 = glEp0Buffer[10];
	#ifdef USB_DEBUG_PRINT
				 		CyU3PDebugPrint (4, "The I2C command is 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
				 				I2CCMDArry[0], I2CCMDArry[1], I2CCMDArry[2], I2CCMDArry[3], I2CCMDArry[4], I2CCMDArry[5],
				 				I2CCMDArry[6], I2CCMDArry[7], I2CCMDArry[8], I2CCMDArry[9], I2CCMDArry[10]);
	#endif
				 		 if(I2CCMDArry[11] != 0xff)//the data availabel.
				 		 {
				 			CyU3PDebugPrint (4, "The I2C current data is not available. try again. %d %d\r\n", I2CCMDArry[9], I2CCMDArry[10]);
				 		 }
				 		 break;
			 	 	 case Ext1ExCtrlSpeedCtlID8:
			 	 		 sendData = SensorGetControl(RegAdd1, devAdd);
			 	 		// sendData &= 0x70;
	 	 				 glEp0Buffer[0] = sendData >> 4; // bit6:4 are significant bits. bit7 is fine shutter & shutter speed control
	 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];
	 	 				 glEp0Buffer[1] = 0;
	 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = glEp0Buffer[1];
	 	 				 sendData = glEp0Buffer[0];
	 	 				 sendData1 = glEp0Buffer[1];
	 	 				 break;
				 	 case Ext1BLCRangeCtlID4:
	 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
	 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];
	 	 				 glEp0Buffer[1] = SensorGetControl(RegAdd1, devAdd);
	 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = glEp0Buffer[1];
	 	 				 sendData = glEp0Buffer[0];
	 	 				 sendData1 = glEp0Buffer[1];
	 	 				 break;
			 	 	 case Ext1BLCWeightCtlID5:
				 	 case ExtShutCtlID0:
				 	 case ExtCtlShutlevCtlID11:
				 		 /* the exposure hysteresis to gamma correction */
			 	 	 case Ext1ExHysterCtlID7:
			 	 	 case Ext1EnhanceModeCtlID9:
			 	 	 case Ext1EnhanceGainCtlID10:
			 	 	 case Ext1EnhanceStarEndCtlID11:
			 	 	 case Ext12DNRGainEnblCtlID12:
			 	 	 case Ext12DNRGainStarEndCtlID13://4bytes
			 	 	 case Ext1GammaCorCtlID14:
			 	 	 case Ext1AGCMaxLimetCtlID15:
					 case ExtExRefCtlID10:
			 	 	 default:
			 	 		 if(curFlag[CtrlID]){
			 	 			 if(Len == 2)
			 	 			 {
			 	 				 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo;//ext_control array;
			 	 				 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[1];
			 	 			 }else if(Len == 4){
			 	 				 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo;//ext_control array;
			 	 				 glEp0Buffer[1] = 0;
			 	 				 glEp0Buffer[2] = pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi;
			 	 				 glEp0Buffer[3] = 0;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[2];
			 	 			 }
			 	 		 }else{
			 	 			 if(Len == 2)
			 	 			 {
			 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
			 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];
			 	 				 glEp0Buffer[1] = 0;
			 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = glEp0Buffer[1];
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[1];
			 	 			 }else if(Len == 4){
			 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd1, devAdd);
			 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = glEp0Buffer[0];
			 	 				 glEp0Buffer[2] = SensorGetControl(RegAdd0, devAdd);
			 	 				 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = glEp0Buffer[2];
			 	 				 glEp0Buffer[1] = 0;
			 	 				 glEp0Buffer[3] = 0;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[2];
			 	 			 }
			 	 			 //curFlag[CtrlID] = CyTrue;
			 	 		 }
			 	 		 break;
			 	 }
		 	 }
		 	 else{
				 switch(CtrlID)
				 {
					 case BrgtCtlID1:
						 /* cancel for 5MP w/b camera
						 Data0 = CtrlParArry[CtrlID][13];  //SensorGetControl(RegAdd0, devAdd); //SensorGetBLCMode();
						 Data1 = CtrlParArry[CtrlID][14];  //SensorGetControl(RegAdd1, devAdd);
						 if (Data1&0x2){ //check the sign bit (bit1)
							 Data1 = ((Data1<<6)&0x40)| (Data0 >> 2);//clear MSB
						 }else{
							 Data1 = ((Data1<<6)|0x80)| (Data0 >> 2);//set MSB
						 }
						 glEp0Buffer[0] = Data1;
						 glEp0Buffer[1] = 0;
						 sendData = glEp0Buffer[0];
						 */
						 if(curFlag[CtrlID]){
							Data0 = pPUCSenCtrl[CtrlID]->UVCCurVLo; // CtrlParArry[CtrlID][13];
						 }else{
							Data0 = SensorGetControl(RegAdd0, devAdd);
							pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
							//curFlag[CtrlID] = CyTrue;
						 }
						 //Data0 = CtrlParArry[CtrlID][13];  //SensorGetControl(RegAdd0, devAdd); //SensorGetBLCMode();
						 /*//new implementation 8/23/2016
						 if(Data0&0x80){
							  Data0 = ~Data0;
						  }else{
							  Data0 = Data0 + 0x80;
						  }
						  */
						 glEp0Buffer[0] = Data0;
						 glEp0Buffer[1] = 0;
						 sendData = glEp0Buffer[0];
						 break;
					 case HueCtlID5:
						 if(curFlag[CtrlID]){
							Data0 = pPUCSenCtrl[CtrlID]->UVCCurVLo; //CtrlParArry[CtrlID][13];
						 }else{
							Data0 = SensorGetControl(RegAdd0, devAdd);
							pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
							//curFlag[CtrlID] = CyTrue;
						 }

						 glEp0Buffer[0] = Data0 + GREEN_BASE;
						 glEp0Buffer[1] = 0;
						 sendData = glEp0Buffer[0];
						 break;
					 case MFreqCtlID4:

						 if(curFlag[CtrlID]){

							 if(is60Hz)
								 glEp0Buffer[0] = 0;//CtrlParArry[CtrlID][13];//ext_control array;
							 else
								 glEp0Buffer[0] = 1;

							 glEp0Buffer[1] = 0; //CtrlParArry[CtrlID][14];
						 }else{
							Data0 = SensorGetControl(0x1, devAdd); //get resolution bit7 for main frequency information
							glEp0Buffer[0] = (Data0&0x80)>>7;
							//glEp0Buffer[0]++;
							pPUCSenCtrl[CtrlID]->UVCCurVLo = glEp0Buffer[0];
							glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCCurVHi; //CtrlParArry[CtrlID][14];
							//curFlag[CtrlID] = CyTrue;
						 }

						 sendData = glEp0Buffer[0];
						 sendData1 = glEp0Buffer[1];
						 break;
					 case WBTLevCtlID11:
						 if(curFlag[CtrlID]){
							 glEp0Buffer[0] = WBMenuCmpArry[0];//using for blue part
							 glEp0Buffer[2] = WBMenuCmpArry[2];//using for red part
						 }else{
							Data0 = SensorGetControl(RegAdd0, devAdd);
							Data1 = SensorGetControl(RegAdd1, devAdd);
							glEp0Buffer[0] = Data0;
							WBMenuCmpArry[0] = glEp0Buffer[0];//using for blue part
							glEp0Buffer[2] = Data1;
							WBMenuCmpArry[2]= glEp0Buffer[2];//using for red part
							//curFlag[CtrlID] = CyTrue;
						 }
						 glEp0Buffer[1] = 0;
						 glEp0Buffer[3] = 0;
						 sendData = glEp0Buffer[0];
						 sendData1 = glEp0Buffer[2];
						 break;
					 case ShapCtlID7:
						 if(curFlag[CtrlID]){
			 	 				 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCCurVLo;//ext_control array;
			 	 				 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCCurVHi;
			 	 				 sendData = glEp0Buffer[0];
			 	 		 }else{
			 	 			 if(Len == 2)
			 	 			 {
			 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd1, devAdd);
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVLo = glEp0Buffer[0];
			 	 				 glEp0Buffer[1] = 0;
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVHi = glEp0Buffer[1];
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[1];
			 	 			 }
			 	 			 //curFlag[CtrlID] = CyTrue;
			 	 		 }
						 break;
					 case BLCCtlID0:
					 case ConsCtlID2:
					 case WBTMdCtlID9:
					 case SaturCtlID6:
					 default:
						 if(curFlag[CtrlID]){
			 	 			 if(Len == 2)
			 	 			 {
			 	 				 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCCurVLo;//ext_control array;
			 	 				 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCCurVHi;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[1];
			 	 			 }else if(Len == 4){
			 	 				 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCCurVLo;//ext_control array;
			 	 				 glEp0Buffer[1] = 0;
			 	 				 glEp0Buffer[2] = pPUCSenCtrl[CtrlID]->UVCCurVHi;
			 	 				 glEp0Buffer[3] = 0;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[2];
			 	 			 }
			 	 		 }else{
			 	 			 if(Len == 2)
			 	 			 {
			 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd0, devAdd);
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVLo = glEp0Buffer[0];
			 	 				 glEp0Buffer[1] = 0;
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVHi = glEp0Buffer[1];
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[1];
			 	 			 }else if(Len == 4){
			 	 				 glEp0Buffer[0] = SensorGetControl(RegAdd1, devAdd);
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVLo = glEp0Buffer[0];
			 	 				 glEp0Buffer[2] = SensorGetControl(RegAdd0, devAdd);
			 	 				 pPUCSenCtrl[CtrlID]->UVCCurVHi = glEp0Buffer[2];
			 	 				 glEp0Buffer[1] = 0;
			 	 				 glEp0Buffer[3] = 0;
			 	 				 sendData = glEp0Buffer[0];
			 	 				 sendData1 = glEp0Buffer[2];
			 	 			 }
			 	 			 //curFlag[CtrlID] = CyTrue;
			 	 		 }
						 break;
				 }
		 	 }

			 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);

//#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The current value 0x%x 0x%x 0x%x, 0x%x, %d\r\n", glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[3], glEp0Buffer[4], Len); // additional debug
//#endif
			  break;
		 case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum BLC = 0. */
		 	 if(CtrlID >= 0x10){//the camera terminal/extension uint command over 0x10.
		 		 if(Len == 2){
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCMinLo;//ExUCtrlParArry[locCtrlID][3];
					 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCMinHi;//ExUCtrlParArry[locCtrlID][4];
					 //CyU3PDebugPrint (4, "The CT/EX control MinLo: 0x%x, MinHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
		 		 }else //if(Len == 4)
		 		 {
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCMinLo;//ExUCtrlParArry[locCtrlID][3];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = pEXTSenCtrl[CtrlID - 0x10]->UVCMinHi;//ExUCtrlParArry[locCtrlID][4];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The CT/EX control MinLo: 0x%x, MinHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }else{
			 	 if(Len ==2){
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCMinLo;//ExUCtrlParArry[locCtrlID][3];
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCMinHi;//ExUCtrlParArry[locCtrlID][4];
					 //CyU3PDebugPrint (4, "The PU control MinLo: 0x%x, MinHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
				 }else{
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCMinLo;//ExUCtrlParArry[locCtrlID][3];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCMinHi;//ExUCtrlParArry[locCtrlID][4];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The PU control MinLo: 0x%x, MinHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }
		 	 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
		 	 if(Len == 2){
		 		 sendData = glEp0Buffer[0];
		 		 sendData1 = glEp0Buffer[1];
		 	 }else{
				  sendData = glEp0Buffer[0];
				  sendData1 = glEp0Buffer[2];
			  }
			  break;
		 case CY_FX_USB_UVC_GET_MAX_REQ:
		 	 if(CtrlID >= 0x10){//the camera terminal/extension uint command over 0x10.
		 		 if(Len == 2){
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCMaxLo;//ExUCtrlParArry[locCtrlID][5];
					 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCMaxHi;//ExUCtrlParArry[locCtrlID][6];
					 //CyU3PDebugPrint (4, "The CT/EX control MaxLo: 0x%x, MaxHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
		 		 }else //if(Len == 4)
		 		 {
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCMaxLo;//ExUCtrlParArry[locCtrlID][5];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = pEXTSenCtrl[CtrlID - 0x10]->UVCMaxHi;//ExUCtrlParArry[locCtrlID][6];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The CT/EX control MaxLo: 0x%x, MaxHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }else{
			 	 if(Len ==2){
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCMaxLo;//ExUCtrlParArry[locCtrlID][5];
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCMaxHi;//ExUCtrlParArry[locCtrlID][6];
					 //CyU3PDebugPrint (4, "The PU control MaxLo: 0x%x, MaxHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
				 }else{
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCMaxLo;//ExUCtrlParArry[locCtrlID][5];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCMaxHi;//ExUCtrlParArry[locCtrlID][6];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The PU control MaxLo: 0x%x, MaxHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }
				  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
				  if(Len == 2){
					  sendData = glEp0Buffer[0];
					  sendData1 = glEp0Buffer[1];
				  }else{
					  sendData = glEp0Buffer[0];
					  sendData1 = glEp0Buffer[2];
				  }
				  break;
		 case CY_FX_USB_UVC_GET_RES_REQ:
		 	 if(CtrlID >= 0x10){
				 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCResLo;//ExUCtrlParArry[locCtrlID][7];//ext_control array;
				 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCResHi;//ExUCtrlParArry[locCtrlID][8];
				 glEp0Buffer[2] = 0;
				 glEp0Buffer[3] = 0;
		 	 }
		 	 else{
		 		 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCResLo;//ExUCtrlParArry[locCtrlID][7];//ext_control array;
		 		 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCResHi;//ExUCtrlParArry[locCtrlID][8];
		 		 glEp0Buffer[2] = 0;
		 		 glEp0Buffer[3] = 0;
		 	 }
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  sendData1 = glEp0Buffer[1];
			  //CyU3PDebugPrint (4, "The control ResLo: 0x%x, ResHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
			  break;
		 case CY_FX_USB_UVC_GET_INFO_REQ:
		 	 if(CtrlID >= 0x10){
				 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCInfoLo;//ExUCtrlParArry[locCtrlID][9];//ext_control array;
				 glEp0Buffer[1] = 0;//pEXTSenCtrl[CtrlID - 0x10]->UVCInfoHi;//ExUCtrlParArry[locCtrlID][10];
				 glEp0Buffer[2] = 0;
				 glEp0Buffer[3] = 0;
		 	 }
		 	 else{
		 		 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCInfoLo;//ExUCtrlParArry[locCtrlID][9];//ext_control array;
		 		 glEp0Buffer[1] = 0;//pPUCSenCtrl[CtrlID]->UVCInfoHi;//ExUCtrlParArry[locCtrlID][10];
		 		 glEp0Buffer[2] = 0;
		 		 glEp0Buffer[3] = 0;
		 	 }
		 	 Len = 1;
		 	 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
		 	 sendData = glEp0Buffer[0];
		 	 sendData1 = glEp0Buffer[1];
		 	 //CyU3PDebugPrint (4, "The control InfoLo: 0x%x, InfoHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
		 	 break;
		 case CY_FX_USB_UVC_GET_DEF_REQ:
		 	 if(CtrlID >= 0x10){//the camera terminal/extension uint command over 0x10.
		 		 if(Len == 2){
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCDefVLo;//ExUCtrlParArry[locCtrlID][11];
					 glEp0Buffer[1] = pEXTSenCtrl[CtrlID - 0x10]->UVCDefVHi;//ExUCtrlParArry[locCtrlID][12];
					 //CyU3PDebugPrint (4, "The CT/EX control DefVLo: 0x%x, DefVHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
		 		 }else //if(Len == 4)
		 		 {
					 glEp0Buffer[0] = pEXTSenCtrl[CtrlID - 0x10]->UVCDefVLo;//ExUCtrlParArry[locCtrlID][11];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[2] = pEXTSenCtrl[CtrlID - 0x10]->UVCDefVHi;//ExUCtrlParArry[locCtrlID][12];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The CT/EX control DefVLo: 0x%x, DefVHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }else{
			 	 if(Len ==2){
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCDefVLo;//ExUCtrlParArry[locCtrlID][11];
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCDefVHi;//ExUCtrlParArry[locCtrlID][12];
					 //CyU3PDebugPrint (4, "The PU control DefVLo: 0x%x, DefVHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[1], Len);
				 }else{
					 glEp0Buffer[0] = pPUCSenCtrl[CtrlID]->UVCDefVLo;//ExUCtrlParArry[locCtrlID][11];
					 glEp0Buffer[1] = 0;
					 glEp0Buffer[1] = pPUCSenCtrl[CtrlID]->UVCDefVHi;//ExUCtrlParArry[locCtrlID][12];
					 glEp0Buffer[3] = 0;
					 //CyU3PDebugPrint (4, "The PU control DefVLo: 0x%x, DefVHi: 0x%x, Len: 0x%x\r\n", glEp0Buffer[0], glEp0Buffer[2], Len);
				 }
			 }
		 	 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
		 	 if(Len == 2){
		 		 sendData = glEp0Buffer[0];
		 		 sendData1 = glEp0Buffer[1];
		 	 }else{
		 		 sendData = glEp0Buffer[0];
		 		 sendData1 = glEp0Buffer[2];
		 	 }
		 	 break;
		 case CY_FX_USB_UVC_SET_CUR_REQ:
			  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				  glEp0Buffer, &readCount);
			  if (apiRetStatus == CY_U3P_SUCCESS )
			   {
				 if(Len == 2){
					 Data0 = glEp0Buffer[0];
					 Data1 = glEp0Buffer[1];
				 }else{
					 Data0 = glEp0Buffer[0];
					 Data1 = glEp0Buffer[2];
				 }
				 dataIdx = 0;
				 CyU3PDebugPrint (4, "\r\nThe set control ctrlID: 0x%x Reg1: 0x%x Reg1:0x%x devADD: 0x%x Len: 0x%x Data0: 0x%x Data1: 0x%x",
						 CtrlID, RegAdd0, RegAdd1, devAdd, Len, Data0, Data1);
			 	 if(CtrlID >= 0x10){
					 switch(CtrlID)
					 {
						 case Ext1BLCGridCtlID6:
							 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;
							 if(Data0 == 1){
								 Data0 = PUCBLC.UVCCurVLo|0x80;
							 }else{
								 Data0 = PUCBLC.UVCCurVLo&0x7f;
							 }
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set grid status
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;
							 break;
					 	 case ExtCamMCtlID12://EXTCamMode
	#if 0 // not be used
							 sendData = CtrlParArry[CtrlID][13];

							 if(CamMode == 1){//720p
								if(sendData >= 3){
									CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
									sendData = 0; //set back to default
									CtrlParArry[CtrlID][13] = 0;
								}
								sendData += 4;
							 }
							//CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, sendData);
							 glEp0Buffer[0] = sendData;
							 glEp0Buffer[1] = 0;
	#endif
							 break;
					 	 case ExtSensorParCtlID14:
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set data
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//AGC. CtrlParArry[CtrlID][14]
							 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = 0;
							 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;
							 break;
					 	 case ExtI2CCtlID15:
					 		 for(idx=0; idx<Len; idx++){
					 			I2CCMDArry[idx] = glEp0Buffer[idx];
					 		 }
					 		 I2CCmdHandler();
							 break;
				 	 	 case ExtAexModCtlID9://4byte
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo != Data0)
							 {
								 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//exposure mode (assume b3:2=00, no BLC window). CtrlParArry[CtrlID][13]
								 Data0 = Data0 | (EXTShutter.UVCCurVLo << 4);
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set first byte
								 dataIdx++;
							 }
							 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi != Data1){
								 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = Data1;//AGC. CtrlParArry[CtrlID][14]
								 if(Data0 == 2 || Data0 == 3){
									 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //AGC
								 }
							 }
							 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CyU3PDebugPrint (4, "ExpM&AGC gotten from host. 0x%x %d; 0x%x 0x%x %d\r\n",
									 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo, pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi,
									 EXTShutter.UVCCurVLo, Data0, Data1);
							 break;
					 	 case ExtShutCtlID0: //special!!
							 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0; //CtrlParArry[CtrlID][13] save new setting
	#if 1	// register setting directly
						     if((EXTAexModGainlev.UVCCurVLo&0x3) != 0)//based on the Aex Mode 2 has been masked in viewer!!!
						     {
						    	 Data0 = (Data0 << 4) | (EXTAexModGainlev.UVCCurVLo);
						    	 dataIdx = 0;
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
								 //cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, 0x00, dataIdx);  //clean Axmode2 bit7
								 //dataIdx++;
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						     }
						     CyU3PDebugPrint (4, "The shutter&exposure 0x%x 0x%x 0x%x 0x%x\r\n",
						    		 Data1, Data0, EXTAexModGainlev.UVCCurVLo, EXTShutter.UVCCurVLo);
						     break;
						 case ExtCtlShutlevCtlID11://shutter level 2bytes standard operation!!!
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo != Data0){
								 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//AGC. CtrlParArry[CtrlID][14]
								 if(EXTAexModGainlev.UVCCurVLo == 1 || EXTAexModGainlev.UVCCurVLo == 3){
									 //cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, 0x80, dataIdx);  //set AxMode2 bit7
									 //dataIdx++;
									 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data0, dataIdx);  //shutter level
								 }
							 }
							 //CtrlParArry[CtrlID][16] = CyTrue;
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 CyU3PDebugPrint (4, "Shutter level gotten from host. 0x%x %d; 0x%x 0x%x %d\r\n",
									 EXTAexModGainlev.UVCCurVLo, EXTAexModGainlev.UVCCurVHi, EXTShutlev.UVCCurVLo, getData, getData1);
							 break;

	#else	// old fashion
							 if(Data0 == 0){//set exposure mode auto
								 if((CTCtrlParArry[AutoExMCtlID1][13] != 8) && (CTCtrlParArry[AutoExMCtlID1][13] != 2)){
									 if(CTCtrlParArry[AutoExMCtlID1][13] == 1) {
										 CTCtrlParArry[AutoExMCtlID1][13] = 8; //aperture priority
									 }else{
										 CTCtrlParArry[AutoExMCtlID1][13] = 2; //auto mode
									 }
								 }
							 }else{
								 Data1 = Data0 - 1;
								 if((CTCtrlParArry[AutoExMCtlID1][13] != 1) && (CTCtrlParArry[AutoExMCtlID1][13] != 4)){
									 if(CTCtrlParArry[AutoExMCtlID1][13] == 8) {
										 CTCtrlParArry[AutoExMCtlID1][13] = 1; //manual mode
									 }else{
										 CTCtrlParArry[AutoExMCtlID1][13] = 4; //shutter priority
									 }
								 }
								 if(Data1 < 8){
									 CTCtrlParArry[ExTmACtlID3][13] = ExTime[Data1][0];
									 CTCtrlParArry[ExTmACtlID3][14] = ExTime[Data1][1];
								 }else{
									 CTCtrlParArry[ExTmACtlID3][13] = ExTime[7][0];
									 CTCtrlParArry[ExTmACtlID3][14] = ExTime[7][1];
								 }
							 }
							 EXTShutter.AvailableF = CyTrue; //CtrlParArry[CtrlID][16] = CyTrue;
							 dataIdx = 0;
							 Data1 = getShutCtrl(Data0, &AxMode); //call setting shutter control Reg. routine.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, AxMode, dataIdx);  //First for Axmode 0
							 if(AxMode){
								 dataIdx++;
								 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, 0x80, dataIdx);  //Second for Axmode 2
								 dataIdx++;
								 cmdSet(cmdQuptr, CtrlID, 0x12, devAdd, Data1, dataIdx);  //Third for fine shutter adjustment
							 }
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 //CyU3PDebugPrint (4, "The shutter&exposure 0x%x 0x%x 0x%x ox%x\r\n", Data1, Data0, CTCtrlParArry[ExTmACtlID3][13], CtrlParArry[CtrlID][13]);
							 break;
	#endif
						 case Ext1BLCRangeCtlID4:
	#if 0 //seperate version
							 //registers value BLD window enable (0x17); position (0x13); size (0x14).getData = Data0&0xF; //get LSB H-Pos.
							 getData1 = Data0>>4; //get MSB V-Pos.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 if(getData1&0x8){//enable BLD window
								 cmdSet(cmdQuptr, CtrlID, 0x17, devAdd, 1, dataIdx); //show BLC window
							 }else{ //disable BLD window
								 cmdSet(cmdQuptr, CtrlID, 0x17, devAdd, 0, dataIdx); //close BLC window
							 }
							 getData1 = getData1&0x7; //mask bit7 ~ bit3/
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData, dataIdx);  //set H-Pos
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, getData1, dataIdx);  //set V-Pos
							 dataIdx++;
							 getData = Data1&0xf; //get LSB H-size.
							 getData1 = Data1>>4; //get MSB V-size.
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, getData, dataIdx);  //set H-size
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, getData1, dataIdx);  //set V-size
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
	#else //combination version
							 //Data0 = Data0&0x7F; //mask window show flag bit.
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set H/V-Pos
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //set H/V-size
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 getData1 = Data1;
	#endif
							 EXTBLCWinPos.UVCCurVLo = Data0; //ExUCtrlParArry[locCtrlID][13] = Data0;//ext_control array;
							 EXTBLCWinPos.UVCCurVHi = Data1; //ExUCtrlParArry[locCtrlID][14] = Data1;
							 EXTBLCWinPos.AvailableF = CyTrue; //ExUCtrlParArry[locCtrlID][16] = CyTrue;
							 break;
				 	 	 case Ext1ExCtrlSpeedCtlID8:
							 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo != Data0){  //2Bytes
								 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//AGC. CtrlParArry[CtrlID][14]
								 Data1 = SensorGetControl(RegAdd0, devAdd); //get the origin data as the fine shutter enable uses the same reg.
								 Data1 &= 0x0F; Data0 = (Data0&0x0F) << 4;
								 Data0 |= Data1;
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set data
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
								 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = 0;
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 }
							 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;

				 	 	 case Ext1BLCWeightCtlID5:
				 	 	 case Ext1ExHysterCtlID7:
				 	 	 case Ext1EnhanceModeCtlID9:
				 	 	 case Ext1EnhanceGainCtlID10:
				 	 	 case Ext12DNRGainEnblCtlID12:
				 	 	 case Ext1GammaCorCtlID14:
				 	 	 case Ext1AGCMaxLimetCtlID15:
				 	 	 case Ext1EnhanceStarEndCtlID11://4bytes
				 	 	 case Ext12DNRGainStarEndCtlID13://4bytes
						 case ExtExRefCtlID10:
				 	 	 default:
				 	 			 if(Len == 2)
				 	 			 {
									 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo != Data0){  //2Bytes
										 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
										 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set data
										 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
										 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//AGC. CtrlParArry[CtrlID][14]
										 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = 0;
									 }
									 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;
				 	 			 }else if(Len == 4){
									 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);   //get mutex 4Btyes
									 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo != Data0)
									 {
										 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo = Data0;//exposure mode (assume b3:2=00, no BLC window). CtrlParArry[CtrlID][13]
										 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set first byte
										 dataIdx++;
									 }
									 if(pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi != Data1){
										 pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi = Data1;//AGC. CtrlParArry[CtrlID][14]
										 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //set second byte
									 }
									 pEXTSenCtrl[CtrlID - 0x10]->AvailableF = CyTrue;
									 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

				 	 			 }
				 	 			 CyU3PDebugPrint (4, "The data gets from host. [0]: 0x%x [1]: 0x%x curLo: 0x%x curHi: 0x%x Len: 0x%x\r\n",
											 Data0, Data1, pEXTSenCtrl[CtrlID - 0x10]->UVCCurVHi, pEXTSenCtrl[CtrlID - 0x10]->UVCCurVLo, Len);
				 	 			 break;
					 }
			 	 }
			 	 else{
					 switch(CtrlID)
					 {
						 case BrgtCtlID1:
							 dataIdx = 0;
	#if 0 //cancel for 5MP w/b camera
								 dataIdx = 0;
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
								  /****** double check the register0 Data1 ******/
								  if(Data0&0x80){
									  Data1 = ((Data0 >> 6)&0x01)|(CtrlParArry[CtrlID][14]&0xfc);
								  }else{
									  Data1 = ((Data0 >> 6)|0x02)|(CtrlParArry[CtrlID][14]&0xfc);
								  }
								 Data1 |= ~0x03;
								 Data1 &= 0xC7;
							  	 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //First
							  	 dataIdx++;

								 Data0 = (Data0 << 2);
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);   //Second
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

								 CtrlParArry[CtrlID][13] = Data0;
								 CtrlParArry[CtrlID][14] = Data1;
								 CtrlParArry[CtrlID][16] = CyTrue;
	#endif
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
								  /****** double check the register0 Data1 ******/
								 /*// new I2C command. direct using Data0 coming from host. 8/23/2016
								 if(Data0&0x80){
									  Data0 = Data0 - 0x80;
								  }else{
									  Data0 = ~Data0;
								  }
								  */
							  	 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

								 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
								 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
							 break;
						 case HueCtlID5://not being used
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, (Data0-GREEN_BASE), dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegMg, devAdd, (Data0-MAGENTA_BASE), dataIdx);  //Second
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegYel, devAdd, (Data0-YELLOW_BASE), dataIdx);  //Third
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegCy, devAdd, (Data0-CYAN_BASE), dataIdx);  //Fourth
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, HuectrlRegRed, devAdd, (Data0-RED_BASE), dataIdx);  //Fifth
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, (glEp0Buffer[0]-BLUE_BASE), dataIdx);   //Sixth
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 pPUCSenCtrl[CtrlID]->UVCCurVLo = glEp0Buffer[0] - GREEN_BASE;
							 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
							 break;
						 case MFreqCtlID4:
							 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
							 //Data0 = Data0 - 1;
							 is60Hz = Data0;
							 if(Data0 < 0)  //for specific check. if it's minor value, set to 0.
							 {
								 Data0 = 0;  // 50Hz (PAL)
								 is60Hz = CyFalse;
							 }
							 else if(Data0 >2)
							 {
								 Data0 = 1;  // 60Hz (NTSC)
								 is60Hz = CyTrue;
							 }
							 CyU3PDebugPrint (4, "Frequency setting is  %d %d\r\n", Data0, is60Hz);
							 if (0/*gpif_initialized == CyTrue*/)
							 {
			                       switch (3/*setRes*/)
			                         {
			                         	case 1: //1944
			                         		SensorSetIrisControl(0x1, 0x30, is60Hz? 0x22:0xA2, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
			                         		CyU3PThreadSleep(500);
			                                CyU3PDebugPrint (4, "FSet the video mode format %x %d\n", is60Hz? 0x22:0xA2, is60Hz);
			                         		break;
			                         	case 2: //1080
			                         		SensorSetIrisControl(0x1, 0x30, is60Hz? 0x12:0x92, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
			                         		CyU3PThreadSleep(500);
			                                CyU3PDebugPrint (4, "FSet the video mode format %x %d\n", is60Hz? 0x12:0x92, is60Hz);
			                         		break;
			                         	case 3: //720
			                         		SensorSetIrisControl(0x1, 0x30, ((is60Hz? 0x02:0x82)&0xFC)|ROIMode, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
			                         		CyU3PThreadSleep(500);
			                                CyU3PDebugPrint (4, "FSet the video mode format %x %d\n", ((is60Hz? 0x02:0x82)&0xFC)|ROIMode, is60Hz);
			                         		break;
			                         	case 4: //VGA
			                         		SensorSetIrisControl(0x1, 0x30, ((is60Hz? 0x32:0xB2)&0xFC)|ROIMode, I2C_DSPBOARD_ADDR_WR/*boardID*/);//start 5MP Res
			                         		CyU3PThreadSleep(500);
			                                CyU3PDebugPrint (4, "FSet the video mode format %x %d\n", ((is60Hz? 0x32:0xB2)&0xFC)|ROIMode, is60Hz);
			                         	default:
			                         		break;
			                         }
							 }

							 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
							 break;
						 case WBTLevCtlID11: //4bytes
							 //Data0 = glEp0Buffer[0]; //blue
							 //Data1 = glEp0Buffer[2]; //red
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 dataIdx++;
							 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //Second
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

							 WBMenuCmpArry[0] = Data0;//using for blue part
							 WBMenuCmpArry[2] = Data1;//using for red part
							 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
							 break;
						 case BLCCtlID0:
							 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
							 if(CamMode == 1) //mode 720p
							 {
								 if(Data0 < 3){
					 				 Data0 += 4;
					 			 }else{
									CyU3PDebugPrint (4, "back light compensation setting is not correct. %d %d\r\n", CamMode, getData);
									Data0 = 4; //set to default.
					 			 }
					 		 }else{
					 			 Data0 |= EXTBLCGrid.UVCCurVLo << 7;
					 		 }

							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
					 		 break;
						 case ShapCtlID7: //for stand sharpness with edge enhancement in 5MP B/W.

							 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;
							 if(Data0 != 0){
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
	#ifdef COLOR
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //Second: set enhancement value.
	#else
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, 0x1, dataIdx);  //First: enable sharpness.
								 dataIdx++;
								 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data0, dataIdx);  //Second: set enhancement value.
	#endif
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 }else{
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
								 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, 0x0, dataIdx);  //First: disable sharpness.
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 }
							 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
							 break;
						 case ConsCtlID2:
						 case WBTMdCtlID9:
						 case SaturCtlID6:
						 default:
			 	 			 if(Len == 2)
			 	 			 {
								 if(pPUCSenCtrl[CtrlID]->UVCCurVLo != Data0){  //2Bytes
									 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
									 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set data
									 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
									 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;//AGC. CtrlParArry[CtrlID][14]
									 pPUCSenCtrl[CtrlID]->UVCCurVHi = 0;
								 }
								 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
			 	 			 }else if(Len == 4){
								 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);   //get mutex 4Btyes
								 if(pPUCSenCtrl[CtrlID]->UVCCurVLo != Data0)
								 {
									 pPUCSenCtrl[CtrlID]->UVCCurVLo = Data0;//exposure mode (assume b3:2=00, no BLC window). CtrlParArry[CtrlID][13]
									 cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, Data0, dataIdx);  //set first byte
									 dataIdx++;
								 }
								 if(pPUCSenCtrl[CtrlID]->UVCCurVHi != Data1){
									 pPUCSenCtrl[CtrlID]->UVCCurVHi = Data1;//AGC. CtrlParArry[CtrlID][14]
									 cmdSet(cmdQuptr, CtrlID, RegAdd1, devAdd, Data1, dataIdx);  //set second byte
								 }
								 pPUCSenCtrl[CtrlID]->AvailableF = CyTrue;
								 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex

			 	 			 }
			 	 			 CyU3PDebugPrint (4, "The data gets from host. [0]: 0x%x [1]: 0x%x curLo: 0x%x curHi: 0x%x Len: 0x%x\r\n",
										 Data0, Data1, pPUCSenCtrl[CtrlID]->UVCCurVHi, pPUCSenCtrl[CtrlID]->UVCCurVLo, Len);
			 	 			 break;
					 }
			 	 }
			   }else{
				   CyU3PDebugPrint (4, "The get data from host fail error code %d.\r\n", apiRetStatus);
			   }
#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The setup sensor value %d, 0x%x 0x%x 0x%x 0x%x %d\r\n", CtrlID, readCount, Data0, Data1, CtrlParArry[CtrlID][14], 0xff); // additional debug
#endif

			  break;
		  default:
			  CyU3PUsbStall (0, CyTrue, CyFalse);
			  break;
		 }
EndofSet:    CyU3PDebugPrint (4, "The Request 0x%x parameter get from host 0x%x 0x%x / send to host 0x%x 0x%x\r\n", reqData, Data0, Data1, sendData, sendData1);
}
/************** CT control requests handler *************************/
#define EXLIMIT  200  //shutter value limit in 30 fps
#if 0
inline void CTControlHandle(uint8_t CtrlID){
    CyU3PReturnStatus_t apiRetStatus = !CY_U3P_SUCCESS;
    VdRingBuf *cmdQuptr = &cmdQu;
    uint16_t readCount;
    uint8_t RegAdd0, RegAdd1, Data0, Data1, Len;
    uint16_t diff, value, diffRd;
    uint8_t i, shutter, index;
    diff = 0xffff;
    shutter = 1;
    index = 1;

    uint8_t devAdd = CTCtrlParArry[CtrlID][15];
    RegAdd0 = CTCtrlParArry[CtrlID][0];
    RegAdd1 = CTCtrlParArry[CtrlID][1];
    Len = CTCtrlParArry[CtrlID][2];
    uint8_t dataIdx, getData=0xFF, getData1=0xff, sendData=0xff, sendData1=0xFF, reqData;
#ifdef USB_DEBUG_PRINT
    CyU3PDebugPrint (4, "The cur sensor value(CT) %d 0x%x 0x%x\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14]); // additional debug
#endif
    reqData = bRequest;

    switch (bRequest)
		 {

		 case CY_FX_USB_UVC_GET_LEN_REQ: /* the length of get length request always setting to 2 */
			  glEp0Buffer[0] = Len;
			  glEp0Buffer[1] = 0;
			  CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_CUR_REQ: /* Current value. */

			 switch(CtrlID)
			 {
				 default:
					 glEp0Buffer[0] = CTCtrlParArry[CtrlID][13];
					 glEp0Buffer[1] = CTCtrlParArry[CtrlID][14];
					 glEp0Buffer[2] = 0;
					 glEp0Buffer[3] = 0;
					 sendData = glEp0Buffer[0];
					 break;
			 }

			 CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);

#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "The get sensor value(CT) %d 0x%x 0x%x, %d %d\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14], glEp0Buffer[0], Len); // additional debug
#endif
			  break;
		 case CY_FX_USB_UVC_GET_MIN_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][3];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][4];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;//1;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_MAX_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][5];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][6];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_RES_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][7];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][8];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_GET_INFO_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][9];
			  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  Len = 1;
			  break;
		 case CY_FX_USB_UVC_GET_DEF_REQ:
			  glEp0Buffer[0] = CTCtrlParArry[CtrlID][11];
			  glEp0Buffer[1] = CTCtrlParArry[CtrlID][12];
			  if(ZmOpRCtlID10 == CtrlID) glEp0Buffer[2] = SPEED;
			  else glEp0Buffer[2] = 0;
			  glEp0Buffer[3] = 0;
			  CyU3PUsbSendEP0Data (Len, (uint8_t *)glEp0Buffer);
			  sendData = glEp0Buffer[0];
			  break;
		 case CY_FX_USB_UVC_SET_CUR_REQ:
			  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
			  glEp0Buffer, &readCount);
			  Data0 = glEp0Buffer[0];
			  Data1 = glEp0Buffer[1];
			  value = Data1;

			  switch(CtrlID)
			  {
		  	      case AutoExMCtlID1:
		  		    //CyU3PDebugPrint (4, "The Ex Mode value(CT) %d 0x%x 0x%x 0x%x 0x%x, %d!\r\n", CtrlID, glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3], readCount); // additional debug

				    CTCtrlParArry[CtrlID][13] = Data0;
				    CTCtrlParArry[CtrlID][16] = CyTrue;
				    getData = glEp0Buffer[0];
		  		    //CyU3PDebugPrint (4, "The Ex Mode set value(CT) %d %d!\r\n", CtrlID, CTCtrlParArry[CtrlID][13]); // additional debug
		  		    switch (getData){
						case 1:
							setIrisauto(cmdQuptr, 0); //set Iris being manual.
							break;
						case 2:
			  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
							dataIdx = 0;
							CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
							CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
				  		    setIrisauto(cmdQuptr, 1); //set Iris being auto.

							break;
						case 4:
			  		    	setIrisauto(cmdQuptr, 1); //set Iris being auto.
							break;
						case 8:
			  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
			  		    	dataIdx = 0;
			  		    	CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
			  		    	cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
			  		    	CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
			  		    	setIrisauto(cmdQuptr, 0); //set Iris being manual.
							break;
		  		    }
#if 0
				    if(getData == 2 || getData == 8){//if exposure mode is auto or aperture priority
		  		    	CtrlParArry[ExtShutCtlID0][13] = 0; //set shutter is auto.
						  dataIdx = 0;
						  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
						  cmdSet(cmdQuptr, ExtShutCtlID0, RegAdd0, devAdd, 0, dataIdx);  //set shutter value to 0
						  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
		  		    }
		  		    if(getData == 2 || getData == 4){//if exposure mode is auto or exprosue priority
		  		    	setIrisauto(cmdQuptr, 1); //set Iris being auto.
		  		    }
#endif
				    break;

			  	  case ExTmACtlID3:
			  		//CyU3PDebugPrint (4, "The Ex Time value(CT) %d 0x%x 0x%x 0x%x 0x%x, %d!\r\n", CtrlID, glEp0Buffer[0], glEp0Buffer[1], glEp0Buffer[2], glEp0Buffer[3], readCount); // additional debug

					  value = (value << 8)|Data0;
					  if(((CTCtrlParArry[AutoExMCtlID1][13] == 1) || (CTCtrlParArry[AutoExMCtlID1][13] == 4))
							  && (value < (EXLIMIT+50)))//shutter set accepted
					  {
						  for(i = 0; i < 8; i++)//find closest shutter No.
						  {
							if(value > ShutValueArry[i]){
								diffRd = value-ShutValueArry[i];
							}else{
								diffRd = ShutValueArry[i]-value;
							}
							  if(diff > diffRd){
								  diff = diffRd;
								  index = i;
							  }
						  }
						  shutter = shutter+index;

						  dataIdx = 0;
						  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
						  cmdSet(cmdQuptr, CtrlID, RegAdd0, devAdd, shutter, dataIdx);  //First
						  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						  //CyU3PDebugPrint (4, "The Ex Time shutter value(CT) %d %d %d %d!\r\n", shutter, index, ShutValueArry[index], diff); // additional debug

						  CTCtrlParArry[CtrlID][13] = Data0;
						  CTCtrlParArry[CtrlID][14] = Data1;
						  CTCtrlParArry[CtrlID][16] = CyTrue;
						  CtrlParArry[ExtShutCtlID0][13] = shutter; //set extension shutter current value
					  }else{
						  CyU3PUsbStall (0, CyTrue, CyFalse);
					  }
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];
					  break;
			  	  case IriACtlID7:
					  if((CTCtrlParArry[AutoExMCtlID1][13] == 1) || (CTCtrlParArry[AutoExMCtlID1][13] == 8))//Iris set accepted
					  {
							 dataIdx = 0;
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, 0x22, RegAdd0, devAdd, Data0, dataIdx);  //First
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 //CyU3PEventSet (&glTimerEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);//set event of the command available.

							 CTCtrlParArry[CtrlID][13] = Data0;
							 CTCtrlParArry[CtrlID][14] = Data1;
							 CTCtrlParArry[CtrlID][16] = CyTrue;
					  }else{
						  CyU3PUsbStall (0, CyTrue, CyFalse);
					  }
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];

					  break;
			  	  case ZmOpRCtlID10:
					  getData = glEp0Buffer[0];
					  getData1 = glEp0Buffer[1];
#if 1
					  dataIdx = 0;
					  CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
					  if(getData == 1)
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, TELEDATA, dataIdx);  //telephoto direction
					  else if(getData == 0xff)
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, WIDEDATA, dataIdx);  //wide-angle direction
					  else
						  cmdSet(cmdQuptr, 0x23, RegAdd0, devAdd, STOP, dataIdx);
					  //dataIdx++;
					  //cmdSet(cmdQuptr, 23, RegAdd0, devAdd, STOP, dataIdx); //for temp implementation for stop zoom
					  CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
#endif
					  CyU3PDebugPrint (4, "Zoom Op receives (CT) 0x%x 0x%x 0x%x\r\n", getData, getData1, glEp0Buffer[2]);
					  break;

			  	  default:
					 CTCtrlParArry[CtrlID][13] = glEp0Buffer[0];
					 CyU3PDebugPrint (4, "default selector (CT) 0x%x 0x%x\r\n", CtrlID, bRequest); // additional debug
			  		 break;
			  }
			  break;
		  default:
			  CyU3PUsbStall (0, CyTrue, CyFalse);
			  CyU3PDebugPrint (4, "default request (CT) 0x%x 0x%x\r\n", CtrlID, bRequest); // additional debug
			  break;
		 }
	//CyU3PDebugPrint (4, "The get sensor value(CT) %d 0x%x 0x%x, %d %d\r\n", CtrlID, CTCtrlParArry[CtrlID][13], CTCtrlParArry[CtrlID][14], glEp0Buffer[0], Len); // additional debug

    CyU3PDebugPrint (4, "The Request 0x%x parameter get from host (CT) 0x%x 0x%x 0x%x / send to host 0x%x 0x%x 0x%x, %d\r\n", reqData, getData, getData1, glEp0Buffer[2], sendData, sendData1, glEp0Buffer[2], Len);
}
#endif // end of the CT controls


/*
 * Entry function for the UVC Application Thread
 */

uint32_t posTick;
CyU3PTimer I2CCmdTimer;

void  I2CCmdCb(uint32_t input){
	CyU3PDebugPrint (4, "I2C pos-timer %d %d\r\n", posTick, input);
	CyU3PThreadSleep(input);
	CyU3PEventSet (&glTimerEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_OR);
}


void
UVCHandleProcessingUnitRqts (uint16_t wValue, uint8_t  bRequest)
{
    uint8_t CtrlAdd;
#ifdef DbgInfo
    CyU3PDebugPrint (4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
#endif
    switch (wValue)
    {
    	case CY_FX_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL:
    		ControlHandle(BLCCtlID0, bRequest);
    		break;
        case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:
   			ControlHandle(BrgtCtlID1, bRequest);
    		break;
       case CY_FX_UVC_PU_CONTRAST_CONTROL:
			ControlHandle(ConsCtlID2, bRequest);
			break;

       case CY_FX_UVC_PU_GAIN_CONTROL: break;

       case CY_FX_UVC_PU_POWER_LINE_FREQUENCY_CONTROL:
      		ControlHandle(MFreqCtlID4, bRequest);
      		break;
       case CY_FX_UVC_PU_HUE_CONTROL:
     		ControlHandle(HueCtlID5, bRequest);
     		break;
       case CY_FX_UVC_PU_SATURATION_CONTROL:
       		ControlHandle(SaturCtlID6, bRequest);
       		break;
       case CY_FX_UVC_PU_SHARPNESS_CONTROL:
       		ControlHandle(ShapCtlID7, bRequest);
       		break;
       case CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL://
       //case CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL:
       case CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
    	   ControlHandle(WBTMdCtlID9, bRequest);
    		break;
       case CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL:
    		ControlHandle(WBTLevCtlID11, bRequest);
    		break;
       case CY_FX_UVC_PU_DIGITAL_MULTIPLIER_CONTROL:
    		ControlHandle(DigZmCtlID14, bRequest);
    		break;

        default:
            /*
             * Only the  control is supported as of now. Add additional code here to support
             * other controls.
             */
        	CyU3PDebugPrint (4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
}

/*
 * Handler for control requests addressed to the UVC Camera Terminal unit.
 */
#if 0 //the CT controls handler
static void
UVCHandleCameraTerminalRqts (
        void)
{
#ifdef UVC_PTZ_SUPPORT
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;
    uint16_t zoomVal;
    int32_t  panVal, tiltVal;
    CyBool_t sendData = CyFalse;
#endif
    uint8_t CtrlAdd;

    switch (wValue)
    {
    	case CY_FX_UVC_CT_SCANNING_MODE_CONTROL:
    		CtrlAdd = CTCtrlParArry[ScanMCtlID0][0];
    		CTControlHandle(ScanMCtlID0, bRequest);
    		break;
        case CY_FX_UVC_CT_AE_MODE_CONTROL:
        	CtrlAdd = CTCtrlParArry[AutoExMCtlID1][0];
   			CTControlHandle(AutoExMCtlID1, bRequest);
    		break;
       case CY_FX_UVC_CT_AE_PRIORITY_CONTROL:
    	    CtrlAdd = CTCtrlParArry[AutoExPCtlID2][0];
			CTControlHandle(AutoExPCtlID2, bRequest);
			break;

       case CY_FX_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
			CtrlAdd = CTCtrlParArry[ExTmACtlID3][0];
			CTControlHandle(ExTmACtlID3, bRequest);
			break;

       case CY_FX_UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL:
     		CtrlAdd = CTCtrlParArry[ExTmRCtlID4][0];
      		CTControlHandle(ExTmRCtlID4, bRequest);
      		break;
       case CY_FX_UVC_CT_FOCUS_ABSOLUTE_CONTROL:
    		CtrlAdd = CTCtrlParArry[FocACtlID5][0];
     		CTControlHandle(FocACtlID5, bRequest);
     		break;
       case CY_FX_UVC_CT_FOCUS_RELATIVE_CONTROL:
          		CtrlAdd = CTCtrlParArry[FocRCtlID6][0];
          		CTControlHandle(FocRCtlID6, bRequest);
          		break;
       case CY_FX_UVC_CT_FOCUS_AUTO_CONTROL:
          		break;
       case CY_FX_UVC_CT_IRIS_ABSOLUTE_CONTROL://
     		CtrlAdd = CTCtrlParArry[IriACtlID7][0];
     		CTControlHandle(IriACtlID7, bRequest);
     		break;

       case CY_FX_UVC_CT_IRIS_RELATIVE_CONTROL:
    		CtrlAdd = CTCtrlParArry[IriRCtlID8][0];
    		CTControlHandle(IriRCtlID8, bRequest);
    		break;
       case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
    		CtrlAdd = CTCtrlParArry[ZmOpACtlID9][0];
    		CTControlHandle(ZmOpACtlID9, bRequest);
    		break;
       case CY_FX_UVC_CT_ZOOM_RELATIVE_CONTROL:
    		CtrlAdd = CTCtrlParArry[ZmOpRCtlID10][0];
    		CTControlHandle(ZmOpRCtlID10, bRequest);
    		break;

        default:
            /*
             * Only the  control is supported as of now. Add additional code here to support
             * other controls.
             */
        	CyU3PDebugPrint (4, "The default setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }

#ifdef UVC_PTZ_SUPPORT
    switch (wValue)
    {
        case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* Support GET/SET queries. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ: /* Current zoom control value. */
                    zoomVal  = CyFxUvcAppGetCurrentZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMinimumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMaximumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution is one unit. */
                    zoomVal  = CyFxUvcAppGetZoomResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ: /* Default zoom setting. */
                    zoomVal  = CyFxUvcAppGetDefaultZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        zoomVal = (glEp0Buffer[0]) | (glEp0Buffer[1] << 8);
                        CyFxUvcAppModifyZoom (zoomVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 2-byte data in zoomVal back to the USB host. */
                glEp0Buffer[0] = CY_U3P_GET_LSB (zoomVal);
                glEp0Buffer[1] = CY_U3P_GET_MSB (zoomVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;

        case CY_FX_UVC_CT_PANTILT_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* GET/SET requests supported for this control */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                    panVal   = CyFxUvcAppGetCurrentPan ();
                    tiltVal  = CyFxUvcAppGetCurrentTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ:
                    panVal   = CyFxUvcAppGetMinimumPan ();
                    tiltVal  = CyFxUvcAppGetMinimumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ:
                    panVal   = CyFxUvcAppGetMaximumPan ();
                    tiltVal  = CyFxUvcAppGetMaximumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ:
                    panVal   = CyFxUvcAppGetPanResolution ();
                    tiltVal  = CyFxUvcAppGetTiltResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ:
                    panVal   = CyFxUvcAppGetDefaultPan ();
                    tiltVal  = CyFxUvcAppGetDefaultTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        panVal = (glEp0Buffer[0]) | (glEp0Buffer[1]<<8) |
                            (glEp0Buffer[2]<<16) | (glEp0Buffer[2]<<24);
                        tiltVal = (glEp0Buffer[4]) | (glEp0Buffer[5]<<8) |
                            (glEp0Buffer[6]<<16) | (glEp0Buffer[7]<<24);

                        CyFxUvcAppModifyPan (panVal);
                        CyFxUvcAppModifyTilt (tiltVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 8-byte PAN and TILT values back to the USB host. */
                glEp0Buffer[0] = CY_U3P_DWORD_GET_BYTE0 (panVal);
                glEp0Buffer[1] = CY_U3P_DWORD_GET_BYTE1 (panVal);
                glEp0Buffer[2] = CY_U3P_DWORD_GET_BYTE2 (panVal);
                glEp0Buffer[3] = CY_U3P_DWORD_GET_BYTE3 (panVal);
                glEp0Buffer[4] = CY_U3P_DWORD_GET_BYTE0 (tiltVal);
                glEp0Buffer[5] = CY_U3P_DWORD_GET_BYTE1 (tiltVal);
                glEp0Buffer[6] = CY_U3P_DWORD_GET_BYTE2 (tiltVal);
                glEp0Buffer[7] = CY_U3P_DWORD_GET_BYTE3 (tiltVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;
            //CyU3PDebugPrint (4, "The camera request received 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
        default:
            //CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
#endif
}
#endif


/*
 * Handler for control requests addressed to the Extension Unit.
 */
void
UVCHandleExtensionUnitRqts (uint16_t wValue, uint8_t  bRequest)
{
    //uint8_t CtrlAdd;  //set control ID -add

#ifdef DbgInfo
    CyU3PDebugPrint (4, "The setup request value 0x%x 0x%x\r\n", wValue, bRequest); // additional debug
#endif
    switch (wValue)
    {
    	case CY_FX_EXT_CONTROL_1SHUTTER: // shutter CONTROL1
      		ControlHandle(ExtShutCtlID0, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_2SENUPMODE: // sense up mode CONTROL2
      		ControlHandle(ExtSenCtlID1, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_3MIRROR: // mirror mode CONTROL3
      		ControlHandle(ExtMirrCtlID2, bRequest);
     		break;
    	case CY_FX_EXT_CONTROL_43DNOISEREDUC_MODE: //3D noise reduce control CONTROL4
      		ControlHandle(Ext3DNReduMCtlID3, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_53DNOISEREDUC_CTRL: //3D noise reduce level CONTROL5
      		ControlHandle(Ext3DNReduLvCtlID4, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_6DAYNIGHT_MODE: // day night mode CONTROL6
      		ControlHandle(ExtDNModCtlID5, bRequest);
     		break;
    	case CY_FX_EXT_CONTROL_7DAYNIGHT_DELAY: //day night switch delay CONTROL7
      		ControlHandle(ExtDNDelytlID6, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_8DAYNIGHT_LEVEL: //day to night level CONTROL8
      		ControlHandle(ExtDNlevCtlID7, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_9NIGHTDAY_LEVEL: //night to day level CONTROL9
      		ControlHandle(ExtNDlevCtlID8, bRequest);
     		break;
    	case CY_FX_EXT_CONTROL_10EXPOSURE_MODE: //AEx mode CONTROL10
    		if(1||CamMode == 1){//only 720p support
				ControlHandle(ExtAexModCtlID9, bRequest);
    		}else/* no support for 1080p camera */
    			CyU3PDebugPrint (4, "The host command is not correct for 1080p camera 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_11AEREFERENCE_LEVEL: //AEx reference level CONTROL11
      		ControlHandle(ExtExRefCtlID10, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_12: //AEx shutter level CONTROL12
      		ControlHandle(ExtCtlShutlevCtlID11, bRequest);
    		break;

    		//ExtCtlShutlevCtlID11
		case CY_FX_EXT_CONTROL_13CAMERA_MODE: //Camera Mode CONTROL13
      		ControlHandle(ExtCamMCtlID12, bRequest);
    		break;
		//case CY_FX_EXT_CONTROL_14SNAP_SHOT: //Still image set CONTROL14
    		//CtrlAdd = CtrlParArry[ExtshotCtlID13][0];
      		//ControlHandle(ExtshotCtlID13);
    		//break;
		case CY_FX_EXT_CONTROL_15SENSOR_PARS: //Sensor Parameters set CONTROL15
      		ControlHandle(ExtSensorParCtlID14, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_16I2C_COMMAND: //I2C commands operation CONTROL16
      		ControlHandle(ExtI2CCtlID15, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_17BLC_RANGE:   //BLD range CONTROL17
      		ControlHandle(Ext1BLCRangeCtlID4, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_18BLC_POSITION:   //BLD gain CONTROL18
      		ControlHandle(Ext1BLCWeightCtlID5, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_19BLC_GRID:   //BLD gain CONTROL19
      		ControlHandle(Ext1BLCGridCtlID6, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_20EXP_HYSTER: //exposure hysteresis CONTROL20
    		//CtrlAdd = ExUCtrlParArry[Ext1ExHysterCtlID7-EXUAOFFSET][0];
      		ControlHandle(Ext1ExHysterCtlID7, bRequest);
			CyU3PDebugPrint (4, "The hyster command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_21EXP_CTRLSPD: //exposure control speed CONTROL21
      		ControlHandle(Ext1ExCtrlSpeedCtlID8, bRequest);
			CyU3PDebugPrint (4, "The ctrlspd command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_22ENHANCE_MODE: //edge enhancement mode CONTROL22
      		ControlHandle(Ext1EnhanceModeCtlID9, bRequest);
			CyU3PDebugPrint (4, "The edgeMode command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_23ENHANCE_GAIN: //edge enhancement gain CONTROL23
      		ControlHandle(Ext1EnhanceGainCtlID10, bRequest);
			CyU3PDebugPrint (4, "The edgegain command 0x%x 0x%x\r\n", wValue, bRequest);
   		break;
		case CY_FX_EXT_CONTROL_24ENHANCE_STED: //edge enhancement start/end CONTROL24
      		ControlHandle(Ext1EnhanceStarEndCtlID11, bRequest);
			CyU3PDebugPrint (4, "The edge start/end command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_262DNR_STED: //2D noise reduction start/end CONTROL26
      		ControlHandle(Ext12DNRGainStarEndCtlID13, bRequest);
			CyU3PDebugPrint (4, "The 2DNR start/end command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_27GAMMA_MODE: //gamma correction CONTROL27
      		ControlHandle(Ext1GammaCorCtlID14, bRequest);
			CyU3PDebugPrint (4, "The gamma command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
		case CY_FX_EXT_CONTROL_28AGC_MAX: //AGC Maximum limitation CONTROL28
      		ControlHandle(Ext1AGCMaxLimetCtlID15, bRequest);
			CyU3PDebugPrint (4, "The AGC Max command 0x%x 0x%x\r\n", wValue, bRequest);
    		break;
   	default:
    		/* No requests supported as of now. Just stall EP0 to fail the request. */
    		CyU3PUsbStall (0, CyTrue, CyFalse);
    		break;
    }

}
#if 0 //the EP0 thread
/*
 * Entry function for the UVC control request processing thread.
 */
void
UVCAppEP0Thread_Entry (
        uint32_t input)
{
    uint32_t eventMask = (CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT | CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT);
    uint32_t eventFlag;
	CyBool_t value;
	CyBool_t *valueptr = &value;


#ifdef USB_DEBUG_INTERFACE
    CyU3PReturnStatus_t apiRetStatus;
    CyU3PDmaBuffer_t    dmaInfo;

    eventMask |= CY_FX_USB_DEBUG_CMD_EVENT;
#endif

    /* for interrupt status test */
    CyU3PReturnStatus_t apiRetStatus;
    eventMask |= VD_FX_INT_STA_EVENT;
    CyU3PDmaBuffer_t    interStabuf;

    for (;;)
    {
        /* Wait for a Video control or streaming related request on the control endpoint. */
        if (CyU3PEventGet (&glFxUVCEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag,
                    CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS)
        {
            /* If this is the first request received during this connection, query the connection speed. */
            if (!isUsbConnected)
            {
                usbSpeed = CyU3PUsbGetSpeed ();
                if (usbSpeed != CY_U3P_NOT_CONNECTED)
                {
                    isUsbConnected = CyTrue;
                }
            }
//#ifdef DbgInfo
            if((eventFlag & eventMask) & ~VD_FX_INT_STA_EVENT)
            CyU3PDebugPrint (4, "USB speed = %d evenflag = 0x%x bmReqType = 0x%x\r\n"
            		"bRequest = 0x%x wValue = 0x%x wIndex = 0x%x wLength = 0x%x isflag 0x%x\r\n",
            		usbSpeed, eventFlag, bmReqType, bRequest, wValue, wIndex, wLength, 0/*isFlag*/); /* additional debug message */
            //CyU3PDebugPrint (4, "fb = %d pb = %d pbc = %d pbcp = %d\r\n", fbbak, pbbak, pbcbak, pbcpbak);
            //fbbak=0;pbbak=0;pbcbak=0;pbcpbak=0;
//#endif
            if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)
            {
            	switch ((wIndex >> 8))
                {

                    case CY_FX_UVC_PROCESSING_UNIT_ID:
                        UVCHandleProcessingUnitRqts ();
                        break;

                    case CY_FX_UVC_CAMERA_TERMINAL_ID:
                        UVCHandleCameraTerminalRqts ();
                        break;

                    case CY_FX_UVC_INTERFACE_CTRL:
                        UVCHandleInterfaceCtrlRqts ();
                        break;

                    case CY_FX_UVC_EXTENSION_UNIT_ID:
                        UVCHandleExtensionUnitRqts ();
                        break;

                    default:
                        /* Unsupported request. Fail by stalling the control endpoint. */
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
            }

            if (eventFlag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT)
            {
                //CyU3PDebugPrint (4, "start a stream req. ctrl. wIndex 0x%x\r\n", wIndex);

                if (wIndex != CY_FX_UVC_STREAM_INTERFACE)
                {
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                }
                else
                {
                    UVCHandleVideoStreamingRqts ();
                }
            }

            /* handle interrupt status event */
            if (eventFlag & VD_FX_INT_STA_EVENT)
            {

            	//CyU3PDebugPrint (4, "start a interrupt req. ctrl. snap flag 0x%x\r\n", snapButFlag);
            	/** preparing interrupt status data **/
            	CyU3PGpioSimpleGetValue (SENSOR_SNAPSHOT_GPIO, valueptr);// get button value 1:release 0:press

				//CyU3PDebugPrint (4, "The interrupt event %d %d\r\n", testSnap, snapButFlag);
            }


#ifdef USB_DEBUG_INTERFACE
            if (eventFlag & CY_FX_USB_DEBUG_CMD_EVENT)
            {
                /* Get the command buffer */
                apiRetStatus = CyU3PDmaChannelGetBuffer (&glDebugCmdChannel, &dmaInfo, CYU3P_WAIT_FOREVER);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to receive debug command, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Decode the command from the command buffer, error checking is not implemented,
                 * so the command is expected to be correctly sent from the host application. First byte indicates
                 * read (0x00) or write (0x01) command. Second and third bytes are register address high byte and
                 * register address low byte. For read commands the fourth byte (optional) can be N>0, to read N
                 * registers in sequence. Response first byte is status (0=Pass, !0=Fail) followed by N pairs of
                 * register value high byte and register value low byte.
                 */
                CyU3PDebugPrint (4, "Debug interface conut %d data %d %d %d\r\n", dmaInfo.count, dmaInfo.buffer[0], dmaInfo.buffer[1], dmaInfo.buffer[2]); //additional debug
                if (dmaInfo.buffer[0] == 0)
                {
                    if (dmaInfo.count == 3)
                    {
                        /*glDebugRspBuffer[0] = SensorRead2B (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(glDebugRspBuffer+1));*/
                        dmaInfo.count = 3;
                    }
                    else if (dmaInfo.count == 4)
                    {
                        if (dmaInfo.buffer[3] > 0)
                        {
                                glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                                		(dmaInfo.buffer[3]*2), (glDebugRspBuffer+1));
                        }
                        dmaInfo.count = dmaInfo.buffer[3]*2+1;
                    }
                    CyU3PDebugPrint (4, "Debug responsR conut %d data %d %d %d\r\n", dmaInfo.count, glDebugRspBuffer[0], glDebugRspBuffer[1], glDebugRspBuffer[2]); //additional debug
                }
                /*  For write commands, the register address is followed by N pairs (N>0) of register value high byte
                 *  and register value low byte to write in sequence. Response first byte is status (0=Pass, !0=Fail)
                 *  followed by N pairs of register value high byte and register value low byte after modification.
                 */
                else if (dmaInfo.buffer[0] == 1)
                {
                        /*glDebugRspBuffer[0] = SensorWrite (SENSOR_ADDR_WR, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (dmaInfo.buffer+3));  original one*/
                        glDebugRspBuffer[0] = SensorWrite2B (SENSOR_ADDR_WR, dmaInfo.buffer[1], dmaInfo.buffer[2],
                                                		0x00, dmaInfo.buffer[3]); //additional debug
                        CyU3PDebugPrint (4, "Debug write %d data %d %d %d\r\n", dmaInfo.count, dmaInfo.buffer[2], dmaInfo.buffer[3], (dmaInfo.buffer+3));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;
                        /*glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (glDebugRspBuffer+1));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;*/
                    dmaInfo.count -= 2;
                }
                /* Default case, prepare buffer for loop back command in response */
                else
                {
                   /* For now, we just copy the command into the response buffer; and send it back to the
                      USB host. This can be expanded to include I2C transfers. */
                    CyU3PMemCopy (glDebugRspBuffer, dmaInfo.buffer, dmaInfo.count);
                    CyU3PDebugPrint (4, "Debug respons conut %d data %d %d %d\r\n", dmaInfo.count, glDebugRspBuffer[0], glDebugRspBuffer[1], glDebugRspBuffer[2]); //additional debug
                }

                dmaInfo.buffer = glDebugRspBuffer;
                dmaInfo.size   = 1024;
                dmaInfo.status = 0;

                /* Free the command buffer to receive the next command. */
                apiRetStatus = CyU3PDmaChannelDiscardBuffer (&glDebugCmdChannel);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to free up command OUT EP buffer, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Wait until the response has gone out. */
                CyU3PDmaChannelWaitForCompletion (&glDebugRspChannel, CYU3P_WAIT_FOREVER);

                apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glDebugRspChannel, &dmaInfo);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    CyU3PDebugPrint (4, "Failed to send debug response, Error code = %d\r\n", apiRetStatus);
                    CyFxAppErrorHandler (apiRetStatus);
                }
            }
#endif
        }
        /* Allow other ready threads to run. */
        CyU3PThreadRelinquish ();
    }
}
#endif

/*
 * Entry function for the internal I2C control handler thread.
 * added 10/2013
 */
/*
static uint8_t timeDelay[64] = {

};
*/
void SenAppThread_Entry(uint32_t input){

	uint16_t count = 0, cmdCopyIdx = 0, count1 = 0, cmdQuIdx = 0; //
    VdRingBuf *cmdQuptr = &cmdQu;
    VdRingBuf *statQuptr = &statQu;
	VdcmdDes  *lcCmdDes;
	VdcmdDes  *lcStaDes;
	uint32_t flag = 0;
	uint8_t  cmdFlag = 0;
	uint8_t regAdd, /*regAdd1,*/ devAdd, data;// data1;
	uint8_t i;
	uint16_t delaytime;
	//CyBool_t trigger = CyFalse;

#if 0 //for test the command queue
	lcCmdDes = cmdQuptr->startAdd;
	for(cmdQuIdx = 0; cmdQuIdx < MAXCMD; cmdQuIdx++){
		CyU3PDebugPrint (4, "Command Queue check cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d\r\n",
				lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, cmdQuIdx);
		lcCmdDes += 1;
	}
	lcCmdDes = statQuptr->startAdd;
	for(cmdQuIdx = 0; cmdQuIdx < MAXCMD; cmdQuIdx++){
		CyU3PDebugPrint (4, "State Queue check cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d\r\n",
				lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, cmdQuIdx);
		lcCmdDes += 1;
	}

#endif
/*** create a timer for I2C commands delay option ***/
	CyU3PTimerCreate(&I2CCmdTimer, I2CCmdCb, 20, 1000, 0, CYU3P_NO_ACTIVATE);
	CyU3PDebugPrint (4, "\r\nI2C per-timer %d", CyU3PGetTime());
	CyU3PThreadSleep(350);
	CyU3PTimerStart(&I2CCmdTimer);
	CyU3PDebugPrint (4, "\r\nI2C start timer %d", CyU3PGetTime());

	while(cmdQuptr->bugFlag == (uint8_t)CyFalse){ //waiting for first command
        /* Allow other ready threads to run. */
		CyU3PDebugPrint (4, "\r\nI2C timer %d", CyU3PGetTime());
        CyU3PThreadRelinquish ();
	}
	CyU3PDebugPrint (4, "The command queue is ready %d %d\r\n", cmdQuptr->bugFlag, cmdQuptr->readPtr->cmdFlag);
	//CamDefSet(); //set default parameters to camera
	/***** add recovery of the current camera settings ****/
	//CyU3PThreadSleep(100);
	//SetCurCmd();
	/*********** the loop of the thread ***********/
	for(;;){

		CyU3PEventGet (&glTimerEvent, VD_FX_I2C_CMD_EVENT, CYU3P_EVENT_AND_CLEAR, &flag, CYU3P_WAIT_FOREVER);//wait command event
/*  // for test GPIO output
		if(trigger)
		{
			CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyFalse);
			{
				CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n", CyFalse);
			}

		}else{
			CyU3PGpioSetValue(SENSOR_RESET_GPIO, CyTrue);
			{
				CyU3PDebugPrint(4, "GPIO Set Value Error, Error Code = %d\n", CyTrue);
			}

		}
*/
			CyU3PMutexGet(statQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
			//CyU3PDebugPrint (4, "get I2C events (0) flag 0x%x cmdflag 0x%x\r\n", flag, cmdFlag);
			/* find an available reading I2C command in the state queue */
			lcStaDes = (VdcmdDes*)statQuptr->readPtr;
			//if(0 && (lcStaDes->cmdFlag == CyTrue)){ /* for state queue it's not used right now. */
				i = 0;
				while((lcStaDes->cmdFlag == deswait) && (i < MAXCMD)){
					i++;
					lcCmdDes = lcStaDes->cmdDesNext;
					statQuptr->readPtr = lcStaDes;
				}
#if 0
				if(lcStaDes->cmdFlag != deswait){
				i = lcStaDes->curNum;
				regAdd = ((lcStaDes->CmdPar)+i)->RegAdd;
				devAdd = ((lcStaDes->CmdPar)+i)->DevAdd;
				data = ((lcStaDes->CmdPar)+i)->Data;
				//delaytime = ((lcStaDes->CmdPar)+i)->DelayT;

				//for(i = 0; i < lcStaDes->NumPara; i++){
					//regAdd = ((lcStaDes->CmdPar)+i)->RegAdd;
					//devAdd = ((lcStaDes->CmdPar)+i)->DevAdd;
					((lcStaDes->CmdPar)+i)->Data = SensorGetControl(regAdd, devAdd); //get state value from I2C bus
#ifdef USB_DEBUG_PRINT
					CyU3PDebugPrint (4, "send I2C state stateID %d cmdCopyIdx %d regAdd 0x%x devAdd 0x%x data 0x%x\r\n",
								lcStaDes->StatID, regAdd, devAdd, data);
#endif
				//}
				lcStaDes->cmdFlag = CyFalse;
				statQuptr->readPtr = (VdcmdDes*)lcStaDes->cmdDesNext; //update command queue read pointer
				cmdFlag = 0xFF; //I2C command done
				/* setting delay */
				delaytime = 300;
				CyU3PTimerModify(&I2CCmdTimer, delaytime, 0);
				CyU3PTimerStart(&I2CCmdTimer);  //start delay timer
			} //end of the if condition statment
#endif
			CyU3PMutexPut(statQuptr->ringMux);  //release the command queue mutex
			if(cmdFlag != 0xFF){ //for during handle command
				CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
				lcCmdDes = cmdQuptr->readPtr;

				/*
				CyU3PDebugPrint (4, "get I2C events (1) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
						flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
				*/

				/* find a available command */
				i = 0;
				while((lcCmdDes->cmdFlag == deswait) && (i < MAXCMD)){
					i++;
					lcCmdDes = lcCmdDes->cmdDesNext;
					cmdQuptr->readPtr = lcCmdDes;
				}
				//CyU3PDebugPrint (4, "i %d Cmf_Flag %d\r\n", i, lcCmdDes->cmdFlag);
				if(lcCmdDes->cmdFlag != deswait){//remove sensor set for WB camera
					i = lcCmdDes->curNum;
					regAdd = ((lcCmdDes->CmdPar)+i)->RegAdd;
					devAdd = ((lcCmdDes->CmdPar)+i)->DevAdd;
					data = ((lcCmdDes->CmdPar)+i)->Data;
					delaytime = ((lcCmdDes->CmdPar)+i)->DelayT;
#if 1
					switch(lcCmdDes->CmdID){
						case 0x20:
							SensorSetIrisControl(regAdd, devAdd, data, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris auto (AF Lens)
							delaytime = 500;
							break;
						case 0x21:
							SensorSetIrisControl(regAdd, devAdd, data, I2C_DSPBOARD_ADDR_WR/*boardID*/);//set Iris auto (non AF Lens)
							delaytime = 500;
							break;
						case 0x22:
							SensorSetIrisControl(regAdd, devAdd, data, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris value (DC manual)
							delaytime = 300;
							break;
						case 0x23:
							SensorSetIrisControl(regAdd, devAdd, data, I2C_AFBOARD_ADDR_WR/*boardID*/);//set Iris value (DC manual)
							delaytime = 300;
							break;
						default:
							SensorSetControl(regAdd, devAdd, data);    //send I2C command
							break;
					}
#endif
					//SensorSetControl(regAdd, devAdd, data);    //send I2C command
					/** timer's ticket modify **/
					//delaytime =100; //temp add -6/17/2015
					CyU3PTimerModify(&I2CCmdTimer, delaytime, 0);
					CyU3PTimerStart(&I2CCmdTimer);  //start delay timer
					//CyU3PDebugPrint (4, "set timer restart(1) %d 0x%x 0x%x %d %d %d %d\r\n", CyU3PGetTime(), regAdd, devAdd, data, delaytime, lcCmdDes->CmdID, i);
					cmdFlag = 0xFF; //I2C command done
//#ifdef USB_DEBUG_PRINT
					CyU3PDebugPrint (4, "\r\nsend I2C command cmdID %d regAdd 0x%x devAdd 0x%x data 0x%x cmdflag 0x%x",
							lcCmdDes->CmdID, regAdd, devAdd, data, lcCmdDes->cmdFlag);
//#endif
					if(lcCmdDes->NumPara == lcCmdDes->curNum){
						lcCmdDes->cmdFlag = deswait;
						if(lcCmdDes->CmdID >= 0x10){// TODO double check
							//ExUCtrlParArry[(lcCmdDes->CmdID-EXUAOFFSET)][16] = CyFalse;
							pEXTSenCtrl[lcCmdDes->CmdID - 0x10]->AvailableF = CyFalse;
						}else{
							//CtrlParArry[lcCmdDes->CmdID][16] = CyFalse; //set flag to false. wait for check.
							pPUCSenCtrl[lcCmdDes->CmdID]->AvailableF = CyFalse;
						}
						cmdQuptr->readPtr = lcCmdDes->cmdDesNext; //update command queue read pointer for next handled command
					}else{
						lcCmdDes->curNum ++;
						lcCmdDes->cmdFlag = desusing;
					}
				}else{
					CyU3PTimerModify(&I2CCmdTimer, 1000, 0);
					CyU3PTimerStart(&I2CCmdTimer);
					//CyU3PDebugPrint (4, "I2C thread beat pace 0x%x\r\n", 1000);
				}
			CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
			}
/*
			CyU3PDebugPrint (4, "get I2C events (2) flag 0x%x cmdflag 0x%x desflag 0x%x lcCmdDes 0x%x\r\n",
					flag, cmdFlag, lcCmdDes->cmdFlag, lcCmdDes);
*/
#ifdef USB_DEBUG_PRINT
			CyU3PDebugPrint (4, "I2C thread checking camera parameters count %d data0 %d data1 %d cmdflag 0x%x.\r\n",
						0/*count*/, CtrlParArry[count][13], CtrlParArry[count][14], cmdFlag);
#endif

			/**** checking the camera registers if it is the same what the current copy is. ****/
			/** this code might be used when a timer is used to schedule the I2C command sent out **/
#if 0
				if((CtrlParArry[cmdCopyIdx][16] != CyTrue)&&(cmdFlag != 0xFF)/*&&(CtrlParArry[cmdCopyIdx][17] != CyFalse)*/){ //checking register value

				regAdd = CtrlParArry[cmdCopyIdx][0];
			    regAdd1 = CtrlParArry[cmdCopyIdx][1];
			    devAdd = CtrlParArry[cmdCopyIdx][15];
			    data = SensorGetControl(regAdd, devAdd); //SensorGetBLCMode();
			    i = 0;
				 switch(cmdCopyIdx)
				 {
					 case BrgtCtlID1:
						 if (CtrlParArry[cmdCopyIdx][14] != data){
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][14], i);
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
							 i++;
						 }
						 else{
							 ;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
						 }

						 CyU3PBusyWait(500);
						 data =SensorGetControl(regAdd1, devAdd);
						 if (CtrlParArry[cmdCopyIdx][13] != data){
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, cmdCopyIdx, regAdd1, devAdd, CtrlParArry[cmdCopyIdx][13], i);
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						 }
						 else{
							 ;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
						 }
						 break;
					 case HueCtlID5:
						 if (CtrlParArry[cmdCopyIdx][13] != data){
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][13], i);
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						 }
						 else{
							 ;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
						 }
						 break;
					 case SaturCtlID6:
					 case WBTLevCtlID10:
					 default:
						 if (CtrlParArry[cmdCopyIdx][13] == data){
							 CyU3PMutexGet(cmdQuptr->ringMux, CYU3P_WAIT_FOREVER);       //get mutex
							 cmdSet(cmdQuptr, cmdCopyIdx, regAdd, devAdd, CtrlParArry[cmdCopyIdx][13], i);
							 CyU3PMutexPut(cmdQuptr->ringMux);  //release the command queue mutex
						 }
						 else{
							 ;//CtrlParArry[cmdCopyIdx][16] = CyTrue; //if they are the same, set flag is true.
						 }
						 break;
				 }
				 //cmdFlag = 0xFF; //one I2C command one available event.
				 CtrlParArry[cmdCopyIdx][16] = CyTrue; //set flag to true. let it sent to camera.
			}
			cmdCopyIdx = (cmdCopyIdx + 1 )& 0x1F;    //update checking index.
#endif
			cmdFlag = 0x00; //clear flag
		/* Allow other ready threads to run. */
			//CyU3PDebugPrint (4, "out of the i2cthread flag 0x%x cmdflag 0x%x\r\n", flag, cmdFlag);
			CyU3PThreadRelinquish ();
		}
}
