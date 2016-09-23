/*********************************************************
 * The cmdqu.c for the ring buffer that can be used any case that needs a buffer.
 * Copyright Videology Imaging Solution Inc. 2013
 * All Rights Reserved
 *
 *  Created on: Oct 30, 2013
 *  Author: wcheng
 *
 *  it's used to create a ring buffer for the commands queue for buffering the host
 *  commands. it improves the commands performace.
 *********************************************************/

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3usb.h>
#include "uvc.h"
#include "sensor.h"
#include "cmdqu.h"

static uint16_t DelayArray[64] = {
		700, //0: BLC
		260, //1: Brightness
		260, //2: Contrast
		260, //3: 0
		700, //4: MainFreq
		260, //5: Hue
		260, //6: Saturation
		260, //7: Sharpness
		260, //8: 0
		260, //9: WBMode
		260, //A: 0
		260, //B: WBComp
		260, //C: 0
		260, //D: 0
		260, //E: DigZoom
		260, //F: 0
		700, //10: Shutter
		700, //11: SenseUp
		400, //12: MirrMode
		300, //13: NoiRedu3DMod
		300, //14: NoiRedu3DLev
		300, //15: DayNightMod
		300, //16: DayNightDly
		260, //17: DayNightLev
		260, //18: NightDayLev
		700, //19: AExModee
		700, //1A: AExReferleve
		260, //1B: 0
		700, //1C: SensorMode
		260, //1D: 0
		500, //1E: SevePars
		260, //1F: 0
		260, //20: Iris auto (AF Lens)
		260, //21: Iris auto (non AF Lens)
		400, //22: Iris value (DC manual)
		400, //23: Iris value (DC manual)
		400, //24: BLCRange
		260, //25: BLCWeight
		260, //26: BLCGrid
		260, //27: 0
		260, //28: 0
		260, //29: 0
		260, //2A: 0
		260, //2B: 0
		260, //2C: 0
		260, //2D: 0
		260, //2E: 0
		260, //2F: 0 end right now
		260, //30: 0
		0

};

void creatqu(uint8_t para){
	CyU3PDebugPrint (4, "The test cmdqu %d \r\n", para); // additional debug
}

/**** it's used test the queue data structure. */
void  cmdquTest(VdRingBuf *quebuf, uint8_t state){
	uint16_t QuIdx = 0;
	VdcmdDes *lcCmdDes;
	CyU3PDebugPrint (4, "Queue %s state %d\r\n", quebuf->bufferName, state);
	CyU3PDebugPrint (4, "Queue check queueID %d startAdd 0x%x endAdd 0x%x write 0x%x read 0x%x queueFlag %d\r\n",
			quebuf->ringbufID, quebuf->startAdd, quebuf->endAdd, quebuf->writePtr, quebuf->readPtr, quebuf->bugFlag);
	lcCmdDes = quebuf->startAdd;
	for(QuIdx = 0; QuIdx < 0x10/*MAXCMD*/; QuIdx++){
		CyU3PDebugPrint (4, "Queue check cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d cmdflag %d\r\n",
				lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, QuIdx, lcCmdDes->cmdFlag);
		lcCmdDes += 1;
	}

	return;
}

/***** create a command buffer. *******/
VdRingBuf  cmdbufCreate(uint16_t size, char * name, uint8_t id, CyU3PMutex *muxPtr){
	VdRingBuf cmdque;

	cmdque.startAdd = CyU3PMemAlloc(sizeof(VdcmdDes)*(size));    //allocate memory for command queue which can be put 256 commands
	cmdque.bugFlag = CyFalse;  // set command queue unavailable.
	cmdque.bufferName = name; //"I2C command queue";
	cmdque.ringbufID = id; //CMDQU0;
	cmdque.numUnit = size;
	cmdque.endAdd = cmdque.startAdd + size;  //the read pointer is initialed one command unit behind of write pointer
	//cmdque.ringMux = CyU3PMemAlloc(sizeof(CyU3PMutex));
	cmdque.ringMux = muxPtr;
	CyU3PMutexCreate(cmdque.ringMux, CYU3P_NO_INHERIT);
	return cmdque;
}

/**** initialize the command queue  *****/
void  cmdquInit(VdRingBuf *cmdqu){
	uint16_t cmdQuIdx = 0;
	VdcmdDes *lcCmdDes;

	for(lcCmdDes = cmdqu->startAdd; lcCmdDes <= cmdqu->endAdd; lcCmdDes++){
		lcCmdDes->CmdID = cmdQuIdx;
		lcCmdDes->cmdFlag = deswait;            //initial the command unavailable
		lcCmdDes->cmdDesNext = cmdqu->startAdd + (uint16_t)((cmdQuIdx + 1)&0x3F);
		lcCmdDes->cmdDesPrevious = cmdqu->startAdd + (uint16_t)((cmdQuIdx - 1)&0x3F);
		if(1/*!cmdQuIdx debug*/) 		CyU3PDebugPrint (4, "Command Queue init 0 cmdID %d CmdDes 0x%x previous 0x%x next 0x%x Idx %d cmdflag %d\r\n",
				lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, 0, lcCmdDes->cmdFlag);
		cmdQuIdx ++;
	}
#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "send a I2C command(11) readptr 0x%x next 0x%x previous 0x%x local 0x%x\r\n",
				cmdqu->readPtr, cmdqu->readPtr->cmdDesNext, cmdqu->readPtr->cmdDesPrevious, lcCmdDes);
	CyU3PDebugPrint (4, "send a I2C command(11) writeptr 0x%x next 0x%x previous 0x%x\r\n",
			cmdqu->writePtr, cmdqu->writePtr->cmdDesNext, cmdqu->writePtr->cmdDesPrevious);
	CyU3PDebugPrint (4, "send a I2C command(11) start 0x%x next 0x%x previous 0x%x\r\n",
			cmdqu->startAdd, cmdqu->startAdd->cmdDesNext, cmdqu->startAdd->cmdDesPrevious);
#endif
	cmdqu->readPtr = cmdqu->startAdd;
	CyU3PThreadSleep(10);
	cmdqu->writePtr = cmdqu->readPtr;
	CyU3PThreadSleep(10);
	cmdqu->bugFlag = (uint8_t)CyTrue; //command queue available.
#ifdef USB_DEBUG_PRINT
	CyU3PDebugPrint (4, "send a I2C command(12) readptr 0x%x next 0x%x previous 0x%x local 0x%x\r\n",
				cmdqu->readPtr, cmdqu->readPtr->cmdDesNext, cmdqu->readPtr->cmdDesPrevious, lcCmdDes);
	CyU3PDebugPrint (4, "send a I2C command(12) writeptr 0x%x next 0x%x previous 0x%x\r\n",
			cmdqu->writePtr, cmdqu->writePtr->cmdDesNext, cmdqu->writePtr->cmdDesPrevious);
	CyU3PDebugPrint (4, "send a I2C command(12) start 0x%x next 0x%x previous 0x%x\r\n",
			cmdqu->startAdd, cmdqu->startAdd->cmdDesNext, cmdqu->startAdd->cmdDesPrevious);
#endif
	return;
}

CyBool_t  cmdbufDestroy(VdRingBuf *cmdqu){
	;  //na
	return CyTrue;
}

/******* set a command into command queue based on the command ID *******/
CyBool_t  cmdSet(VdRingBuf *cmdqu, uint8_t cmdID, uint8_t RegAdd, uint8_t DevAdd, uint8_t Data, uint8_t dataIdx){
	VdcmdDes *lcCmdDes;

	lcCmdDes = cmdqu->startAdd; //get a command descriptor
	lcCmdDes = lcCmdDes + cmdID;
	if(lcCmdDes->cmdFlag != desusing){
		((lcCmdDes->CmdPar)+dataIdx)->DevAdd = DevAdd;
		((lcCmdDes->CmdPar)+dataIdx)->RegAdd = RegAdd;
		((lcCmdDes->CmdPar)+dataIdx)->Data = Data;
		((lcCmdDes->CmdPar)+dataIdx)->DelayT = DelayArray[cmdID];
		lcCmdDes->NumPara = dataIdx;
		lcCmdDes->curNum = 0;
		lcCmdDes->cmdFlag = desReady;
#ifdef USB_DEBUG_PRINT
		CyU3PDebugPrint (4, "\r\nCommand Queue set cmdID %d CmdDes 0x%x next 0x%x cmdflag %d dataIdx %d data 0x%x",
					lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesNext, lcCmdDes->cmdFlag, dataIdx, Data);
#endif
	}else{
#ifdef USB_DEBUG_PRINT
		CyU3PDebugPrint (4, "Command Queue set fail as transferring cmdID %d CmdDes 0x%x\r\n",
					lcCmdDes->CmdID, lcCmdDes);
#endif
	}
	return CyTrue;
}

#if 0 // alternate putting queue approach
/******* set a command into command queue based on write pointer *******/
CyBool_t  cmdSet(VdRingBuf *cmdqu, uint8_t RegAdd, uint8_t DevAdd, uint8_t Data){
	VdcmdDes *lcCmdDes;

	lcCmdDes = cmdqu->writePtr; //get a command descriptor
	CyU3PDebugPrint (4, "Command Queue init 0 cmdID %d CmdDes 0x%x previous 0x%x next 0x%x qunext 0x%x cmdflag %d\r\n",
					lcCmdDes->CmdID, lcCmdDes,	lcCmdDes->cmdDesPrevious, lcCmdDes->cmdDesNext, cmdqu->writePtr->cmdDesNext, lcCmdDes->cmdFlag);
	//cmdquTest(cmdqu, 0);
	if(lcCmdDes == cmdqu->readPtr){//at first state.
		if(lcCmdDes->cmdFlag == deswait){//the writing available
			lcCmdDes->DevAdd = DevAdd;
			lcCmdDes->RegAdd = RegAdd;
			lcCmdDes->Data[0] = Data;
			lcCmdDes->curNum = 0;
			lcCmdDes->cmdFlag = desReady;
#ifdef USB_DEBUG_PRINT
			CyU3PDebugPrint (4, "CmdQu first state cmdID %d CmdDes 0x%x reader 0x%x next 0x%x\r\n",
					lcCmdDes->CmdID, lcCmdDes, cmdqu->readPtr, lcCmdDes->cmdDesNext);
#endif
			cmdqu->writePtr = lcCmdDes->cmdDesNext;

		}else{ //queue fullness
			CyU3PDebugPrint (4, "CmdQu fullness cmdID %d CmdDes 0x%x reader 0x%x\r\n",
					lcCmdDes->CmdID, lcCmdDes, cmdqu->readPtr);
			return 0;
		}
	}else{ //in normal state
		//the command descriptor is available.
		if(lcCmdDes->cmdFlag == dewait){
			lcCmdDes->DevAdd = DevAdd;
			lcCmdDes->RegAdd = RegAdd;
			lcCmdDes->Data[0] = Data;
			lcCmdDes->curNum = 0;
			lcCmdDes->cmdFlag = desReady;
#ifdef USB_DEBUG_PRINT
			CyU3PDebugPrint (4, "Command descriptor setting cmdID %d nextID %d data %d cmdAdd 0x%x flag %d %d\r\n",
					lcCmdDes->CmdID, lcCmdDes->cmdDesNext->CmdID, lcCmdDes->Data[0], lcCmdDes, cmdqu->writePtr->cmdFlag, (lcCmdDes+1)->cmdFlag);
#endif
			cmdqu->writePtr = lcCmdDes->cmdDesNext; // update the command queue writer pointer.
		}else{//reset writer
			while(lcCmdDes != cmdqu->readPtr){
				lcCmdDes = lcCmdDes->cmdDesNext; //update cmdDes.
				if(lcCmdDes->cmdFlag == dewait){
					lcCmdDes->DevAdd = DevAdd;
					lcCmdDes->RegAdd = RegAdd;
					lcCmdDes->Data[0] = Data;
					lcCmdDes->curNum = 0;
					lcCmdDes->cmdFlag = desReady;
#ifdef USB_DEBUG_PRINT
					CyU3PDebugPrint (4, "Command descriptor setting cmdID %d nextID %d data %d cmdAdd 0x%x flag %d %d\r\n",
							lcCmdDes->CmdID, lcCmdDes->cmdDesNext->CmdID, lcCmdDes->Data[0], lcCmdDes, cmdqu->writePtr->cmdFlag, (lcCmdDes+1)->cmdFlag);
#endif
					break;
				}
			}
			if(lcCmdDes != cmdqu->readPtr)
				cmdqu->writePtr = lcCmdDes->cmdDesNext; //update writer.
			else cmdqu->writePtr = cmdqu->readPtr; //reset writer.
		}
	}

	return CyTrue;
}
#endif

/******* get a state from camera register. *******
 * it might be unused, if state request performs immediately
 *****/
void statGet(VdRingBuf *statqu, uint8_t statID){
#if 1 // copy
	VdstateDes *lcStatDes;
	uint8_t Data0, Data1;
	uint8_t sendData[2];

	lcStatDes = (VdstateDes*)((statqu->startAdd) + statID);//get a state descriptor, and find any available state to send to host

	if(lcStatDes->statFlag == 0x0F){//statFlag: 0x00:noCom; 0xFF:cmdready; 0x0F:state ready.
		switch(statID){
			case BrgtCtlID1:
					Data0 = lcStatDes->staPar->Data;
					Data1 = ((lcStatDes->staPar)+1)->Data;
					//CtrlParArry[CtrlID][13] = Data0;
					//CtrlParArry[CtrlID][14] = Data1;
					if (Data1&0x2){ //check the sign bit (bit1)
						Data1 = ((Data1<<6)&0x40)| (Data0 >> 2);//clear MSB
					}else{
						Data1 = ((Data1<<6)|0x80)| (Data0 >> 2);//set MSB
					}
					glEp0Buffer[0] = Data1;
					glEp0Buffer[1] = 0;
					break;
				case HueCtlID5:
					Data0 = lcStatDes->staPar->Data;
					glEp0Buffer[0] = Data0 + GREEN_BASE;
					sendData[1] = 0;
					break;
				//case SaturCtlID6:
				//case WBTLevCtlID10:
				default:
					glEp0Buffer[0] = lcStatDes->staPar->Data;
					glEp0Buffer[1] = 0;
					break;
			 }
			  CyU3PUsbSendEP0Data (2, (uint8_t *)glEp0Buffer);
#ifdef USB_DEBUG_PRINT
			  CyU3PDebugPrint (4, "Command Queue init 0 cmdID %d CmdDes 0x%x previous 0x%x next 0x%x qunext 0x%x cmdflag %d sendData %d\r\n",
					  lcStatDes->StatID, lcStatDes, lcStatDes->staDesPrevious, lcStatDes->staDesNext, statqu->writePtr->cmdDesNext, lcStatDes->statFlag, sendData[0]);
#endif
		}
		lcStatDes++;
#endif
	return;
}

/******* find a command form command queue ********/
VdcmdDes *cmdFind(VdRingBuf *cmdqu, uint8_t cmdID){
	VdcmdDes *cmd = cmdqu->startAdd;
	;  //na
	return cmd;
}

//end of the cmdqu.c
