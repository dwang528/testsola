/************************************************************************************
*																					*
*	File: MLink.h																	*
*	Revision: 1																		*
*	Create Date: 04/28/2003  2:20PM													*
*	Last Modified: 10/22/2004  3:45AM												*
*	Author:																			*
*	Description:																	*
*		Header file for MLink.c.													*
*																					*
*************************************************************************************
*																					*
*							COPYRIGHT (C) 1998 - 2011								*
*	This software or any other copies thereof may not be provided or otherwise		*
*	made available to any other person not affiliated with the Company				*
*																					*
************************************************************************************/
#ifndef _MLINK_H_
#define _MLINK_H_

// Public definitions ==============================================================

#define MGMT_BRCM_LINK			0
#define MGMT_DCHAIN_LINK		2
#if defined(MCM_REVX0)
#define MGMT_RCM_LINK			0
#define MGMT_BMM_LINK			1
#define MGMT_DCHAINP_LINK		2
#define MGMT_DCHAINA_LINK		3
#endif // !defined(MCM_REVX0)
#define MGMT_MAX_LINKS			4

#define MGMT_SEND_RETRIES		10
//#define MGMT_PACKETARG_SIZE		254

// Channel selection
#define MGMT_MAIN_CHANNEL		0
#define MGMT_AUX_CHANNEL		1
#define MGMT_MAX_CHANNELS		2

// Public communications status, coarse states
#define MGMT_STATUS_IDLE		0
//#define MGMT_STATUS_DONE		1
//#define MGMT_STATUS_PAUSED		2
//#define MGMT_STATUS_WORKING		3
#define MGMT_STATUS_SENDING		4
#define MGMT_STATUS_RECVING		5
#define MGMT_STATUS_SNDDONE		6
#define MGMT_STATUS_RCVDONE		7

// End result values
#define MGMT_RESULT_NONE		0x00
#define MGMT_RESULT_SUCCESS		0x10
#define MGMT_RESULT_FAILURE		0x1F
#define MGMT_RESULT_ERRBUSY		0x18

// Public variables
extern const char* MGMT_DEVICE[MGMT_MAX_LINKS];
extern C_UBYTE gmgmtlnkStatus[MGMT_MAX_LINKS];
//extern C_UBYTE gmgmtRXINT;
//extern C_UBYTE gmgmtTXINT;


// Public Prototypes  ==============================================================

// Initializes the Management Link, returns TRUE on success.
C_UBYTE InitMgmtLnk(C_UBYTE tindex);

// Starts sending a packet through the Management Link, returns MGMT_RESULT_WORKING on start success.
C_UBYTE
StartMgmtLnkSend (C_UBYTE tindex);

// Starts receiving a packet from the Management Link, returns MGMT_RESULT_WORKING on start success.
C_UBYTE
StartMgmtLnkRecv (C_UBYTE tindex);

// Starts receiving a packet through the Management Link, returns TRUE on start success.

// Get the final completion status of send or recv through the Management Link, sendflag TRUE requests
// for send status completion and FALSE requests for receive status completion.
// returns one of the result values
C_UBYTE
UpdateMgmtLnkStatus(C_UBYTE tindex);

// Clear all the intermediate states and start all over
void
ResetMgmtLnkStatus(C_UBYTE tindex);

C_UBYTE
GetMgmtLnkAddress(C_UBYTE tindex);

C_UBYTE *
GetMgmtRecvPacket(C_UBYTE tindex);

C_UWORD
GetMgmtRecvPacketSize(C_UBYTE tindex);

C_UBYTE
GetMgmtRecvAddress(C_UBYTE tindex);

C_UBYTE *
GetMgmtSendPacket(C_UBYTE tindex, C_UBYTE largeflg);

C_UWORD
GetMgmtSendPacketSize(C_UBYTE tindex);

C_UBYTE
GetMgmtSendAddress(C_UBYTE tindex);

void
SendMgmtLnk(C_UBYTE tindex, C_UBYTE setuprecv, C_UBYTE blockflag, C_UBYTE envelopeflag);

C_UBYTE
RecvMgmtLnk(C_UBYTE tindex, C_UWORD linktimeout, C_UBYTE envelopeflag);

C_UBYTE
RecvRawMgmtLnk(C_UBYTE tindex, C_UWORD linktimeout, C_UBYTE *pbuf, C_UWORD bufsize);

int
CheckRecvMgmtLnk(C_UBYTE tindex);

void
PrepRecvMgmtLnk(C_UBYTE tindex);

void
FlushRecvMgmtLnk(C_UBYTE tindex);

#endif // _MLINK_H_


