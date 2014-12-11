/************************************************************************************
*																					*
*	File: MLink.c																	*
*	Revision: 1																		*
*	Create Date: 04/28/2003  3:20PM													*
*	Last Modified:  5/16/2011  2:45PM												*
*	Author:																			*
*	Description:																	*
*		Access to the serial link for management.									*
*																					*
*************************************************************************************
*																					*
*							COPYRIGHT (C) 1998 - 2011								*
*																					*
*							-- ALL RIGHTS RESERVED --								*
*																					*
*	This software or any other copies thereof may not be provided or otherwise		*
*	made available to any other person not affiliated with the Company				*
*																					*
*	Confidential and/or Proprietary Information.									*
*																					*
************************************************************************************/

// Test Switches       ==============================================================

// Public definitions & prototypes (include files) ==================================
#ifdef WIN32
#include "winstubincs.h"
#include "winsomioctl.h"
#else // not WIN32
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#include <stdint.h>
#include <errno.h>
#if !defined(SOMHOST)
// SOM's ioctl header (in kernel tree)
#include "som/somioctl.h"
#endif // !defined(SOMHOST)
#endif // WIN32
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "cldefs.h"
#include "epigvars.h"
#include "mlcmds.h"
#include "mlink.h"
#include "crc.h"
#include "epitools.h"


int SerialRead (int fd, unsigned char *pBuffer, size_t bufferSize)
{
#if defined(SOMHOST)
	memset(pBuffer, 0, bufferSize);
	return 0;
#else
	return read(fd, pBuffer, bufferSize);
#endif // defined(SOMHOST)
}

C_BOOL SerialWrite (int fd, const void *pData, size_t dataSize)
{
#if defined(SOMHOST)
	return TRUE;
#else
	if (write(fd, pData, dataSize) == dataSize)
		return TRUE;

	return FALSE;
#endif // defined(SOMHOST)
}

int SerialIsReceiveDataAvailable (int fd)
{
#if defined(SOMHOST)
	return 0;
#else
	int bytes = 0;
	int ret = ioctl(fd, FIONREAD, &bytes);
	if (ret < 0)
	{
		epi_printf("%s: failed ioctl, errno=%d\n", __FUNCTION__, errno);
		return ret;
	}

	return bytes;
#endif // defined(SOMHOST)
}

// Public variables	   ==============================================================
C_UBYTE gmgmtlnkStatus[MGMT_MAX_LINKS];
//C_UBYTE gmgmtRXINT;
C_UBYTE gmgmtTXINT[MGMT_MAX_LINKS];

// Private definitions ==============================================================

#define RCM_MGMT_DEVICE					"/dev/ttyS2"
#define BRCM_MGMT_DEVICE				"/dev/ttyS2"
#define BMM_MGMT_DEVICE					"/dev/ttyS4"
#define DCHAIN_MGMT_DEVICE				"/dev/ttyS1"
#define DCHAINP_MGMT_DEVICE				"/dev/ttyS1"
#define DCHAINA_MGMT_DEVICE				"/dev/ttyS3"

const char* MGMT_DEVICE[MGMT_MAX_LINKS] = {
	RCM_MGMT_DEVICE,		// alias to BRCM_MGMT_DEVICE
	BMM_MGMT_DEVICE,
	DCHAINP_MGMT_DEVICE,	// alias to DCHAIN_MGMT_DEVICE
	DCHAINA_MGMT_DEVICE
};

// Enable USART RX interrupt
#define ENABLE_RXINT()		gmgmtRXINT[tindex] = 1

// Enable USART TX interrupt
#define ENABLE_TXINT()		gmgmtTXINT[tindex] = 1

// Disable USART RX interrupt
#define DISABLE_RXINT()		gmgmtRXINT[tindex] = 0

// Disable USART TX interrupt
#define DISABLE_TXINT()		gmgmtTXINT[tindex] = 0

// Set transmit address byte
//#define SET_SENDADDR()		

// Set receive address byte
//#define SET_RECVADDR()		

// Set receive any byte
//#define SET_RECVANY()		

// Clear receive register
//#define CLR_RECVREG()		{C_UBYTE dummy=RX_RECVREG();}

//#define TX_SENDREG(d)		putch(d)
//#define RX_RECVREG()		getch()

typedef struct
{
	C_UBYTE myaddr;		// my assigned hardware address/ID
	C_UBYTE state;		// state of send/receive of the packet (internal use only)
	C_UBYTE retries;	// number of retries attempted on this packet (internal use only)
	
	C_UBYTE sresult;	// result of send
	C_UBYTE saddr;		// module address to send/receive to/from
	C_UBYTE scmd;		// command code on send
	C_UWORD sargsize;	// size of valid argument information in the send argbuf buffer
	C_UBYTE sbuf[MGMT_MAXARG_SIZE + MGMT_PACKOVHD_SIZE];		// send buffer
	C_UBYTE lsbuf[MGMT_XLARGEARG_SIZE + MGMT_PACKOVHD_SIZE];	// large send buffer
	C_UBYTE uselsbuf;	// flag indicates use of large send buffer
	C_UBYTE scount;		// count of argument being sent
	C_UWORD scrc;		// crc of the entire send packet: addr, cmdrsp, argsize, argbuf
	
	C_UBYTE rresult;	// result of receive
	C_UBYTE raddr;		// module address to send/receive to/from
	C_UBYTE rcmd;		// command code on receive
	C_UBYTE rcount;		// count of argument being received
	C_UWORD rargsize;	// size of valid argument information in the receive argbuf buffer
	C_UBYTE rbuf[MGMT_MAXARG_SIZE + MGMT_PACKOVHD_SIZE];		// send buffer
	C_UBYTE lrbuf[MGMT_XLARGEARG_SIZE + MGMT_PACKOVHD_SIZE];	// large receive buffer
	C_UBYTE uselrbuf;	// flag indicates use of large receive buffer
	C_UWORD rcrc;		// crc of the entire receive packet: addr, cmdrsp, argsize, argbuf
	
	C_UWORD tmpcrc;		// temp crc for comparison

	int ttySfd;			// handle for corresponding ttySx port
	
} MGMTLINKS;

// detailed states of the communication link
#define MGMT_STATE_IDLE			0

#define MGMT_STATE_SNDADDR		1
#define MGMT_STATE_SNDCMDR		2
#define MGMT_STATE_SNDACNT		3
#define MGMT_STATE_SNDARGS		4
#define MGMT_STATE_SNDCRCH		5
#define MGMT_STATE_SNDCRCL		6
#define MGMT_STATE_SNDDONE		7

#define MGMT_STATE_RCVADDR		10
#define MGMT_STATE_RCVCMDR		11
#define MGMT_STATE_RCVACNT		12
#define MGMT_STATE_RCVARGS		13
#define MGMT_STATE_RCVCRCH		14
#define MGMT_STATE_RCVCRCL		15
#define MGMT_STATE_RCVDONE		16

#define MGMT_STATE_COMPLETE		20
#define MGMT_STATE_RETRY		30

#define MGMT_PACKPOS_ADDR		0
#define MGMT_PACKPOS_CMDRSP		1
#define MGMT_PACKPOS_ARGCNT		2
#define MGMT_PACKPOS_ARGBAS		3

// Private Prototypes  ==============================================================


// Private variables   ==============================================================
static MGMTLINKS mlnk[MGMT_MAX_LINKS];



// Public routines	   ==============================================================

// One time initialization of variables and hardware
C_UBYTE
InitMgmtLnk(C_UBYTE tindex)
{
	struct termios tm;
	MGMTLINKS *pml = &mlnk[tindex];
	const char *dev = MGMT_DEVICE[tindex];

	memset (pml, 0, sizeof(MGMTLINKS));

	pml->state = MGMT_STATE_IDLE;
	pml->ttySfd = -1;
	pml->rbuf[0] = 0;
	pml->sresult = MGMT_RESULT_NONE;
	pml->rresult = MGMT_RESULT_NONE;
	gmgmtlnkStatus[tindex] = MGMT_STATUS_IDLE;

	// initialize hw dependent stuff
//FIXME	DISABLE_RXINT();
	DISABLE_TXINT();

	pml->ttySfd = open(dev, O_RDWR | O_NOCTTY);
	if (pml->ttySfd < 0)
		epi_printf ("%s() ********* Failed open[%s]", __FUNCTION__, dev);
	else
		epi_printf ("%s() ********* Succeded open[%s]", __FUNCTION__, dev);
	fcntl(pml->ttySfd, F_SETFL, 0);
	if (tcgetattr(pml->ttySfd, &tm) < 0)
	{
		epi_printf("%s(): tcgetattr() failed!\n", __FUNCTION__);
	}
	// tm.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);	// turn to raw mode
	tm.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// turn to raw mode
	//tm.c_lflag |= IEXTEN;
	//tm.c_lflag &= ~(ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | PENDIN);	// turn to raw mode
	tm.c_oflag &= ~(OPOST | ONLCR);
	// tm.c_iflag = IGNPAR;
	tm.c_iflag = IGNPAR;
	tm.c_cflag = (CLOCAL | CREAD | CS8);
	/*
#define SOM_MLINK_SPEED		B115200
	if (cfsetispeed(&tm, SOM_MLINK_SPEED) < 0)
	{
		epi_printf("%s(): cfsetispeed() failed!\n", __FUNCTION__);
	}
	if (cfsetospeed(&tm, SOM_MLINK_SPEED) < 0)
	{
		epi_printf("%s(): cfsetospeed() failed!\n", __FUNCTION__);
	}
	*/
	memset (tm.c_cc, 0, sizeof(tm.c_cc));
	tm.c_cc[VMIN] = 0;
	//tm.c_cc[VTIME] = 100;
	tm.c_cc[VTIME] = 1;
	tcflush(pml->ttySfd, TCIFLUSH);
	if (tcsetattr(pml->ttySfd, TCSANOW, &tm) < 0)
	{
		epi_printf("%s(): tcsetattr() failed!\n", __FUNCTION__);
	}
	epi_printf("termios: iflag=%o, lflag=%o, oflag=%o, cflag=%o, ccvmin=%d, ccvtime=%d\n",
		tm.c_iflag, tm.c_lflag, tm.c_oflag, tm.c_cflag, tm.c_cc[VMIN], tm.c_cc[VTIME]);

#define SETLOWLATENCY
#if defined(SETLOWLATENCY)
	{
		struct serial_struct serinfo;
		ioctl(pml->ttySfd, TIOCGSERIAL, &serinfo);
		serinfo.flags |= ASYNC_LOW_LATENCY;
		if (ioctl(pml->ttySfd, TIOCSSERIAL, &serinfo) >= 0) 
		{
			epi_printf("%s(): set low latency.\n", __FUNCTION__);
		}
		else
		{
			epi_printf("%s(): failed to set low latency! errno %d [%s].\n",
				__FUNCTION__, errno, strerror(errno));
		}
	}
#endif // defined(SETLOWLATENCY)

	return (TRUE);
}

// Start the sending task by providing with a send packet
C_UBYTE
StartMgmtLnkSend (C_UBYTE tindex)
{
	MGMTLINKS *pml = &mlnk[tindex];

//FIXME	DISABLE_RXINT();
	DISABLE_TXINT();

	pml->state = MGMT_STATE_SNDADDR;
	pml->retries = 0;
	pml->scount = 0;
	if (pml->uselsbuf)
	{
		pml->saddr = pml->lsbuf[MGMT_PACKADDR_POS];
		pml->scmd = pml->lsbuf[MGMT_PACKCMD_POS];
		if (pml->lsbuf[MGMT_PACKCNT_POS] == 0xFF)
			pml->sargsize = MGMT_XLARGEARG_SIZE;
		else
			pml->sargsize = pml->lsbuf[MGMT_PACKCNT_POS] << 4;
	}
	else
	{
		pml->saddr = pml->sbuf[MGMT_PACKADDR_POS];
		pml->scmd = pml->sbuf[MGMT_PACKCMD_POS];
		if (pml->sbuf[MGMT_PACKCNT_POS] == 0xFF)
			pml->sargsize = MGMT_MAXARG_SIZE;
		else
			pml->sargsize = pml->sbuf[MGMT_PACKCNT_POS];
	}
	pml->scrc = 0;
	pml->sresult = MGMT_RESULT_NONE;
	gmgmtlnkStatus[tindex] = MGMT_STATUS_SENDING;

	ENABLE_TXINT();
	tcflush(pml->ttySfd, TCOFLUSH);

	return (MGMT_STATUS_SENDING);
}

// Start the receiving task, it will allocate the needed receive packet buffer
C_UBYTE
StartMgmtLnkRecv (C_UBYTE tindex)
{
	MGMTLINKS *pml = &mlnk[tindex];

//FIXME	DISABLE_RXINT();
	DISABLE_TXINT();
	tcflush(pml->ttySfd, TCIFLUSH);

	pml->state = MGMT_STATE_RCVADDR;
	pml->retries = 0;
	pml->raddr = 0;
	pml->rcount = 0;
	pml->rcmd = 0;
	pml->rargsize = 0;
	pml->rcrc = 0;
	pml->rbuf[0] = 0;
	pml->uselrbuf = 0;
	pml->rresult = MGMT_RESULT_NONE;
	gmgmtlnkStatus[tindex] = MGMT_STATUS_RECVING;

//	SET_RECVANY();

//	CLR_RECVREG();
//	tcflush(pml->ttySfd, TCIFLUSH);

//FIXME	ENABLE_RXINT();

	return (MGMT_STATUS_RECVING);
}


// get the final completion status and helps transition between sending to receiving states
// and viceversa.  At the end a call resets all status and states variables
// to a fresh idle state and allow new starts to occur. 
C_UBYTE
UpdateMgmtLnkStatus(C_UBYTE tindex)
{
	C_UBYTE result = MGMT_RESULT_NONE;
	MGMTLINKS *pml = &mlnk[tindex];

	if (pml->state == MGMT_STATE_RCVDONE)
	{
		result = pml->rresult;
	}
	else if (pml->state == MGMT_STATE_SNDDONE)
	{
		// finish the receive send cycle go back to idle
		pml->state = MGMT_STATE_IDLE;
		gmgmtlnkStatus[tindex] = MGMT_STATUS_IDLE;
		result = pml->sresult;
	}
	return (result);
}


// Clear the link status and prep for new start
void
ResetMgmtLnkStatus(C_UBYTE tindex)
{
	MGMTLINKS *pml = &mlnk[tindex];

	pml->state = MGMT_STATE_IDLE;
	pml->rbuf[0] = 0;
	pml->sbuf[0] = 0;
	gmgmtlnkStatus[tindex] = MGMT_STATUS_IDLE;
	
	// what about the large buffer management???
}

C_UBYTE
GetMgmtLnkAddress(C_UBYTE tindex)
{
	return (mlnk[tindex].myaddr);
}

C_UBYTE *
GetMgmtRecvPacket(C_UBYTE tindex)
{
	if (mlnk[tindex].uselrbuf)
		return (mlnk[tindex].lrbuf);
	else
		return (mlnk[tindex].rbuf);
}

C_UWORD
GetMgmtRecvPacketSize(C_UBYTE tindex)
{
	if (mlnk[tindex].uselrbuf)
		return (MGMT_XLARGEARG_SIZE);
	else
		return (MGMT_MAXARG_SIZE);
}

C_UBYTE
GetMgmtRecvAddress(C_UBYTE tindex)
{
	return (mlnk[tindex].raddr);
}

C_UBYTE *
GetMgmtSendPacket(C_UBYTE tindex, C_UBYTE largeflg)
{
	if (largeflg)
	{
		mlnk[tindex].uselsbuf = 1;
		return (mlnk[tindex].lsbuf);
	}
	else
	{
		mlnk[tindex].uselsbuf = 0;
		return (mlnk[tindex].sbuf);
	}
}

C_UWORD
GetMgmtSendPacketSize(C_UBYTE tindex)
{
	if (mlnk[tindex].uselsbuf)
		return (MGMT_XLARGEARG_SIZE);
	else
		return (MGMT_MAXARG_SIZE);
}

C_UBYTE
GetMgmtSendAddress(C_UBYTE tindex)
{
	return (mlnk[tindex].saddr);
}

// Private routines	   ==============================================================

// for debug only
// #define MLINK_DUMPDATA

void
SendMgmtLnk(C_UBYTE tindex, C_UBYTE setuprecv, C_UBYTE blockflag, C_UBYTE envelopeflag)
{
	MGMTLINKS *pml = &mlnk[tindex];
	C_UWORD len;
	C_UBYTE envelbuf[3];
	C_UBYTE *psbuf;

	len = pml->sargsize + MGMT_PACKOVHD_SIZE;
	if (pml->uselsbuf)
	{
#if defined(MLINK_DUMPDATA)
// debug
if (pml->lsbuf[MGMT_PACKPOS_CMDRSP] == MGMT_FIELDS_LSET) 
{
char f[32];
epi_dumpfile(som_tsfilename(f), pml->lsbuf, len);
//epi_dump_buffer(__FUNCTION__, pml->lsbuf, len);
}
#endif // defined(MLINK_DUMPDATA)

		psbuf = pml->lsbuf;
//epi_printf("*S*%x-%d\n", psbuf[MGMT_PACKPOS_CMDRSP], pml->sargsize);
	}
	else
		psbuf = pml->sbuf;
	pml->scrc = AccCRC(0, psbuf, (C_UWORD) (len - 2));
	// big endian?
	psbuf[len - 2] = (pml->scrc >> 8) & 0xFF;
	psbuf[len - 1] = pml->scrc & 0xFF;
	// disable interrupts to make sure the whole packet is sent together
	// in the same time slice.  Undesireable to disable all interrupts
	// but there is no other way to disable the serial port xmit only
	// InterruptDisable();
	if (setuprecv)
		StartMgmtLnkRecv (tindex);
	tcflush(pml->ttySfd, TCOFLUSH);
	
	if (envelopeflag)
	{ // send the leader and byte count
		envelbuf[0] = MGMT_RAWPREFIX1;
		envelbuf[1] = MGMT_RAWPREFIX2;
		envelbuf[2] = (C_UBYTE) len;
		SerialWrite (pml->ttySfd, envelbuf, 3);
	}

	SerialWrite (pml->ttySfd, psbuf, len);

//epi_printf ("%s(%d) send data %d bytes\n", __FUNCTION__, __LINE__, len);
	if (envelopeflag)
	{ // send the trailer
		envelbuf[0] = MGMT_RAWSUFFIX1;
		envelbuf[1] = MGMT_RAWSUFFIX2;
		SerialWrite (pml->ttySfd, envelbuf, 2);
//epi_printf ("%s(%d) send suffix 2 bytes\n", __FUNCTION__, __LINE__);
	}
	// InterruptEnable();
	DISABLE_TXINT();
	pml->sresult = MGMT_RESULT_SUCCESS;
	if (!setuprecv)
	{
		pml->state = MGMT_STATE_SNDDONE;
		gmgmtlnkStatus[tindex] = MGMT_STATUS_SNDDONE;
	}
	/*
	if (len >= 4096)
	{
		static int findex = 0;
		char filename[64];
		sprintf (filename, "/tmp/send%d", findex++);
		epi_dumpfile(filename, psbuf, len);
		//epi_dump_buffer(__FUNCTION__, psbuf, len);
	}
	*/
	if (blockflag)
	{	// wait until all the bytes have exited the hardware port, i.e. the xmit buffer is empty
		tcdrain(pml->ttySfd);
		epi_msleep(1);	// wait for character in shift register to be sent out
//epi_printf ("%s(%d) waited for xmit drain before exiting\n", __FUNCTION__, __LINE__);
	}
}

#define DEBUG_RECVMGMTLNK

// return 0 if no data received else 1
C_UBYTE
RecvMgmtLnk(C_UBYTE tindex, C_UWORD linktimeout, C_UBYTE envelopeflag)
{
#ifdef DEBUG_RECVMGMTLNK
	static int goodcnt = 0;
#endif // DEBUG_RECVMGMTLNK
	C_UWORD len, i;
	//C_UWORD tlen;
	MGMTLINKS *pml = &mlnk[tindex];
	int curcnt, mlnksize;
	C_UBYTE result = FALSE;
	int timeoutcnt = (linktimeout/10) + 1;	// milliseconds timeout converted to decamilliseconds
	int abortcnt = 0;
	C_UBYTE envelbuf[3];
	C_UBYTE *prbuf = NULL;

	pml->rresult = MGMT_RESULT_FAILURE;
	gmgmtlnkStatus[tindex] = MGMT_STATUS_IDLE;

	if (envelopeflag)
	{ // skip through the envelope's 3 bytes

		mlnksize = 3;
		curcnt = 0;	// start with 0 received
		for (;;)
		{
			len = SerialRead (pml->ttySfd, &envelbuf[curcnt], mlnksize - curcnt);
			if (len > 0)
			{
				if ((len + curcnt) == 3)
				{
					if (envelbuf[0] == MGMT_RAWPREFIX1)
					{
						if (envelbuf[1] == MGMT_RAWPREFIX2)
						{
							curcnt = 3;
							break;	// got it
						}
					}
					else if (envelbuf[1] == MGMT_RAWPREFIX1)
					{
						if (envelbuf[2] == MGMT_RAWPREFIX2)
						{
							envelbuf[0] = MGMT_RAWPREFIX1;
							envelbuf[1] = MGMT_RAWPREFIX2;
							curcnt = 2;	// missing the LEN byte
						}
					}
					else if (envelbuf[2] == MGMT_RAWPREFIX1)
					{
						envelbuf[0] = MGMT_RAWPREFIX1;
						curcnt = 1; // missing 2nd prefix and LEN byte
					}
					else
						curcnt = 0; // nothing matched
				}
				else
					curcnt += len;
			}
			epi_msleep(10);
			if (--timeoutcnt <= 0)
				goto RecvMgmtLnkExit;
		}
	}

	timeoutcnt = (linktimeout/10) + 1;	// milliseconds timeout converted to decamilliseconds
	curcnt = 0;	// start with 0 received
	mlnksize = MGMT_PACKOVHD_SIZE;	// initially get the minimum overhead amount
	prbuf = pml->rbuf;
	while (curcnt < mlnksize)
	{
		len = SerialRead (pml->ttySfd, &prbuf[curcnt], mlnksize - curcnt);
		if (len > 0)
		{
if (0)//len != (mlnksize - curcnt))
epi_printf("sr len: %d, want %d, tindex=%d, addr=0x%X, abortcnt=%d, tocnt=%d\n", len, mlnksize - curcnt, tindex, GetMgmtSendAddress(tindex), abortcnt, timeoutcnt);
		}

		if (len > 0)
		{
			curcnt += len;
			abortcnt = 0;
		}
		else
			abortcnt++;

		if ((curcnt == MGMT_PACKOVHD_SIZE) && (mlnksize == MGMT_PACKOVHD_SIZE))
		{	// we can get in and start getting the rest of the packet if any
			if (prbuf[MGMT_PACKPOS_CMDRSP] >= (C_UBYTE) 0x80)
			{
//epi_printf("*R*%x-%u\n", prbuf[MGMT_PACKPOS_CMDRSP], prbuf[MGMT_PACKPOS_ARGCNT]);
				if (prbuf[MGMT_PACKPOS_ARGCNT] == 0xFF)
					pml->rargsize = MGMT_XLARGEARG_SIZE;
				else
					pml->rargsize = prbuf[MGMT_PACKPOS_ARGCNT] << 4;
				if (pml->rargsize <= MGMT_XLARGEARG_SIZE)
				{	// valid argsize
					pml->uselrbuf = 1;
					memcpy (pml->lrbuf, pml->rbuf, curcnt);
					prbuf = pml->lrbuf;
					// increase the timeout for the larger packets
					timeoutcnt += ((linktimeout/10) + 1) * (pml->rargsize >> 8);
				}
				else
					prbuf = NULL;
			}
			else
			{
				pml->uselrbuf = 0;
				if (prbuf[MGMT_PACKPOS_ARGCNT] == 0xFF)
					pml->rargsize = MGMT_MAXARG_SIZE;
				else
					pml->rargsize = prbuf[MGMT_PACKPOS_ARGCNT];
				if (pml->rargsize <= MGMT_MAXARG_SIZE)
				{	// valid argsize
					// keep it in the normal sized buffer
				}
				else
					prbuf = NULL;
			}
			if (!prbuf)
			{	// error packet too large must be an overrun case
				epi_printf("(a(%d,%d))", len, pml->rargsize);
				goto RecvMgmtLnkExit;
			}
			mlnksize += pml->rargsize;
		}
		if (curcnt == mlnksize)
		{	// we got the entire packet
			len = MGMT_PACKOVHD_SIZE + pml->rargsize;
			i = 0;
			// now we can process the complete packet
			pml->raddr = prbuf[i++];
			pml->rcmd = prbuf[i++];
			i++; //skip already have this pml->rargsize = prbuf[i++];
			// for big endian
			pml->tmpcrc = (prbuf[len - 2] << 8) & 0xFF00;
			pml->tmpcrc |= prbuf[len - 1] & 0xFF;
			pml->rcrc = AccCRC(0, prbuf, (C_UWORD) (len - 2));	
			/*
			if (len >= 4096)
			{
				static int findex = 0;
				char filename[64];
				sprintf (filename, "/tmp/recv%d", findex++);
				epi_dumpfile(filename, prbuf, len);
				//epi_dump_buffer(__FUNCTION__, prbuf, len);
			}
			*/
			if (pml->tmpcrc != pml->rcrc)
			{
//#ifdef DEBUG_RECVMGMTLNK
				epi_printf("*f%d,%d, ccrc=%X, rcrc=%X\n", pml->rcmd, len, pml->rcrc, pml->tmpcrc);
//#endif // DEBUG_RECVMGMTLNK
				goto RecvMgmtLnkExit;
			}
			else
			{
#if defined(MLINK_DUMPDATA)
// debug
if (prbuf[MGMT_PACKPOS_CMDRSP] == MGMT_FIELDS_LSET) 
{
char f[32];
epi_dumpfile(som_tsfilename(f), prbuf, len);
//epi_dump_buffer("MGMT_FIELDS_LSET", prbuf, len);
}
#endif // defined(MLINK_DUMPDATA)
				pml->rresult = MGMT_RESULT_SUCCESS;
//#ifdef DEBUG_RECVMGMTLNK
				if (goodcnt++ > 1000)
				{
					epi_printf("1000 good ones\n");
					goodcnt = 0;
				}
				if (envelopeflag)
				{ // skip through the envelope's 2 bytes suffix
					curcnt = 0;	// start with 0 received
					for (;;)
					{
						len = SerialRead (pml->ttySfd, &envelbuf[curcnt], 2-curcnt);
						if ((len > 0) && ((curcnt + len) >= 2))
							break;
						epi_msleep(10);
						if (--timeoutcnt <= 0)
							goto RecvMgmtLnkExit;
					}
				}
				if (SerialIsReceiveDataAvailable(pml->ttySfd) > 0)
				{
					//tcflush(pml->ttySfd, TCIFLUSH);
					epi_printf("**O**\n");
				}
//#endif // DEBUG_RECVMGMTLNK
				result = TRUE;
				pml->state = MGMT_STATE_RCVDONE;
				gmgmtlnkStatus[tindex] = MGMT_STATUS_RCVDONE;
				goto RecvMgmtLnkExit;
			}
		}
		epi_msleep(10);
		if (--timeoutcnt <= 0)
			goto RecvMgmtLnkExit;
#if defined(MCM_REVX0)
		if (((tindex == MGMT_RCM_LINK) || (tindex == MGMT_BMM_LINK)) &&
			(abortcnt > 2))
#else // not defined(MCM_REVX0)
		if ((tindex == MGMT_BRCM_LINK) && (abortcnt > 2))
#endif // !defined(MCM_REVX0)
			goto RecvMgmtLnkExit;
	}

RecvMgmtLnkExit:
	tcflush(pml->ttySfd, TCIFLUSH);
	return (result);
}


C_UBYTE
RecvRawMgmtLnk(C_UBYTE tindex, C_UWORD linktimeout, C_UBYTE *pbuf, C_UWORD bufsize)
{
	C_UBYTE result = 0;
	MGMTLINKS *pml = &mlnk[tindex];
	int curcnt, len;
	int timeoutcnt = (linktimeout/100) + 1;	// milliseconds timeout converted to decamilliseconds

	if (!pbuf)
		goto RecvRawMgmtLnkExit;

//epi_printf("%s, tindex=%d, bufsize=%d\n", __FUNCTION__, tindex, bufsize);
	curcnt = 0;
	while ((curcnt < bufsize) && (--timeoutcnt > 0))
	{
		len = SerialRead (pml->ttySfd, &pbuf[curcnt], bufsize - curcnt);
		if (len > 0)
			curcnt += len;
//epi_printf("%s, len=%d, curcnt=%d\n", __FUNCTION__, len, curcnt);
		epi_msleep(100);
	}
	result = curcnt;

RecvRawMgmtLnkExit:
	return (result);
}


// return TRUE if there are bytes received or FALSE if none
int
CheckRecvMgmtLnk(C_UBYTE tindex)
{
	int ret;

	ret = SerialIsReceiveDataAvailable(mlnk[tindex].ttySfd);
	return ret;
}

void
PrepRecvMgmtLnk(C_UBYTE tindex)
{
	C_UBYTE bbuf = 0;
	//tcflush(mlnk[tindex].ttySfd, TCIFLUSH); doesn't work
	//tcflush(mlnk[tindex].ttySfd, TCOFLUSH); doesn't work
	//SerialRead (mlnk[tindex].ttySfd, &bbuf, 1); doesn't work
	//ioctl(mlnk[tindex].ttySfd, TCXONC, TCOON); doesn't work
	//ioctl(mlnk[tindex].ttySfd, TCXONC, TCOOFF); doesn't work

	// only this works to enable SerialIsReceiveDataAvailable()
	// should find a better way in the future
	SerialWrite (mlnk[tindex].ttySfd, &bbuf, 1);
}

void
FlushRecvMgmtLnk(C_UBYTE tindex)
{
	tcflush(mlnk[tindex].ttySfd, TCIFLUSH);
}
