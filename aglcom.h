/* Xenia Mailer - FidoNet Technology Mailer
 * Copyright (C) 1987-2001 Arjen G. Lentz
 * AGL's async experiment (DOS)
 *
 * This file is part of Xenia Mailer.
 * Xenia Mailer is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */


#ifndef __COM_DEF_
#define __COM_DEF_

#ifdef __cplusplus
extern "C" {
#endif


#if 0
typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned long  dword;
typedef int boolean;
enum { false, true };
#endif


/* ---------------------------------- UART port base / Interrupt assignments */
/* PIC and INT number/mask automatically calculated from IRQ number	     */
typedef struct _agl_portinfo COM_PORTINFO;
struct _agl_portinfo {
	word portbase;			/* UART port base		     */
	byte irq;			/* IRQ number			     */
};

/* ---------------------------------- DOS function and interrupt defintions  */
enum {	DOS_INT        = 0x21,		/* DOS function dispatcher interrupt */
	DOS_SETINTVECT = 0x25,		/* DOS get interrupt vector function */
	DOS_GETINTVECT = 0x35,		/* DOS set interrupt vector function */
	DOS_SETTIMVECT = 0x2508,	/* Set timer interrupt vect int 08h  */
	DOS_GETTIMVECT = 0x3508 	/* Get timer interrupt vect int 08h  */
};

/* ---------------------------------- 8259 Programmable Interrupt Controller */
/* CMD = Control register (EOI), CTL = Interrupt mask register (1=mask off)  */
enum {	PIC1_PORTBASE = 0x20,		/* Port 20-21  1st 8259 all PCs      */
	PIC1_VECTBASE = 0x08,		/* VectBase IRQ 0 = BIOS/DOS INT 08h */
	PIC2_PORTBASE = 0xA0,		/* Port A0-21  2nd 8259 only AT & up */
	PIC2_VECTBASE = 0x70,		/* VectBase IRQ 8 = BIOS/DOS INT 70h */
	PIC_CMD       = 0,		/* 0  W PIC command register (EOI)   */
	PIC_CTL       = 1,		/* 1 RW PIC control register (masks) */
	PIC_EOI       = 0x20		/* Nonspecific end of irq (to CMD)   */
};

/* ---------------------------------- UART ports --------------------------- */
enum {	UART_RBR,			/* 0 R	Receiver Buffer Register     */
	UART_THR = 0,			/* 0  W Transmitter Holding Register */
	UART_IER,			/* 1 RW Interrupt Enable Register    */
	UART_IIR,			/* 2 R	Interrupt Identification Reg */
	UART_FCR = 2,			/* 2  W FIFO Control Register	     */
	UART_LCR,			/* 3 RW Line Control Register	     */
	UART_MCR,			/* 4 RW Modem Control Register	     */
	UART_LSR,			/* 5 RW Line Status Register	     */
	UART_MSR,			/* 6 RW Modem Status Register	     */
	UART_SCR,			/* 7 RW Modem Scratch Register	     */
	UART_DLL = 0,			/* 0 RW DLAB=1 BPS Divisor Latch LSB */
	UART_DLM			/* 1 RW DLAB=1 BPS Divisor Latch MSB */
};

/* ---------------------------------- UART port bits ----------------------- */
enum {	IER_ERBFI  = 0x01,		/* Enable Recv Data Avail. Int	     */
	IER_ETBEI  = 0x02,		/* Enable Xmit Hold. Reg. Empty Int  */
	IER_ELSI   = 0x04,		/* Enable Recv Line Status Int	     */
	IER_EDSSI  = 0x08,		/* Enable Modem Status Int	     */

	IIR_PEND   = 0x01,		/* Irq ID 0  NOT interrupt pending   */
	IIR_ID1    = 0x02,		/* Interrupt ID bit 1		     */
	IIR_ID2    = 0x04,		/* Interrupt ID bit 2		     */
	IIR_ID3    = 0x08,		/* Interrupt ID bit 3 (FIFO only)    */
	 IIR_RLSTS  = IIR_ID1 | IIR_ID2,   /* | 3 HIGH Receiver Line Status  */
	 IIR_RDAVL  = IIR_ID2,		   /* | 2 2nd  Received Data Avail.  */
	 IIR_CHRTI  = IIR_ID2 | IIR_ID3,   /* | 6 2nd  Char. Timeout Indic.  */
	 IIR_THRE   = IIR_ID1,		   /* | 1 3rd  Xmit Hold Reg. Empty  */
	 IIR_MDSTS  = 0,		   /* | 0 4th  Modem status	     */
	IIR_FIFO1  = 0x40,		/* FIFOs Enabled 1		     */
	IIR_FIFO2  = 0x80,		/* FIFOs Enabled 2		     */

	FCR_ENABLE = 0x01,		/* FIFO Enable			     */
	FCR_RCVRST = 0x02,		/* Receiver FIFO Reset		     */
	FCR_XMTRST = 0x04,		/* Transmitter FIFO Reset	     */
	FCR_DMA    = 0x08,		/* DMA Mode Select		     */
	FCR_TRIGL  = 0x40,		/* Receiver Trigger LSb 	     */
	FCR_TRIGM  = 0x80,		/* Receiver Trigger MSb 	     */
	 FCR_TRIG1  = 0,		     /* | Trigger level  1	     */
	 FCR_TRIG4  = FCR_TRIGL,	     /* | Trigger level  4	     */
	 FCR_TRIG8  = FCR_TRIGM,	     /* | Trigger level  8	     */
	 FCR_TRIG14 = FCR_TRIGL | FCR_TRIGM, /* | Trigger level 14	     */

	LCR_WLS0   = 0x01,		/* Word Length Select bit 0	     */
	LCR_WLS1   = 0x02,		/* Word Length Select bit 1	     */
	 LCR_WORD5  = 0,		     /* | 5 bit chars		     */
	 LCR_WORD6  = LCR_WLS0, 	     /* | 6 bit chars		     */
	 LCR_WORD7  = LCR_WLS1, 	     /* | 7 bit chars		     */
	 LCR_WORD8  = LCR_WLS0 | LCR_WLS1,   /* | 8 bit chars		     */
	LCR_STB    = 0x04,		/* Number of Stop Bits		     */
	LCR_PEN    = 0x08,		/* Parity Enable		     */
	LCR_EPS    = 0x10,		/* Even Parity Select		     */
	LCR_STICKP = 0x20,		/* Stick Parity 		     */
	LCR_SETBRK = 0x40,		/* Set Break			     */
	LCR_DLAB   = 0x80,		/* Divisor Latch Access Bit	     */

	MCR_DTR    = 0x01,		/* Data Terminal Ready		     */
	MCR_RTS    = 0x02,		/* Request to Send		     */
	MCR_OUT1   = 0x04,		/* Out 1 (User bit)		     */
	MCR_OUT2   = 0x08,		/* Out 2 (UART interrupt enable)     */
	MCR_LOOP   = 0x10,		/* Loopback mode		     */

	LSR_DR	   = 0x01,		/* Data Ready			     */
	LSR_OE	   = 0x02,		/* Overrun Error		     */
	LSR_PE	   = 0x04,		/* Parity Error 		     */
	LSR_FE	   = 0x08,		/* Framing Error		     */
	LSR_BI	   = 0x10,		/* Break Interrupt		     */
	LSR_THRE   = 0x20,		/* Transmitter Hold. Register Empty  */
	LSR_TEMT   = 0x40,		/* Transmitter Empty		     */
	LSR_FERR   = 0x80,		/* Error in receiver FIFO	     */

	MSR_DCTS   = 0x01,		/* Delta Clear To Send		     */
	MSR_DDSR   = 0x02,		/* Delta Data Set Ready 	     */
	MSR_TERI   = 0x04,		/* Trailing Edge Ring Indicator      */
	MSR_DDCD   = 0x08,		/* Delta Data Carrier Detect	     */
	MSR_CTS    = 0x10,		/* Clear To Send		     */
	MSR_DSR    = 0x20,		/* Data Set Ready		     */
	MSR_RI	   = 0x40,		/* Ring Indicator		     */
	MSR_DCD    = 0x80		/* Data Carrier Detect		     */
};

/* ---------------------------------- BPS rate divisor base ---------------- */
#define BRD_BASE	(115200UL)	/* BRD=Base/BPS  BPS=Base/BRD	     */
#define BRD_BASEL	(0xC200)	/* Base LSB  (115200 = 0001C200h)    */
#define BRD_BASEM	(0x0001)	/* Base MSB			     */


/* ---------------------------------- General defitions -------------------- */
#define COM_MAXPORTS	(16)		/* Max. no. com ports [0-15]	     */
#define COM_MINBUFFER	(128)		/* Min. size of xmit/recv buffers    */
#define COM_MAXBUFFER	(8192)		/* Max. size of xmit/recv buffers    */
#define COM_BUFSIZE	(4096)		/* Default xmit/recv buffer size     */
#define COM_FIFOSIZE	(16)		/* # of places in UART tx FIFO	     */
#define COM_MINSPEED	(2UL)		/* Minimum speed of comport	     */
#define COM_MAXSPEED	(115200UL)	/* Maximum speed of comport	     */
#define COM_TXTIMEOUT	(546)		/* 30 secs (18.2 ticks/sec)	     */
#define COM_TICKMSECS	(55)		/* 55 milliseconds in a clock tick   */
#define COM_THRESHOLD	(4)		/* low water 1/4th high water 3/4th  */
#define COM_EOF 	(-1)		/* No character available	     */
#define COM_XOFF	('S' - '@')	/* XOFF character (Ctrl-S)	     */
#define COM_XON 	('Q' - '@')	/* XON character (Ctrl-Q)	     */

typedef byte COM_PARITY;				/* Parity options    */
enum {	PAR_NONE, PAR_ODD, PAR_EVEN };

enum {	COM_MINBITS = 5, COM_MAXBITS = 8,		/* Data bits	     */
	COM_MINSTOP = 1, COM_MAXSTOP = 2		/* Stop bits	     */
};

typedef byte COM_FLOW;					/* Flow control opts */
enum { FLOW_NONE, FLOW_SOFT = 0x01, FLOW_HARD = 0x02 };

typedef byte COM_TXSTATE;				/* Transmitter state */
enum { TX_DISABLE = 0x01,
       TX_XOFF	  = 0x02, TX_LOWCTS = 0x04,
       TX_NODATA  = 0x08, TX_BRK    = 0x10
};

typedef byte COM_RXSTATE;				/* Receiver state    */
enum { RX_DISABLE = 0x01,
       RX_HIGHBUF = 0x02
};
typedef byte COM_RXFLOW;				/* Receiver flow ctl */
enum { RX_XOFF	  = 0x01, RX_LOWRTS = 0x02
};

typedef byte COM_FIFO;					/* FIFO state	     */
enum { FIFO_PRESENT = 0x01, FIFO_ALLOWED = 0x02, FIFO_ACTIVE = 0x04 };

typedef byte COM_PGMUART;				/* Pgm UART options  */
enum { PGM_AND, PGM_OR, PGM_XOR, PGM_PUT };

enum { NOUART, NS8250, NS16450, NS16550, NS16550A };

/* ---------------------------------- Available vars & Function prototypes - */
extern COM_PORTINFO agl_portinfo[COM_MAXPORTS]; /* 16x portbase,irq struct   */

byte far *agl_alloc    (word size);		/* User or default allocfunc */
void	  agl_free     (byte far *buf); 	/* User or default freefunc  */

boolean  agl_open      (byte port);		/* Open comport 	     */
boolean  agl_close     (boolean restoremcr);	/* Close comport	     */

boolean  agl_settxbuf  (word bufsize);		/* Set/change txbuf size     */
word	 agl_gettxbuf  (void);			/* Get current txbuf size    */
boolean  agl_setrxbuf  (word bufsize);		/* Set/change rxbuf size     */
word	 agl_getrxbuf  (void);			/* Get current rxbuf size    */

dword	 agl_setspeed  (dword newspeed);	/* Set com speed (1-115200)  */
dword	 agl_getspeed  (void);			/* Get com speed (1-115200)  */
boolean  agl_setparity (COM_PARITY parity, byte databits, byte stopbits);
byte	 agl_getbits   (void);			/* Get UART parity/data/stop */
boolean  agl_setmask   (byte newmask);		/* Set rx byte mask (7/8bit) */
byte	 agl_getmask   (void);			/* Get current rx byte mask  */

boolean  agl_setflow   (COM_FLOW newflow);	/* Set flow control flags    */
COM_FLOW agl_getflow   (void);			/* Get current flow settings */

boolean  agl_havefifo  (byte port);		/* FIFO detect on spec. port */
boolean  agl_setfifo   (byte newtrigger);	/* Set/change trigger level  */
byte	 agl_getfifo   (void);			/* Get current trigger level */
boolean  agl_usefifo   (boolean enable);	/* Allow FIFO usage	     */

boolean  agl_txbyte    (byte c);		/* Transmit byte (no wait)   */
boolean  agl_txpush    (byte c);		/* Insert at top of queue    */
word	 agl_txblock   (byte far *buf, word len);	/* TxBlock (no wait) */
boolean  agl_txempty   (void);			/* True if txbuf empty	     */
boolean  agl_txready   (void);			/* True if txbuf not full    */
word	 agl_txfill    (void);			/* # bytes in tx buffer      */
boolean  agl_txclear   (void);			/* Clear tx buffer	     */
boolean  agl_txdisable (boolean disable);	/* Disable transmitter	     */

int	 agl_rxbyte    (void);			/* Receive byte (no wait)    */
boolean  agl_rxpush    (byte c);		/* Insert at top of queue    */
word	 agl_rxblock   (byte far *buf, word len);	/* RxBlock (no wait) */
int	 agl_rxpeek    (void);			/* Nondestr. read next byte  */
boolean  agl_rxempty   (void);			/* True if rxbuf empty	     */
boolean  agl_rxready   (void);			/* True if rxbuf not empty   */
word	 agl_rxfill    (void);			/* # bytes in rx buffer      */
boolean  agl_rxclear   (void);			/* Clear rx buffer	     */
boolean  agl_rxdisable (boolean disable);	/* Disable receiver	     */

boolean  agl_break     (word millisecs);	/* Send break for n msecs    */
boolean  agl_carrier   (void);			/* True if DCD mask high     */
boolean  agl_setdcd    (byte dcdmask);		/* Set carrier detect mask   */
byte	 agl_getdcd    (void);			/* Get carrier detect mask   */
boolean  agl_setdtr    (boolean enable);	/* Set/lower DTR line (MCR)  */
boolean  agl_setloop   (boolean enable);	/* Select UART loopback mode */
boolean  agl_setout1   (boolean enable);	/* Set/lower OUT1 line (MCR) */

byte	 agl_getlsr    (void);			/* Get last line status reg  */
byte	 agl_getmsr    (void);			/* Get last line status reg  */
byte	 agl_geterr    (void);			/* Get error flags (resets!) */

byte	 agl_uart      (byte port);		/* Return UART type	     */
boolean  agl_modem     (byte port);		/* True if modem on port     */
boolean  agl_putuart   (byte port, byte uartreg, COM_PGMUART method, byte value);
byte	 agl_getuart   (byte port, byte uartreg);  /* Program/Read UART regs */


#ifdef __cplusplus
}
#endif

#endif/*__COM_DEF_*/

/* end of aglcom.h --------------------------------------------------------- */
