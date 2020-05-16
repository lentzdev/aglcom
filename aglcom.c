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


#pragma inline
#include <stddef.h>
#include <dos.h>
typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned long  dword;
typedef int boolean;
enum { false, true };
#include "aglcom.h"


/* ------------------------------------------------------------------------- */
COM_PORTINFO agl_portinfo[COM_MAXPORTS] = {
	{ 0x3F8, 4 }, { 0x2F8, 3 }, { 0x3E8, 4 }, { 0x2E8, 3 },
	{ 0x3F8, 4 }, { 0x2F8, 3 }, { 0x3E8, 4 }, { 0x2E8, 3 },
	{ 0x3F8, 4 }, { 0x2F8, 3 }, { 0x3E8, 4 }, { 0x2E8, 3 },
	{ 0x3F8, 4 }, { 0x2F8, 3 }, { 0x3E8, 4 }, { 0x2E8, 3 }
};


/* ------------------------------------------------------------------------- */
static	byte	agl_port = 0;			/* Logical com port number   */
static	word	agl_ds	 = 0;			/* Dataseg for these vars    */

static	void interrupt (*old_comisr)(void) = NULL;    /* Storage old COM ISR */
static	void interrupt (*old_timerisr)(void) = NULL;  /* Storage old TIM ISR */
static	byte	old_ier    = 0, 		/* Storage old UART IER      */
		old_lcr    = 0, 		/* Storage old UART LCR      */
		old_mcr    = 0, 		/* Storage old UART MCR      */
		old_picctl = 0; 		/* Storage old PIC CTL mask  */

static	word	uart_port  = 0; 		/* UART base address	     */

static	byte	agl_irq    = 0; 		/* IRQ number used	     */
static	byte	agl_vector = 0; 		/* Corresponding INT vector  */

static	word	pic_port   = 0; 		/* PIC base address	     */
static	byte	pic_mask   = 0; 		/* IER bit set 1 to mask off */


/* ------------------------------------------------------------------------- */
static		boolean      active = false;	/* True if routines opened   */

static		word	     txsize = 0,	/* Current tx buffer size    */
			     rxsize = 0;	/* Current rx buffer size    */

static		byte	far *txbuf   = NULL,	/* pointer to tx buffer      */
			far *rxbuf   = NULL;	/* pointer to tx buffer      */

static volatile byte	     ctlbyte = 0;	/* Send cmd byte (XON/XOFF)  */

static		dword	     speed = 0UL;	/* Current speed	     */
static		byte	     lcr   = 0; 	/* Current LCR settings      */
static		byte	     mask  = 0xff;	/* rx byte mask (strip 8th)  */
static		byte	     dcd   = MSR_DCD;	/* Modem carrier detect mask */

static		COM_FLOW     flow = FLOW_NONE;	/* Flow control settings     */

static volatile COM_TXSTATE  txstate   = 0;	/* Transmitter state	     */
static volatile COM_RXSTATE  rxstate   = 0;	/* Receiver state	     */
static volatile COM_RXFLOW   rxflow    = 0;	/* Receiver flow state	     */
static volatile word	     xofftimer = 0,	/* XOFF timeout countdown    */
			     brktimer  = 0;	/* Break timing countdown    */

static		COM_FIFO     fifo     = FIFO_ALLOWED;	/* FIFO use flags    */
static		byte	     trigger  = FCR_TRIG8;	/* rx FIFO trig lvl  */

static volatile byte	     agl_lsr	= 0,	/* Copy of last LSR read     */
			     agl_msr	= 0,	/* Copy of last MSR read     */
			     agl_errors = 0;	/* LSR reads ORed to here    */

static /*vola*/ word	  txin	   = 0, rxin	 = 0,	/* Buf write pointer */
			  txout    = 0, rxout	 = 0;	/* Buf read pointer  */

static volatile word	  txtop    = 0, rxtop	 = 0,	/* Buftop offset     */
			  txfill   = 0, rxfill	 = 0,	/* # bytes in buffer */
					rxlow	 = 0,	/* Flow lowwater     */
					rxhigh	 = 0;	/* Flow highwater    */


/* ------------------------------------------------------------------------- */
#define WAITJMP()  { asm jmp $+2; }
#define WORD(var)  (*((word *) &var))
#define UART(reg)  (uart_port + (reg))
#define PIC(reg)   (pic_port + (reg))
#define LABEL(lbl) } lbl: asm {

/* Temporarily here, will be in separate module: user can replace by linking */
#include <alloc.h>
#define agl_alloc(bufsize) ((byte far *) farmalloc(bufsize))
#define agl_free(buf)	   (farfree(buf))


/* ------------------------------------------------------------------------- */
static void near agl_txstart (void)   /* Start xmitter, assumes CLI, DX=base */
{
asm {
	inc  dx 					/* IER=BASE+1	     */
	in   al, dx
	or   al, IER_ETBEI				/* ENABLE XMIT INTR! */
	jmp  $+2
	out  dx, al
	dec  dx 					/* Back to base      */
    }
}/*agl_txstart()*/


/* ------------------------------------------------------------------------- */
static void near agl_rxstart (void)	      /* Start receiver, assumes CLI */
{
asm {
	mov  bh, [rxflow]				/* Read in rxflow    */
	mov  dx, [uart_port]				/* Get UART base     */

	test bh, RX_LOWRTS				/* Was RTS low?      */
	jz   norts					/* RTS wasn't low    */
	and  bh, NOT(RX_LOWRTS) 			/* Reset RTS flag    */
	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx
	or   al, MCR_RTS				/* Raise RTS line    */
	jmp  $+2
	out  dx, al
	sub  dx, 4					/* Back to base      */

LABEL(norts)
	test bh, RX_XOFF				/* Sent XOFF?	     */
	jz   noxoff					/* Didn't send XOFF  */
	and  bh, NOT(RX_XOFF)				/* Reset XOFF flag   */
	cmp  [ctlbyte], COM_XOFF			/* Had XOFF queued?  */
	jne  noqueue					/* Not in queue      */
	mov  [ctlbyte], 0				/* Scrap that XOFF!  */
	jmp  noxoff					/* XON not required  */

LABEL(noqueue)
	mov  [ctlbyte], COM_XON 			/* Send XON byte     */
	call agl_txstart				/* Start transmitter */

LABEL(noxoff)
	mov  [rxflow], bh				/* Store new rxflow  */
    }
}/*agl_rxstart()*/


/* ------------------------------------------------------------------------- */
static void near agl_rxstop (void)	      /* Stop receiver, assumes CLI, */
{					      /* DX=uart, BL=flow	     */
asm {
	push bx
	mov  bh, [rxflow]

	test bl, FLOW_HARD				/* Hard flow control */
	jz   nohard					/* No hard flow ctrl */
	test bh, RX_LOWRTS
	jnz  nohard

	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx
	and  al, NOT(MCR_RTS)				/* Drop RTS line     */
	jmp  $+2
	out  dx, al
	sub  dx, 4					/* Back to base      */
	or   bh, RX_LOWRTS				/* Flag RTS low      */
LABEL(nohard)

	test bl, FLOW_SOFT				/* Soft flow control */
	jz   nosoft					/* No soft flow ctrl */
	test bh, RX_XOFF
	jnz  nosoft

	mov  [ctlbyte], COM_XOFF			/* Send XOFF byte    */
	call agl_txstart				/* Start transmitter */
	or   bh, RX_XOFF				/* Flag XOFF sent    */
LABEL(nosoft)

LABEL(fini)
	mov  [rxflow], bh
	pop  bx
    }
}/*agl_rxstop()*/


/* ------------------------------------------------------------------------- */
static void interrupt agl_isr	      (void);
static void interrupt timer_isr       (void);
static void near      isr_modemstatus (void);
static void near      isr_transmit    (void);
static void near      isr_receive     (void);
static void near      isr_linestatus  (void);

static void near (*isr_funcs[])(void) = {
	isr_modemstatus,	/* 00 Modem Status			     */
	isr_transmit,		/* 01 Transmitter Holding Register Empty     */
	isr_receive,		/* 10 Received Data Available		     */
	isr_linestatus		/* 11 Receiver Line Status		     */
};


/* ------------------------------------------------------------------------- */
static void interrupt agl_isr (void)
{
asm {
	sti						/* Yes, this is okay */

	mov  dx, [uart_port]				/* Get/remember port */
	mov  bl, [flow] 				/* Get/remember flow */
	cld						/* Clear direction   */

LABEL(again)
	add  dx, 2					/* IIR=BASE+2	     */
	in   al, dx					/* Get IIR register  */
	sub  dx, 2					/* Back to base      */

	and  ax, (IIR_ID2 OR IIR_ID1 OR IIR_PEND)	/* Mask 16/ints !ID2 */
	test al, IIR_PEND				/* Interrupt pending */
	jnz  fini					/* 1 = False alarm   */

	mov  di, ax
	call [offset isr_funcs + di]			/* call funcs[type]  */
	jmp  again					/* Next please!      */

LABEL(fini)
	/* 8250s and 16450s sometimes lose THRE interrupt if also ELSI/EDSSI */
	/* and handling full duplex data transfer (important for Hydra!)     */
	/* Also, when byte moved from hold > xmitter, stuff in another byte! */
	add  dx, 5					/* LSR=BASE+5	     */
	in   al, dx
	test al, LSR_THRE
	jz   nothre
	sub  dx, 4					/* IER=LSR-4	     */
	in   al, dx
	test al, IER_ETBEI				/* Is TX intr. on?   */
	jz   nothre
	dec  dx 					/* Back to base      */
	call isr_transmit
LABEL(nothre)

if 0
	/* Disable and re-enable UART interrupt register for a negative edge */
	/* I don't think this is necessary nowadays... uncomment if needed   */
	inc  dx
	xor  ah, ah
	jmp  $+2
	in   al, dx
	xchg al, ah
	jmp  $+2
	out  dx, al
	xchg al, ah
	jmp  $+2
	out  dx, al
	dec  dx
endif

	mov  dx, [pic_port]
	mov  al, PIC_EOI				/* Nonspec. end int. */
	cli						/* Disable INTs now! */
	out  dx, al					/* And get it out... */
    }
}/*agl_isr()*/


/* ------------------------------------------------------------------------- */
static void near isr_modemstatus (void)
{
asm {
	add  dx, 6					/* MSR=BASE+6	     */
	in   al, dx					/* Get Modem Status  */
	sub  dx, 6
	mov  [agl_msr], al				/* Store for user    */

	test bl, FLOW_HARD				/* Hard flow control */
	jz   fini					/* No hard flow ctrl */
	test al, MSR_DCTS				/* CTS line changed? */
	jz   fini					/* Nope, so ignore   */

	test al, MSR_CTS				/* Check CTS line    */
	jnz  highcts					/* Is high now	     */

	or   [txstate], TX_LOWCTS			/* Set tx inhibit    */
	jmp  fini

LABEL(highcts)
	test [txstate], TX_LOWCTS			/* Check CTS state   */
	jz   fini					/* Wasn't low before */
	and  [txstate], NOT(TX_LOWCTS)			/* Reset CTS flowflg */
	jnz  fini					/* Still tx inihibit */
	call agl_txstart				/* START TRANSMITTER */

LABEL(fini)
    }
}/*isr_modemstatus()*/


/* ------------------------------------------------------------------------- */
static void near isr_transmit (void)
{
asm {
	mov  bh, [txstate]				/* Read in txstate   */

	test bh, TX_BRK 				/* Test break state  */
	jnz  notx					/* If so, don't send */

LABEL(again)
	mov  cx, 1					/* Xmit max 1 byte   */
	test [fifo], FIFO_ACTIVE			/* Test FIFO active  */
	jz   nofifo					/* FIFO not active   */
	mov  cx, COM_FIFOSIZE				/* Xmit max 16 bytes */
LABEL(nofifo)

	mov  al, [ctlbyte]				/* Get control byte  */
	or   al, al					/* Set CPU flags     */
	jz   noctl					/* No ctlchr to send */
	out  dx, al					/* Send control byte */
	mov  [ctlbyte], 0				/* Reset after send  */
	dec  cx 					/* One less to send  */
	jz   fini					/* No more allowed?  */
LABEL(noctl)

	or   bh, bh					/* Test ALL txstate  */
	jnz  notx					/* tx inhibited      */

	mov  di, [txfill]				/* Get # bytes in tx */
	or   di, di					/* Set CPU flags     */
	jz   nodata					/* No data, stop tx  */

	mov  es, [word ptr txbuf + 2]			/* Get txout buf seg */
	mov  si, [txout]				/* Get txout buf ptr */
	mov  bp, [txtop]				/* Get tx top of buf */

LABEL(more)
	cmp  si, bp					/* Test for buf top  */
	jl   notop					/* Not yet	     */
	mov  si, [word ptr txbuf]			/* Reset to bottom   */
LABEL(notop)

	lods BYTE PTR es:[si]				/* Get byte from buf */
	out  dx, al					/* Put chr info UART */

	dec  di 					/* One less to send  */
	loopnz more					/* Go send next byte */
	/* loopnz exits when either cx==0 (--UARTcount) or di==ZF (--txfill) */

	mov  [txout], si				/* Store new out ptr */
	mov  [txfill], di				/* Store bytes left  */
	or   di, di					/* Anything left?    */
	jnz  fini					/* See you next time */

LABEL(nodata)
	or   [txstate], TX_NODATA			/* Flag no data      */

LABEL(notx)
	inc  dx 					/* IER=BASE+1	     */
	in   al, dx
	and  al, NOT(IER_ETBEI) 			/* Disable TX intr.  */
	jmp  $+2
	out  dx, al
	dec  dx 					/* Back to base      */

LABEL(fini)
    }
}/*isr_transmit()*/


/* ------------------------------------------------------------------------- */
static void near isr_receive (void)
{
asm {
	mov  bh, [rxstate]				/* Get rx state      */
	mov  si, [rxfill]				/* Get # bytes in rx */
	mov  cx, [rxsize]				/* Get rxbuf size    */
	mov  es, [word ptr rxbuf + 2]			/* Get rxin buf seg  */
	mov  di, [rxin] 				/* Get rxin buf ptr  */
	mov  bp, [rxtop]				/* Get rx top of buf */
	mov  ah, [mask] 				/* Get char mask     */

LABEL(rxdata)
	add  dx, 5					/* LSR=BASE+5	     */
	in   al, dx
	sub  dx, 5					/* Back to base      */
	test al, LSR_DR 				/* Test Data Ready   */
	jz   fini					/* No (more) data    */

	cmp  si, cx					/* Check for rx room */
	jnl  fini					/* No, no more room  */

	in   al, dx					/* Get chr from UART */
	and  al, ah					/* Mask out unwanted */

	test bl, FLOW_SOFT				/* Soft flow control */
	jz   store					/* No soft flow ctrl */

	cmp  al, COM_XON				/* Is it an XON ??   */
	jne  doxoff					/* No, go check XOFF */
	test [txstate], TX_XOFF 			/* Have XOFF state?  */
	jz   rxdata					/* No, just ignore   */
	mov  [xofftimer], 0				/* Stop XOFF timer   */
	and  [txstate], NOT(TX_XOFF)			/* Yes, reset XOFF   */
	jnz  rxdata					/* Still tx inhibit? */
	call agl_txstart				/* START TRANSMITTER */
	jmp  rxdata					/* Go get next byte  */

LABEL(doxoff)
	cmp  al, COM_XOFF				/* Is it an XOFF ??  */
	jne  store					/* No, go on normal  */
	test [txstate], TX_XOFF 			/* Have XOFF state?  */
	jnz  rxdata					/* Yes, just ignore  */
	or   [txstate], TX_XOFF 			/* No, flag XOFF     */
	mov  [xofftimer], COM_TXTIMEOUT 		/* Start XOFF timer  */
	jmp  rxdata					/* Go get next byte  */

LABEL(store)
	cmp  di, bp					/* Test for buf top  */
	jl   notop					/* Not yet	     */
	mov  di, [word ptr rxbuf]			/* Reset to bottom   */
LABEL(notop)

	stosb						/* Put byte in rxbuf */
	inc  si 					/* One more received */

	test bh, RX_HIGHBUF				/* Already high buf? */
	jnz  rxdata					/* Yes, just go on   */
	cmp  si, [rxhigh]				/* Check highwater   */
	jl   rxdata					/* Plenty of room    */
	or   bh, RX_HIGHBUF				/* Flag high water   */
	mov  [rxstate], bh				/* Store new rxstate */
	call agl_rxstop 				/* Handle flow ctrl  */
	jmp  rxdata					/* Go get next byte  */

LABEL(fini)
	mov  [rxfill],	si				/* Store new rxfill  */
	mov  [rxin],	di				/* Store new rxin    */
    }
}/*isr_receive()*/


/* ------------------------------------------------------------------------- */
static void near isr_linestatus (void)
{
asm {
	add  dx, 5					/* LSR=BASE+5	     */
	in   al, dx					/* Get Line  Status  */
	sub  dx, 5
	mov  [agl_lsr], al				/* Store for user    */

	and  al, NOT(LSR_DR OR LSR_THRE OR LSR_TEMT)	/* Mask out unwanted */
	or   [agl_errors], al				/* OR rest in errors */
    }
}/*isr_linestatus()*/


/* ------------------------------------------------------------------------- */
static void interrupt timer_isr (void)	       /* Break timing, XOFF timeout */
{					       /* Interrupt 8 handler TIMER  */
asm {
	cmp  [brktimer], 0				/* brktimer running? */
	jz   brkdone
	dec  [brktimer] 				/* Yes, decrease one */
	jnz  brkdone					/* Expired now?      */
	mov  dx, [uart_port]
	add  dx, 3					/* LCR=BASE+4	     */
	in   al, dx					/* Get LCR from UART */
	and  al, NOT(LCR_SETBRK)			/* Clear LCR BRK bit */
	jmp  $+2
	out  dx, al					/* Put LCR into UART */
	sub  dx, 3					/* Back to base      */
	and  [txstate], NOT(TX_BRK)			/* Undo BRK state    */
	jnz  brkdone					/* Xmit allowed now? */
	call agl_txstart				/* START TRANSMITTER */
LABEL(brkdone)

	cmp  [xofftimer], 0				/* xofftimer running */
	jz   xoffdone
	dec  [xofftimer]				/* Yes, decrease one */
	jnz  xoffdone					/* Expired now?      */
	and  [txstate], NOT(TX_XOFF)			/* Undo XOFF state   */
	jnz  xoffdone					/* Xmit allowed now? */
	mov  dx, [uart_port]
	call agl_txstart				/* START TRANSMITTER */
LABEL(xoffdone)

	pushf	/* Pretend we're doing an interrupt call ourselves */
	call dword ptr old_timerisr			/* Call timer chain  */
    }
}/*timer_isr()*/


/* ------------------------------------------------------------------------- */
boolean agl_open (byte port)
{
	if (active) return (true);

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (false);
	if (agl_portinfo[port].irq > 15) return (false);

	if (!txsize) txsize = COM_BUFSIZE;
	if (!rxsize) rxsize = COM_BUFSIZE;
	if (!(txbuf = agl_alloc(txsize)) || !(rxbuf = agl_alloc(rxsize))) {
	   if (txbuf) agl_free(txbuf);
	   return (false);
	}
	ctlbyte = 0;

	txin	  =
	txout	  = WORD(txbuf);
	txtop	  = WORD(txbuf) + txsize;
	txfill	  = 0;
	txstate   = TX_NODATA;
	xofftimer = 0;

	rxin	 =
	rxout	 = WORD(rxbuf);
	rxtop	 = WORD(rxbuf) + rxsize;
	rxlow	 = rxsize / 4;
	rxhigh	 = rxsize - rxlow;
	rxfill	 = 0;
	rxstate  = 0;
	rxflow	 = 0;

	uart_port = agl_portinfo[port].portbase;
	agl_irq   = agl_portinfo[port].irq;
	if (agl_irq < 8) {
	   agl_vector = PIC1_VECTBASE + agl_irq;
	   pic_port   = PIC1_PORTBASE;
	}
	else {
	   agl_vector = PIC2_VECTBASE + agl_irq;
	   pic_port   = PIC2_PORTBASE;
	}
	pic_mask = (1 << (agl_irq & 0x07));

asm {
	cli						/* DISABLE ALL INTS  */

	mov  dx, [uart_port]
	inc  dx 					/* IER=BASE+1	     */
	jmp  $+2
	in   al, dx
	mov  [old_ier], al				/* Save int. enable  */
	add  dx, 2					/* LCR=IER+2	     */
	jmp  $+2
	in   al, dx
	mov  [old_lcr], al				/* Save line ctrl    */
	and  al, NOT(LCR_SETBRK OR LCR_DLAB)		/* lcr=NOT(BRK|DLAB) */
	mov  [lcr], al
	jmp  $+2
	out  dx, al					/* No BRK/DLAB garb! */

	mov  cl, al					/* LSR for bitsshift */
	not  cl 					/* Invert 0s and 1s  */
	and  cl, 003h					/* Just keep 2 bits  */
	mov  al, 0ffh					/* Mask for 8 bit    */
	shr  al, cl					/* Filter cl bits    */
	mov  [mask], al 				/* Store our mask    */

	inc  dx 					/* MCR=LCR+1	     */
	jmp  $+2
	in   al, dx
	mov  [old_mcr], al				/* Save modem ctrl   */
   }

	WAITJMP();
	outportb(UART(UART_IER),0);			/* Set int.enable 0  */
	WAITJMP();
	if (inportb(UART(UART_IER)) != 0) {		/* Read & check 0    */
	   WAITJMP();
	   outportb(UART(UART_LCR),old_lcr);		/* Back just in case */
	   outportb(UART(UART_IER),old_ier);		/* Back just in case */
	   asm sti
	   agl_free(txbuf);
	   agl_free(rxbuf);
	   return (false);				/* Fail = no comport */
	}

asm {
	mov  [agl_ds], ds				/* Remember data seg */

	mov  ah, DOS_GETINTVECT 			/* Old COMINT vect   */
	mov  al, [agl_vector]
	int  DOS_INT
	mov  [word ptr old_comisr],bx
	mov  [word ptr old_comisr + 2],es

	mov  ax, DOS_GETTIMVECT 			/* Old TIMINT vector */
	int  DOS_INT
	mov  [word ptr old_timerisr], bx
	mov  [word ptr old_timerisr + 2], es

	mov  ah, DOS_SETINTVECT 			/* Set own COMINT    */
	mov  al, [agl_vector]				/* Requires DS !     */
/***/	push ds
	mov  bx, cs
	mov  ds, bx
	mov  dx, offset agl_isr
	int  DOS_INT
	mov  ax, DOS_SETTIMVECT 			/* Insert own TIMINT */
	mov  ds, bx
	mov  dx, offset timer_isr
	int  DOS_INT
/***/	pop  ds

	mov  dx, [pic_port]				/* Program 6259 PIC  */
	inc  dx
	in   al, dx
	mov  [old_picctl], al				/* Save old IRQ mask */
	mov  ah, [pic_mask]				/* Get our bit mask  */
	not  ah 					/* Invert for enable */
	and  al, ah					/* AND into mask     */
	jmp  $+2
	out  dx, al					/* to enable our IRQ */

	mov  dx, [uart_port]				/* RBR=base	     */
	jmp  $+2
	in   al, dx					/* Read RBR clear rx */

	add  dx, 4					/* MCR=RBR+4	     */
	mov  al, (MCR_DTR OR MCR_RTS OR MCR_OUT2)	/* Mdm:DTR+RTS+OUT2  */
	jmp  $+2
	out  dx, al

	/* The speed chapter */
	dec  dx 					/* LCR=MCR-1	     */
	mov  cl, [lcr]					/* Read and remember */
	mov  al, cl					/* Move to use now   */
	or   al, LCR_DLAB				/* Select brd latch  */
	out  dx, al

	sub  dx, 3					/* DLL=LCR-3	     */
	jmp  $+2
	in   al, dx
	jmp  $+2
	out  dx, al					/* Write to wake up  */
	mov  bl, al					/* Remember LSB      */
	inc  dx 					/* DLM=DLL+1	     */
	jmp  $+2
	in   al, dx
	jmp  $+2
	out  dx, al					/* Write to wake up  */
	mov  bh, al					/* Remember MSB      */

	add  dx, 2					/* LCR=DLM+2	     */
	mov  al, cl					/* Get back old LCR  */
	out  dx, al					/* Deselect brdlatch */

	mov  dx, BRD_BASEM				/* Divide cur speed  */
	mov  ax, BRD_BASEL
	cmp  bx, 1
	jle  nodiv					/* Don't do /1 or /0 */
	div  bx
LABEL(nodiv)
	mov  [word ptr speed], ax			/* Store cur speed   */
	mov  [word ptr speed + 2], dx

	/* On to the FIFO stuff */
	mov  dx, [uart_port]
	add  dx, 2					/* FCR/IIR=BASE+2    */
	mov  bl, [fifo] 				/* Read and remember */
	and  bl, FIFO_ALLOWED				/* Reset FIFO flags  */
	mov  al, (FCR_ENABLE OR FCR_RCVRST OR FCR_XMTRST)
	out  dx, al
	jmp  $+2
	in   al, dx
	and  al, (IIR_FIFO1 OR IIR_FIFO2)		/* Test FIFOs	     */
	cmp  al, (IIR_FIFO1 OR IIR_FIFO2)		/* BOTH bits=16550A  */
	jne  nofifo					/* Sorry, no okidoki */
	or   bl, FIFO_PRESENT				/* Ok, FIFOs present */
	test bl, FIFO_ALLOWED				/* Use them too?     */
	jz   nofifo
	mov  al, FCR_ENABLE				/* FIFO enable flag  */
	or   al, [trigger]				/* Set FIFO rcv trig */
	jmp  $+2
	out  dx, al
	or   bl, FIFO_ACTIVE				/* Note active now   */
	jmp  didfifo
LABEL(nofifo)
	xor  al, al					/* No? reset FIFOreg */
	jmp  $+2
	out  dx, al
LABEL(didfifo)
	mov  [fifo], bl 				/* Store fifo info   */

	/* Go wake up LSR and MSR regs, and store their contents */
	add  dx, 3					/* LSR=FCR+3	     */
	jmp  $+2
	in   al, dx					/* Read Line Status  */
	mov  [agl_lsr], al
	inc  dx 					/* MSR=LSR+1	     */
	jmp  $+2
	in   al, dx					/* Read Modem Status */
	mov  [agl_msr], al
if 0
	/* Disable hardware handshake if CTS line low */
	test al, MSR_CTS
	jnz  ctshigh
	and  [flow], NOT(FLOW_HARD)
LABEL(ctshigh)
endif

	mov  [agl_errors], 0				/* Clear error flags */

	/* Enable receive, line status and modem status interrupts, no xmit */
	sub  dx, 5					/* IER=MSR-5	     */
	mov  al, (IER_ERBFI OR IER_ELSI OR IER_EDSSI)
	jmp  $+2
	out  dx, al

	sti						/* ENABLE INTERRUPTS */

	mov  al, [port]
	mov  [agl_port], al

	mov  ax, true
	mov  [active], ax
    }
}/*agl_open()*/


/* ------------------------------------------------------------------------- */
boolean agl_close (boolean restoremcr)
{
asm {
	mov  ax, true
	cmp  [active], true
	je   doit
	jmp  fini

LABEL (doit)
	cli						/* DISABLE ALL INTS  */

	mov  dx, [pic_port]				/* Restore 8259 PIC  */
	inc  dx
	mov  bl, [pic_mask]				/* Our IRQ bit	     */
	mov  bh, [old_picctl]				/* Get old value     */
	and  bh, bl					/* Only keep our bit */
	jmp  $+2
	in   al, dx					/* Get current value */
	not  bl 					/* Invert our bit    */
	and  al, bl					/* Filter out IRQ    */
	or   al, bh					/* Put in old bit    */
	jmp  $+2
	out  dx, al					/* Back to old glory */

	mov  dx, [uart_port]
	inc  dx 					/* IER=BASE+1	     */
	xor  al, al
	jmp  $+2
	out  dx, al					/* Set int.enable 0  */
	inc  dx 					/* FCR=IER+1	     */
	jmp  $+2
	out  dx, al					/* Reset FIFO ctrl   */

	inc  dx 					/* LCR=FCR+1	     */
	mov  al, [old_lcr]
	jmp  $+2
	out  dx, al					/* Restore line ctrl */

	inc  dx 					/* MCR=LCR+1	     */
	mov  al, [old_mcr]
	cmp  [word ptr restoremcr], true		/* Leave DTR/RTS?    */
	je   domcr
	or   al, (MCR_DTR OR MCR_RTS)			/* No, pull 'em high */
LABEL(domcr)
	jmp  $+2
	out  dx, al					/* Restore mdm ctrl  */

	sub  dx, 4					/* RBR=MCR-4	     */
	jmp  $+2
	in   al, dx					/* Read to clear RBR */
	add  dx, 2					/* IIR=RBR+2	     */
	jmp  $+2
	in   al, dx					/* Read to clear IIR */
	add  dx, 3					/* LSR=IIR+3	     */
	jmp  $+2
	in   al, dx					/* Read to clear LSR */
	inc  dx 					/* MSR=LSR+1	     */
	jmp  $+2
	in   al, dx					/* Read to clear MSR */

	sub  dx, 5					/* IER=MSR-5	     */
	jmp  $+2
	mov  al, [old_ier]
	out  dx, al					/* Restore int enab. */

	mov  ah, DOS_SETINTVECT 			/* Restore COMINT    */
	mov  al, [agl_vector]				/* Requires DS !     */
/***/	mov  bx, ds
	lds  dx, [dword ptr old_comisr]
	int  DOS_INT
/***/	mov  ds, bx
	mov  ax, DOS_SETTIMVECT 			/* Restore TIMINT    */
	lds  dx, [dword ptr old_timerisr]
	int  DOS_INT
/***/	mov  ds, bx

	sti						/* ENABLE INTERRUPTS */

	xor  ax, ax
	mov  [agl_lsr], al
	mov  [agl_msr], al
	mov  [agl_errors], al
	mov  [txfill], ax
	mov  [rxfill], ax

	mov  [active], false

	mov  ax, true

LABEL(fini)
    }
}/*agl_close()*/


/* ------------------------------------------------------------------------- */
boolean agl_settxbuf (word bufsize)
{
	if	(bufsize < COM_MINBUFFER) bufsize = COM_MINBUFFER;
	else if (bufsize > COM_MAXBUFFER) bufsize = COM_MAXBUFFER;

	if (active) {
	   if (bufsize != txsize) {
	      byte far *newbuf; /* First try to allocate new, then free old! */

	      if (!(newbuf = agl_alloc(bufsize))) return (false);
	      asm cli
	      agl_free(txbuf);
	      txbuf    = newbuf;
	      txin     =
	      txout    = WORD(txbuf);
	      txtop    = WORD(txbuf) + txsize;
	      txfill   = 0;
	      txstate |= TX_NODATA;
	      asm sti
	   }
	}
	else
	   txsize = bufsize;

	return (true);
}/*agl_settxbuf()*/


/* ------------------------------------------------------------------------- */
word agl_gettxbuf (void)
{
	asm mov ax, [txsize];
}/*agl_gettxbuf()*/


/* ------------------------------------------------------------------------- */
boolean agl_setrxbuf (word bufsize)
{
	if	(bufsize < COM_MINBUFFER) bufsize = COM_MINBUFFER;
	else if (bufsize > COM_MAXBUFFER) bufsize = COM_MAXBUFFER;

	if (active) {
	   if (bufsize != rxsize) {
	      byte far *newbuf; /* First try to allocate new, then free old! */

	      if (!(newbuf = agl_alloc(bufsize))) return (false);
	      asm cli
	      agl_free(rxbuf);
	      rxbuf    = newbuf;
	      rxin     =
	      rxout    = WORD(rxbuf);
	      rxtop    = WORD(rxbuf) + rxsize;
	      rxfill   = 0;
	      asm and  [rxstate], NOT(RX_HIGHBUF)
	      asm jnz  norx
	      asm call agl_rxstart
norx:	      asm sti
	   }
	}
	else
	   rxsize = bufsize;

	return (true);
}/*agl_setrxbuf()*/


/* ------------------------------------------------------------------------- */
word agl_getrxbuf (void)
{
	asm mov ax, [rxsize];
}/*agl_getrxbuf()*/


/* ------------------------------------------------------------------------- */
dword agl_setspeed (dword newspeed)
{
asm {
	xor  ax, ax
	mov  dx, ax
	cmp  [active], true
	jne  fini

	mov  bx, [word ptr newspeed]
	mov  cx, [word ptr newspeed + 2]

	or   cx, cx					/* Test > 65535      */
	jz   nohigh					/* Not that high     */
	mov  bx, 1					/* Set 1 = 115200bps */
	jmp  setbdl					/* Go program UART   */

LABEL(nohigh)
	cmp  bx, 1					/* Test <= 1	     */
	ja   nolow					/* Not that low      */
	mov  bx, 65535					/* Set 65535 = 1bps  */
	jmp  setbdl					/* Go program UART   */

LABEL(nolow)
	mov  dx, BRD_BASEM				/* Divide cur speed  */
	mov  ax, BRD_BASEL
	div  bx
	mov  bx, ax					/* Move result	     */

LABEL(setbdl)
	cli

	mov  dx, [uart_port]				/* Get port base     */
	add  dx, 3					/* LCR=BASE+3	     */
	in   al, dx					/* Read Line Control */
	mov  cl, al					/* and remember      */
	jmp  $+2
	or   al, LCR_DLAB				/* Select brd latch  */
	out  dx, al

	sub  dx, 3					/* DLL=LCR-3	     */
	mov  al, bl
	jmp  $+2
	out  dx, al					/* Write LSB	     */

	inc  dx 					/* DLM=DLL+1	     */
	mov  al, bh
	jmp  $+2
	out  dx, al					/* Write MSB	     */

	add  dx, 2					/* LCR=DLM+2	     */
	mov  al, cl					/* Get back old LCR  */
	out  dx, al					/* Deselect brdlatch */

	sti

	mov  dx, BRD_BASEM				/* Divide cur speed  */
	mov  ax, BRD_BASEL
	cmp  bx, 1
	jle  nodiv					/* Don't do /1 or /0 */
	div  bx
LABEL(nodiv)
	mov  [word ptr speed], ax			/* Store cur speed   */
	mov  [word ptr speed + 2], dx			/* (Real as IN UART) */

LABEL(fini)
    }
}/*agl_setspeed()*/


/* ------------------------------------------------------------------------- */
dword agl_getspeed (void)
{
asm {
	xor  ax, ax
	mov  dx, ax
	cmp  [active], true
	jne  fini

	mov  ax, [word ptr speed]
	mov  dx, [word ptr speed + 2]

LABEL(fini)
    }
}/*agl_getspeed()*/


/* ------------------------------------------------------------------------- */
boolean agl_setparity (COM_PARITY parity, byte databits, byte stopbits)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	xor  cl, cl

	mov  bl, [parity]				/* Load new parity   */
	or   bl, bl					/* Set CPU flags     */
	jz   noparity					/* Want no parity    */
	cmp  bl, PAR_EVEN				/* Check highest     */
	ja   fini					/* Above is invalid  */
	jne  noteven					/* Want odd parity   */
	or   cl, LCR_EPS				/* Set even parity   */
LABEL(noteven)
	or   cl, LCR_PEN				/* Parity enable     */
LABEL(noparity)

	mov  bl, [databits]				/* Load new databits */
	cmp  bl, COM_MINBITS				/* Test bits < 5     */
	jl   fini					/* Less is invalid   */
	cmp  bl, COM_MAXBITS				/* Test bits > 8     */
	ja   fini					/* More is invalid   */
	sub  bl, COM_MINBITS				/* Convert to 2 bits */
	or   cl, bl					/* Add to new LSR    */

	mov  bl, [stopbits]				/* Load new stopbits */
	cmp  bl, COM_MINSTOP				/* Test bits < 1     */
	jl   fini					/* Less is invalid   */
	cmp  bl, COM_MAXSTOP				/* Test bits > 2     */
	ja   fini					/* More is invalid   */
	dec  bl 					/* Convert to 1 bits */
	shl  bl, 1
	shl  bl, 1
	or   cl, bl					/* Add to new LCR    */

	mov  [lcr], cl					/* Store new LCR     */

	mov  dx, [uart_port]				/* Get UART base     */
	add  dx, 3					/* LCR=BASE+3	     */

	cli

	in   al, dx					/* Get LCR from UART */
	and  al, LCR_SETBRK				/* Retain this bit   */
	or   al, cl					/* Add new setting   */
	jmp  $+2
	out  dx, al					/* Write out again   */

	not  cl 					/* Invert 0s and 1s  */
	and  cl, 003h					/* Just keep 2 bits  */
	mov  al, 0ffh					/* Mask for 8 bit    */
	shr  al, cl					/* Filter >>cl bits  */
	mov  [mask], al 				/* Store our mask    */

	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_setparity()*/


/* ------------------------------------------------------------------------- */
byte agl_getbits (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  al, [lcr]					/* Get line settings */

LABEL(fini)
    }
}/*agl_getbits()*/


/* ------------------------------------------------------------------------- */
boolean agl_setmask (byte newmask)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	mov  al, [newmask]				/* Load new mask     */
	mov  [mask], al 				/* Store new mask    */

	mov  ax, true

LABEL(fini)
    }
}/*agl_setmask()*/


/* ------------------------------------------------------------------------- */
byte agl_getmask (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  al, [mask] 				/* Get bitmask value */

LABEL(fini)
    }
}/*agl_getmask()*/


/* ------------------------------------------------------------------------- */
boolean agl_setflow (COM_FLOW newflow)
{
asm {
	mov  bl, [newflow]

	cli

	mov  bh, [flow]

	cmp  bl, bh
	jne  change					/* Same as before    */
	jmp  noneed

LABEL(change)
	mov  ax, false
	cmp  bl, (FLOW_HARD OR FLOW_SOFT)
	jle  action
	jmp  fini					/* Invalid value     */

LABEL(action)
	mov  [flow], bl 				/* Store new setting */

	cmp  [active], true				/* Is port active?   */
	je   doit					/* Yes, go change    */
	jmp  noneed					/* No, just store    */

LABEL(doit)
	mov  bh, [rxflow]
	mov  ch, [txstate]
	mov  cl, [rxstate]
	mov  dx, [uart_port]

	test bl, FLOW_HARD				/* Hardware flow?    */
	jz   rxhard					/* Not switched on   */

	/* RX RTS handshake now on */
	or   cl, cl					/* Any rx inhibit?   */
	jz   nortsstop
	call agl_rxstop 				/* Act on it now     */
LABEL(nortsstop)

	/* TX CTS handshake now on */
	test [agl_msr], MSR_CTS 			/* Check CTS status  */
	jnz  soft					/* Is high now	     */
	or   ch, TX_LOWCTS				/* Set tx inhibit    */
	jmp  soft  

LABEL(rxhard)
	test cl, RX_LOWRTS				/* Was RTS low?      */
	jz   txhard					/* RTS wasn't low    */
	/* RTS handshake now off */
	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx
	or   al, MCR_RTS				/* Raise RTS line    */
	jmp  $+2
	out  dx, al
	sub  dx, 4					/* Back to base      */
	and  cl, NOT(RX_LOWRTS) 			/* Reset LOWRTS flag */

LABEL(txhard)
	test ch, TX_LOWCTS				/* Check CTS state   */
	jz   soft					/* Wasn't low before */
	/* TX CTS handshake now off */
	and  ch, NOT(TX_LOWCTS) 			/* Reset CTS flowflg */
	jnz  soft					/* Still tx inihibit */
	call agl_txstart				/* START TRANSMITTER */

LABEL(soft)
	test bl, FLOW_SOFT				/* Software flow?    */
	jz   txsoft					/* Not switched on   */

	/* RX XOFF handshake now on */
	or   cl, cl					/* Any rx inhibit?   */
	jz   noxoffstop
	call agl_rxstop 				/* Act on it now     */
LABEL(noxoffstop)

	/* TX XOFF handshake now on */
	/* need to do nothing here */
	jmp  done

LABEL(rxsoft)
	test bl, FLOW_SOFT				/* Used to have on?  */
	jz   txsoft					/* Didn't have soft  */
	/* RX XOFF handshake now off */
	and  cl, NOT(RX_XOFF)				/* Remove XOFF flag  */
	cmp  [ctlbyte], COM_XOFF			/* Had XOFF queued?  */
	jne  noqueue					/* Not in queue      */
	mov  [ctlbyte], 0				/* Scrap that XOFF!  */
	jmp  txsoft					/* XON not required  */
LABEL(noqueue)
	mov  [ctlbyte], COM_XON 			/* Send XON byte     */
	call agl_txstart				/* Start transmitter */

LABEL(txsoft)
	test ch, TX_XOFF				/* Flow in action?   */
	jz   done  
	/* TX XOFF handshake now off */
	and  ch, NOT(TX_XOFF)				/* Yes, clear now    */
	mov  [xofftimer], 0				/* Clear xoff timer  */
	jnz  done					/* Still tx inhibit? */
	call agl_txstart				/* START TRANSMITTER */

LABEL(done)
	mov  [txstate], ch
	mov  [rxstate], cl
	mov  [rxflow],	bh

LABEL(noneed)
	mov  ax, true

LABEL(fini)
	sti
    }
}/*agl_setflow()*/


/* ------------------------------------------------------------------------- */
COM_FLOW agl_getflow (void)
{
asm {
	xor  ah, ah
	mov  al, [flow]
    }
}/*agl_getflow()*/


/* ------------------------------------------------------------------------- */
boolean agl_havefifo (byte port)
{
	word p;

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (false);

	if (active && port == agl_port)
	   return ((fifo & FIFO_PRESENT) ? true : false);

	p = agl_portinfo[port].portbase;

asm {
	mov  bx, false					/* Assume no FIFO    */
	cli

	mov  dx, [p]
	add  dx, 2					/* FCR/IIR=BASE+2    */

	/* As FCR is read-only, we can't possibly restore original content;  */
	/* Therefore we first do a read-only check so that if the FIFO were  */
	/* already activated by another application, we see but do no harm.  */
	in   al, dx					/* Do a safe check   */
	and  al, (IIR_FIFO1 OR IIR_FIFO2)		/* Test FIFOs	     */
	cmp  al, (IIR_FIFO1 OR IIR_FIFO2)		/* BOTH bits=16550A  */
	jne  check					/* Do a real check   */
	mov  bx, true					/* FIFOs present     */
	jmp  fini

LABEL(check)
	mov  al, FCR_ENABLE
	jmp  $+2
	out  dx, al
	jmp  $+2
	in   al, dx
	and  al, (IIR_FIFO1 OR IIR_FIFO2)		/* Test FIFOs	     */
	cmp  al, (IIR_FIFO1 OR IIR_FIFO2)		/* BOTH bits=16550A  */
	jne  nofifo					/* Sorry, no okidoki */
	mov  bx, true					/* Ok, FIFOs present */
LABEL(nofifo)
	xor  al, al					/* Reset FIFOs again */
	jmp  $+2
	out  dx, al

LABEL(fini)
	sti
	mov  ax, bx
    }
}/*agl_havefifo()*/


/* ------------------------------------------------------------------------- */
boolean agl_setfifo (byte newtrigger)
{
asm {
	mov  ah, [newtrigger]

	mov  al, FCR_TRIG14				/* Trigger level 14  */
	cmp  ah, 14
	jge  ok
	mov  al, FCR_TRIG8				/* Trigger level 8   */
	cmp  ah, 8
	jge  ok
	mov  al, FCR_TRIG4				/* Trigger level 4   */
	cmp  ah, 4
	jge  ok
	mov  al, FCR_TRIG1				/* Trigger level 1   */

LABEL(ok)
	cmp  al, [trigger]
	je   fini					/* Same as before    */
	mov  [trigger], al

	cmp  [active], true				/* Are we active?    */
	jne  fini
	test [fifo], FIFO_ACTIVE			/* FIFOs enabled?    */
	jz   fini

	mov  dx, [uart_port]
	add  dx, 2					/* FCR=BASE+2	     */
	or   al, FCR_ENABLE				/* Add enable flag   */
	cli
	out  dx, al					/* New level to UART */
	sti

LABEL(fini)
	mov  ax, true
    }
}/*agl_setfifo()*/


/* ------------------------------------------------------------------------- */
byte agl_getfifo (void)
{
asm {
	xor  ah, ah
	mov  bl, [trigger]
	and  bl, FCR_TRIG14
	mov  al, 14
	cmp  bl, FCR_TRIG14
	je   fini
	mov  al, 8
	cmp  bl, FCR_TRIG8
	je   fini
	mov  al, 4
	cmp  bl, FCR_TRIG4
	je   fini
	mov  al, 1
LABEL(fini)
    }
}/*agl_getfifo()*/


/* ------------------------------------------------------------------------- */
boolean agl_usefifo (boolean enable)
{
asm {
	cli
	mov  ah, [fifo]

	cmp  [word ptr enable], true
	jne  disable

	cmp  [active], true				/* Are we active?    */
	jne  enno
	test ah, FIFO_ACTIVE				/* FIFOs enabled?    */
	jnz  enno
	mov  dx, [uart_port]
	add  dx, 2					/* FCR=BASE+2	     */
	mov  al, FCR_ENABLE				/* FIFO enable flag  */
	or   al, [trigger]				/* Set FIFO rcv trig */
	out  dx, al					/* New level to UART */
	or   ah, FIFO_ACTIVE				/* Set enabled flag  */
LABEL(enno)
	or   ah, FIFO_ALLOWED				/* Set allowed flag  */
	jmp  fini

LABEL(disable)
	cmp  [active], true				/* Are we active?    */
	jne  disno
	test ah, FIFO_ACTIVE				/* FIFOs enabled?    */
	jz   disno
	mov  dx, [uart_port]
	add  dx, 2					/* FCR=BASE+2	     */
	xor  al, al					/* Disable FIFOs     */
	out  dx, al					/* Write to UART     */
LABEL(disno)
	and  ah, NOT(FIFO_ALLOWED OR FIFO_ACTIVE)	/* Clear these flags */

LABEL(fini)
	mov  [fifo], ah
	sti

	mov  ax, true
    }
}/*agl_usefifo()*/


/* ------------------------------------------------------------------------- */
boolean agl_txbyte (byte c)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	mov  ax, [txsize]
	cmp  [txfill], ax				/* Check for space   */
	jnl  full					/* tx buffer is full */

	mov  al, [c]					/* Get byte to send  */

	mov  es, [word ptr txbuf + 2]			/* Get txin buf seg  */
	mov  di, [txin] 				/* Get txin buf ptr  */

	cmp  di, [txtop]				/* Test for buf top  */
	jl   notop					/* Not yet	     */
	mov  di, [word ptr txbuf]			/* Reset to bottom   */
LABEL(notop)

	cld
	stosb						/* Store byte in buf */

	mov  [txin], di 				/* Store new txin    */

	inc  [txfill]					/* One more to send  */

	test [txstate], TX_NODATA			/* Was empty before? */
	jz   nostart					/* Wasn't empty      */
	and  [txstate], NOT(TX_NODATA)			/* Reset NODATA flag */
	jnz  nostart					/* Still tx inihibit */
	mov  dx, [uart_port]
	call agl_txstart				/* START TRANSMITTER */

LABEL(nostart)
	mov  ax, true

LABEL(full)
	sti

LABEL(fini)
    }
}/*agl_txbyte()*/


/* ------------------------------------------------------------------------- */
boolean agl_txpush (byte c)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	mov  ax, [txsize]
	cmp  [txfill], ax				/* Check for space   */
	jnl  full					/* tx buffer is full */

	mov  al, [c]					/* Get byte to send  */

	mov  es, [word ptr txbuf + 2]			/* Get txout buf seg */
	mov  di, [txout]				/* Get txout buf ptr */

	cmp  di, [word ptr txbuf]			/* Test for buf bot. */
	ja   notop					/* Nope 	     */
	mov  di, [txtop]				/* Reset to top      */
LABEL(notop)

	dec  di 					/* First go backward */
	mov  [es:di], al				/* Push byte in buf  */

	mov  [txout], di				/* Store new txout   */

	inc  [txfill]					/* One more to send  */

	test [txstate], TX_NODATA			/* Was empty before? */
	jz   nostart					/* Wasn't empty      */
	and  [txstate], NOT(TX_NODATA)			/* Reset NODATA flag */
	jnz  nostart					/* Still tx inihibit */
	mov  dx, [uart_port]
	call agl_txstart				/* START TRANSMITTER */

LABEL(nostart)
	mov  ax, true

LABEL(full)
	sti

LABEL(fini)
    }
}/*agl_txpush()*/


/* ------------------------------------------------------------------------- */
word agl_txblock (byte far *buf, word len)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  cx, [len]					/* Send 0 bytes!?    */
	jcxz fini					/* Get out of here   */

	mov  ax, [txsize]				/* Get size of txbuf */
	sub  ax, [txfill]				/* Calc # bytes free */
	jz   fini					/* No space at all   */

	cmp  cx, ax					/* Enough space free */
	jle  txall					/* Yes, transfer all */
	mov  cx, ax					/* Nope, limit xfer  */
LABEL(txall)
	push cx 					/* Push # bytes sent */
	mov  dx, cx					/* # bytes left over */

	mov  es, [word ptr txbuf + 2]			/* Get txin buf seg  */
	mov  di, [txin] 				/* Get txin buf ptr  */
	mov  bx, [word ptr txbuf]			/* Get txbuf bottom  */
	mov  ax, [txtop]				/* Get tx top of buf */

	cld						/* Clear direction   */
	push ds 					/* Save DS	     */
	lds  si, [buf]					/* DS:SI = userbuf   */

	sub  ax, di					/* Calc free at top  */
	jz   bottom					/* Nowt, goto bottom */

	cmp  cx, ax					/* Enough top space  */
	jle  alltop					/* Yes, transfer all */
	mov  cx, ax					/* Nope, limit top   */
LABEL(alltop)

	sub  dx, cx					/* # left for bottom */

	shr  cx, 1
	jnc  eventop
	movsb						/* Copy byte to top  */
	jcxz endtop					/* Was only byte?    */
LABEL(eventop)
	rep  movsw					/* Copy words to top */
LABEL(endtop)
	mov  cx, dx					/* Get # remaining   */
	jcxz nomore					/* Nothing left      */

LABEL(bottom)
	mov  di, bx					/* Reset to bottom   */

	shr  cx, 1
	jnc  evenbottom
	movsb						/* Copy byte to bot  */
	jcxz nomore					/* Was only byte?    */
LABEL(evenbottom)
	rep  movsw					/* Copy words to bot */

LABEL(nomore)
	pop  ds 					/* Restore DS	     */

	mov  [txin], di 				/* Store new txin    */

	cli

	test [txstate], TX_NODATA			/* Was empty before? */
	jz   nostart					/* Wasn't empty      */
	and  [txstate], NOT(TX_NODATA)			/* Reset NODATA flag */
	jnz  nostart					/* Still tx inihibit */
	mov  dx, [uart_port]
	call agl_txstart				/* START TRANSMITTER */
LABEL(nostart)

	pop  ax 					/* Pop # bytes sent  */
	add  [txfill], ax				/* # more to send    */

	sti

LABEL(fini)
    }
}/*agl_txblock()*/


/* ------------------------------------------------------------------------- */
boolean agl_txempty (void)
{
asm {
	mov  ax, true
	cmp  [active], true
	jne  fini

	cmp  [txfill], 0
	je   fini

	mov  ax, false

LABEL(fini)
    }
}/*agl_txempty()*/


/* ------------------------------------------------------------------------- */
boolean agl_txready (void)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	mov  bx, [txsize]
	cmp  [txfill], bx
	jnl  fini

	mov  ax, true

LABEL(fini)
    }
}/*agl_txempty()*/


/* ------------------------------------------------------------------------- */
word agl_txfill (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  ax, [txfill]

LABEL(fini)
    }
}/*agl_txfill()*/


/* ------------------------------------------------------------------------- */
boolean agl_txclear (void)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	cmp  [txfill], 0
	je   noneed

	mov  ax, [word ptr txbuf]
	mov  [txin],   ax
	mov  [txout],  ax
	mov  [txfill], 0

	or   [txstate], TX_NODATA

LABEL(noneed)
	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_txclear()*/


/* ------------------------------------------------------------------------- */
boolean agl_txdisable (boolean disable)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	cmp  [word ptr disable], true
	jne  enable

	or   [txstate], TX_DISABLE
	jmp  done

LABEL(enable)
	test [txstate], TX_DISABLE
	jz   done
	and  [txstate], NOT(TX_DISABLE)
	jnz  done
	mov  dx, [uart_port]
	call agl_txstart

LABEL(done)
	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_txdisable()*/


/* ------------------------------------------------------------------------- */
int agl_rxbyte (void)
{
asm {
	mov  ax, COM_EOF
	cmp  [active], true
	jne  fini

	cli

	mov  cx, [rxfill]				/* Anything in buf?  */
	jcxz empty					/* Nothing in there  */

	mov  es, [word ptr rxbuf + 2]			/* Get rxout buf seg */
	mov  si, [rxout]				/* Get rxout buf ptr */

	cmp  si, [rxtop]				/* Test for buf top  */
	jl   notop					/* Not yet	     */
	mov  si, [word ptr rxbuf]			/* Reset to bottom   */
LABEL(notop)

	cld
	lods BYTE PTR es:[si]				/* Get byte from buf */

	mov  [rxout], si				/* Store new rxout   */
	dec  cx 					/* One less in buf   */

	test [rxstate], RX_HIGHBUF			/* Had high water?   */
	jz   done					/* Nope 	     */
	cmp  cx, [rxlow]				/* Buf low again?    */
	ja   done					/* No, still above   */
	and  [rxstate], NOT(RX_HIGHBUF) 		/* Undo highwater    */
	jnz  done					/* Still rx inhibit? */
	mov  ah, al					/* Remember rx byte! */
	call agl_rxstart				/* Undo flow control */
	mov  al, ah					/* Get rx byte back! */

LABEL(done)
	mov  [rxfill], cx				/* Store new rxfill  */

	xor  ah, ah

LABEL(empty)
	sti

LABEL(fini)
    }
}/*agl_rxbyte()*/


/* ------------------------------------------------------------------------- */
int agl_rxpeek (void)
{
asm {
	mov  ax, COM_EOF
	cmp  [active], true
	jne  fini

	cli

	cmp  [rxfill], 0				/* Anything in buf?  */
	jz   empty					/* Nothing in there  */

	mov  es, [word ptr rxbuf + 2]			/* Get rxout buf seg */
	mov  si, [rxout]				/* Get rxout buf ptr */

	cmp  si, [rxtop]				/* Test for buf top  */
	jl   notop					/* Not yet	     */
	mov  si, [word ptr rxbuf]			/* Reset to bottom   */
LABEL(notop)

	xor  ah, ah
	mov  al, [es:si]				/* Just take a peek  */

LABEL(empty)
	sti

LABEL(fini)
    }
}/*agl_rxpeek()*/


/* ------------------------------------------------------------------------- */
boolean agl_rxpush (byte c)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	mov  cx, [rxfill]
	cmp  cx, [rxsize]				/* Check for space   */
	jnl  full					/* rx buffer is full */

	mov  al, [c]					/* Get byte to stuff */

	mov  es, [word ptr rxbuf + 2]			/* Get rxout buf seg */
	mov  di, [rxout]				/* Get rxout buf ptr */

	cmp  di, [word ptr rxbuf]			/* Test for buf bot. */
	ja   notop					/* Nope 	     */
	mov  di, [rxtop]				/* Reset to top      */
LABEL(notop)

	dec  di 					/* First go backward */
	mov  [es:di], al				/* Push byte in buf  */

	mov  [rxout], di				/* Store new rxout   */
	inc  cx 					/* One more in rxbuf */

	test [rxstate], RX_HIGHBUF			/* Already high buf? */
	jnz  done					/* Yes, just go on   */
	cmp  cx, [rxhigh]				/* Check highwater   */
	jl   done					/* Plenty of room    */
	or   [rxstate], RX_HIGHBUF			/* Flag high water   */
	mov  dx, [uart_port]
	mov  bl, [flow]
	call agl_rxstop 				/* Handle flow ctrl  */

LABEL(done)
	mov  [rxfill], cx				/* Store new rxfill  */

	mov  ax, true

LABEL(full)
	sti

LABEL(fini)
    }
}/*agl_rxpush()*/


/* ------------------------------------------------------------------------- */
word agl_rxblock (byte far *buf, word len)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  cx, [len]					/* Get 0 bytes!?     */
	jcxz fini					/* Get out of here   */

	mov  ax, [rxfill]				/* Get # bytes avail */
	or   ax, ax
	jz   fini					/* Nothing in here   */

	cmp  cx, ax					/* Enough available? */
	jle  rxall					/* Yes, transfer all */
	mov  cx, ax					/* Nope, limit xfer  */
LABEL(rxall)
	push cx 					/* Push # bytes rcvd */
	mov  dx, cx					/* # bytes left over */

	cld						/* Clear direction   */
	push ds 					/* Save DS	     */
	les  di, [buf]					/* ES:DI = userbuf   */

	mov  bx, [word ptr rxbuf]			/* Get rxbuf bottom  */
	mov  ax, [rxtop]				/* Get rx top of buf */
	mov  si, [rxout]				/* Get rxout buf ptr */
	/* Get this one last because DS is changed */
	mov  ds, [word ptr rxbuf + 2]			/* Get rxout buf seg */

	sub  ax, si					/* Calc bytes at top */
	jz   bottom					/* Nowt, goto bottom */

	cmp  cx, ax					/* Enough to satisfy */
	jle  alltop					/* Yes, transfer all */
	mov  cx, ax					/* Nope, limit top   */
LABEL(alltop)

	sub  dx, cx					/* # left for bottom */

	shr  cx, 1
	jnc  eventop
	movsb						/* Cpy byte frm top  */
	jcxz endtop					/* Was only byte?    */
LABEL(eventop)
	rep  movsw					/* Cpy words frm top */
LABEL(endtop)
	mov  cx, dx					/* Get # left to do  */
	jcxz nomore					/* Nothing left      */

LABEL(bottom)
	mov  si, bx					/* Reset to bottom   */

	shr  cx, 1
	jnc  evenbottom
	movsb						/* Cpy byte frm bot  */
	jcxz nomore					/* Was only byte?    */
LABEL(evenbottom)
	rep  movsw					/* Cpy words frm bot */

LABEL(nomore)
	pop  ds 					/* Restore DS	     */

	mov  [rxout], si				/* Store new rxout   */
	pop  ax 					/* Pop # bytes rcvd  */

	cli

	sub  [rxfill], ax				/* # less in buf now */

	test [rxstate], RX_HIGHBUF			/* Had high water?   */
	jz   done					/* Nope 	     */
	mov  cx, [rxfill]				/* Get #bytes in buf */
	cmp  cx, [rxlow]				/* Buf low again?    */
	ja   done					/* No, still above   */
	and  [rxstate], NOT(RX_HIGHBUF) 		/* Undo highwater    */
	jnz  done					/* Still rx inhibit? */
	mov  cx, ax					/* Remember # bytes! */
	call agl_rxstart				/* Undo flow control */
	mov  ax, cx					/* Get # bytes back! */
LABEL(done)
	sti

LABEL(fini)
    }
}/*agl_rxblock()*/


/* ------------------------------------------------------------------------- */
boolean agl_rxempty (void)
{
asm {
	mov  ax, true
	cmp  [active], true
	jne  fini

	cmp  [rxfill], 0
	je   fini

	mov  ax, false

LABEL(fini)
    }
}/*agl_rxempty()*/


/* ------------------------------------------------------------------------- */
boolean agl_rxready (void)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cmp  [rxfill], 0
	je   fini

	mov  ax, true

LABEL(fini)
    }
}/*agl_rxempty()*/


/* ------------------------------------------------------------------------- */
word agl_rxfill (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  ax, [rxfill]

LABEL(fini)
    }
}/*agl_rxfill()*/


/* ------------------------------------------------------------------------- */
boolean agl_rxclear (void)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	cmp  [rxfill], 0
	je   done

	mov  ax, [word ptr rxbuf]
	mov  [rxin],   ax
	mov  [rxout],  ax
	mov  [rxfill], 0

	test [rxstate], RX_HIGHBUF			/* Had high water?   */
	jz   done					/* Nope 	     */
	and  [rxstate], NOT(RX_HIGHBUF) 		/* Undo highwater    */
	jnz  done					/* Still rx inhibit? */
	call agl_rxstart				/* Undo flow control */

LABEL(done)
	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_rxclear()*/


/* ------------------------------------------------------------------------- */
boolean agl_rxdisable (boolean disable)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli

	cmp  [word ptr disable], true
	jne  enable

	test [rxstate], RX_DISABLE			/* Already disabled? */
	jnz  done
	or   [rxstate], RX_DISABLE			/* Flag rx disable   */
	mov  dx, [uart_port]
	mov  bl, [flow]
	call agl_rxstop 				/* Do flow control   */
	jmp  done

LABEL(enable)
	test [rxstate], RX_DISABLE			/* Already enabled?  */
	jz   done
	and  [rxstate], NOT(RX_DISABLE) 		/* Undo rx disable   */
	jnz  done					/* Still rx inhibit? */
	call agl_rxstart				/* Undo flow control */

LABEL(done)
	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_rxdisable()*/


/* ------------------------------------------------------------------------- */
boolean agl_break (word millisecs)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli  

	mov  bl, [txstate]				/* Get txstate	     */
	test bl, TX_BRK 				/* Break already on? */
	jnz  done					/* Yes, so ignore    */
	/* One could make a case for just restarting the timer, but....      */
	/* some modems do funny stuff if a break is sustained for very long  */
	or   bl, TX_BRK
	mov  [txstate], TX_BRK

	mov  dx, [uart_port]
	add  dx, 3					/* LCR=BASE+3	     */
	in   al, dx					/* Get LCR from UART */
	or   al, LCR_SETBRK				/* Set BRK bit to on */
	jmp  $+2
	out  dx, al					/* Put LCR into UART */

	xor  dx, dx					/* Clear upper word  */
	mov  ax, [millisecs]				/* Get lower word    */
	add  ax, (COM_TICKMSECS - 1)			/* Upwards next tick */
	/* Above 'add' also prevents division by zero is user behaves silly! */
	cmp  ax, 1000					/* More than 1 sec?  */
	jle  nothigh
	mov  ax, 1000					/* Limit this cookie */
LABEL(nothigh)
	mov  bx, COM_TICKMSECS				/* Get divisor (55)  */
	div  bx 					/* Divide msec->tick */
	mov  [brktimer], ax				/* Set break timer   */
	/* Timer will start countdown when we re-enable interrupts, and also */
	/* automagically reset the break condition upon expiration; cute ay? */

LABEL(done)
	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_break()*/


/* ------------------------------------------------------------------------- */
boolean agl_carrier (void)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	mov  bl, [agl_msr]				/* Get modem status  */
	test bl, [dcd]					/* Mask DCD bit(s)   */
	jz   fini					/* Not high = false  */

	mov  ax, true					/* High     = true   */

LABEL(fini)
    }
}/*agl_carrier()*/


/* ------------------------------------------------------------------------- */
boolean agl_setdcd (byte dcdmask)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	mov  al, [dcdmask]				/* Load new DCDmask  */
	mov  [dcd], al					/* Store new DCDmask */

	mov  ax, true

LABEL(fini)
    }
}/*agl_setdcd()*/


/* ------------------------------------------------------------------------- */
byte agl_getdcd (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  al, [dcd]					/* Get DCDmask value */

LABEL(fini)
    }
}/*agl_getdcd()*/


/* ------------------------------------------------------------------------- */
boolean agl_setdtr (boolean enable)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli  

	mov  dx, [uart_port]
	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx					/* Get MCR from UART */

	cmp  [word ptr enable], true			/* Up or down?	     */
	jne  disable
	or   al, MCR_DTR				/* Raise DTR line    */
	jmp  store
LABEL(disable)
	and  al, NOT(MCR_DTR)				/* Lower DTR line    */
LABEL(store)
	jmp  $+2
	out  dx, al					/* Put MCR into UART */

	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_setdtr()*/


/* ------------------------------------------------------------------------- */
boolean agl_setloop (boolean enable)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli  

	mov  dx, [uart_port]
	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx					/* Get MCR from UART */

	cmp  [word ptr enable], true			/* Up or down?	     */
	jne  disable
	or   al, MCR_LOOP				/* Raise LOOP flag   */
	jmp  store
LABEL(disable)
	and  al, NOT(MCR_LOOP)				/* Lower LOOP flag   */
LABEL(store)
	jmp  $+2
	out  dx, al					/* Put MCR into UART */

	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_setloop()*/


/* ------------------------------------------------------------------------- */
boolean agl_setout1 (boolean enable)
{
asm {
	mov  ax, false
	cmp  [active], true
	jne  fini

	cli  

	mov  dx, [uart_port]
	add  dx, 4					/* MCR=BASE+4	     */
	in   al, dx					/* Get MCR from UART */

	cmp  [word ptr enable], true			/* Up or down?	     */
	jne  disable
	or   al, MCR_OUT1				/* Raise OUT1 line   */
	jmp  store
LABEL(disable)
	and  al, NOT(MCR_OUT1)				/* Lower OUT1 line   */
LABEL(store)
	jmp  $+2
	out  dx, al					/* Put MCR into UART */

	sti

	mov  ax, true

LABEL(fini)
    }
}/*agl_setout1()*/


/* ------------------------------------------------------------------------- */
byte agl_getlsr (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  al, [agl_lsr]

LABEL(fini)
    }
}/*agl_getlsr()*/


/* ------------------------------------------------------------------------- */
byte agl_getmsr (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	mov  al, [agl_msr]

LABEL(fini)
    }
}/*agl_getmsr()*/


/* ------------------------------------------------------------------------- */
byte agl_geterr (void)
{
asm {
	xor  ax, ax
	cmp  [active], true
	jne  fini

	cli
	xchg al, [agl_errors]				/* Read/reset status */
	sti

LABEL(fini)
    }
}/*agl_geterr()*/


/* ------------------------------------------------------------------------- */
byte agl_uart (byte port)
{
	byte type;
	byte b, save;

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (NOUART);

	if (active && port == agl_port && (fifo & FIFO_PRESENT))
	   return (NS16550A);

	disable();

	b = inportb(agl_portinfo[port].portbase + UART_IIR);
	if (b == 0xff)
	   type = NOUART;
	else {
	   b &= IIR_FIFO1 | IIR_FIFO2;
	   if (b)
	      type = (b == (IIR_FIFO1 | IIR_FIFO2)) ? NS16550A : NS16550;
	   else {
	      save = inportb(agl_portinfo[port].portbase + UART_IER);
	      outportb(agl_portinfo[port].portbase + UART_IER,0);
	      b = inportb(agl_portinfo[port].portbase + UART_IER);
	      outportb(agl_portinfo[port].portbase + UART_IER,save);
	      if (b)
		 type = NOUART;
	      else {
		 outportb(agl_portinfo[port].portbase + UART_FCR,FCR_ENABLE);
		 b = inportb(agl_portinfo[port].portbase + UART_IIR);
		 outportb(agl_portinfo[port].portbase + UART_FCR,0);
		 b &= IIR_FIFO1 | IIR_FIFO2;
		 if (b)
		    type = (b == (IIR_FIFO1 | IIR_FIFO2)) ? NS16550A : NS16550;
		 else {
		    save = inportb(agl_portinfo[port].portbase + UART_SCR);
		    b = save ^ 0xff;
		    outportb(agl_portinfo[port].portbase + UART_SCR,b);
		    type = (inportb(agl_portinfo[port].portbase + UART_SCR) == b) ? NS16450 : NS8250;
		    outportb(agl_portinfo[port].portbase + UART_SCR,save);
		 }
	      }
	   }
	}

	enable();

	return (type);
}/*agl_uart()*/


/* ------------------------------------------------------------------------- */
boolean agl_modem (byte port)
{
	byte m;

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (false);

	if (active && port == agl_port)
	   m = agl_msr;
	else
	   m = inportb(agl_portinfo[port].portbase + UART_MSR);

	return ((m & (MSR_CTS | MSR_DSR)) == (MSR_CTS | MSR_DSR) ? true : false);
}/*agl_modem()*/


/* ------------------------------------------------------------------------- */
boolean agl_putuart (byte port, byte uartreg, COM_PGMUART method, byte value)
{
	word p;
	byte c;

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (false);
	if (method > PGM_PUT) return (false);

	asm cli

	p = agl_portinfo[port].portbase + uartreg;
	if (method == PGM_PUT)
	   c = value;
	else {
	   c = inportb(p);
	   switch (method) {
		  case PGM_AND: c &= value; break;
		  case PGM_OR:	c |= value; break;
		  case PGM_XOR: c ^= value; break;
		  case PGM_PUT: break;
	   }
	}
	outportb(p,c);

	asm sti

	return (true);
}/*agl_putuart()*/


/* ------------------------------------------------------------------------- */
byte agl_getuart (byte port, byte uartreg)
{
	byte val;

	if (port >= COM_MAXPORTS || !agl_portinfo[port].portbase)
	   return (0);

	asm cli
	val = inportb(agl_portinfo[port].portbase + uartreg);
	asm sti
	asm xor ah, ah

	return (val);
}/*agl_getuart()*/


/* end of aglcom.c --------------------------------------------------------- */
