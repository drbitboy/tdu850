#ifndef _LINUX_TDU_H
#define _LINUX_TDU_H

#include <linux/lp.h>

/* #define TDU_MAJOR LP_MAJOR */

#define TDU_MAJOR 60		/* experimental */

/*
 * Ocean Data TDU-850 printer driver
 * - slave mode
 * - with Ocean Data-delivered cable (no ACK to parport => no interrupt):
 *
 *                           TDU
 *   TDU          DIR       J1 PIN      PP PIN   PP              REGISTER
 *   NAME         TDU<>PP   (DB37)      (DB25)   NAME            BIT
 *   =======================================================================
 *
 *   D7-D0           <       6-9,12-15   9-2      DATA7-DATA0     D7-D0
 *   (data)
 *
 *   =======================================================================
 *
 *   -CKI/-STB       <       11          14       -AUTOFD         C1-  0x02
 *   (clkin/strobe)
 *
 *   -EOLI           <       16          16       -INIT           C2+  0x04
 *   (eol in)
 *
 *   -KYI            <       18          17       -SELECT         C3-  0x08
 *   (key in)
 *
 *   Not Used        <       17          1        -STROBE         C0-  0x01
 *
 *   =======================================================================
 *
 *   -RDYO            >      4           11       +BUSY           S7- 0x80
 *   (ready out)
 *
 *   -PO              >      10          12       +PAPEREND       S5+ 0x20
 *   (paper out)
 *
 *   reserved         >      19          10       -ACK            S6+ 0x40
 *
 *   =======================================================================
 *
 *   -ACK             >      1           X
 *   (strobeack)
 *
 *   -EOLO            >      2           X (wired to TDU J1 pin 11 (-CKI/-STB)
 *   (eol out)                              via 1 kohm resistor)
 *
 *   -CKO             >      3           X
 *   (clock out)
 *
 *   -KYO             >      5           X
 *   (key out)
 *
 *   =======================================================================
 *
 *   - what about end of line out (-EOLO) wired to -CKI/-STB?
 *
 * - BASED ON lp (tdu/TDU replaces lp/LP)
 *
 * usr/include/linux/tdu.h c.2000 Brian Carcich
 *
 * usr/include/linux/lp.h c.1991-1992 James Wiegand
 * many modifications copyright (C) 1992 Michael K. Johnson
 * Interrupt support added 1993 Nigel Gamble
 */

/*
 * Per POSIX guidelines, this module reserves the TDU and tdu prefixes
 * These are the tdu_table[minor].flags flags...
 */

#define TDU_EXIST 0x0001
#define TDU_SELEC 0x0002
#define TDU_BUSY	 0x0004
#define TDU_BUSY_BIT_POS 2
#define TDU_OFFL	 0x0008
#define TDU_NOPA  0x0010
#define TDU_ERR   0x0020
#define TDU_ABORT 0x0040
#define TDU_CAREFUL 0x0080
#define TDU_ABORTOPEN 0x0100

#define TDU_MAXWIDTH 1728

/* timeout for each character.  This is relative to bus cycles -- it
 * is the count in a busy loop.  THIS IS THE VALUE TO CHANGE if you
 * have extremely slow printing, or if the machine seems to slow down
 * a lot when you print.  If you have slow printing, increase this
 * number and recompile, and if your system gets bogged down, decrease
 * this number.  This can be changed with the tunetdu(8) command as well.
 */

#define TDU_INIT_CHAR 1000

/* The parallel port specs apparently say that there needs to be
 * a .5usec wait before and after the strobe.
 */

#define TDU_INIT_WAIT 1

/* This is the amount of time that the driver waits for the printer to
 * catch up when the printer's buffer appears to be filled.  If you
 * want to tune this and have a fast printer (i.e. HPIIIP), decrease
 * this number, and if you have a slow printer, increase this number.
 * This is in hundredths of a second, the default 2 being .05 second.
 * Or use the tunetdu(8) command, which is especially nice if you want
 * change back and forth between character and graphics printing, which
 * are wildly different...
 */

#define TDU_INIT_TIME 2

/***************
 * IOCTL numbers
 */
#define TDUGETSTATUS 0x3C01  /* return TDU_S(minor) */
#define TDURESET     0x3C02  /* reset printer */
#define TDUTIME      0x3C03  /* corresponds to TDU_INIT_TIME */
#define TDUCHAR      0x3C04  /* corresponds to TDU_INIT_CHAR */
#define TDUWAIT      0x3C05  /* corresponds to TDU_INIT_WAIT */
#define TDUABORT     0x3C06  /* call with TRUE arg to abort on error, */
                             /* FALSE to retry.  Default is retry.  */
#define TDUABORTOPEN 0x3C07  /* call with TRUE arg to abort open() on error, */
                             /*  FALSE to ignore error.  Default is ignore.  */
#define TDUGETFLAGS  0x3C08  /* get TDU_F(minor) flags */
#define TDUGETWIDTH  0x3C09  /* get current width */
#define TDUGETCOL    0x3C0A  /* get current column */

#define TDUGETCONTROL 0x3C0B /* get control port bits */
#define TDUGETDATA   0x3C0C  /* get control port bits */

/******************************************************************************
 * IOCTL numbers greater than or equal to TDUROOTREQD - root privilege required
 */
#define TDUROOTREQD   TDUCTOGGLEBIT
#define TDUCTOGGLEBIT 0x3CF0                      /* toggle control port bits */
#define TDUSETDATA   0x3CF1                             /* set data port bits */

/**********************
 * end of IOCTL numbers
 **********************/

/********************************************************************
 * timeout for printk'ing a timeout, in jiffies (100ths of a second).
 * This is also used for re-checking error conditions if TDU_ABORT is
 * not set.  This is the default behavior.
 */

#define TDU_TIMEOUT_POLLED	(10 * HZ)

/**************************************************************************
 * The following constants describe the various signals of the printer port
 * hardware.  Note that the hardware inverts some signals and that some
 * signals are active low.  An example is TDU_CKI, which must be programmed
 * with 1 for being active and 0 for being inactive, because the strobe signal
 * gets inverted, but it is also active low.
 */

/**********************************
 * bit defines for 8255 status port
 * base + 1
 * accessed with TDU_S(minor), which gets the byte...
 */
#define TDU_TRDYO	0x80  /* inverted input, active low:  1 = active */
#define TDU_TBSYO	0x80  /* inverted input, active high:  0 = active */
#define TDU_TPO		0x20  /* unchanged input, active high:  1 = active */

/**************************************************************
 * unused control bits we want to set appropriately for the TDU
 * -except for strobe, no pins, port bit 1 => active
 */
#define TDU_PSTROBE	0x01  /* parport strobe pin DB25-1; 1=>low=>active */
#define TDU_PINTEN	0x10  /* port hardware interrupt enable */
#define TDU_PINPMOD	0x20  /* IEEE-1284 input enable */

/***********************
 * MACROs to test Status
 *
 * LO/HI => active when electrically low/high
 * DIR => BIT=1 = electrical hi (i.e. DIRect)
 * INV => BIT=0 = electrical lo (i.e. INVerted)
 */

/* TDU_TST*:  TEST */

#define TDU_TSTHIDIR(A,BIT) ((A) & BIT)			/* 1=active:  mask */
#define TDU_TSTLODIR(A,BIT) (TDU_TSTHIDIR(A,BIT) ^ BIT)	/* 0=active:  mask,XOR*/
#define TDU_TSTHIINV TDU_TSTLODIR			/* ~1=0=active */
#define TDU_TSTLOINV TDU_TSTHIDIR			/* ~0=1=active */

/* TDU_S*:  printer status, test only  */

#define	TDU_SRDYO(A)	TDU_TSTLOINV(A,TDU_TRDYO)	/* S7- -ReadyOut */
#define	TDU_SPO(A)	TDU_TSTLODIR(A,TDU_TPO)		/* S5+ -PaperOut */

/* special, BUSY = ~RDYO */

#define TDU_SBSYO(A)    TDU_TSTHIINV(A,TDU_TBSYO)	/* S7- +Busy= ~(-RDYO)*/

/* 
 * defines for 8255 control port
 * base + 2 
 * accessed with TDU_C(minor)
 */
#define TDU_TKYI	0x08  /* start line:  inv output, act low:  1 = act */
#define TDU_TEOLI	0x04  /* end line: unch output, act low:  0 = act */
#define TDU_TCKI	0x02  /* clock in:  inv output, act low:  1 = act */

/*******************************************
 * MACROs to set & reset test Control lines
 */

/* TDU_SET*:  SET */

#define TDU_SETHIDIR(BIT) | BIT			/* 1 => active */
#define TDU_SETLODIR(BIT) & ~BIT		/* 0 => active */
#define TDU_SETHIINV(BIT) TDU_SETLODIR(BIT)	/* ~1 = 0 => active */
#define TDU_SETLOINV(BIT) TDU_SETHIDIR(BIT)	/* ~0 = 1 => active */

/* TDU_RST*:  RESET */

#define TDU_RSTHIDIR(BIT) TDU_SETLODIR(BIT)
#define TDU_RSTLODIR(BIT) TDU_SETHIDIR(BIT)
#define TDU_RSTHIINV(BIT) TDU_RSTLODIR(BIT)
#define TDU_RSTLOINV(BIT) TDU_RSTHIDIR(BIT)

/* TDU_C*:  printer control
 * TDU_CSET*:  set active
 * TDU_CRST*:  reset (set inactive)
 * TDU_CTST*:  test
 */

#define	TDU_CSETCKI(A)	(A TDU_SETLOINV(TDU_TCKI))	/* C1- -ClocKIn/Strobe*/
#define	TDU_CRSTCKI(A)	(A TDU_RSTLOINV(TDU_TCKI))
#define	TDU_CTSTCKI(A)	TDU_TSTLOINV(A,TDU_TCKI)

#define	TDU_CSETEOLI(A)	(A TDU_SETLODIR(TDU_TEOLI))	/* C2+ -EOLI */
#define	TDU_CRSTEOLI(A)	(A TDU_RSTLODIR(TDU_TEOLI))
#define	TDU_CTSTEOLI(A)	TDU_TSTLODIR(A,TDU_TEOLI)

#define	TDU_CSETKYI(A)	(A TDU_SETLOINV(TDU_TKYI))	/* C2- -KeyIn */
#define	TDU_CRSTKYI(A)	(A TDU_RSTLOINV(TDU_TKYI))
#define	TDU_CTSTKYI(A)	TDU_TSTLOINV(A,TDU_TKYI)

/*******************************************************************
 * printer reset - reset all control bits in which we are interested
 */
#define	TDU_CRSTALL	TDU_CRSTCKI( TDU_CRSTEOLI( TDU_CRSTKYI( 0)))

/**************************************
 * fix unused control bits when writing
 * - disable interrupts:  +C4
 * - set data direction to output:  +C5
 * - set strobe pin low:  -C0
 */
#define TDUFIXCTL(Y) (((Y) & ~(TDU_PINTEN|TDU_PINPMOD)) | TDU_PSTROBE)

/* mask of all control bits in which we are interested */

#define TDU_CTLALL	(TDU_TKYI|TDU_TEOLI|TDU_TCKI)

/*****************************
 * stuff only needed by kernel
 */
#ifdef __KERNEL__

/* Magic numbers for defining port-device mappings */
#define TDU_PARPORT_OFF -2
#define TDU_PARPORT_NONE -1

#define TDU_F(minor)	tdu_table[(minor)].flags	/* flags for busy, &c */
#define TDU_CHAR(minor)	tdu_table[(minor)].chars	/* busy timeout */
#define TDU_TIME(minor)	tdu_table[(minor)].time		/* wait time */
#define TDU_WAIT(minor)	tdu_table[(minor)].wait		/* strobe wait */
#define TDU_WIDTH(minor) tdu_table[(minor)].width	/* 1728 max */
#define TDU_COL(minor)	tdu_table[(minor)].col		/* current column */
#define TDU_BUFFER_SIZE 256		/* FYI, 256 * 6.75 = 1728 */

#define TDU_BASE(x)	tdu_table[(x)].dev->port->base

struct tdu_struct {
	struct pardevice *dev;
	unsigned long flags;	/* flags for busy, &c */
	unsigned int chars;	/* number of 2500 us delays to wait for ready */
	unsigned int time;	/* */
	unsigned int wait;	/* usecond delay to hold control pin pulses */
	unsigned int width;	/* width to print, 1728 max */
	unsigned int col;	/* current column */
	char *buffer;
	struct wait_queue *wait_q;
	unsigned int last_error;
};

#define TDU_PREEMPTED(minor) (tdu_table[(minor)].dev->port->waithead != NULL)

/*********************
 * function prototypes
 */
extern int tdu_init(void);

#endif	/* __KERNEL__ */

#endif /* _LINUX_TDU_H */
