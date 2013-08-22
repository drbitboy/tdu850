/********************************************
 * Ocean Data TDU-850 character device driver
 *
 *****************************************
 * - BASED ON lp  (tdu/TDU replaces lp/LP)
 *   - see "Generic parallel printer driver" below
 *
 *******************************
 * - see tdu.h for cable pinouts
 *   - if you have a homemade cable, it should be easier to change the 
 *     bit macros in tdu.h than to make a new cable.  
 *     ***N.B. TDUFIXCTL() macro may need to be modified depending on 
 *             what you do with TDU_PSTROBE
 *
 *****************************************
 * - To create device (see Makefile also):
 *
 *   # rm -f /dev/tdu0
 *   # mknod -m 0666 /dev/tdu0 c 60 0
 *
 *   - ***N.B. "60" is TDU_MAJOR in tdu.h
 *
 *************************************
 * - To load tdu device driver module:
 *
 *   # insmod tdu.o parport=0 reset=1
 *
 ***************************************************
 * - to automatically load this via kerneld, add something like this to 
 *   /etc/conf.modules (or modules.conf?):
 *
 *   keep
 *   path[misc]=/root/tdu/modules
 *   alias char-major-60 tdu
 *   options tdu parport=0 reset=1
 *
 *
 **************************************************
 * - To send a raster image to the tdu (/dev/tdu0):
 *
 *   (1) Open /dev/tduN
 *   (2) set the width to the number of columns in each line (1728 max)
 *   (3) send the bytes to the device
 *   (4) close the device
 *
 *   **********
 *   program e.g. (stuff on right after '#' are comments)
 *
 *   #include <stdio.h>
 *   #include <tdu.h>
 *   char *pbuf;
 *   char tduname[] = { "/dev/tdu0" };
 *   int fd, width;
 *
 *   ...               # alloc pbuf, load source bytes into pbuf, set width
 *
 *   fd = open(tduname, O_WRONLY, 0);                                   # (1)
 *   if (ioctl(fd, TDURESET, width) < 0) { exit(-1); }                  # (2)
 *   for ( i=0; i<nl; ++i, pbuf+=width) { write( fd, pbuf, width); }    # (3)
 *   close(fd);                                                         # (4)
 *   return 0;
 *
 *   - ***N.B. write() may not print all characters sent due to 
 *             error/offline condition
 *     - to be more rigorous, program should check the number of characters 
 *       sent by write (the return value) and resend unwritten characters 
 *       after condition is cleared
 *
 *   **********
 *   shell e.g.
 *
 *   % tunetdu /dev/tdu0 -r 1024                      # (1), (2)
 *   % tail +4 xyz.pgm > /dev/tdu0                    # (1), (3), (4)
 *
 *   - assumes xyz.pgm is raw PGM file with 3 header lines & width = 1024
 *
 * brian carcich October, 2000
 *****************************
 *
 * Generic parallel printer driver
 *
 * Copyright (C) 1992 by Jim Weigand and Linus Torvalds
 * Copyright (C) 1992,1993 by Michael K. Johnson
 * - Thanks much to Gunter Windau for pointing out to me where the error
 *   checking ought to be.
 * Copyright (C) 1993 by Nigel Gamble (added interrupt code)
 * Copyright (C) 1994 by Alan Cox (Modularised it)
 * TDUCAREFUL, TDUABORT, TDUGETSTATUS added by Chris Metcalf,metcalf@lcs.mit.edu
 * Statistics and support for slow printers by Rob Janssen, rob@knoware.nl
 * "tdu=" command line parameters added by Grant Guenther, grant@torque.net
 * tdu_read (Status readback) support added by Carsten Gross,
 *                                             carsten@sol.wohnheim.uni-ulm.de
 * Support for parport by Philip Blundell <Philip.Blundell@pobox.com>
 * Parport sharing hacking by Andrea Arcangeli
 * Fixed kernel_(to/from)_user memory copy to check for errors
 * 				by Riccardo Facchetti <fizban@tin.it>
 * Redesigned interrupt handling for handle printers with buggy handshake
 *				by Andrea Arcangeli, 11 May 1998
 * Full efficient handling of printer with buggy irq handshake (now I have
 * understood the meaning of the strange handshake). This is done sending new
 * characters if the interrupt is just happened, even if the printer say to
 * be still BUSY. This is needed at least with Epson Stylus Color. To enable
 * the new TRUST_IRQ mode read the `TDU OPTIMIZATION' section below...
 * Fixed the irq on the rising edge of the strobe case.
 * Obsoleted the CAREFUL flag since a printer that doesn' t work with
 * CAREFUL will block a bit after in tdu_check_status().
 *				Andrea Arcangeli, 15 Oct 1998
 */

/* This driver should, in theory, work with any parallel port that has an
 * appropriate low-level driver; all I/O is done through the parport
 * abstraction layer.
 *
 * If this driver is built into the kernel, you can configure it using the
 * kernel command-line.  For example:
 *
 *	tdu=parport1,none,parport2	(bind tdu0 to parport1, disable tdu1 and
 *					 bind tdu2 to parport2)
 *
 *	tdu=reset			(reset the printer during 
 *					 initialisation)
 *
 *	tdu=off				(disable the printer driver entirely)
 *
 * If the driver is loaded as a module, similar functionality is available
 * using module parameters.  The equivalent of the above commands would be:
 *
 *	# insmod tdu.o parport=1,none,2
 *
 *	# insmod tdu.o parport=auto
 *
 *	# insmod tdu.o reset=1
 */

/*
 * TDU OPTIMIZATIONS
 *
 * - TDU_WAIT time
 *
 * You can use this setting if your printer is fast enough and/or your
 * machine is slow enough ;-).
 *
 * tunetdu /dev/tdu? -w 0
 *
 */

/* COMPATIBILITY WITH OLD KERNELS
 *
 * TDU:  probably none
 *
 * Under Linux 2.0 and previous versions, tdu devices were bound to ports at
 * particular I/O addresses, as follows:
 *
 *	tdu0		0x3bc
 *	tdu1		0x378
 *	tdu2		0x278
 *
 * The new driver, by default, binds tdu devices to parport devices as it
 * finds them.  This means that if you only have one port, it will be bound
 * to tdu0 regardless of its I/O address.  If you need the old behaviour, you
 * can force it using the parameters described above.
 */

#include <linux/module.h>
#include <linux/init.h>

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/devfs_fs_kernel.h>
/* #include <linux/malloc.h> */
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>

#include <linux/parport.h>
/*OUTFORDEVELOPMENT:  #include <linux/tdu.h> */
#include <tdu.h>

#include <asm/uaccess.h>
#include <asm/system.h>

/* if you have more than 3 TDU printers, remember to increase TDU_NO */

#define TDU_NO 3

struct tdu_struct tdu_table[TDU_NO] =
{
  [0 ... TDU_NO-1] = {NULL, 0, TDU_INIT_CHAR, TDU_INIT_TIME, TDU_INIT_WAIT,
                      TDU_MAXWIDTH, 0,
                      NULL, NULL, 0}
};

/* --- parport support ----------------------------------------- */

#define tdu_parport_release(x) do { parport_release(tdu_table[(x)].dev); \
                                   } while (0);
#define tdu_parport_claim(x) do { parport_claim_or_block(tdu_table[(x)].dev); \
                                } while (0);

/*****************************
 * Disabled 20030321
 */
#if 0
static int tdu_preempt(void *handle)
{
struct tdu_struct *tdus = (struct tdu_struct *)handle;

  if (waitqueue_active (&tdus->wait_q))
    wake_up_interruptible(&tdus->wait_q);

  /* Don't actually release the port now */
  return 1;
}
#endif

static __inline__ void tdu_yield (int minor)
{
  if (!parport_yield_blocking (tdu_table[minor].dev))
  {
    if (current->need_resched)
      schedule ();
  }
}

static __inline__ void tdu_schedule(int minor, long timeout)
{
  struct pardevice *dev = tdu_table[minor].dev;
  register unsigned long int timeslip = (jiffies - dev->time);
  if ((timeslip > dev->timeslice) && (dev->port->waithead != NULL)) {
    tdu_parport_release(minor);
    schedule_timeout(timeout);
    tdu_parport_claim(minor);
  } 
  else
    schedule_timeout(timeout);
}

/* --- low-level port access ----------------------------------- */
/* dtr=>data  str=status  ctr=control */

/*************
 * port macros
 * - read data, status & control ports
 * - write control & data ports
 */
#  define r_dtr(x)	(parport_read_data(tdu_table[(x)].dev->port))
#  define r_str(x)	(parport_read_status(tdu_table[(x)].dev->port))
#  define r_ctr(x)	(parport_read_control(tdu_table[(x)].dev->port))
#  define w_ctr(x,y)	do { \
                          parport_write_control(tdu_table[(x)].dev->port \
                          , TDUFIXCTL(y)); \
                        } while (0)
#  define w_dtr(x,y)	do { \
                          parport_write_data(tdu_table[(x)].dev->port, (y)); \
                        } while (0)

#define	tdu_wait(minor)	udelay(TDU_WAIT(minor))

/************************
 * reset printer & driver
 * - resets control bits CKI, KYI & EOLI
 *   - toggles EOLI in case printer is in the middle of a line
 * - sets width of future lines (so driver knows when to send EOLI)
 * - sets column to 0
 * - has its own claim & release
 *   - release if necessary before claiming
 */
static int tdu_reset(int minor, int newwidth)
{
  int retval;
  unsigned char ctr;

  tdu_parport_claim (minor);

  TDU_COL(minor) = 0;				/* set to first column */

  newwidth = newwidth ? newwidth : TDU_MAXWIDTH;  /* 0 becomes TDU_MAXWIDTH */

  if ( newwidth >= 0 && newwidth <= TDU_MAXWIDTH) {
    TDU_WIDTH(minor) = newwidth;
    printk(KERN_INFO 
    "tdu%d reset; width = %d\n"
    , minor, newwidth);
  } else if ( newwidth ) {
    TDU_WIDTH(minor) = TDU_MAXWIDTH;
    printk(KERN_INFO 
    "tdu%d invalid width (%d) requested; resetting to %d\n"
    , minor, newwidth, TDU_MAXWIDTH);
  }

  /* reset control port pins */
  ctr = TDU_CRSTALL;
  w_ctr(minor, ctr);

  /* send EOLI in case printer thinks it is in the middle of a line */
  tdu_wait(minor);
  w_ctr( minor, ctr = TDU_CSETEOLI(ctr));
  tdu_wait(minor);
  w_ctr( minor, ctr = TDU_CRSTEOLI(ctr));
  tdu_wait(minor);

  retval = r_str(minor);
  tdu_parport_release (minor);
  return retval;
} /* static int tdu_reset(int minor, int newwidth) */

/*******************************************************************************
 * tdu_char - send one character
 *            - special -KYI processing for 1st character (TDU_COL == 0)
 *            - special -EOLI processing for last char (++TDU_COL == TDU_WIDTH)
 *
 * no ACK or INTERRUPT/IRQ, the only feedback comes via -RDYO & -PO, 
 * and -RDYO will always be on when -PO is on.  given that, tdu_char 
 * needs to 1) put data on data pins, 2) set -CKI, 3) wait, 4) reset -CKI,
 * 5) delay
 * - take yields out, the tdu cannot share the port
 */

static inline int tdu_char(char tduchar, int minor)
{
long count = 0;
unsigned char rctr;	/* actual control status */
unsigned char ctr;	/* desired control status */
unsigned char status;

  if (signal_pending(current))
    return 0;

  /* compare actual & desired (all reset) control status */

  rctr = r_ctr(minor) & TDU_CTLALL;
  ctr = TDU_CRSTALL;

  if ( rctr != ctr ) {
    printk(KERN_INFO "tdu%d->tdu_char():  ctrl port state was 0x%02x"
                   , minor, rctr);
    w_ctr( minor, ctr);	                 /* reset control port to known state */
    printk("; set to 0x%02x", ctr);
    tdu_wait(minor);	                 /* allow time after change e.g. Tkyh */
    rctr = r_ctr(minor);
    printk("; now 0x%02x\n", rctr);
  }

  status = r_str(minor);                            /* test for ready (-RDYO) */

  while (!TDU_COL(minor)) {	     /* first column, wait for RDYO, send KYI */

    if (TDU_SRDYO(status)) {
      ctr = TDU_CSETKYI(ctr);
      w_ctr( minor, ctr);
      tdu_wait(minor);	                         /* Thkck - host -KYI to -CKI */
      break;
    }

    if (++count >= TDU_CHAR(minor))
    {
      printk(KERN_INFO 
              "tdu%d->tdu_char():  Start of line NOT RDY after %ld tries\n"
            , minor, count);
      break;
    }
    udelay( 2500);		/* should this be a schedule_timeout? */
    status = r_str(minor);
  }

  if (TDU_SBSYO(status)) {
    return 0;
  }

  /* TIMING - from TDU-850 Op & Maint Manual:
   * Tsudck:  min 0 ns before strobe that data must be on data pins
   * Thdck:   min 400 ns data hold following strobe low
   * Tckl:    min 200 ns strobe low
   * Tckh:    min 200 ns strobe high
   * Tpck:    min 500 ns total strobe cycle
   * ***N.B. minimum wait in driver is udelay(1) = 1 microsecond
   *         - 1us is greater than all minima above, so use that 
   *           i.e. tdu_wait(minor) macro, defaults to 1 us
   */

  w_dtr(minor, tduchar);			/* put d0-d7 onto data pins */
						/* Tsudck = 0 */
  w_ctr(minor, ctr = TDU_CSETCKI(ctr));		/* -CKI on i.e. strobe */
  tdu_wait(minor);				/* Tckl */
  if  (!TDU_COL(minor)) 			/* for 1st line ... */
    ctr = TDU_CRSTKYI(ctr);			/* ... reset -KYI */
  w_ctr(minor, ctr = TDU_CRSTCKI(ctr));		/* -CKI off */
  tdu_wait(minor);				/* Tckh, Tsukck */

  if ( ++TDU_COL(minor) < TDU_WIDTH(minor)) return 1; /* return if not at eol */

  /***************************************
   * special handling at end of line (eol)
   */
  TDU_COL(minor) = 0;				/* reset column to 0 */
  w_ctr( minor, ctr=TDU_CSETEOLI(ctr));		/* -EOLI on */
  tdu_wait(minor);				/* Teolil */
  w_ctr( minor, ctr = TDU_CRSTEOLI(ctr));	/* -EOLI off */

  return 1;
} /* static inline int tdu_char(char tduchar, int minor) */

/*****************************************
 * fancy wait - TDU_TIMEOUT_POLLED jiffies
 */
static void tdu_errorwait(int minor)
{
  if ( TDU_PREEMPTED(minor)) {
    current->state = TASK_INTERRUPTIBLE;
    tdu_parport_release(minor);
    current->state = TASK_INTERRUPTIBLE;
    schedule_timeout(TDU_TIMEOUT_POLLED);
    tdu_parport_claim(minor);
  }
}

/*************************************************************
 * look for error status (offline, paper out)
 * - if error exists:
 *   - if ABORT ON ERROR set then return 1
 *   - else do tdu_errorwait() & return 0
 */
static int tdu_check_status(int minor)
{
  unsigned int last = tdu_table[minor].last_error;
  unsigned char status = r_str(minor);

  if (TDU_SRDYO(status)) {
      last = 0;
  } else {
    if (TDU_SPO(status)) {
      printk(KERN_INFO "tdu%d out of paper\n", minor);
      last = TDU_TPO;
    } else {
      printk(KERN_INFO "tdu%d offline or error\n", minor);
      last = TDU_TRDYO;
    }
  }

  tdu_table[minor].last_error = last;

  if (last != 0) {
    if (TDU_F(minor) & TDU_ABORT) return 1;
    tdu_errorwait(minor);
  }

  return 0;
} /* static int tdu_check_status(int minor) */

/***********************************
 * write buffer of characters to tdu
 * - assumes TDU_WIDTH(minor) & TDU_COL(minor) set/handled elsewhere
 */
static int tdu_write_buf(unsigned int minor, const char *buf, int count)
{
  unsigned long copy_size;
  unsigned long total_bytes_written = 0;
  unsigned long bytes_written;
  struct tdu_struct *tdu = &tdu_table[minor];

  if (minor >= TDU_NO) return -ENXIO;
  if (tdu->dev == NULL) return -ENXIO;

  tdu_table[minor].last_error = 0;

  do {
    bytes_written = 0;
    copy_size = (count < TDU_BUFFER_SIZE ? count : TDU_BUFFER_SIZE);

    if (copy_from_user(tdu->buffer, buf, copy_size))
    {
      w_ctr(minor, TDU_CRSTALL);
      return -EFAULT;
    }

    while (copy_size) {
      if (tdu_char(255 -((unsigned char)tdu->buffer[bytes_written]), minor)) {
        --copy_size;
        ++bytes_written;
      } else {
        int rc = total_bytes_written + bytes_written;


        if (signal_pending(current))       /* test for interrupt */
        {
          w_ctr(minor, TDU_CRSTALL);
          return rc ? rc : -EINTR;
        }

        /* tdu_check_status checks for error, 
         * returns 1 if TDU_ABORT set in TDU_F (.flags),
	 * else waits TDU_TIMEOUT_POLLED jiffies & returns 0
         */
        if (tdu_check_status(minor))
        {
          w_ctr(minor, TDU_CRSTALL);
          return rc ? rc : -EIO;
        }

        /* wait before trying again */

        current->state = TASK_INTERRUPTIBLE;
        tdu_schedule(minor, TDU_TIME(minor));

      } /* if tdu_char(...) ... else ... */

    } /* while copy_size */

    total_bytes_written += bytes_written;
    buf += bytes_written;
    count -= bytes_written;

  } while (count > 0); /* do ... */

  w_ctr(minor, TDU_CRSTALL);
  return total_bytes_written;
} /* tdu_write_buf(unsigned int minor, const char *buf, int count) */


/*************************************************
 * device driver interface (wrapper) for tdu_write_buf() above
 */
static ssize_t tdu_write(struct file * file, const char * buf,
            size_t count, loff_t *ppos)
{
  unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
  ssize_t retv;

  /* Claim Parport or sleep until it becomes available
   */
  tdu_parport_claim (minor);

  retv = tdu_write_buf(minor, buf, count);

  tdu_parport_release (minor);
  return retv;
} /* tdu_write(struct file * file, const char * buf, ... */

/**********************************
 * Disabled 20030321
 */
#if 0
 * tdu_lseek should never be called
 */
static long long tdu_lseek(struct file * file, long long offset, int origin)
{
  return -ESPIPE;
}
#endif

/*************************************************
 * open tdu
 */
static int tdu_open(struct inode * inode, struct file * file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  if (minor >= TDU_NO) return -ENXIO;
  if ((TDU_F(minor) & TDU_EXIST) == 0) return -ENXIO;
  if (test_and_set_bit(TDU_BUSY_BIT_POS, &TDU_F(minor))) return -EBUSY;

  MOD_INC_USE_COUNT;

  /* If ABORTOPEN is set and the printer is offline or out of paper,
   * we may still want to open it to perform ioctl()s.  Therefore we
   * have commandeered O_NONBLOCK, even though it is being used in
   * a non-standard manner.  This is strictly a Linux hack, and
   * should most likely only ever be used by the tunetdu application.
   */
  if ((TDU_F(minor) & TDU_ABORTOPEN) && !(file->f_flags & O_NONBLOCK)) {
    int status;
    tdu_parport_claim (minor);
    status = r_str(minor);
    tdu_parport_release (minor);
    if TDU_SPO(status) {
      printk(KERN_INFO "tdu%d out of paper\n", minor);
      MOD_DEC_USE_COUNT;
      TDU_F(minor) &= ~TDU_BUSY;
      return -ENOSPC;
    } else if TDU_SBSYO(status) {
      printk(KERN_INFO "tdu%d off-line or error\n", minor);
      MOD_DEC_USE_COUNT;
      TDU_F(minor) &= ~TDU_BUSY;
      return -EIO;
    }
  }

  tdu_table[minor].buffer = (char *) kmalloc(TDU_BUFFER_SIZE, GFP_KERNEL);
  if (!tdu_table[minor].buffer) {
    MOD_DEC_USE_COUNT;
    TDU_F(minor) &= ~TDU_BUSY;
    return -ENOMEM;
  }
  return 0;
}

/************
 * close tdu?
 */
static int tdu_release(struct inode * inode, struct file * file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  /* kfree_s(tdu_table[minor].buffer, TDU_BUFFER_SIZE); */
  kfree(tdu_table[minor].buffer);
  tdu_table[minor].buffer = NULL;
  MOD_DEC_USE_COUNT;
  TDU_F(minor) &= ~TDU_BUSY;
  return 0;
}

/***********
 * ioctl tdu
 */
static int tdu_ioctl(struct inode *inode, struct file *file,
        unsigned int cmd, unsigned long arg)
{
  unsigned int minor = MINOR(inode->i_rdev);
  int status;
  int rctr;
  int retval = 0;

  if (minor >= TDU_NO) return -ENODEV;
  if ((TDU_F(minor) & TDU_EXIST) == 0) return -ENODEV;

  /* must be root user for some commands */

  if ( !(cmd < TDUROOTREQD || suser()) ) return -EINVAL;

  switch ( cmd ) {

    /*********************************************************
     * diagnostics to set &/or get status, control & data bits
     */
    case TDUGETSTATUS:
      tdu_parport_claim(minor);
      status = r_str(minor);
      status <<= 24;                    /* assumes sizeof(int) > 3 */
      status |= TDU_WIDTH(minor);       /* and width < (1 << 24) */
      tdu_parport_release(minor);
      if (copy_to_user((int *) arg, &status, sizeof(int)))
        return -EFAULT;
      break;

    case TDUGETCONTROL:
      tdu_parport_claim(minor);
      status = r_ctr(minor);
      if (copy_to_user((int *) arg, &status, sizeof(int)))
        return -EFAULT;
      tdu_parport_release(minor);
      break;

    case TDUGETDATA:
      tdu_parport_claim(minor);
      status = r_dtr(minor);
      if (copy_to_user((int *) arg, &status, sizeof(int)))
        return -EFAULT;
      tdu_parport_release(minor);
      break;

    case TDUCTOGGLEBIT:                                 /* toggle control bit */
      tdu_parport_claim(minor);
      rctr = r_ctr(minor);
      printk(KERN_INFO "tdu%d toggling control 0x%02x ^ 0x%02x;\n"
                     , minor, rctr, (int)arg);
      rctr ^= arg;
      printk(KERN_INFO "tdu%d - writing control port = 0x%02x\n", minor, rctr);
      w_ctr(minor, rctr);

      for ( rctr=0; rctr < 2000 ; ++rctr) udelay( 1000);  /* 5 seconds */
      rctr = r_ctr(minor);
      printk(KERN_INFO "tdu%d - control = 0x%02x\n", minor, rctr);

      tdu_parport_release(minor);
      break;

    /************************************************************************
     * set width, reset column, reset control bits (clock in, key in, eol in)
     */
    case TDURESET:                      /* *****N.B. here is how width is set */
      tdu_reset(minor,arg);
      break;

    /*********************************************************************
     * set width, timing, timeout, number-of-tries, ABORT, abort parameters 
     */
    case TDUTIME:/* jiffies to wait between trying to write single characters */
      TDU_TIME(minor) = arg * HZ/100;
      break;

    case TDUCHAR:    /* # of times to wait at start of line for printer ready */
      TDU_CHAR(minor) = arg;
      break;

    case TDUWAIT:                             /* microseconds to hold signals */
      TDU_WAIT(minor) = arg;
      break;

    case TDUABORT:                                          /* abort on error */
      if (arg)
        TDU_F(minor) |= TDU_ABORT;
      else
        TDU_F(minor) &= ~TDU_ABORT;
      break;

    case TDUABORTOPEN:                          /* abort on error during open */
      if (arg)
        TDU_F(minor) |= TDU_ABORTOPEN;
      else
        TDU_F(minor) &= ~TDU_ABORTOPEN;
      break;

    case TDUGETFLAGS:                           /* get tdu_table[minor].flags */
      status = TDU_F(minor);
      if (copy_to_user((int *) arg, &status, sizeof(int)))
        return -EFAULT;
      break;

    default:
      retval = -EINVAL;
  }
  return retval;
}


/**************************************************
 * file operation routines for TDU character device
 */
static struct file_operations tdu_fops = {
        owner:          THIS_MODULE,
        write:          tdu_write,
        ioctl:          tdu_ioctl,
        open:           tdu_open,
        release:        tdu_release,
};

/**************************************************
 * Old code, replaced 20030321:
 */
#if 0
static struct file_operations tdu_fops = {
  tdu_lseek,
  NULL,		/* tdu_read */
  tdu_write,
  NULL,		/* tdu_readdir */
  NULL,		/* tdu_poll */
  tdu_ioctl,
  NULL,		/* tdu_mmap */
  tdu_open,
  NULL,		/* flush */
  tdu_release
};
#endif

/*****************************************************************/
/* --- initialisation code ------------------------------------- */
/*****************************************************************/

/**********************************************
 * read reset int & parport[] string parameters
 *
 * - MODULAR:  from insmod command or alias in modules.dep? e.g.
 *     # insmod tdu.o parport=0 reset=1
 *
 * - KERNEL:  from boot line, anywhere else?
 *
 */
#ifdef MODULE

static int parport_nr[TDU_NO] = { [0 ... TDU_NO-1] = TDU_PARPORT_NONE };
static char *parport[TDU_NO] = { NULL,  };
static int reset = 0;

MODULE_PARM(parport, "1-" __MODULE_STRING(TDU_NO) "s");
MODULE_PARM(reset, "i");

/***************************/
#else /* #ifdef MODULE ... */       /* **********N.B. this section not tested */

static int parport_nr[TDU_NO] __initdata = 
					{ [0 ... TDU_NO-1] = TDU_PARPORT_NONE };
static int reset __initdata = 0;

static int parport_ptr = 0;

__initfunc(void tdu_setup(char *str, int *ints))
{
  if (!str) {
    if (ints[0] == 0 || ints[1] == 0) {
      /* disable driver on "tdu=" or "tdu=0" */
      parport_nr[0] = TDU_PARPORT_OFF;
    } else {
      printk(KERN_WARNING "warning: 'tdu=0x%x' is deprecated, ignored\n"
            , ints[1]);
    }
  } else if (!strncmp(str, "parport", 7)) {
    int n = simple_strtoul(str+7, NULL, 10);
    if (parport_ptr < TDU_NO)
      parport_nr[parport_ptr++] = n;
    else
      printk(KERN_INFO "tdu: too many ports, %s ignored.\n", str);
  } else if (!strcmp(str, "off")) {
    parport_nr[0] = TDU_PARPORT_OFF;
  } else if (!strcmp(str, "none")) {
    parport_nr[parport_ptr++] = TDU_PARPORT_NONE;
  } else if (!strcmp(str, "reset")) {
    reset = 1;
  }
}

#endif /* #ifdef MODULE ... #else ... */
/*************************************/

/*************************************************************************
 * register parport device from (struct parport *) port into tdu_table[nr]
 * - called by tdu_init below
 */
int tdu_register(int nr, struct parport *port)
{
  tdu_table[nr].dev = parport_register_device(port, "tdu", 
               NULL,		/* preempt */ /* tdu_preempt out 20030321 */
               NULL,		/* wakeup */
               NULL		/* interrupt */, 
               PARPORT_DEV_EXCL,
               (void *) &tdu_table[nr]);
  if (tdu_table[nr].dev == NULL)
    return 1;
  tdu_table[nr].flags |= TDU_EXIST;

  if (reset) tdu_reset(nr,TDU_MAXWIDTH);

  printk(KERN_INFO "tdu%d (tdu_register()): using %s at 0x%04x.\n"
                 , nr, port->name, (int) port->base);
  return 0;
}

/******************************************
 * go through parports (parport_enumerate),
 * - call tdu_register for each port where 
 *   port->number matches one of parport_nr[] 
 * - if at least one tdu is registered, 
 *   - register tdu as a character device
 */
int tdu_init(void)
{
  unsigned int count = 0;
  unsigned int i;
  struct parport *port;

  if (parport_nr[0] == TDU_PARPORT_OFF) return 0;

  for (i = 0; i < TDU_NO; i++) {
    for (port = parport_enumerate(); port; port = port->next) {
      if (port->number == parport_nr[i]) {
        if (!tdu_register(i, port))
          count++;
        break;
      }
    }
  }

  if (count) {
    if (devfs_register_chrdev(TDU_MAJOR, "tdu", &tdu_fops)) {
      printk(KERN_INFO "tdu: unable to get major %d\n", TDU_MAJOR);
      return -EIO;
    }
  } else {
    printk(KERN_INFO "tdu: no devices found\n");
    return -ENODEV;
  }
  return 0;
}

#ifdef MODULE

/****************************
 * MODULAR TDU initialisation
 * - interpret parport[] string parameters, load parport_nr[]
 * - call tdu_init() above to go through parport_nr[]
 */
int init_module(void)
{
  if (parport[0]) {
    /* The user gave some parameters.  Let's see what they were.  */
    int n;
    for (n = 0; n < TDU_NO && parport[n]; n++) {
      if (!strncmp(parport[n], "none", 4))
        parport_nr[n] = TDU_PARPORT_NONE;
      else {
        char *ep;
        unsigned long r = simple_strtoul(parport[n], &ep, 0);
        if (ep != parport[n]) 
          parport_nr[n] = r;
        else {
          printk(KERN_ERR "tdu: bad port specifier `%s'\n", parport[n]);
          return -ENODEV;
        }
      }
    }
  }

  return tdu_init();
}

/*********************
 * MODULAR TDU cleanup
 * - probably in response to remove module command e.g.
 *   # rmmod tdu
 */
void cleanup_module(void)
{
  unsigned int offset;

  devfs_unregister_chrdev(TDU_MAJOR, "tdu");
  for (offset = 0; offset < TDU_NO; offset++) {
    if (tdu_table[offset].dev == NULL)
      continue;
    parport_unregister_device(tdu_table[offset].dev);
  }
}
#endif /* #ifdef MODULE */
