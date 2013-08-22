/*
tunetdu.c - tunelp for /dev/tdu? (ocean data tdu-850 printer)

FUNCTIONS

This source file includes following functions. 

  1.print_usage 
  2.mylloc 
  3.get_val 
  4.get_onoff 
  5.main 

*/

/****************************************************************************\
*       Copyright (C) 1992-1997 Michael K. Johnson, johnsonm@redhat.com      *
*                                                                            *
*       This file is licensed under the terms of the GNU General             *
*       Public License, version 2, or any later version.  See file COPYING   *
*       for information on distribution conditions.                          *
\****************************************************************************/

/* $Id: tunelp.c,v 1.8 1997/07/06 00:14:06 aebr Exp $
 * $Log: tunelp.c,v $
 * Revision 1.8  1997/07/06 00:14:06  aebr
 * Fixes to silence -Wall.
 *
 * Revision 1.7  1997/06/20 16:10:38  janl
 * tunelp refreshed from authors archive.
 *
 * Revision 1.9  1997/06/20 12:56:43  johnsonm
 * Finished fixing license terms.
 *
 * Revision 1.8  1997/06/20 12:34:59  johnsonm
 * Fixed copyright and license.
 *
 * Revision 1.7  1995/03/29 11:16:23  johnsonm
 * TYPO fixed...
 *
 * Revision 1.6  1995/03/29  11:12:15  johnsonm
 * Added third argument to ioctl needed with new kernels
 *
 * Revision 1.5  1995/01/13  10:33:43  johnsonm
 * Chris's changes for new ioctl numbers and backwards compatibility
 * and the reset ioctl.
 *
 * Revision 1.4  1995/01/03  17:42:14  johnsonm
 * -s isn't supposed to take an argument; removed : after s in getopt...
 *
 * Revision 1.3  1995/01/03  07:36:49  johnsonm
 * Fixed typo
 *
 * Revision 1.2  1995/01/03  07:33:44  johnsonm
 * revisions for lp driver updates in Linux 1.1.76
 *
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
/* #include <linux/lp.h> */
/* #include <linux/tdu.h> */
#include "tdu.h"
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>

struct command {
  long op;
  long val;
  struct command *next;
};


void print_usage(char *progname) {
  printf("Usage: %s <device> [ options ]\n  OPTIONS:
-h            print this message
-t <TIME>     set char timeout
-c <CHARS>    set # tries per char
-w <WAIT>     set lo/hi (CKI, EOLI, KYI) duration
-a on|off     whether to abort on any operation
-o on|off     whether to abort on open
-r <WIDTH>    set width, reset tdu driver
-S            return status port bits & width
-C            return control port bits
-D            return data port bits
-T 0|1|2|3    toggle control port bit C0|C1|C2|C3 (requires root privilege)
", progname);

  exit (1);
}





void *mylloc(long size) {
  void *ptr;
  if(!(ptr = (void*)malloc(size))) {
    perror("malloc error");
    exit(2);
  }
  return ptr;
}



long get_val(char *val) {
  long ret;
  if (!(sscanf(val, "%ld", &ret) == 1)) {
    perror("sscanf error");
    exit(3);
  }
  return ret;
}


long get_onoff(char *val) {
  if (!strncasecmp("on", val, 2))
    return 1;
  return 0;
}



int main (int argc, char ** argv) {
  int c, fd, status, retval;
  char *progname;
  char *filename;
  struct stat statbuf;
  struct command *cmds, *cmdst;


  progname = argv[0];
  if (argc < 2) print_usage(progname);

  filename = strdup(argv[1]);
  fd = open(filename, O_WRONLY|O_NONBLOCK, 0);
  /* Need to open O_NONBLOCK in case ABORTOPEN is already set and
     printer is off or off-line or in an error condition.  Otherwise
     we would abort... */
  if (fd < 0) {
    perror(argv[1]);
    return -1;
  }

  fstat(fd, &statbuf);

  if((!S_ISCHR(statbuf.st_mode)) || (MAJOR(statbuf.st_rdev) != TDU_MAJOR )
     || (MINOR(statbuf.st_rdev) > 3)) {
    printf("%s: %s not a tdu device.\n", progname, argv[1]);
    print_usage(progname);
  }

  cmdst = cmds = mylloc(sizeof(struct command));
  cmds->next = 0;

  while ((c = getopt(argc, argv, "hSDCT:r:t:c:w:a:o:")) != EOF) {
    switch (c) {
    case 'h':			/* help */
      print_usage(progname);
      break;

    case 'S':			/* get status */
      cmds->op = TDUGETSTATUS;
      cmds->val = 0;
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;

    case 'D':			/* get data port bits */
      cmds->op = TDUGETDATA;
      cmds->val = 0;
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;

    case 'C':		/* get control port bits */
    case 'T':		/* toggle control port bit - get before & after */

      cmds->op = TDUGETCONTROL;
      cmds->val = 0;
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      if ( c == 'C') break;

      cmds->op = TDUCTOGGLEBIT;
      cmds->val = get_val(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;

      cmds->op = TDUGETCONTROL;
      cmds->val = 0;
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;

      break;

    case 'r':			/* reset */
      cmds->op = TDURESET;
      cmds->val = get_val(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    case 't':			/* set char timeout if char not taken */
      cmds->op = TDUTIME;
      cmds->val = get_val(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    case 'c':			/* number of times to try to output a char */
      cmds->op = TDUCHAR;
      cmds->val = get_val(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    case 'w':			/* strobe (CKI) duration between transitions */
      cmds->op = TDUWAIT;
      cmds->val = get_val(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    case 'a':			/* whether to abort operation on error */
      cmds->op = TDUABORT;
      cmds->val = get_onoff(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    case 'o':			/* whether to abort open() on error */
      cmds->op = TDUABORTOPEN;
      cmds->val = get_onoff(optarg);
      cmds->next = mylloc(sizeof(struct command));
      cmds = cmds->next; cmds->next = 0;
      break;
    default: 
      print_usage(progname);
      break;
    }
  }

  cmds = cmdst;
  while (cmds->next) {
    switch ( cmds->op) {
    case TDUGETSTATUS:
      {
      int width;
        status = 0xdeadbeef;
        retval = ioctl(fd, TDUGETSTATUS, &status);
        if (retval < 0)
          perror("TDUGETSTATUS error");
        else {
          if (status == 0xdeadbeef)      /* a few 1.1.7x kernels will do this */
            status = retval;
          printf("%s status port bits are 0x%08x", filename, status);
          width = status & 0xffffff;
          printf( ", width=%d", width);
          status >>= 24;
          if TDU_SRDYO(status) printf(", ready");
          if TDU_SBSYO(status) printf(", busy");
          if TDU_SPO(status) printf(", out of paper");
          printf("\n");
        }
      }
      break;

    case TDUGETCONTROL:
      status = 0xdeadbeef;
      retval = ioctl(fd, TDUGETCONTROL, &status);
      if (retval < 0)
        perror("TDUGETCONTROL");
      else {
        if (status == 0xdeadbeef)      /* a few 1.1.7x kernels will do this */
          status = retval;
        printf("%s control port bits are 0x%02x", filename, status);
        printf( ", -CKI/C1- %s", TDU_CTSTCKI(status) ? "on" : "off");
        printf( ", -EOLI/C2+ %s", TDU_CTSTEOLI(status) ? "on" : "off");
        printf( ", -KYI/C3- %s", TDU_CTSTKYI(status) ? "on" : "off");
        printf("\n");
      }
      break;

    case TDUGETDATA:
      status = 0xdeadbeef;
      retval = ioctl(fd, TDUGETDATA, &status);
      if (retval < 0)
        perror("TDUGETDATA error");
      else {
        if (status == 0xdeadbeef)      /* a few 1.1.7x kernels will do this */
          status = retval;
        printf("%s data port bits are 0x%02x\n", filename, status);
      }
      break;

    case TDUCTOGGLEBIT:
      if ( cmds->val < 0 || cmds->val > 3) {
        print_usage(progname);
        break;
      }
      status = 1 << cmds->val;
      retval = ioctl(fd, TDUCTOGGLEBIT, status);
      if (retval < 0)
        perror("TDUGETSTATUS error");
      /* sleep( 1); */
      break;

    default:
      if (ioctl(fd, cmds->op, cmds->val) < 0) {
        perror("tunetdu: ioctl");
      } else printf( "%04x\n", (int) cmds->op);
      break;
    } /* switch cmd->op */
    cmdst = cmds;
    cmds = cmds->next;
    free(cmdst);
  } /* while cmds->next */

  close(fd);

  return 0;
}
