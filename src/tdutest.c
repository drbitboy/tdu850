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
  printf("Usage: %s <device> [ -t <TIME> | -c <CHARS> | -w <WAIT> | \n"
         "          -a [on|off] | -o [on|off] | -s ]\n", progname);
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
  int c, fd;
  char *progname;
  char *filename;
  struct stat statbuf;
  unsigned char buf[256+1728], *pbuf;


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
  
  if (ioctl(fd, TDURESET, 1728) < 0) {
      perror("tunetdu: ioctl");
  }

  for ( pbuf=buf+(c=0); pbuf<(buf+256+1728); ++pbuf) *pbuf = c++;

  for ( pbuf=buf; pbuf<(buf+256); ++pbuf) {
    printf( "%d,%03d/", write( fd, pbuf, 1728), (int) *pbuf);
    fflush( stdout);
  }
  printf( "\n");

  close(fd);

  return 0;
}
