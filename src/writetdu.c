#include <stdio.h>
#include <stdlib.h>
#include <tdu.h>
#include <fcntl.h>

/*
 * cc -c -I. -o writetdu.o writetdu.c
 *
 * "-I." needed until tdu.h is under /usr/include/
 */

void writetdu( char* pbuf   // pointer to start of 2-D array
             , int width   // width of 2-D array pbuf in pixels
             , int nl      // number of lines (rows) in 2-D array pbuf
             ) {
int fd, i;

  fd = open("/dev/tdu0", O_WRONLY, 0);
  if (ioctl(fd, TDURESET, width) < 0) { exit(-1); }
  for ( i=0; i<nl; ++i, pbuf+=width) { write( fd, pbuf, width); }
  close(fd);
  return;
}

