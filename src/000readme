tdu850
======

########################################################################
Linux device driver for Ocean Data TDU-850, continuous thermal printer,
300dpi, 8-bit gray levels.

Written and working at Cornell University ca. 2000-2003 to print Mars
Global Surveyor "noodle" images from the Mars Orbiter Camera (MOC).

Shameless copy of lp driver.

Placed on Github in 2013.

########################################################################

Device driver:

  tdu.c
  tdu.h
  Makefile
  N.B. uses lp and parport.


Command-line IOCTL(2) interface:

  tunetdu.c
  - a la tunelp


Device names:

   /dev/tduN (e.g. /dev/tdu0)


Building and Loading Driver:

  See tdu.c


Usage Quick Start (after building and loading driver):


  From shell:

    tunetdu /dev/tdu0 -r 1024   # Reset TDU 0, set width to 1024 pixels
    cat image.dat > /dev/tdu0   # Send 1024 x N byte data to TDU 0


  From C (writetdu.c; see tdu.c for more details):

    #include <stdio.h>
    #include <stdlib.h>
    #include <tdu.h>
    #include <fcntl.h>

    void writetdu( char* pbuf   // pointer to start of 2-D array of bytes
                 , int width    // width of 2-D array pbuf in pixels
                 , int nl       // number of lines (rows) in 2-D array pbuf
                 ) {
    int fd, i;

      fd = open("/dev/tdu0", O_WRONLY, 0);
      if (ioctl(fd, TDURESET, width) < 0) { exit(-1); }
      for ( i=0; i<nl; ++i, pbuf+=width) { write( fd, pbuf, width); }
      close(fd);
      return;
    }


  From GDL/IDL:  see idl_examples/

    - http://gnudatalanguage.sourceforge.net/

    - http://www.exelisvis.com/ProductsServices/IDL.aspx


Miscellany:

  See tdu.c for brief comments about using and building this package.

  See tdu.h for cable pinouts.

  Will need some work (Makefile) to function with current linux kernels.


########################################################################
Copyright Brian Carcich, 2013.
Many other copyrights from lp driver also apply; see tdu.c.
Latchmoor Services, LLC.
