

COMPILER_PREFIX=arm-angstrom-linux-gnueabi
CC=$(COMPILER_PREFIX)-gcc
CPP=$(COMPILER_PREFIX)-g++
STRIP=$(COMPILER_PREFIX)-strip

KERNEL_DIR = /data2/sdonati/products/0458_otomelara_omap3/kernel/include

CFLAGS	 = -I$(KERNEL_DIR)  -O2 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp -mthumb-interwork -mno-thumb -fexpensive-optimizations -fomit-frame-pointer -frename-registers -O2 -ggdb2  $(DEFINES) $(OPTIMIZE) -Wall -Winline -fno-strength-reduce
CPPFLAGS = -I$(KERNEL_DIR)  -O2 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp -mthumb-interwork -mno-thumb -fexpensive-optimizations -fomit-frame-pointer -frename-registers -O2 -ggdb2 -Wall -Winline -fno-strength-reduce

PROGS_O  = camera_2.6.37.o
PROG    = camera
all: $(PROG)

$(PROG): objs
	$(CPP) $(CFLAGS) $(LFLAGS) -o camera camera_2.6.37.o $(LIBS)
#	$(STRIP) $(PROG)
xd:  xd.o
	gcc  -o xd xd.o 
xd.o:  xd.c
	gcc  -c xd.c 
objs:   $(PROGS_O)


.c.o:
	$(CC) $(CFLAGS)  -c -o $*.o $< 

.o:
	$(CPP) $(CFLAGS) $(LFLAGS) -o $* $(PROGS_O) $(LIBS)

clean:       cleanbin
	rm -f *.o *~ 

cleanbin:
	rm -f $(PROG)

