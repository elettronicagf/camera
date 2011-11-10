include ../makefile_conf/Makefile.rules
CPPFLAGS += $(DEFINES) $(OPTIMIZE)  -Wall 
CFLAGS  = -I$(KERNEL_DIR)   -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp -mthumb-interwork -mno-thumb -fomit-frame-pointer -frename-registers -ggdb2 -Wall -Winline -fno-strength-reduce
PROGS_O  = camera.o
PROG    = camera
all: $(PROG)

$(PROG): objs
	$(CPP) $(CFLAGS) $(LFLAGS) -o $(PROG) $(PROGS_O) $(LIBS)
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

