include ../Makefile.inc

CFLAGS	= $(PRJCFLAGS) -I..
OBJLIBS	= ../libslam.a 
SRC =$(wildcard *.cpp) 
OBJS=$(SRC:.cpp=.o)

all : $(OBJLIBS)

../libslam.a : $(OBJS)
	$(ECHO) $(AR) $(ARFLAGS) rv ../libslam.a $?
	$(AR) $(ARFLAGS) rv ../libslam.a $?
	$(ECHO) $(RANLIB) ../libslam.a
	$(RANLIB) ../libslam.a

clean :
	$(ECHO) cleaning up in SLAM
	-$(RM) -f $(OBJS)

force_look :
	true