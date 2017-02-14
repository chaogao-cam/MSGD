include ../Makefile.inc

CFLAGS	= $(PRJCFLAGS) -I..
OBJLIBS	= ../libmath.a
SRC =$(wildcard *.cpp) 
OBJS=$(SRC:.cpp=.o)

all : $(OBJLIBS)

../libmath.a : $(OBJS)
	$(ECHO) $(AR) $(ARFLAGS) rv ../libmath.a $?
	$(AR) $(ARFLAGS) rv ../libmath.a $?
	$(ECHO) $(RANLIB) ../libmath.a
	$(RANLIB) ../libmath.a

clean :
	$(ECHO) cleaning up in MathLib
	-$(RM) -f $(OBJS)

force_look :
	true