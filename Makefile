include Makefile.inc

DIRS	= MathLib SLAM
EXE	= msgd
OBJS	= main.o
OBJLIBS	= libmath.a libslam.a 
LIBS	= -L. -lmath -lslam

all : $(EXE)

$(EXE) : main.o $(OBJLIBS)
	$(ECHO) $(LD) -o $(EXE) $(OBJS) $(LIBS)
	$(LD) -o $(EXE) $(OBJS) $(LIBS)

libmath.a : force_look
	$(ECHO) looking into MathLib : $(MAKE) $(MFLAGS)
	cd MathLib; $(MAKE) $(MFLAGS)

libslam.a : force_look
	$(ECHO) looking into SLAM : $(MAKE) $(MFLAGS)
	cd SLAM; $(MAKE) $(MFLAGS)

clean :
	$(ECHO) cleaning up in .
	-$(RM) -f $(EXE) $(OBJS) $(OBJLIBS)
	-for d in $(DIRS); do (cd $$d; $(MAKE) clean ); done

force_look :
	true