# 
CPP  = g++
LIBS =  -L/usr/X11R6/lib -lglut -lGL -lGLU -lX11 -lXi -lXmu -lm
INCS =  
CXXINCS = 
CXXFLAGS = -Wall $(CXXINCS)
CFLAGS = $(INCS)  -fmessage-length=0

# 
all: TD_animation_3D

# 
TD_animation_3D:	TD_animation_3D.o MDVectors.o
	$(CPP) TD_animation_3D.o MDVectors.o -o $@ $(CXXFLAGS) $(LIBS)

TD_animation_3D.o:	 TD_animation_3D.cpp
	$(CPP) -c $^ -o $@ $(CXXFLAGS)

MDVectors.o:	 MDVectors.cpp
	$(CPP) -c $^ -o $@ $(CXXFLAGS)

# 
clean: clear
clear:
	rm -f *.o TD_animation_3D *.*~

