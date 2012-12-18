
CC=gcc
CPP=g++
CFLAGS=-Wall -ansi -lGL -lGLU -lglut -lGLEW `libpng12-config --cflags --libs` -lz
LFLAGS=-Wall -ansi -lGL -lGLU -lglut `libpng12-config --cflags --libs`   
SFLAGS=-lm -lpthread -lGL -lglut -lGLU
EXEC=anim


all: $(EXEC)

anim : mrproper
	$(CC) -o td5 TD_GL_5_1.c  $(SFLAGS)

clean:
	rm *.o

mrproper: 
	rm -f $(EXEC)

