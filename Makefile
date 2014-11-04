CC = g++
LD = g++
LDFLAGS = -L/usr/local/lib/
CFLAGS=-g -Wall `pkg-config --cflags libfreenect` -lopengl32 -lglut32 -I/usr/local/include/libfreenect -I/usr/local/include/libusb-1.0 
#12 -lGL -lGLU -lglut
LIBS = -lfreenect -framework GLUT -framework OpenGL -lglfw3 -lglew  #-lGL -lGLU -lglut 
OBJECTS = mutex.o myfreenectdevice.o main.o
PROG = 3Dscene

all:$(PROG)

$(PROG): $(OBJECTS)
	$(LD) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $(PROG)

%.o: %.cpp
	$(CC) $(CFLAGS)  $(LIBS) -c $<

clean:
	rm -rf *.o $(PROG)

