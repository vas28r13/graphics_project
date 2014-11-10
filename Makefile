CC = g++
LD = g++
LDFLAGS = -L/usr/local/lib/
CFLAGS=-g -Wall `pkg-config --cflags libfreenect` -lopengl32 -lglut32 -I/usr/local/include/libfreenect -I/usr/local/include/libusb-1.0 -I/usr/local/include/opencv
#-lGL -lGLU -lglut
LIBS = -lfreenect -framework GLUT -framework OpenGL -lglfw3 -lglew -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -lopencv_nonfree
OBJECTS = mutex.o myfreenectdevice.o main.o
PROG = 3Dscene

all:$(PROG)

$(PROG): $(OBJECTS)
	$(LD) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $(PROG)

%.o: %.cpp
	$(CC) $(CFLAGS)  $(LIBS) -c $<

clean:
	rm -rf *.o $(PROG)

