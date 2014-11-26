CC = g++
LD = g++
LDFLAGS = -L/usr/local/lib/
#`pkg-config --cflags libfreenect`
CFLAGS=-g -Wall -lopengl32 -lglut32 -I/usr/local/include/libfreenect -I/usr/local/include/libusb-1.0 -I/usr/local/include/opencv -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.7
#-lGL -lGLU -lglut
LIBS = -lfreenect -framework GLUT -framework OpenGL -lglfw3 -lglew -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -lopencv_nonfree -lpcl_common -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_keypoints -lpcl_search -lboost_system
OBJECTS = mutex.o myfreenectdevice.o main.o
PROG = 3Dscene

all:$(PROG)

$(PROG): $(OBJECTS)
	$(LD) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $(PROG)

%.o: %.cpp
	$(CC) $(CFLAGS)  $(LIBS) -c $<

clean:
	rm -rf *.o $(PROG)

