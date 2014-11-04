#include <libfreenect.hpp>
#include <vector>
#include <cmath>
//Local headers
#include "mutex.h"
#include "defs.h"

using namespace std;

/*
 * Number of pixels in the kinect image frame
 * width*height pixels in the image
 */
#define         FREENECT_FRAME_PIX   (FREENECT_FRAME_H*FREENECT_FRAME_W) 

 /*
  * Number of bytes in the image frame
  * 3 Bytes per pixel
  */
#define         FREENECT_VIDEO_RGB_SIZE   (FREENECT_FRAME_PIX*3)

 /*
  * Number of bytes in the image frame
  * 32 bits per depth info
  */
#define         FREENECT_VIDEO_DEPTH_SIZE   (FREENECT_FRAME_PIX*4) 

class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice();
		MyFreenectDevice(freenect_context *_ctx, int _index);

	  	// ~MyFreenectDevice() {
    // 		stopVideo();
	   //  	stopDepth();
	  	// }

		void VideoCallback(void* _rgb, uint32_t timestamp);

		void DepthCallback(void* _depth, uint32_t timestamp);
		
		bool getRGB(vector<uint8_t> &buffer);

		bool getDepth(vector<uint8_t> &buffer);

		bool getDepthInMM(vector<uint16_t> &buffer);

	private:
		vector<uint8_t> m_buffer_depth;
		vector<uint16_t> m_depth;
		vector<uint8_t> m_buffer_video;
		vector<uint16_t> m_gamma;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};