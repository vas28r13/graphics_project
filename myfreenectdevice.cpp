#include "myfreenectdevice.h"

MyFreenectDevice::MyFreenectDevice(freenect_context *_ctx, int _index) 
			: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_VIDEO_RGB_SIZE), m_depth(FREENECT_VIDEO_DEPTH_SIZE), m_buffer_video(FREENECT_VIDEO_RGB_SIZE), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false)
{

			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
}


void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
};

void MyFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	for( unsigned int i = 0 ; i < FREENECT_FRAME_PIX ; i++) {
		//cout << "The depth info for pixel " << i << " is: " << depth[i] << "\n";
		m_depth[i] = depth[i];
		int pval = m_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			case 1:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = lb;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 2:
				m_buffer_depth[3*i+0] = 255-lb;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 3:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = lb;
				break;
			case 4:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255;
				break;
			case 5:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			default:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 0;
				break;
		}
	}
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
} 

bool MyFreenectDevice::getRGB(vector<uint8_t> &buffer) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		buffer.swap(m_buffer_video);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool MyFreenectDevice::getDepth(vector<uint8_t> &buffer) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		buffer.swap(m_buffer_depth);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}

bool MyFreenectDevice::getDepthInMM(vector<uint16_t> &buffer) {
	m_depth_mutex.lock();
	if(m_new_depth_frame) {
		buffer.swap(m_depth);
		m_new_depth_frame = false;
		m_depth_mutex.unlock();
		return true;
	} else {
		m_depth_mutex.unlock();
		return false;
	}
}