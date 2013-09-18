#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "libv4l2.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

/* some macros */
#define CLEAR(x) memset(&(x), 0, sizeof(x))


/* class definition */
class CamGrabber {

	/*****************************************************************/	
	private:
		
		/* MMAP buffer */
		struct buffer {
				void   *start;
				size_t length;
		};
    
		/* globals */
	    struct v4l2_format              fmt;
	    struct v4l2_buffer              buf;
	    struct v4l2_requestbuffers      req;
	    enum v4l2_buf_type              type;
	    fd_set                          fds;
	    struct timeval                  tv;
	    int                             r, fd;
	    unsigned int                    i, n_buffers;
	    char                            *dev_name;
	    char                            out_name[256];
	    FILE                            *fout;
	    struct buffer                   *buffers;
	    unsigned char *pixelBuffer; //pixel buffer
	    unsigned int maxPixelCount;

	    /*prototypes */
	    void YUYVtoGRAY(unsigned int width, unsigned int height, unsigned char *frame);

	/*****************************************************************/
	public:
	    CamGrabber(char *device, unsigned int width, unsigned int height);
	    ~CamGrabber();
	    
	    int grabFrame(unsigned char ** pixelData, bool onlyGrayFromYUYV);
	    


	    
};

/* prototypes */
static void xioctl(int fh, int request, void *arg);

