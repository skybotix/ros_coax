#include <stdio.h>
#include <stdlib.h>
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

#include "CamGrabber.hpp"


static void xioctl(int fh, int request, void *arg) {
        int r;

        do {
                r = v4l2_ioctl(fh, request, arg);
        } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

        if (r == -1) {
                fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}


CamGrabber::CamGrabber(char *device, unsigned int width, unsigned int height) {

	/* some initialization */
	fd = -1;
	dev_name = device;

	/* initialize the cam*/
    fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
            perror("Cannot open device");
            exit(EXIT_FAILURE);
    }

    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; //V4L2_PIX_FMT_MJPEG
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    xioctl(fd, VIDIOC_S_FMT, &fmt);
//    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
//            printf("Libv4l didn't accept RGB24 format. Can't proceed.\n");
//            exit(EXIT_FAILURE);
//    }


    CLEAR(req);
    req.count = 2;	
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(fd, VIDIOC_REQBUFS, &req);

    buffers = (buffer *)calloc(req.count, sizeof(*buffers));
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
            CLEAR(buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = n_buffers;

            xioctl(fd, VIDIOC_QUERYBUF, &buf);

            buffers[n_buffers].length = buf.length;
            buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
                          PROT_READ | PROT_WRITE, MAP_SHARED,
                          fd, buf.m.offset);

            if (MAP_FAILED == buffers[n_buffers].start) {
                    perror("mmap");
                    exit(EXIT_FAILURE);
            }
    }

    for (i = 0; i < n_buffers; ++i) {
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            xioctl(fd, VIDIOC_QBUF, &buf);
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd, VIDIOC_STREAMON, &type);


      do {
          FD_ZERO(&fds);
          FD_SET(fd, &fds);

          /* Timeout. */
          tv.tv_sec = 2;
          tv.tv_usec = 0;

          r = select(fd + 1, &fds, NULL, NULL, &tv);
      } while ((r == -1 && (errno = EINTR)));
      if (r == -1) {
          perror("select");
          //return errno;
      }



      /* allocate space for output frame */
      maxPixelCount = 3*width*height;
      pixelBuffer = (unsigned char*)malloc(maxPixelCount * sizeof(unsigned char));



}

/* deinit device */
CamGrabber::~CamGrabber() {

    /* stop capturing */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    
    /* release output buffer */
    free(pixelBuffer);

    /* release memory */
    for (i = 0; i < n_buffers; ++i)
            v4l2_munmap(buffers[i].start, buffers[i].length);
    v4l2_close(fd);

}

/* convert from camera YUYV (YUV422) format to GRAY8 */
void CamGrabber::YUYVtoGRAY(unsigned int width, unsigned int height, unsigned char *frame)
{
	/* this conversion only works on YUYV data streams*/
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
		printf("ERROR: Conversion to GRAY8 selected but image stream is not in V4L2_PIX_FMT_YUYV.\n");
		exit(EXIT_FAILURE);
	}

	int numPixels = width * height;

	/* extract lumo */
	for(int i=0; i<numPixels; i++)
		frame[i] = frame[2*i];

}


int CamGrabber::grabFrame(unsigned char ** pixelData, bool onlyGrayFromYUYV) {

  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  xioctl(fd, VIDIOC_DQBUF, &buf);

  /* copy the buffer */
  memcpy(pixelBuffer, (unsigned char *)buffers[buf.index].start, maxPixelCount * sizeof(unsigned char));

  /* release buffer back to driver queue */
  xioctl(fd, VIDIOC_QBUF, &buf);

  /* convert to grayscale if needed (to avoid YUYV --> RGB --> GRAY conversion which is quite slow...) */
  if(onlyGrayFromYUYV)
  YUYVtoGRAY(fmt.fmt.pix.width, fmt.fmt.pix.height, (unsigned char *)pixelBuffer);

  /* return the pointer */
  *pixelData = pixelBuffer;

  return 1;
}

