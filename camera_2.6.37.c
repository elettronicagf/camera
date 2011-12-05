/*
 * saMmapLoopback.c
 *
 * Application used to do NTSC loopback in MMAP memory mode
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>


/* structure used to store information of the buffers */
struct buf_info {
	int index;
	unsigned int length;
	char *start;
};

/* Changing the following will result in different number of buffers used */
#define NUM_BUFFERS     3

#define CAPTURE_MAX_BUFFER		NUM_BUFFERS
#define DISPLAY_MAX_BUFFER		NUM_BUFFERS
/* device to be used for capture */
#define CAPTURE_DEVICE		"/dev/video0"
#define CAPTURE_NAME		"Capture"
/* device to be used for display */
#define DISPLAY_DEVICE		"/dev/video1"
#define DISPLAY_NAME		"Display"
/* number of frames to be captured and displayed */
#define MAXLOOPCOUNT		1000

#define DEF_PIX_FMT		V4L2_PIX_FMT_UYVY

#define IMG_WIDTH 720
#define IMG_HEIGHT 576
#define USBCAM_WIDTH 320
#define USBCAM_HEIGHT 240
#define USBCAM_DEF_PIX_FMT	V4L2_PIX_FMT_YUYV
/* capture_buff_info and display_buff_info stores buffer information of capture
   and display respectively. */
static struct buf_info capture_buff_info[CAPTURE_MAX_BUFFER];
static struct buf_info display_buff_info[DISPLAY_MAX_BUFFER];
static int is_usb = 0;

/*===============================initCapture==================================*/
/* This function initializes capture device. It selects an active input
 * and detects the standard on that input. It then allocates buffers in the
 * driver's memory space and mmaps them in the application space.
 */
static int initCapture(int *capture_fd, int *numbuffers, char *inputname,
				int capt_input, char *stdname, struct v4l2_format *fmt)
{
	int ret, i, j;
	struct v4l2_requestbuffers reqbuf;
	struct v4l2_buffer buf;
	struct v4l2_capability capability;
	struct v4l2_input input;
	v4l2_std_id std_id;
	struct v4l2_standard standard;
	int index;

	/* Open the capture device */
	*capture_fd  = open((const char *) CAPTURE_DEVICE, O_RDWR);
	if (*capture_fd  <= 0) {
		printf("Cannot open = %s device\n", CAPTURE_DEVICE);
		return -1;
	}
	printf("\n%s: Opened Channel\n", CAPTURE_NAME);

	/* Get any active input */
	if (ioctl(*capture_fd, VIDIOC_G_INPUT, &index) < 0) {
		perror("VIDIOC_G_INPUT");
		goto ERROR;
	}

	/* Enumerate input to get the name of the input detected */
	memset(&input, 0, sizeof(input));
	input.index = index;
	if (ioctl(*capture_fd, VIDIOC_ENUMINPUT, &input) < 0) {
		perror("VIDIOC_ENUMINPUT");
		goto ERROR;
	}

	printf("%s: Current Input: %s\n", CAPTURE_NAME, input.name);

	if (!is_usb)
		index = capt_input;

	if (ioctl(*capture_fd, VIDIOC_S_INPUT, &index) < 0) {
		perror("VIDIOC_S_INPUT");
		goto ERROR;
	}
	memset(&input, 0, sizeof(input));
	input.index = index;
	if (ioctl(*capture_fd, VIDIOC_ENUMINPUT, &input) < 0) {
		perror("VIDIOC_ENUMINPUT");
		goto ERROR;
	}
	printf("%s: Input changed to: %s\n", CAPTURE_NAME,
						 input.name);

	/* Store the name of the output as per the input detected */
	strcpy(inputname, (char*)input.name);

	if (!is_usb) {
		/* Detect the standard in the input detected */
		if (ioctl(*capture_fd, VIDIOC_QUERYSTD, &std_id) < 0) {
			perror("VIDIOC_QUERYSTD");
			goto ERROR;
		}

		/* Get the standard*/
		if (ioctl(*capture_fd, VIDIOC_G_STD, &std_id) < 0) {
			/* Note when VIDIOC_ENUMSTD always returns EINVAL this
			   is no video device or it falls under the USB exception,
			   and VIDIOC_G_STD returning EINVAL is no error. */
			perror("VIDIOC_G_STD");
			goto ERROR;
		}
		memset(&standard, 0, sizeof(standard));
		standard.index = 0;
		while (1) {
			if (ioctl(*capture_fd, VIDIOC_ENUMSTD, &standard) < 0) {
				perror("VIDIOC_ENUMSTD");
				goto ERROR;
			}

			/* Store the name of the standard */
			if (standard.id & std_id) {
				strcpy(stdname, (char*)standard.name);
				printf("%s: Current standard: %s\n",
						CAPTURE_NAME, standard.name);
				break;
			}
			standard.index++;
		}
	}

	/* Check if the device is capable of streaming */
	if (ioctl(*capture_fd, VIDIOC_QUERYCAP, &capability) < 0) {
		perror("VIDIOC_QUERYCAP");
		goto ERROR;
	}
	if (capability.capabilities & V4L2_CAP_STREAMING)
		printf("%s: Capable of streaming\n", CAPTURE_NAME);
	else {
		printf("%s: Not capable of streaming\n", CAPTURE_NAME);
		goto ERROR;
	}

	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(*capture_fd, VIDIOC_G_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_G_FMT");
		goto ERROR;
	}

	if (is_usb) {
		fmt->fmt.pix.width = USBCAM_WIDTH;
		fmt->fmt.pix.height = USBCAM_HEIGHT;
		fmt->fmt.pix.pixelformat = USBCAM_DEF_PIX_FMT;
	} else {
		fmt->fmt.pix.width = IMG_WIDTH;
		fmt->fmt.pix.height = IMG_HEIGHT;
		fmt->fmt.pix.pixelformat = DEF_PIX_FMT;
	}
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(*capture_fd, VIDIOC_S_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_S_FMT");
		goto ERROR;
	}

	ret = ioctl(*capture_fd, VIDIOC_G_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_G_FMT");
		goto ERROR;
	}

	if (is_usb) {
		if (fmt->fmt.pix.pixelformat != USBCAM_DEF_PIX_FMT) {
			printf("%s: Requested pixel format not supported\n",
					CAPTURE_NAME);
			goto ERROR;
		}
	} else {
		if (fmt->fmt.pix.pixelformat != DEF_PIX_FMT) {
			printf("%s: Requested pixel format not supported\n",
					CAPTURE_NAME);
			goto ERROR;
		}
	}

	/* Buffer allocation
	 * Buffer can be allocated either from capture driver or
	 * user pointer can be used
	 */
	/* Request for MAX_BUFFER input buffers. As far as Physically contiguous
	 * memory is available, driver can allocate as many buffers as
	 * possible. If memory is not available, it returns number of
	 * buffers it has allocated in count member of reqbuf.
	 * HERE count = number of buffer to be allocated.
	 * type = type of device for which buffers are to be allocated.
	 * memory = type of the buffers requested i.e. driver allocated or
	 * user pointer */
	reqbuf.count = *numbuffers;
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	ret = ioctl(*capture_fd, VIDIOC_REQBUFS, &reqbuf);
	if (ret < 0) {
		perror("Cannot allocate memory");
		goto ERROR;
	}
	/* Store the number of buffers actually allocated */
	*numbuffers = reqbuf.count;
	printf("%s: Number of requested buffers = %d\n", CAPTURE_NAME,
			*numbuffers);

	memset(&buf, 0, sizeof(buf));

	/* Mmap the buffers
	 * To access driver allocated buffer in application space, they have
	 * to be mmapped in the application space using mmap system call */
	for (i = 0; i < (*numbuffers); i++) {
		buf.type = reqbuf.type;
		buf.index = i;
		buf.memory = reqbuf.memory;
		ret = ioctl(*capture_fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			perror("VIDIOC_QUERYCAP");
			*numbuffers = i;
			goto ERROR1;
		}


		capture_buff_info[i].length = buf.length;
		capture_buff_info[i].index = i;
		capture_buff_info[i].start = mmap(NULL, buf.length,
				PROT_READ | PROT_WRITE, MAP_SHARED, *capture_fd,
				buf.m.offset);

		if (capture_buff_info[i].start == MAP_FAILED) {
			printf("Cannot mmap = %d buffer\n", i);
			*numbuffers = i;
			goto ERROR1;
		}

		memset((void *) capture_buff_info[i].start, 0x80,
				capture_buff_info[i].length);
		/* Enqueue buffers
		 * Before starting streaming, all the buffers needs to be
		 * en-queued in the driver incoming queue. These buffers will
		 * be used by thedrive for storing captured frames. */
		ret = ioctl(*capture_fd, VIDIOC_QBUF, &buf);
		if (ret < 0) {
			perror("VIDIOC_QBUF");
			*numbuffers = i + 1;
			goto ERROR1;
		}
	}

	printf("%s: Init done successfully\n\n", CAPTURE_NAME);
	return 0;

ERROR1:
	for (j = 0; j < *numbuffers; j++)
		munmap(capture_buff_info[j].start,
				capture_buff_info[j].length);
ERROR:
	close(*capture_fd);

	return -1;
}

/*===============================initDisplay==================================*/
/* This function initializes display device. It sets output and standard for
 * LCD. These output and standard are same as those detected in capture device.
 * It, then, allocates buffers in the driver's memory space and mmaps them in
 * the application space */
static int initDisplay(int *display_fd, int *numbuffers, char *stdname,
				struct v4l2_format *fmt)
{
	int ret, i, j;
	struct v4l2_requestbuffers reqbuf;
	struct v4l2_buffer buf;
	struct v4l2_capability capability;
	struct v4l2_control control;

	/* Open the video display device */
	*display_fd = open((const char *) DISPLAY_DEVICE, O_RDWR);
	if (*display_fd <= 0) {
		printf("Cannot open = %s device\n", DISPLAY_DEVICE);
		return -1;
	}
	printf("\n%s: Opened Channel\n", DISPLAY_NAME);

	/* Check if the device is capable of streaming */
	if (ioctl(*display_fd, VIDIOC_QUERYCAP, &capability) < 0) {
		perror("VIDIOC_QUERYCAP");
		goto ERROR;
	}

	if (capability.capabilities & V4L2_CAP_STREAMING)
		printf("%s: Capable of streaming\n", DISPLAY_NAME);
	else {
		printf("%s: Not capable of streaming\n", DISPLAY_NAME);
		goto ERROR;
	}

	/* Rotate by 90 degree so that 480x640 resolution will become 640x480 */
        control.id = V4L2_CID_ROTATE;
        control.value = 90;
	ret = ioctl(*display_fd, VIDIOC_S_CTRL, &control);
	if (ret < 0) {
		perror("VIDIOC_S_CTRL");
		goto ERROR;
	}

	/* Get the format */
	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ret = ioctl(*display_fd, VIDIOC_G_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_G_FMT");
		goto ERROR;
	}

	if (is_usb) {
		fmt->fmt.pix.width = USBCAM_WIDTH;
		fmt->fmt.pix.height = USBCAM_HEIGHT;
		fmt->fmt.pix.pixelformat = USBCAM_DEF_PIX_FMT;
	} else {
		fmt->fmt.pix.width = IMG_WIDTH;
		fmt->fmt.pix.height = IMG_HEIGHT;
		fmt->fmt.pix.pixelformat = DEF_PIX_FMT;
	}
	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ret = ioctl(*display_fd, VIDIOC_S_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_S_FMT");
		goto ERROR;
	}

	ret = ioctl(*display_fd, VIDIOC_G_FMT, fmt);
	if (ret < 0) {
		perror("VIDIOC_G_FMT");
		goto ERROR;
	}

	if (is_usb) {
		if (fmt->fmt.pix.pixelformat != USBCAM_DEF_PIX_FMT) {
			printf("%s: Requested pixel format not supported\n",
					CAPTURE_NAME);
			goto ERROR;
		}
	} else {
		if (fmt->fmt.pix.pixelformat != DEF_PIX_FMT) {
			printf("%s: Requested pixel format not supported\n",
					CAPTURE_NAME);
			goto ERROR;
		}
	}

	/* Buffer allocation
	 * Buffer can be allocated either from capture driver or
	 * user pointer can be used
	 */
	/* Request for MAX_BUFFER input buffers. As far as Physically contiguous
	 * memory is available, driver can allocate as many buffers as
	 * possible. If memory is not available, it returns number of
	 * buffers it has allocated in count member of reqbuf.
	 * HERE count = number of buffer to be allocated.
	 * type = type of device for which buffers are to be allocated.
	 * memory = type of the buffers requested i.e. driver allocated or
	 * user pointer */
	reqbuf.count = *numbuffers;
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	ret = ioctl(*display_fd, VIDIOC_REQBUFS, &reqbuf);
	if (ret < 0) {
		perror("Cannot allocate memory");
		goto ERROR;
	}
	/* Store the numbfer of buffers allocated */
	*numbuffers = reqbuf.count;
	printf("%s: Number of requested buffers = %d\n", DISPLAY_NAME,
	       (*numbuffers));

	memset(&buf, 0, sizeof(buf));

	/* Mmap the buffers
	 * To access driver allocated buffer in application space, they have
	 * to be mmapped in the application space using mmap system call */
	for (i = 0; i < (*numbuffers); i++) {
		/* Query physical address of the buffers */
		buf.type = reqbuf.type;
		buf.index = i;
		buf.memory = reqbuf.memory;
		ret = ioctl(*display_fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			perror("VIDIOC_QUERYCAP");
			(*numbuffers) = i;
			goto ERROR1;
		}

		/* Mmap the buffers in application space */
		display_buff_info[i].length = buf.length;
		display_buff_info[i].index =  i;
		display_buff_info[i].start = mmap(NULL, buf.length,
				PROT_READ | PROT_WRITE, MAP_SHARED, *display_fd,
				buf.m.offset);

		if (display_buff_info[i].start == MAP_FAILED) {
			printf("Cannot mmap = %d buffer\n", i);
			(*numbuffers) = i;
			goto ERROR1;
		}
		memset((void *) display_buff_info[i].start, 0x80,
		       display_buff_info[i].length);

		/* Enqueue buffers
		 * Before starting streaming, all the buffers needs to be
		 * en-queued in the driver incoming queue. These buffers will
		 * be used by thedrive for storing captured frames. */
		ret = ioctl(*display_fd, VIDIOC_QBUF, &buf);
		if (ret < 0) {
			perror("VIDIOC_QBUF");
			(*numbuffers) = i + 1;
			goto ERROR1;
		}
	}
	printf("%s: Init done successfully\n\n", DISPLAY_NAME);
	return 0;

ERROR1:
	for (j = 0; j < *numbuffers; j++)
		munmap(display_buff_info[j].start,
			display_buff_info[j].length);
ERROR:
	close(*display_fd);

	return -1;
}

static void usage(void)
{
	printf("Usage: saMmapLoopback [-u <usb/tvp>] [-h help]");
	printf("\t[-u <usb/tvp>]	: 0 - TVP Capture 1 - USB Camera\n"
			"[-i <S-Vid/CVBS>]	: 0: CVBS 1 - S-Video\n");
}
static int timeval_subtract(struct timeval *result, struct timeval *x,
		                     struct timeval *y)
{
	/* Perform the carry for the later subtraction by updating y.
	 * */
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) /
			1000000 + 1;
		y->tv_usec -= 1000000 *	nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) /
			1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	   tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is
	 * negative. */
	return x->tv_sec < y->tv_sec;
}
int main(int argc, char *argv[])
{
	char shortoptions[] = "u:h:i:";
	int capture_fd, display_fd;
	struct v4l2_format capture_fmt;
	struct v4l2_format display_fmt;
	char inputname[15], stdname[15];
	int i = 0, ret = 0, capt_input = 0, a, c, index;
	int capture_numbuffers = CAPTURE_MAX_BUFFER, display_numbuffers = DISPLAY_MAX_BUFFER;
	struct v4l2_buffer display_buf, capture_buf;
	struct timeval before, after, result;

	for (;;) {
		c = getopt_long(argc, argv, shortoptions, (void *) NULL,
				&index);
		if (-1 == c)
			break;
		switch (c) {
			case 0:
				break;
			case 'u':
			case 'U':
				is_usb = atoi(optarg);
				break;
			case 'i':
			case 'I':
				capt_input = atoi(optarg);
				break;
			default:
				usage();
				exit(1);
		}

	}
	for(i = 0; i < CAPTURE_MAX_BUFFER; i++) {
		capture_buff_info[i].start = NULL;
	}
	for(i = 0; i < DISPLAY_MAX_BUFFER; i++) {
		display_buff_info[i].start = NULL;
	}
	/* STEP1:
	 * Initialization section
	 * Initialize capture and display devices.
	 * Here one capture channel is opened and input and standard is
	 * detected on that channel.
	 * Display channel is opened with the same standard that is detected at
	 * capture channel.
	 * */
	ret = initCapture(&capture_fd, &capture_numbuffers, inputname,
			capt_input, stdname, &capture_fmt);
	if(ret < 0) {
		printf("Error in opening capture device for channel 0\n");
		return ret;
	}

	/* open display channel */
	ret = initDisplay(&display_fd, &display_numbuffers,
			  stdname, &display_fmt);
	if(ret < 0) {
		printf("Error in opening display device\n");
		goto ERROR_1;
	}

	/* run section
	 * STEP2:
	 * Here display and capture channels are started for streaming. After
	 * this capture device will start capture frames into enqueued
	 * buffers and display device will start displaying buffers from
	 * the qneueued buffers */

	/* Start Streaming. on display device */
	a = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ret = ioctl(display_fd, VIDIOC_STREAMON, &a);
	if (ret < 0) {
		perror("VIDIOC_STREAMON");
		goto ERROR;
	}
	printf("%s: Stream on...\n", DISPLAY_NAME);

	/* Start Streaming. on capture device */
	a = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(capture_fd, VIDIOC_STREAMON, &a);
	if (ret < 0) {
		perror("VIDIOC_STREAMON");
		goto ERROR;
	}
	printf("%s: Stream on...\n", CAPTURE_NAME);

	/* Set the display buffers for queuing and dqueueing operation */
	display_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	display_buf.index = 0;
	display_buf.memory = V4L2_MEMORY_MMAP;

	/* Set the capture buffers for queuing and dqueueing operation */
	capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	capture_buf.index = 0;
	capture_buf.memory = V4L2_MEMORY_MMAP;

	/* One buffer is dequeued from display and capture channels.
	 * Capture buffer will be copied to display buffer.
	 * All two buffers are put back to respective channels.
	 * This sequence is repeated in loop.
	 * After completion of this loop, channels are stopped.
	 */
	gettimeofday(&before, NULL);
	for (i = 0; i < MAXLOOPCOUNT; i++) {
		int h;
		unsigned char *cap_ptr, *dis_ptr;
		/* Dequeue display buffer */
		ret = ioctl(display_fd, VIDIOC_DQBUF, &display_buf);
		if (ret < 0) {
			perror("VIDIOC_DQBUF");
			goto ERROR;
		}

		/* Dequeue capture buffer */
		ret = ioctl(capture_fd, VIDIOC_DQBUF, &capture_buf);
		if (ret < 0) {
			perror("VIDIOC_DQBUF");
			goto ERROR;
		}

		cap_ptr = (unsigned char*)capture_buff_info[capture_buf.index].start;
		dis_ptr = (unsigned char*)display_buff_info[display_buf.index].start;
		for (h = 0; h < display_fmt.fmt.pix.height; h++) {
#if defined (CONFIG_AM3517)
			int j;
			for(j = 0; j < (display_fmt.fmt.pix.width * 2); j++)
				dis_ptr[j] = alaw_mapping_table[cap_ptr[j]];
#else
			memcpy(dis_ptr, cap_ptr, display_fmt.fmt.pix.width * 2);
#endif
			cap_ptr += capture_fmt.fmt.pix.width * 2;
			dis_ptr += display_fmt.fmt.pix.width * 2;
		}

		ret = ioctl(capture_fd, VIDIOC_QBUF, &capture_buf);
		if (ret < 0) {
			perror("VIDIOC_QBUF");
			goto ERROR;
		}

		ret = ioctl(display_fd, VIDIOC_QBUF, &display_buf);
		if (ret < 0) {
			perror("VIDIOC_QBUF");
			goto ERROR;
		}
	}
	gettimeofday(&after, NULL);

	a = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ret = ioctl(display_fd, VIDIOC_STREAMOFF, &a);
	if (ret < 0) {
		perror("VIDIOC_STREAMOFF");
		goto ERROR;
	}
	printf("\n%s: Stream off!!\n", DISPLAY_NAME);

	a = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(capture_fd, VIDIOC_STREAMOFF, &a);
	if (ret < 0) {
		perror("VIDIOC_STREAMOFF");
		goto ERROR;
	}
	printf("%s: Stream off!!\n", CAPTURE_NAME);

	printf("\nLoopback Successful\n\n");
	printf("Timing Analysis:\n");
	printf("----------------\n");
	printf("Before Time:\t%lu %lu\n",before.tv_sec, before.tv_usec);
	printf("After Time:\t%lu %lu\n",after.tv_sec, after.tv_usec);
	timeval_subtract(&result, &after, &before);
	printf("Result Time:\t%ld %ld\n",result.tv_sec, result.tv_usec);
	printf("Calculated Frame Rate:\t%ld Fps\n\n", MAXLOOPCOUNT/result.tv_sec);

ERROR:
	/* Un-map the buffers */
	for (i = 0; i < display_numbuffers; i++) {
		munmap(display_buff_info[i].start,
		       display_buff_info[i].length);
		display_buff_info[i].start = NULL;
	}
	/* Close the file handle */
	close(display_fd);

ERROR_1:
	/* Un-map the buffers */
	for (i = 0; i < capture_numbuffers; i++) {
		munmap(capture_buff_info[i].start,
		       capture_buff_info[i].length);
		capture_buff_info[i].start = NULL;
	}
	/* Close the file handle */
	close(capture_fd);

	return 0;
}
