/****************************************************************************/
/* $Id: 
 ****************************************************************************/
#include <linux/fb.h>
#include <linux/ioctl.h>
#include <linux/types.h>
/*
 * define the IOCTL to get the frame buffer handle info.
 * currently only image handle is returned
 */

/* define this for testing H/w acceleation funcitons */
/* #define FB_TEST_HW_ACCELERATION */


/* assign an accelerator type (we have no official, so we do not add them to linux/fb.h */
#define FB_ACCEL_PNX849X        0x90    /* Trident PNX849X             */

/*
 * structure which contains the image handle
 */
typedef struct _cnxtfb_handle
{
  void     *hImage;
  void     *hVPP_SD;
  void     *hTvEnc_SD;
  void     *hVPP;
  void     *hTvEnc;
  void     *hImage_SD;
} cnxtfb_handle;

typedef struct _cnxtfb_resolution
{
  u_int32_t uWidth;
  u_int32_t uHeight;

} cnxtfb_resolution;

/* To use with ioctl FBIO_CHANGEOUTPUTFORMAT */
typedef enum {
    CNXTFB_VIDEO_STANDARD_ATSC_1080I = 0,
    CNXTFB_VIDEO_STANDARD_NTSC_M,
    CNXTFB_VIDEO_STANDARD_ATSC_480P,
    CNXTFB_VIDEO_STANDARD_ATSC_720P,
    CNXTFB_VIDEO_STANDARD_PAL_B_WEUR,
    CNXTFB_VIDEO_STANDARD_SECAM_L,
    CNXTFB_VIDEO_STANDARD_ATSC_576P,
    CNXTFB_VIDEO_STANDARD_ATSC_720P_50HZ,
    CNXTFB_VIDEO_STANDARD_ATSC_1080I_50HZ
} CNXTFB_VIDEO_STANDARD;

typedef enum
{
  CNXTFB_BLEND_MODE_GLOBAL_ALPHA = 0,
  CNXTFB_BLEND_MODE_PIXEL_ALPHA,
  CNXTFB_BLEND_MODE_ALPHA_MULTIPLIED,
} CNXTFB_BLEND_MODE;

typedef enum
{
    CNXTFB_INVALID = -1,
    CNXTFB_480I = 0,
    CNXTFB_480P,
    CNXTFB_576I,
    CNXTFB_576P,
    CNXTFB_720P,
    CNXTFB_720P_50,
    CNXTFB_1080I,
    CNXTFB_1080I_50,
    CNXTFB_1080P,
    CNXTFB_DISPLAY_MODE_LAST = CNXTFB_1080P
} cnxtfb_displaymode;

typedef enum
{
   CNXTFB_TYPE_SD = 1, /* 1 << 0 */
   CNXTFB_TYPE_HD = 2  /* 1 << 1 */
} CNXTFB_FB_TYPE;

typedef struct
{
   unsigned char Type; /* Bitmask of type CNXTFB_FB_TYPE */
   cnxtfb_displaymode SDMode;
   cnxtfb_displaymode HDMode;
} CNXTFB_OUTPUT_FORMAT_CHANGE;

#define CNXTFB_IO(type)            _IO('F', type)
#define CNXTFB_IOW(type, dtype)    _IOW('F', type, dtype)
#define CNXTFB_IOR(type, dtype)    _IOR('F', type, dtype)

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC       _IOW('F', 0x20, u_int32_t)
#endif

#define FBIO_GETCNXTFBHANDLE      CNXTFB_IOR(0x21, cnxtfb_handle) // 0x4620
#define FBIO_STARTDISPLAY         CNXTFB_IO(0x22)
#define FBIO_STOPDISPLAY          CNXTFB_IO(0x23)
#define FBIO_SETOPACITY           CNXTFB_IOW(0x24, u_int8_t)
#define FBIO_SETBLENDMODE         CNXTFB_IOW(0x25, u_int8_t)
#define FBIO_CHANGEOUTPUTFORMAT   CNXTFB_IOW(0x26, u_int32_t)
#define FBIO_GETFBRESOLUTION      CNXTFB_IOR(0x27, cnxtfb_resolution)
#define FBIO_SETFLICKERFILTER     CNXTFB_IOW(0x28, u_int8_t)
#define FBIO_SCALE_SD_OSD         CNXTFB_IO(0x28)

#ifdef FB_TEST_HW_ACCELERATION
#define FBIO_FILL_RECT            CNXTFB_IOW(0x29, struct fb_fillrect)
#define FBIO_COPY_AREA            CNXTFB_IOW(0x2a, struct fb_copyarea)
#define FBIO_IMAGE_BLT            CNXTFB_IOW(0x2b, struct fb_image)
#endif
