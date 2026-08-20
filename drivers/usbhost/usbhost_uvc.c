/****************************************************************************
 * drivers/usbhost/usbhost_uvc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/endian.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/uvc.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A camera is not required to describe itself sanely.  These bound what we
 * are willing to allocate on its say-so.
 */

#define UVC_MAX_FORMATS         8
#define UVC_MAX_FRAMES          32
#define UVC_MAX_ALTSETTINGS     16

/* Device naming.  A camera appears as an ordinary NuttX video device. */

#define UVC_DEVNAME_FMT         "/dev/video%d"
#define UVC_DEVNAME_LEN         16
#define UVC_MAX_DEVICES         8

/* A UVC payload header is at least two bytes, the length and the flags, and
 * cannot be longer than the packet carrying it.
 */

#define UVC_HEADER_MINLEN       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One frame size supported by one format */

struct uvc_frameinfo_s
{
  uint8_t  index;                  /* bFrameIndex, used in probe/commit */
  uint16_t width;
  uint16_t height;
  uint32_t maxbuffersize;          /* dwMaxVideoFrameBufferSize */
  uint32_t definterval;            /* dwDefaultFrameInterval, 100ns units */
  uint32_t mininterval;            /* Fastest interval offered */
  uint32_t maxinterval;            /* Slowest interval offered */
};

/* One format supported by the streaming interface */

struct uvc_formatinfo_s
{
  uint8_t  index;                  /* bFormatIndex, used in probe/commit */
  uint8_t  subtype;                /* UVC_VS_FORMAT_* */
  uint8_t  defframe;               /* bDefaultFrameIndex */
  uint8_t  bpp;                    /* bBitsPerPixel, 0 for compressed */
  uint32_t pixfmt;                 /* IMGDATA_PIX_FMT_* equivalent */
  uint8_t  nframes;
  FAR struct uvc_frameinfo_s *frames;
};

/* One alternate setting of the streaming interface, with the bandwidth its
 * endpoint offers.  Streaming starts by choosing the cheapest one that can
 * carry the negotiated payload.
 */

struct uvc_altinfo_s
{
  uint8_t  alt;                    /* bAlternateSetting */
  uint16_t pktsize;                /* Bytes per microframe */
};

/* One transfer queued on the streaming endpoint.  Several are kept in flight
 * and each is resubmitted from its own completion, so the endpoint is never
 * idle between transfers.
 */

struct uvc_req_s
{
  FAR struct uvc_state_s *priv;    /* Back pointer for the completion */
  struct usbhost_isoc_s isoc;      /* Isochronous transfer description */
  FAR uint8_t *buffer;             /* Transfer buffer */
  FAR uint16_t *pktlen;            /* Per-packet lengths, filled on
                                    * completion */
  bool queued;                     /* This request is in flight */
};

/* This structure describes an instance of the driver.  The class structure
 * must come first so that a usbhost_class_s pointer can be cast to it.
 */

struct uvc_state_s
{
  struct usbhost_class_s usbclass;

  volatile bool disconnected;      /* The device is gone */
  uint8_t crefs;                   /* Reference count */
  mutex_t lock;
  struct work_s work;              /* For deferred destruction */

  /* What the descriptors said */

  uint16_t bcduvc;                 /* Reported UVC release */
  uint8_t  probelen;               /* Probe/commit payload length for it */
  uint8_t  ctrlif;                 /* VideoControl interface number */
  uint8_t  streamif;               /* VideoStreaming interface number */
  bool     havestream;             /* A streaming interface was found */

  uint8_t  epaddr;                 /* Streaming endpoint address */
  uint8_t  eptype;                 /* USB_EP_ATTR_XFER_ISOC or _BULK */
  uint8_t  epinterval;

  uint8_t  nalts;
  struct uvc_altinfo_s alts[UVC_MAX_ALTSETTINGS];

  uint8_t  nformats;
  FAR struct uvc_formatinfo_s *formats;

  /* Driver-allocated buffers for endpoint 0 traffic */

  FAR uint8_t *ctrlreq;            /* struct usb_ctrlreq_s */
  FAR uint8_t *ctrlbuf;            /* Probe/commit payload */

  /* What the last successful negotiation settled on */

  FAR struct uvc_formatinfo_s *curfmt;
  FAR struct uvc_frameinfo_s  *curframe;
  uint32_t curinterval;            /* Frame interval, 100ns units */
  uint32_t maxframesize;           /* dwMaxVideoFrameSize */
  uint32_t maxpayload;             /* dwMaxPayloadTransferSize */
  uint8_t  curalt;                 /* Alternate setting that carries it */
  uint16_t curpktsize;             /* Its endpoint's bytes per microframe */

  /* Streaming */

  usbhost_ep_t epstream;           /* The streaming endpoint */
  bool     streaming;              /* Transfers are queued */
  uint8_t  inflight;               /* How many are still queued */
  sem_t    donesem;                /* Posted when the last one completes */
  struct uvc_req_s reqs[CONFIG_USBHOST_UVC_NREQS];

  /* Frame assembly */

  FAR uint8_t *framebuf;           /* Buffer the framework gave us */
  uint32_t framebufsize;
  uint32_t frameoffset;            /* How much of it is filled */
  uint8_t  lastfid;                /* Frame ID of the frame being built */
  bool     framestarted;
  bool     framebad;               /* A payload error spoiled this frame */

  /* The video capture framework */

  struct imgsensor_s sensor;
  struct imgdata_s data;
  imgdata_capture_t capture_cb;
  FAR void *capture_arg;
  int      devno;
  char     devpath[UVC_DEVNAME_LEN];

  FAR struct v4l2_fmtdesc *fmtdescs;
  size_t fmtdescs_num;
  FAR struct v4l2_frmsizeenum *frmsizes;
  size_t frmsizes_num;
  FAR struct v4l2_frmivalenum *frmintervals;
  size_t frmintervals_num;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Class management */

static FAR struct usbhost_class_s *
  usbhost_uvc_create(FAR struct usbhost_hubport_s *hport,
                     FAR const struct usbhost_id_s *id);
static void usbhost_uvc_destroy(FAR void *arg);
static void usbhost_uvc_freeformats(FAR struct uvc_state_s *priv);

/* struct usbhost_class_s methods */

static int usbhost_uvc_connect(FAR struct usbhost_class_s *usbclass,
                               FAR const uint8_t *configdesc, int desclen);
static int usbhost_uvc_disconnected(FAR struct usbhost_class_s *usbclass);

/* Descriptor parsing */

static uint32_t usbhost_uvc_pixfmt(FAR const uint8_t *guid);
static int usbhost_uvc_allocbuffers(FAR struct uvc_state_s *priv);
static void usbhost_uvc_freebuffers(FAR struct uvc_state_s *priv);
static int usbhost_uvc_vsrequest(FAR struct uvc_state_s *priv, uint8_t req,
                                 uint8_t cs, FAR uint8_t *data,
                                 uint16_t len);
static int usbhost_uvc_negotiate(FAR struct uvc_state_s *priv,
                                 FAR struct uvc_formatinfo_s *fmt,
                                 FAR struct uvc_frameinfo_s *frame,
                                 uint32_t interval, bool commit);
static void usbhost_uvc_selectalt(FAR struct uvc_state_s *priv);
static void usbhost_uvc_defaults(FAR struct uvc_state_s *priv,
                                 FAR struct uvc_formatinfo_s **fmt,
                                 FAR struct uvc_frameinfo_s **frame);
static int usbhost_uvc_setinterface(FAR struct uvc_state_s *priv,
                                    uint8_t alt);

/* Streaming */

static void usbhost_uvc_payload(FAR struct uvc_state_s *priv,
                                FAR const uint8_t *data, uint16_t len);
static void usbhost_uvc_complete(FAR void *arg, ssize_t result);
static int usbhost_uvc_submit(FAR struct uvc_state_s *priv,
                              FAR struct uvc_req_s *req);
static int usbhost_uvc_startstream(FAR struct uvc_state_s *priv);
static void usbhost_uvc_stopstream(FAR struct uvc_state_s *priv);

/* Video capture framework: image sensor operations */

static bool usbhost_uvc_is_available(FAR struct imgsensor_s *sensor);
static int usbhost_uvc_sensor_init(FAR struct imgsensor_s *sensor);
static int usbhost_uvc_sensor_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
  usbhost_uvc_get_driver_name(FAR struct imgsensor_s *sensor);
static int usbhost_uvc_sensor_validate(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_fmt,
                                       FAR imgsensor_format_t *fmt,
                                       FAR imgsensor_interval_t *interval);
static int usbhost_uvc_sensor_start(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_fmt,
                                    FAR imgsensor_format_t *fmt,
                                    FAR imgsensor_interval_t *interval);
static int usbhost_uvc_sensor_stop(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type);

/* Video capture framework: image data operations */

static int usbhost_uvc_data_init(FAR struct imgdata_s *data);
static int usbhost_uvc_data_uninit(FAR struct imgdata_s *data);
static int usbhost_uvc_data_setbuf(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts,
                                   FAR uint8_t *addr, uint32_t size);
static int usbhost_uvc_data_validate(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval);
static int usbhost_uvc_data_start(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *datafmts,
                                  FAR imgdata_interval_t *interval,
                                  imgdata_capture_t callback,
                                  FAR void *arg);
static int usbhost_uvc_data_stop(FAR struct imgdata_s *data);

/* Device registration */

static int usbhost_uvc_buildenum(FAR struct uvc_state_s *priv);
static int usbhost_uvc_register(FAR struct uvc_state_s *priv);
static int usbhost_uvc_devno_alloc(FAR struct uvc_state_s *priv);
static void usbhost_uvc_devno_free(FAR struct uvc_state_s *priv);
static int usbhost_uvc_parse(FAR struct uvc_state_s *priv,
                             FAR const uint8_t *configdesc, int desclen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A camera is offered to this driver in one of two shapes.  A device whose
 * VideoControl and VideoStreaming interfaces are grouped by an interface
 * association descriptor, which the UVC specification requires and which
 * essentially every camera does, arrives as a composite function identified
 * by the association: video / video interface collection.  A camera that
 * declares no class in its device descriptor instead arrives one interface
 * at a time, and it is the VideoControl interface that identifies it.
 */

static const struct usbhost_id_s g_uvc_id[] =
{
  {
    USB_CLASS_VIDEO,                   /* base */
    UVC_SC_VIDEO_INTERFACE_COLLECTION, /* subclass */
    0,                                 /* proto */
    0,                                 /* vid */
    0                                  /* pid */
  },
  {
    USB_CLASS_VIDEO,                   /* base */
    UVC_SC_VIDEOCONTROL,               /* subclass */
    0,                                 /* proto */
    0,                                 /* vid */
    0                                  /* pid */
  }
};

static const struct imgsensor_ops_s g_uvc_sensor_ops =
{
  .is_available           = usbhost_uvc_is_available,
  .init                   = usbhost_uvc_sensor_init,
  .uninit                 = usbhost_uvc_sensor_uninit,
  .get_driver_name        = usbhost_uvc_get_driver_name,
  .validate_frame_setting = usbhost_uvc_sensor_validate,
  .start_capture          = usbhost_uvc_sensor_start,
  .stop_capture           = usbhost_uvc_sensor_stop,
};

static const struct imgdata_ops_s g_uvc_data_ops =
{
  .init                   = usbhost_uvc_data_init,
  .uninit                 = usbhost_uvc_data_uninit,
  .set_buf                = usbhost_uvc_data_setbuf,
  .validate_frame_setting = usbhost_uvc_data_validate,
  .start_capture          = usbhost_uvc_data_start,
  .stop_capture           = usbhost_uvc_data_stop,
};

/* Bitmap of the /dev/videoN minor numbers this driver has taken */

static uint8_t g_uvc_devinuse;
static mutex_t g_uvc_devlock = NXMUTEX_INITIALIZER;

static struct usbhost_registry_s g_uvc =
{
  NULL,                                /* flink */
  usbhost_uvc_create,                  /* create */
  2,                                   /* nids */
  g_uvc_id                             /* id */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_uvc_getle16
 *
 * Description:
 *   Read a little endian 16-bit value from a descriptor field that is
 *   declared as a byte array because it is not naturally aligned.
 *
 ****************************************************************************/

static uint16_t usbhost_uvc_getle16(FAR const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_uvc_putle16
 *
 * Description:
 *   Store a little endian 16-bit value into a descriptor or request field
 *   that is declared as a byte array.
 *
 ****************************************************************************/

static void usbhost_uvc_putle16(FAR uint8_t *dest, uint16_t val)
{
  dest[0] = (uint8_t)(val & 0xff);
  dest[1] = (uint8_t)(val >> 8);
}

/****************************************************************************
 * Name: usbhost_uvc_pixfmt
 *
 * Description:
 *   Map the GUID of an uncompressed format descriptor onto the equivalent
 *   pixel format of the video framework.  Only the first four bytes, the
 *   FourCC, distinguish the formats we care about, but the whole GUID is
 *   compared so that a vendor format that happens to share a FourCC is not
 *   mistaken for a standard one.
 *
 * Returned Value:
 *   The IMGDATA_PIX_FMT_* value, or UINT32_MAX if the format is one we
 *   cannot describe.
 *
 ****************************************************************************/

static uint32_t usbhost_uvc_pixfmt(FAR const uint8_t *guid)
{
  static const uint8_t g_yuy2[UVC_GUID_LEN] = UVC_GUID_FORMAT_YUY2;
  static const uint8_t g_nv12[UVC_GUID_LEN] = UVC_GUID_FORMAT_NV12;
  static const uint8_t g_uyvy[UVC_GUID_LEN] = UVC_GUID_FORMAT_UYVY;

  if (memcmp(guid, g_yuy2, UVC_GUID_LEN) == 0)
    {
      return IMGDATA_PIX_FMT_YUYV;
    }

  if (memcmp(guid, g_uyvy, UVC_GUID_LEN) == 0)
    {
      return IMGDATA_PIX_FMT_UYVY;
    }

  if (memcmp(guid, g_nv12, UVC_GUID_LEN) == 0)
    {
      return IMGDATA_PIX_FMT_NV12;
    }

  return UINT32_MAX;
}

/****************************************************************************
 * Name: usbhost_uvc_parseframe
 *
 * Description:
 *   Extract one frame descriptor.  The uncompressed and MJPEG frame
 *   descriptors share a layout; they differ only in subtype.  The interval
 *   list at the end is either bFrameIntervalType discrete values or, when
 *   that field is zero, a continuous minimum, maximum and step triple.
 *
 ****************************************************************************/

static void usbhost_uvc_parseframe(FAR struct uvc_frameinfo_s *frame,
                                   FAR const uint8_t *desc)
{
  FAR const struct uvc_frame_desc_s *fdesc =
    (FAR const struct uvc_frame_desc_s *)desc;
  uint8_t nintervals = fdesc->bframeintervaltype;
  uint8_t len        = fdesc->blength;
  int i;

  frame->index         = fdesc->bframeindex;
  frame->width         = le16toh(fdesc->wwidth);
  frame->height        = le16toh(fdesc->wheight);
  frame->maxbuffersize = le32toh(fdesc->dwmaxvideoframebuffersize);
  frame->definterval   = le32toh(fdesc->dwdefaultframeinterval);
  frame->mininterval   = frame->definterval;
  frame->maxinterval   = frame->definterval;

  /* A frame interval is a period, so the fastest frame rate is the smallest
   * interval.  Ignore a list that does not fit in the descriptor rather than
   * reading past it.
   */

  if (nintervals == 0)
    {
      if (len >= UVC_FRAME_MINLEN + 3 * sizeof(uint32_t))
        {
          frame->mininterval = le32toh(fdesc->dwframeinterval[0]);
          frame->maxinterval = le32toh(fdesc->dwframeinterval[1]);
        }
    }
  else
    {
      for (i = 0; i < nintervals; i++)
        {
          uint32_t interval;

          if (len < UVC_FRAME_MINLEN + (i + 1) * sizeof(uint32_t))
            {
              break;
            }

          interval = le32toh(fdesc->dwframeinterval[i]);
          if (interval == 0)
            {
              continue;
            }

          if (i == 0 || interval < frame->mininterval)
            {
              frame->mininterval = interval;
            }

          if (i == 0 || interval > frame->maxinterval)
            {
              frame->maxinterval = interval;
            }
        }
    }

  if (frame->definterval == 0)
    {
      frame->definterval = frame->mininterval;
    }
}

/****************************************************************************
 * Name: usbhost_uvc_parse
 *
 * Description:
 *   Walk the configuration descriptor and extract everything the driver
 *   needs: the VideoControl interface and the UVC release it implements, the
 *   VideoStreaming interface with its alternate settings and streaming
 *   endpoint, and the formats and frame sizes it offers.
 *
 *   The walk is deliberately tolerant.  A camera carries descriptors this
 *   driver has no interest in, such as processing units, extension units and
 *   still image descriptors, as well as vendor descriptors it does not
 *   document at all; those are skipped.  Only a descriptor that is required
 *   to stream at all makes the whole device fail.
 *
 * Returned Value:
 *   Zero on success, or a negated errno value.
 *
 ****************************************************************************/

static int usbhost_uvc_parse(FAR struct uvc_state_s *priv,
                             FAR const uint8_t *configdesc, int desclen)
{
  FAR struct uvc_formatinfo_s *format = NULL;
  FAR const struct usb_desc_s *desc;
  uint8_t curif    = 0xff;    /* Interface the walk is currently inside */
  uint8_t cursub   = 0;       /* Its subclass */
  uint8_t curalt   = 0;
  bool    instream = false;   /* Inside the chosen streaming interface */
  int     nformats = 0;
  int     offset;
  int     len;
  int     pass;

  priv->ctrlif   = 0xff;
  priv->streamif = 0xff;

  /* Two passes: the first counts what has to be allocated, the second fills
   * it in.  Counting first avoids reallocating while parsing, and means a
   * descriptor set that turns out to be nonsense is rejected before any
   * memory has been committed to it.
   */

  for (pass = 0; pass < 2; pass++)
    {
      int fmtidx = -1;

      curif    = 0xff;
      instream = false;
      format   = NULL;

      for (offset = 0; offset + (int)sizeof(struct usb_desc_s) <= desclen;
           offset += len)
        {
          desc = (FAR const struct usb_desc_s *)&configdesc[offset];
          len  = desc->len;

          /* A zero or overlong length would make this loop spin or walk off
           * the end of the buffer.  Stop; what has been parsed so far may
           * still be usable.
           */

          if (len < (int)sizeof(struct usb_desc_s) ||
              offset + len > desclen)
            {
              uwarn("WARNING: Malformed descriptor at offset %d\n", offset);
              break;
            }

          switch (desc->type)
            {
              case USB_DESC_TYPE_INTERFACE:
                {
                  FAR const struct usb_ifdesc_s *ifdesc =
                    (FAR const struct usb_ifdesc_s *)desc;

                  if (len < USB_SIZEOF_IFDESC)
                    {
                      break;
                    }

                  curif  = ifdesc->ifno;
                  cursub = ifdesc->subclass;
                  curalt = ifdesc->alt;

                  if (ifdesc->classid != USB_CLASS_VIDEO)
                    {
                      instream = false;
                      break;
                    }

                  if (cursub == UVC_SC_VIDEOCONTROL)
                    {
                      priv->ctrlif = curif;
                      instream     = false;
                    }
                  else if (cursub == UVC_SC_VIDEOSTREAMING)
                    {
                      /* Take the first streaming interface offered.  A
                       * camera with several independent streams is exposed
                       * as its first one; the rest are ignored.
                       */

                      if (priv->streamif == 0xff)
                        {
                          priv->streamif  = curif;
                          priv->havestream = true;
                        }

                      instream = (curif == priv->streamif);

                      /* Alternate setting 0 of a streaming interface has no
                       * endpoint by definition; the others each offer one,
                       * recorded when its endpoint descriptor is reached.
                       */
                    }
                }
                break;

              case USB_DESC_TYPE_ENDPOINT:
                {
                  FAR const struct usb_epdesc_s *epdesc =
                    (FAR const struct usb_epdesc_s *)desc;
                  uint8_t attr;
                  uint16_t mxpacket;

                  if (!instream || len < USB_SIZEOF_EPDESC || pass != 0)
                    {
                      break;
                    }

                  attr = epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK;
                  if ((epdesc->addr & USB_EP_ADDR_NUMBER_MASK) == 0 ||
                      (epdesc->addr & USB_DIR_IN) == 0)
                    {
                      break;
                    }

                  if (attr != USB_EP_ATTR_XFER_ISOC &&
                      attr != USB_EP_ATTR_XFER_BULK)
                    {
                      break;
                    }

                  /* A high speed isochronous endpoint may transfer up to
                   * three transactions per microframe, and the multiplier
                   * lives in the upper bits of wMaxPacketSize.
                   */

                  mxpacket = usbhost_uvc_getle16(epdesc->mxpacketsize);
                  mxpacket = (mxpacket & USB_EP_MAX_PACKET_MASK) *
                             (USB_EP_MAX_PACKET_MULT(mxpacket) + 1);

                  priv->epaddr     = epdesc->addr;
                  priv->eptype     = attr;
                  priv->epinterval = epdesc->interval;

                  if (priv->nalts < UVC_MAX_ALTSETTINGS)
                    {
                      priv->alts[priv->nalts].alt     = curalt;
                      priv->alts[priv->nalts].pktsize = mxpacket;
                      priv->nalts++;
                    }
                }
                break;

              case UVC_CS_INTERFACE:
                {
                  uint8_t subtype = configdesc[offset + 2];

                  if (len < 3)
                    {
                      break;
                    }

                  /* VideoControl: the header carries the UVC release, which
                   * fixes the probe and commit payload length.
                   */

                  if (curif == priv->ctrlif && cursub == UVC_SC_VIDEOCONTROL)
                    {
                      if (subtype == UVC_VC_HEADER &&
                          len >= UVC_VC_HEADER_MINLEN && pass == 0)
                        {
                          FAR const struct uvc_vc_header_desc_s *hdr =
                            (FAR const struct uvc_vc_header_desc_s *)desc;

                          priv->bcduvc = le16toh(hdr->bcduvc);
                        }

                      break;
                    }

                  if (!instream)
                    {
                      break;
                    }

                  switch (subtype)
                    {
                      case UVC_VS_INPUT_HEADER:
                        break;

                      case UVC_VS_FORMAT_UNCOMPRESSED:
                      case UVC_VS_FORMAT_MJPEG:
                        {
                          uint32_t pixfmt;
                          uint8_t index;
                          uint8_t defframe;
                          uint8_t bpp = 0;

                          if (subtype == UVC_VS_FORMAT_UNCOMPRESSED)
                            {
                              FAR const struct
                                uvc_format_uncompressed_desc_s *f =
                                (FAR const struct
                                 uvc_format_uncompressed_desc_s *)desc;

                              if (len < UVC_FORMAT_UNCOMPRESSED_LEN)
                                {
                                  break;
                                }

                              pixfmt   = usbhost_uvc_pixfmt(f->guidformat);
                              index    = f->bformatindex;
                              defframe = f->bdefaultframeindex;
                              bpp      = f->bbitsperpixel;
                            }
                          else
                            {
                              FAR const struct uvc_format_mjpeg_desc_s *f =
                                (FAR const struct
                                 uvc_format_mjpeg_desc_s *)desc;

                              if (len < UVC_FORMAT_MJPEG_LEN)
                                {
                                  break;
                                }

                              pixfmt   = IMGDATA_PIX_FMT_JPEG;
                              index    = f->bformatindex;
                              defframe = f->bdefaultframeindex;
                            }

                          if (pixfmt == UINT32_MAX)
                            {
                              /* A format we have no name for.  Its frame
                               * descriptors follow and must not be attached
                               * to the previous format.
                               */

                              uinfo("Ignoring unsupported format %d\n",
                                    index);
                              format = NULL;
                              fmtidx = -1;
                              break;
                            }

                          if (fmtidx + 1 >= UVC_MAX_FORMATS)
                            {
                              uwarn("WARNING: Ignoring format %d, "
                                    "table full\n", index);
                              format = NULL;
                              break;
                            }

                          fmtidx++;

                          if (pass == 0)
                            {
                              nformats = fmtidx + 1;
                              format   = NULL;
                            }
                          else
                            {
                              format           = &priv->formats[fmtidx];
                              format->index    = index;
                              format->subtype  = subtype;
                              format->defframe = defframe;
                              format->bpp      = bpp;
                              format->pixfmt   = pixfmt;
                              format->nframes  = 0;
                            }
                        }
                        break;

                      case UVC_VS_FRAME_UNCOMPRESSED:
                      case UVC_VS_FRAME_MJPEG:
                        {
                          if (len < UVC_FRAME_MINLEN)
                            {
                              break;
                            }

                          if (pass == 0)
                            {
                              /* Nothing to count: every format is given the
                               * same maximum-sized frame table.
                               */
                            }
                          else if (format != NULL &&
                                   format->nframes < UVC_MAX_FRAMES)
                            {
                              usbhost_uvc_parseframe(
                                &format->frames[format->nframes],
                                (FAR const uint8_t *)desc);
                              format->nframes++;
                            }
                        }
                        break;

                      default:

                        /* Colour matching, still image, and anything a
                         * future revision adds.  Not needed to stream.
                         */

                        break;
                    }
                }
                break;

              default:

                /* Interface association, and any descriptor type this
                 * driver has no use for.
                 */

                break;
            }
        }

      /* Between the passes, commit the memory the first pass sized */

      if (pass == 0)
        {
          int i;

          if (priv->ctrlif == 0xff)
            {
              uerr("ERROR: No VideoControl interface\n");
              return -ENODEV;
            }

          if (!priv->havestream)
            {
              uerr("ERROR: No VideoStreaming interface\n");
              return -ENODEV;
            }

          if (nformats == 0)
            {
              uerr("ERROR: Camera offers no format this driver knows\n");
              return -ENOTSUP;
            }

          if (priv->nalts == 0)
            {
              uerr("ERROR: No usable streaming endpoint\n");
              return -ENODEV;
            }

          priv->formats = kmm_zalloc(nformats *
                                     sizeof(struct uvc_formatinfo_s));
          if (priv->formats == NULL)
            {
              return -ENOMEM;
            }

          priv->nformats = nformats;

          for (i = 0; i < nformats; i++)
            {
              priv->formats[i].frames =
                kmm_zalloc(UVC_MAX_FRAMES *
                           sizeof(struct uvc_frameinfo_s));
              if (priv->formats[i].frames == NULL)
                {
                  usbhost_uvc_freeformats(priv);
                  return -ENOMEM;
                }
            }
        }
    }

  /* The probe and commit payload length follows the reported release */

  if (priv->bcduvc >= 0x0150)
    {
      priv->probelen = UVC_PROBE_COMMIT_SIZE_15;
    }
  else if (priv->bcduvc >= 0x0110)
    {
      priv->probelen = UVC_PROBE_COMMIT_SIZE_11;
    }
  else
    {
      priv->probelen = UVC_PROBE_COMMIT_SIZE_10;
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_allocbuffers
 *
 * Description:
 *   Allocate the endpoint 0 buffers through the host controller driver, so
 *   that they satisfy whatever alignment or memory region the controller
 *   needs for control traffic.
 *
 ****************************************************************************/

static int usbhost_uvc_allocbuffers(FAR struct uvc_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  size_t maxlen;
  int ret;

  ret = DRVR_ALLOC(hport->drvr, &priv->ctrlreq, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of the control request failed: %d\n", ret);
      return ret;
    }

  if (maxlen < sizeof(struct usb_ctrlreq_s))
    {
      uerr("ERROR: Control buffer too small: %zu\n", maxlen);
      return -ENOMEM;
    }

  ret = DRVR_ALLOC(hport->drvr, &priv->ctrlbuf, &maxlen);
  if (ret < 0)
    {
      uerr("ERROR: DRVR_ALLOC of the control buffer failed: %d\n", ret);
      return ret;
    }

  if (maxlen < UVC_PROBE_COMMIT_SIZE_15)
    {
      uerr("ERROR: Control buffer too small for a probe payload: %zu\n",
           maxlen);
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_freebuffers
 ****************************************************************************/

static void usbhost_uvc_freebuffers(FAR struct uvc_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;

  if (priv->ctrlreq != NULL)
    {
      DRVR_FREE(hport->drvr, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  if (priv->ctrlbuf != NULL)
    {
      DRVR_FREE(hport->drvr, priv->ctrlbuf);
      priv->ctrlbuf = NULL;
    }
}

/****************************************************************************
 * Name: usbhost_uvc_vsrequest
 *
 * Description:
 *   Issue one class-specific request to the VideoStreaming interface.  The
 *   direction is taken from the request code, as the UVC specification
 *   encodes it there: the GET_* codes have bit 7 set, SET_CUR does not.
 *
 * Input Parameters:
 *   priv - The driver state
 *   req  - UVC_SET_CUR, UVC_GET_CUR, UVC_GET_MIN, UVC_GET_MAX, UVC_GET_RES,
 *          UVC_GET_DEF or UVC_GET_LEN
 *   cs   - The control selector, UVC_VS_PROBE_CONTROL or
 *          UVC_VS_COMMIT_CONTROL
 *   data - The payload, sent or received
 *   len  - Its length
 *
 * Returned Value:
 *   Zero on success, or a negated errno value.  A camera that does not
 *   implement an optional request stalls it, which arrives here as an
 *   error and is a normal answer rather than a failure.
 *
 ****************************************************************************/

static int usbhost_uvc_vsrequest(FAR struct uvc_state_s *priv, uint8_t req,
                                 uint8_t cs, FAR uint8_t *data,
                                 uint16_t len)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  FAR struct usb_ctrlreq_s *ctrlreq;
  bool in = (req & USB_REQ_DIR_IN) != 0;

  if (priv->disconnected)
    {
      return -ENODEV;
    }

  ctrlreq       = (FAR struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = (in ? USB_REQ_DIR_IN : USB_REQ_DIR_OUT) |
                  USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
  ctrlreq->req  = req;

  /* The control selector goes in the high byte of wValue, the low byte is
   * zero.  wIndex names the interface the control lives on.
   */

  usbhost_uvc_putle16(ctrlreq->value, (uint16_t)cs << 8);
  usbhost_uvc_putle16(ctrlreq->index, priv->streamif);
  usbhost_uvc_putle16(ctrlreq->len, len);

  if (in)
    {
      return DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, data);
    }

  return DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, data);
}

/****************************************************************************
 * Name: usbhost_uvc_negotiate
 *
 * Description:
 *   Run the UVC streaming negotiation for one format, frame size and frame
 *   interval.
 *
 *   Negotiation is a conversation, not a command.  The host proposes a
 *   configuration with SET_CUR on the probe control, and the camera answers
 *   GET_CUR with the configuration it will actually deliver, which is not
 *   necessarily the one proposed: it fills in the payload and frame sizes,
 *   and it may move the frame interval, or even the format and frame index,
 *   to something it can sustain.  Whatever comes back is what gets
 *   committed, so this function reports the camera's answer rather than the
 *   proposal.
 *
 *   The proposal starts from the camera's own defaults, read with GET_DEF,
 *   so that the fields this driver has no opinion about, key frame rate and
 *   compression quality among them, carry values the camera likes.  A
 *   camera that does not implement GET_DEF stalls it, which is not fatal.
 *
 * Input Parameters:
 *   priv     - The driver state
 *   fmt      - The format to ask for
 *   frame    - The frame size to ask for
 *   interval - The frame interval to ask for, in 100ns units
 *   commit   - Commit the result, which makes it take effect as soon as the
 *              streaming interface is switched to an alternate setting with
 *              bandwidth.  When false the camera is only asked what it
 *              would do, which is how the driver validates a format without
 *              disturbing the current one.
 *
 * Returned Value:
 *   Zero on success, or a negated errno value.
 *
 ****************************************************************************/

static int usbhost_uvc_negotiate(FAR struct uvc_state_s *priv,
                                 FAR struct uvc_formatinfo_s *fmt,
                                 FAR struct uvc_frameinfo_s *frame,
                                 uint32_t interval, bool commit)
{
  FAR struct uvc_streaming_control_s *ctrl =
    (FAR struct uvc_streaming_control_s *)priv->ctrlbuf;
  uint8_t len = priv->probelen;
  uint32_t maxpayload;
  uint32_t maxframe;
  int ret;

  /* Start from the camera's defaults where it offers them */

  memset(ctrl, 0, UVC_PROBE_COMMIT_SIZE_15);

  ret = usbhost_uvc_vsrequest(priv, UVC_GET_DEF, UVC_VS_PROBE_CONTROL,
                              priv->ctrlbuf, len);
  if (ret < 0)
    {
      uinfo("GET_DEF is not supported (%d), proposing from zero\n", ret);
      memset(ctrl, 0, UVC_PROBE_COMMIT_SIZE_15);
    }

  /* Ask for this format, frame size and frame interval.  bmHint bit 0 tells
   * the camera to keep the frame interval fixed and vary everything else.
   */

  ctrl->bmhint          = htole16(1);
  ctrl->bformatindex    = fmt->index;
  ctrl->bframeindex     = frame->index;
  ctrl->dwframeinterval = htole32(interval);

  ret = usbhost_uvc_vsrequest(priv, UVC_SET_CUR, UVC_VS_PROBE_CONTROL,
                              priv->ctrlbuf, len);
  if (ret < 0)
    {
      uerr("ERROR: Probe SET_CUR rejected: %d\n", ret);
      return ret;
    }

  /* Read back what the camera will really do */

  ret = usbhost_uvc_vsrequest(priv, UVC_GET_CUR, UVC_VS_PROBE_CONTROL,
                              priv->ctrlbuf, len);
  if (ret < 0)
    {
      uerr("ERROR: Probe GET_CUR failed: %d\n", ret);
      return ret;
    }

  maxframe   = le32toh(ctrl->dwmaxvideoframesize);
  maxpayload = le32toh(ctrl->dwmaxpayloadtransfersize);

  if (ctrl->bformatindex != fmt->index ||
      ctrl->bframeindex != frame->index)
    {
      /* The camera substituted something else.  That is its right, and the
       * substitution is what will be delivered, so take it.
       */

      uwarn("WARNING: Camera answered with format %d frame %d, "
            "not the requested format %d frame %d\n",
            ctrl->bformatindex, ctrl->bframeindex, fmt->index,
            frame->index);
    }

  /* A camera is allowed to report no payload size, and a few report
   * nonsense.  Fall back to the frame size, which always bounds a payload.
   */

  if (maxpayload == 0 || maxpayload > maxframe + UVC_PAYLOAD_HEADER_LEN)
    {
      maxpayload = maxframe != 0 ? maxframe : frame->maxbuffersize;
    }

  if (maxframe == 0)
    {
      maxframe = frame->maxbuffersize;
    }

  if (maxframe == 0)
    {
      uerr("ERROR: Camera reports no frame size\n");
      return -EIO;
    }

  if (commit)
    {
      ret = usbhost_uvc_vsrequest(priv, UVC_SET_CUR, UVC_VS_COMMIT_CONTROL,
                                  priv->ctrlbuf, len);
      if (ret < 0)
        {
          uerr("ERROR: Commit rejected: %d\n", ret);
          return ret;
        }
    }

  priv->curfmt      = fmt;
  priv->curframe    = frame;
  priv->curinterval = le32toh(ctrl->dwframeinterval);
  priv->maxframesize = maxframe;
  priv->maxpayload  = maxpayload;

  usbhost_uvc_selectalt(priv);

  uinfo("Negotiated %s %dx%d at %" PRIu32 " fps: frame %" PRIu32 " bytes, "
        "payload %" PRIu32 " bytes, alt %d (%d bytes per microframe)\n",
        fmt->subtype == UVC_VS_FORMAT_MJPEG ? "MJPEG" : "uncompressed",
        frame->width, frame->height,
        priv->curinterval ?
          (uint32_t)(10000000ul / priv->curinterval) : 0,
        priv->maxframesize, priv->maxpayload, priv->curalt,
        priv->curpktsize);

  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_selectalt
 *
 * Description:
 *   Choose the alternate setting to stream on.  Alternate settings of a
 *   streaming interface differ only in how much isochronous bandwidth their
 *   endpoint reserves, and the bus grants that bandwidth for as long as the
 *   setting is selected.  Take the cheapest setting that can still carry the
 *   negotiated payload, so that the camera does not hold bandwidth it will
 *   not use and other devices can still be enumerated.
 *
 *   A bulk streaming endpoint has no alternate settings to choose between.
 *
 ****************************************************************************/

static void usbhost_uvc_selectalt(FAR struct uvc_state_s *priv)
{
  uint32_t needed;
  int best = -1;
  int i;

  if (priv->eptype != USB_EP_ATTR_XFER_ISOC)
    {
      priv->curalt     = priv->nalts > 0 ? priv->alts[0].alt : 0;
      priv->curpktsize = priv->nalts > 0 ? priv->alts[0].pktsize : 0;
      return;
    }

  /* dwMaxPayloadTransferSize is what the camera wants to move in one
   * microframe, so it is directly comparable with wMaxPacketSize.
   */

  needed = priv->maxpayload;

  for (i = 0; i < priv->nalts; i++)
    {
      if (priv->alts[i].pktsize >= needed &&
          (best < 0 || priv->alts[i].pktsize < priv->alts[best].pktsize))
        {
          best = i;
        }
    }

  if (best < 0)
    {
      /* Nothing is big enough.  Take the largest and let the camera split
       * the payload across microframes, which the payload header allows.
       */

      for (i = 0; i < priv->nalts; i++)
        {
          if (best < 0 || priv->alts[i].pktsize > priv->alts[best].pktsize)
            {
              best = i;
            }
        }

      uwarn("WARNING: No alternate setting carries %" PRIu32 " bytes, "
            "using the largest\n", needed);
    }

  priv->curalt     = priv->alts[best].alt;
  priv->curpktsize = priv->alts[best].pktsize;
}

/****************************************************************************
 * Name: usbhost_uvc_defaults
 *
 * Description:
 *   Choose the format and frame size to use when the application has not
 *   asked for anything in particular.  The camera names a default frame for
 *   each format, and the format preference is a configuration choice:
 *   MJPEG costs far less bandwidth, uncompressed needs no decoder.
 *
 ****************************************************************************/

static void usbhost_uvc_defaults(FAR struct uvc_state_s *priv,
                                 FAR struct uvc_formatinfo_s **fmt,
                                 FAR struct uvc_frameinfo_s **frame)
{
  FAR struct uvc_formatinfo_s *chosen = &priv->formats[0];
  int i;
  int j;

  for (i = 0; i < priv->nformats; i++)
    {
      bool mjpeg = priv->formats[i].subtype == UVC_VS_FORMAT_MJPEG;

#ifdef CONFIG_USBHOST_UVC_PREFER_MJPEG
      if (mjpeg)
#else
      if (!mjpeg)
#endif
        {
          chosen = &priv->formats[i];
          break;
        }
    }

  *fmt   = chosen;
  *frame = &chosen->frames[0];

  for (j = 0; j < chosen->nframes; j++)
    {
      if (chosen->frames[j].index == chosen->defframe)
        {
          *frame = &chosen->frames[j];
          break;
        }
    }
}

/****************************************************************************
 * Name: usbhost_uvc_setinterface
 *
 * Description:
 *   Select an alternate setting of the streaming interface.  This is what
 *   actually reserves or releases isochronous bandwidth on the bus: setting
 *   0 has no endpoint and so costs nothing, and the camera sends nothing
 *   until a setting with an endpoint is selected.
 *
 ****************************************************************************/

static int usbhost_uvc_setinterface(FAR struct uvc_state_s *priv,
                                    uint8_t alt)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  FAR struct usb_ctrlreq_s *ctrlreq;

  if (priv->disconnected)
    {
      return -ENODEV;
    }

  ctrlreq       = (FAR struct usb_ctrlreq_s *)priv->ctrlreq;
  ctrlreq->type = USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD |
                  USB_REQ_RECIPIENT_INTERFACE;
  ctrlreq->req  = USB_REQ_SETINTERFACE;

  usbhost_uvc_putle16(ctrlreq->value, alt);
  usbhost_uvc_putle16(ctrlreq->index, priv->streamif);
  usbhost_uvc_putle16(ctrlreq->len, 0);

  return DRVR_CTRLOUT(hport->drvr, hport->ep0, ctrlreq, NULL);
}

/****************************************************************************
 * Name: usbhost_uvc_deliver
 *
 * Description:
 *   Hand the assembled frame to the video capture framework and start a new
 *   one.  A frame that was spoiled by a payload error is dropped: passing a
 *   torn frame up would be worse than missing one.
 *
 ****************************************************************************/

static void usbhost_uvc_deliver(FAR struct uvc_state_s *priv)
{
  struct timespec ts;
  struct timeval tv;

  if (priv->frameoffset > 0 && !priv->framebad &&
      priv->capture_cb != NULL && priv->framebuf != NULL)
    {
      clock_gettime(CLOCK_MONOTONIC, &ts);
      TIMESPEC_TO_TIMEVAL(&tv, &ts);

      /* The framework takes the buffer back here and gives us the next one
       * through set_buf(), so stop writing into this one.
       */

      priv->framebuf = NULL;
      priv->capture_cb(0, priv->frameoffset, &tv, priv->capture_arg);
    }
  else if (priv->frameoffset > 0)
    {
      uinfo("Dropping a spoiled frame of %" PRIu32 " bytes\n",
            priv->frameoffset);
    }

  priv->frameoffset  = 0;
  priv->framebad     = false;
  priv->framestarted = false;
}

/****************************************************************************
 * Name: usbhost_uvc_payload
 *
 * Description:
 *   Consume one UVC payload and add it to the frame being assembled.
 *
 *   Every payload starts with a header giving its own length and a set of
 *   flags.  Two of those flags carry the framing.  FID toggles between
 *   consecutive frames, so a change of FID means the previous frame ended
 *   and a new one starts here; EOF marks the last payload of a frame.  Both
 *   are honoured, because not every camera sets EOF reliably, and relying on
 *   FID alone would delay a frame until the first payload of the next one
 *   arrives.
 *
 *   The data here comes straight off the wire from a device that may be
 *   faulty or hostile, so every length is checked before it is used.
 *
 ****************************************************************************/

static void usbhost_uvc_payload(FAR struct uvc_state_s *priv,
                                FAR const uint8_t *data, uint16_t len)
{
  uint8_t hdrlen;
  uint8_t flags;
  uint8_t fid;
  uint32_t paylen;

  /* An empty packet is normal: an isochronous endpoint keeps its slot on the
   * bus whether or not the camera has anything to send.
   */

  if (len == 0)
    {
      return;
    }

  if (len < UVC_HEADER_MINLEN)
    {
      uwarn("WARNING: Runt payload of %d bytes\n", len);
      priv->framebad = true;
      return;
    }

  hdrlen = data[0];
  flags  = data[1];

  if (hdrlen < UVC_HEADER_MINLEN || hdrlen > len)
    {
      uwarn("WARNING: Payload header claims %d bytes of %d\n", hdrlen, len);
      priv->framebad = true;
      return;
    }

  fid = (flags & UVC_STREAM_FID) != 0;

  /* The camera reports that it could not produce this payload correctly */

  if ((flags & UVC_STREAM_ERR) != 0)
    {
      priv->framebad = true;
    }

  if (!priv->framestarted)
    {
      priv->framestarted = true;
      priv->lastfid      = fid;
      priv->frameoffset  = 0;
    }
  else if (fid != priv->lastfid)
    {
      /* The frame ID toggled, so the previous frame ended without an EOF */

      usbhost_uvc_deliver(priv);
      priv->framestarted = true;
      priv->lastfid      = fid;
    }

  paylen = len - hdrlen;

  if (paylen > 0)
    {
      if (priv->framebuf == NULL)
        {
          /* No buffer to fill.  The framework has not given us the next one
           * yet, so this frame is lost; keep tracking the framing so that
           * the next frame starts cleanly.
           */

          priv->framebad = true;
        }
      else if (priv->frameoffset + paylen > priv->framebufsize)
        {
          /* More data than the negotiated frame size.  Never write past the
           * buffer; drop the frame instead.
           */

          uwarn("WARNING: Frame exceeds %" PRIu32 " bytes, dropping\n",
                priv->framebufsize);
          priv->framebad = true;
        }
      else
        {
          memcpy(priv->framebuf + priv->frameoffset, data + hdrlen, paylen);
          priv->frameoffset += paylen;
        }
    }

  if ((flags & UVC_STREAM_EOF) != 0)
    {
      usbhost_uvc_deliver(priv);
    }
}

/****************************************************************************
 * Name: usbhost_uvc_complete
 *
 * Description:
 *   One streaming transfer finished.  Consume its packets and queue it
 *   again, unless streaming has been stopped or the camera is gone.
 *
 *   This runs on the host controller driver's completion context, which is a
 *   task context rather than an interrupt handler, so it may take the lock
 *   and submit the next transfer directly.
 *
 ****************************************************************************/

static void usbhost_uvc_complete(FAR void *arg, ssize_t result)
{
  FAR struct uvc_req_s *req = (FAR struct uvc_req_s *)arg;
  FAR struct uvc_state_s *priv = req->priv;
  int i;

  req->queued = false;

  if (result < 0)
    {
      /* A failed transfer loses whatever the camera sent during it, so the
       * frame being assembled has a hole in it.
       */

      uinfo("Streaming transfer failed: %zd\n", result);
      priv->framebad = true;
    }
  else if (priv->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      for (i = 0; i < req->isoc.npackets; i++)
        {
          uint16_t len = req->pktlen[i];

          /* A device could report more than the packet can hold */

          if (len > req->isoc.pktsize)
            {
              uwarn("WARNING: Packet %d claims %d of %d bytes\n", i, len,
                    req->isoc.pktsize);
              priv->framebad = true;
              continue;
            }

          usbhost_uvc_payload(priv,
                              req->buffer + i * req->isoc.pktsize, len);
        }
    }
  else
    {
      /* A bulk payload is delimited by the transfer itself */

      usbhost_uvc_payload(priv, req->buffer, (uint16_t)result);
    }

  if (priv->streaming && !priv->disconnected)
    {
      if (usbhost_uvc_submit(priv, req) >= 0)
        {
          return;
        }
    }

  /* This request is done for good.  Wake the stopper once the last one has
   * drained, so that no transfer is still touching a buffer we are about to
   * free.
   */

  if (--priv->inflight == 0)
    {
      nxsem_post(&priv->donesem);
    }
}

/****************************************************************************
 * Name: usbhost_uvc_submit
 *
 * Description:
 *   Queue one streaming transfer.
 *
 ****************************************************************************/

static int usbhost_uvc_submit(FAR struct uvc_state_s *priv,
                              FAR struct uvc_req_s *req)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  int ret;

  if (priv->eptype == USB_EP_ATTR_XFER_ISOC)
    {
      ret = DRVR_ISOCASYNCH(hport->drvr, priv->epstream, &req->isoc,
                            usbhost_uvc_complete, req);
    }
  else
    {
      ret = DRVR_ASYNCH(hport->drvr, priv->epstream, req->buffer,
                        priv->maxpayload, usbhost_uvc_complete, req);
    }

  if (ret >= 0)
    {
      req->queued = true;
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_uvc_freereqs
 ****************************************************************************/

static void usbhost_uvc_freereqs(FAR struct uvc_state_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_USBHOST_UVC_NREQS; i++)
    {
      if (priv->reqs[i].buffer != NULL)
        {
          kmm_free(priv->reqs[i].buffer);
          priv->reqs[i].buffer = NULL;
        }

      if (priv->reqs[i].pktlen != NULL)
        {
          kmm_free(priv->reqs[i].pktlen);
          priv->reqs[i].pktlen = NULL;
        }
    }
}

/****************************************************************************
 * Name: usbhost_uvc_startstream
 *
 * Description:
 *   Commit the negotiated configuration, open the streaming endpoint and
 *   fill it with transfers.
 *
 *   The order matters.  The configuration is committed while the interface
 *   is still on alternate setting 0, because that is when the camera is
 *   allowed to accept it; selecting a setting with bandwidth is what starts
 *   the camera sending.
 *
 ****************************************************************************/

static int usbhost_uvc_startstream(FAR struct uvc_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  struct usbhost_epdesc_s epdesc;
  uint16_t npackets;
  uint32_t bufsize;
  int ret;
  int i;

  if (priv->streaming)
    {
      return OK;
    }

  if (priv->curfmt == NULL || priv->curframe == NULL)
    {
      return -EINVAL;
    }

  /* Commit what was negotiated */

  ret = usbhost_uvc_negotiate(priv, priv->curfmt, priv->curframe,
                              priv->curinterval, true);
  if (ret < 0)
    {
      return ret;
    }

  npackets = priv->eptype == USB_EP_ATTR_XFER_ISOC ?
             CONFIG_USBHOST_UVC_NPACKETS : 1;
  bufsize  = priv->eptype == USB_EP_ATTR_XFER_ISOC ?
             (uint32_t)npackets * priv->curpktsize : priv->maxpayload;

  for (i = 0; i < CONFIG_USBHOST_UVC_NREQS; i++)
    {
      FAR struct uvc_req_s *req = &priv->reqs[i];

      req->priv   = priv;
      req->queued = false;
      req->buffer = kmm_malloc(bufsize);
      if (req->buffer == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_reqs;
        }

      if (priv->eptype == USB_EP_ATTR_XFER_ISOC)
        {
          req->pktlen = kmm_zalloc(npackets * sizeof(uint16_t));
          if (req->pktlen == NULL)
            {
              ret = -ENOMEM;
              goto errout_with_reqs;
            }

          req->isoc.buffer   = req->buffer;
          req->isoc.pktlen   = req->pktlen;
          req->isoc.pktsize  = priv->curpktsize;
          req->isoc.npackets = npackets;
        }
    }

  /* Ask for the bandwidth.  Until this point the camera has sent nothing. */

  ret = usbhost_uvc_setinterface(priv, priv->curalt);
  if (ret < 0)
    {
      uerr("ERROR: Cannot select alternate setting %d: %d\n",
           priv->curalt, ret);
      goto errout_with_reqs;
    }

  epdesc.hport        = hport;
  epdesc.addr         = priv->epaddr & USB_EP_ADDR_NUMBER_MASK;
  epdesc.in           = true;
  epdesc.xfrtype      = priv->eptype;
  epdesc.interval     = priv->epinterval;
  epdesc.mxpacketsize = priv->curpktsize;

  ret = DRVR_EPALLOC(hport->drvr, &epdesc, &priv->epstream);
  if (ret < 0)
    {
      uerr("ERROR: Cannot allocate the streaming endpoint: %d\n", ret);
      goto errout_with_altsetting;
    }

  priv->frameoffset  = 0;
  priv->framestarted = false;
  priv->framebad     = false;
  priv->streaming    = true;
  priv->inflight     = 0;

  for (i = 0; i < CONFIG_USBHOST_UVC_NREQS; i++)
    {
      ret = usbhost_uvc_submit(priv, &priv->reqs[i]);
      if (ret < 0)
        {
          if (priv->inflight == 0)
            {
              uerr("ERROR: Cannot queue a streaming transfer: %d\n", ret);
              priv->streaming = false;
              goto errout_with_ep;
            }

          /* Some transfers are queued, which is enough to stream, just with
           * less tolerance for delay.
           */

          uwarn("WARNING: Only %d of %d transfers could be queued\n",
                priv->inflight, CONFIG_USBHOST_UVC_NREQS);
          break;
        }

      priv->inflight++;
    }

  uinfo("Streaming %dx%d, %d transfers of %d packets\n",
        priv->curframe->width, priv->curframe->height,
        priv->inflight, npackets);

  return OK;

errout_with_ep:
  DRVR_EPFREE(hport->drvr, priv->epstream);
  priv->epstream = NULL;

errout_with_altsetting:
  usbhost_uvc_setinterface(priv, 0);

errout_with_reqs:
  usbhost_uvc_freereqs(priv);
  return ret;
}

/****************************************************************************
 * Name: usbhost_uvc_stopstream
 *
 * Description:
 *   Stop the camera and release everything streaming was using.  Nothing is
 *   freed until every queued transfer has come back, because a transfer
 *   still in flight is writing into one of those buffers.
 *
 ****************************************************************************/

static void usbhost_uvc_stopstream(FAR struct uvc_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;

  if (!priv->streaming)
    {
      return;
    }

  /* Tell the completions to stop resubmitting, then let the transfers that
   * are already queued drain.
   */

  priv->streaming = false;

  if (priv->inflight > 0)
    {
      if (priv->epstream != NULL)
        {
          DRVR_CANCEL(hport->drvr, priv->epstream);
        }

      while (priv->inflight > 0)
        {
          if (nxsem_wait_uninterruptible(&priv->donesem) < 0)
            {
              break;
            }
        }
    }

  /* Give the bandwidth back.  Alternate setting 0 has no endpoint, so the
   * camera stops sending and the bus reservation is released.
   */

  if (!priv->disconnected)
    {
      usbhost_uvc_setinterface(priv, 0);
    }

  if (priv->epstream != NULL)
    {
      DRVR_EPFREE(hport->drvr, priv->epstream);
      priv->epstream = NULL;
    }

  usbhost_uvc_freereqs(priv);

  priv->framebuf     = NULL;
  priv->frameoffset  = 0;
  priv->framestarted = false;
  priv->capture_cb   = NULL;

  uinfo("Streaming stopped\n");
}

/****************************************************************************
 * Name: usbhost_uvc_report
 *
 * Description:
 *   Log what the camera turned out to be.  This is the driver's account of
 *   the device and the first thing to look at when a camera misbehaves.
 *
 ****************************************************************************/

static void usbhost_uvc_report(FAR struct uvc_state_s *priv)
{
  int i;
  int j;

  uinfo("UVC %d.%02x camera: VideoControl if %d, VideoStreaming if %d\n",
        priv->bcduvc >> 8, priv->bcduvc & 0xff, priv->ctrlif,
        priv->streamif);
  uinfo("  streaming endpoint %02x, %s, %d alternate settings, "
        "probe payload %d bytes\n", priv->epaddr,
        priv->eptype == USB_EP_ATTR_XFER_ISOC ? "isochronous" : "bulk",
        priv->nalts, priv->probelen);

  for (i = 0; i < priv->nalts; i++)
    {
      uinfo("    alt %d: %d bytes per microframe\n",
            priv->alts[i].alt, priv->alts[i].pktsize);
    }

  for (i = 0; i < priv->nformats; i++)
    {
      FAR struct uvc_formatinfo_s *fmt = &priv->formats[i];

      uinfo("  format %d: %s, %d frame sizes\n", fmt->index,
            fmt->subtype == UVC_VS_FORMAT_MJPEG ? "MJPEG" : "uncompressed",
            fmt->nframes);

      for (j = 0; j < fmt->nframes; j++)
        {
          FAR struct uvc_frameinfo_s *frame = &fmt->frames[j];

          /* An interval is in 100ns units, so 10000000/interval is the
           * frame rate in Hz.
           */

          uinfo("    frame %d: %dx%d, up to %" PRIu32 " fps, "
                "%" PRIu32 " byte buffer\n", frame->index, frame->width,
                frame->height,
                frame->mininterval ?
                  (uint32_t)(10000000ul / frame->mininterval) : 0,
                frame->maxbuffersize);
        }
    }
}

/****************************************************************************
 * Name: usbhost_uvc_freeformats
 ****************************************************************************/

static void usbhost_uvc_freeformats(FAR struct uvc_state_s *priv)
{
  int i;

  if (priv->formats == NULL)
    {
      return;
    }

  for (i = 0; i < priv->nformats; i++)
    {
      if (priv->formats[i].frames != NULL)
        {
          kmm_free(priv->formats[i].frames);
        }
    }

  kmm_free(priv->formats);
  priv->formats  = NULL;
  priv->nformats = 0;
}

/****************************************************************************
 * Name: usbhost_uvc_findformat
 *
 * Description:
 *   Find the format and frame size matching what the application asked for.
 *   An exact match on width and height is required; a camera reports the
 *   sizes it has and the framework has already offered them to the caller.
 *
 ****************************************************************************/

static int usbhost_uvc_findformat(FAR struct uvc_state_s *priv,
                                  uint32_t pixfmt, uint16_t width,
                                  uint16_t height,
                                  FAR struct uvc_formatinfo_s **fmtout,
                                  FAR struct uvc_frameinfo_s **frameout)
{
  int i;
  int j;

  for (i = 0; i < priv->nformats; i++)
    {
      FAR struct uvc_formatinfo_s *fmt = &priv->formats[i];

      if (fmt->pixfmt != pixfmt)
        {
          continue;
        }

      for (j = 0; j < fmt->nframes; j++)
        {
          if (fmt->frames[j].width == width &&
              fmt->frames[j].height == height)
            {
              *fmtout   = fmt;
              *frameout = &fmt->frames[j];
              return OK;
            }
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: usbhost_uvc_interval
 *
 * Description:
 *   Convert a frame rate expressed as the framework's interval fraction into
 *   the UVC frame interval, which is a period in 100ns units.  Clamp it to
 *   what the frame descriptor offers rather than asking for a rate the
 *   camera cannot produce.
 *
 ****************************************************************************/

static uint32_t usbhost_uvc_interval(FAR struct uvc_frameinfo_s *frame,
                                     FAR imgdata_interval_t *interval)
{
  uint64_t value;

  if (interval == NULL || interval->denominator == 0)
    {
      return frame->definterval;
    }

  value = (uint64_t)interval->numerator * 10000000ull /
          interval->denominator;

  if (value < frame->mininterval)
    {
      value = frame->mininterval;
    }
  else if (value > frame->maxinterval)
    {
      value = frame->maxinterval;
    }

  return (uint32_t)value;
}

/****************************************************************************
 * Name: Image sensor operations
 *
 * Description:
 *   A UVC camera is a single device that answers for both halves of the
 *   video capture framework's model.  The sensor half describes what the
 *   camera can do and validates requests; the data half, below, moves the
 *   pixels.
 *
 ****************************************************************************/

static bool usbhost_uvc_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct uvc_state_s *priv =
    container_of(sensor, struct uvc_state_s, sensor);

  return !priv->disconnected;
}

static int usbhost_uvc_sensor_init(FAR struct imgsensor_s *sensor)
{
  return OK;
}

static int usbhost_uvc_sensor_uninit(FAR struct imgsensor_s *sensor)
{
  return OK;
}

static FAR const char *
  usbhost_uvc_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "USB Video Class";
}

static int usbhost_uvc_sensor_validate(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_fmt,
                                       FAR imgsensor_format_t *fmt,
                                       FAR imgsensor_interval_t *interval)
{
  FAR struct uvc_state_s *priv =
    container_of(sensor, struct uvc_state_s, sensor);
  FAR struct uvc_formatinfo_s *format;
  FAR struct uvc_frameinfo_s *frame;

  if (nr_fmt != 1)
    {
      return -ENOTSUP;
    }

  return usbhost_uvc_findformat(priv, fmt[IMGSENSOR_FMT_MAIN].pixelformat,
                                fmt[IMGSENSOR_FMT_MAIN].width,
                                fmt[IMGSENSOR_FMT_MAIN].height,
                                &format, &frame);
}

static int usbhost_uvc_sensor_start(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_fmt,
                                    FAR imgsensor_format_t *fmt,
                                    FAR imgsensor_interval_t *interval)
{
  return OK;
}

static int usbhost_uvc_sensor_stop(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type)
{
  return OK;
}

/****************************************************************************
 * Name: Image data operations
 ****************************************************************************/

static int usbhost_uvc_data_init(FAR struct imgdata_s *data)
{
  FAR struct uvc_state_s *priv =
    container_of(data, struct uvc_state_s, data);

  return priv->disconnected ? -ENODEV : OK;
}

static int usbhost_uvc_data_uninit(FAR struct imgdata_s *data)
{
  return OK;
}

static int usbhost_uvc_data_setbuf(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   FAR imgdata_format_t *datafmts,
                                   FAR uint8_t *addr, uint32_t size)
{
  FAR struct uvc_state_s *priv =
    container_of(data, struct uvc_state_s, data);
  irqstate_t flags;

  if (addr == NULL || size == 0)
    {
      return -EINVAL;
    }

  /* The completion handler reads these while assembling a frame */

  flags = enter_critical_section();
  priv->framebuf     = addr;
  priv->framebufsize = size;
  leave_critical_section(flags);

  return OK;
}

static int usbhost_uvc_data_validate(FAR struct imgdata_s *data,
                                     uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval)
{
  FAR struct uvc_state_s *priv =
    container_of(data, struct uvc_state_s, data);
  FAR struct uvc_formatinfo_s *format;
  FAR struct uvc_frameinfo_s *frame;

  if (nr_datafmts != 1)
    {
      return -ENOTSUP;
    }

  return usbhost_uvc_findformat(priv,
                                datafmts[IMGDATA_FMT_MAIN].pixelformat,
                                datafmts[IMGDATA_FMT_MAIN].width,
                                datafmts[IMGDATA_FMT_MAIN].height,
                                &format, &frame);
}

static int usbhost_uvc_data_start(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmts,
                                  FAR imgdata_format_t *datafmts,
                                  FAR imgdata_interval_t *interval,
                                  imgdata_capture_t callback,
                                  FAR void *arg)
{
  FAR struct uvc_state_s *priv =
    container_of(data, struct uvc_state_s, data);
  FAR struct uvc_formatinfo_s *format;
  FAR struct uvc_frameinfo_s *frame;
  int ret;

  if (nr_datafmts != 1)
    {
      return -ENOTSUP;
    }

  ret = usbhost_uvc_findformat(priv,
                               datafmts[IMGDATA_FMT_MAIN].pixelformat,
                               datafmts[IMGDATA_FMT_MAIN].width,
                               datafmts[IMGDATA_FMT_MAIN].height,
                               &format, &frame);
  if (ret < 0)
    {
      verr("No such format: %" PRIu32 " %dx%d\n",
           datafmts[IMGDATA_FMT_MAIN].pixelformat,
           datafmts[IMGDATA_FMT_MAIN].width,
           datafmts[IMGDATA_FMT_MAIN].height);
      return ret;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->disconnected)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  priv->capture_cb  = callback;
  priv->capture_arg = arg;

  /* Negotiate this configuration before committing to it, so that a format
   * the camera turns out to dislike is refused here rather than producing a
   * stream that never delivers a frame.
   */

  ret = usbhost_uvc_negotiate(priv, format, frame,
                              usbhost_uvc_interval(frame, interval), false);
  if (ret >= 0)
    {
      ret = usbhost_uvc_startstream(priv);
    }

  if (ret < 0)
    {
      priv->capture_cb = NULL;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int usbhost_uvc_data_stop(FAR struct imgdata_s *data)
{
  FAR struct uvc_state_s *priv =
    container_of(data, struct uvc_state_s, data);
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  usbhost_uvc_stopstream(priv);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_devno_alloc
 *
 * Description:
 *   Take the lowest free /dev/videoN minor number, so that several cameras
 *   can be attached at once.
 *
 ****************************************************************************/

static int usbhost_uvc_devno_alloc(FAR struct uvc_state_s *priv)
{
  int devno;
  int ret;

  ret = nxmutex_lock(&g_uvc_devlock);
  if (ret < 0)
    {
      return ret;
    }

  for (devno = 0; devno < UVC_MAX_DEVICES; devno++)
    {
      uint8_t bit = 1 << devno;

      if ((g_uvc_devinuse & bit) == 0)
        {
          g_uvc_devinuse |= bit;
          priv->devno = devno;
          nxmutex_unlock(&g_uvc_devlock);
          return OK;
        }
    }

  nxmutex_unlock(&g_uvc_devlock);
  return -EMFILE;
}

/****************************************************************************
 * Name: usbhost_uvc_devno_free
 ****************************************************************************/

static void usbhost_uvc_devno_free(FAR struct uvc_state_s *priv)
{
  if (priv->devno >= 0 && priv->devno < UVC_MAX_DEVICES)
    {
      nxmutex_lock(&g_uvc_devlock);
      g_uvc_devinuse &= ~(1 << priv->devno);
      nxmutex_unlock(&g_uvc_devlock);
      priv->devno = -1;
    }
}

/****************************************************************************
 * Name: usbhost_uvc_v4l2fmt
 *
 * Description:
 *   Map the framework's internal pixel format onto the V4L2 four character
 *   code that user space sees.  The enumeration ioctls report the four
 *   character code, while the sensor and data operations are called with the
 *   internal value, so the driver has to speak both.
 *
 ****************************************************************************/

static uint32_t usbhost_uvc_v4l2fmt(uint32_t pixfmt)
{
  switch (pixfmt)
    {
      case IMGDATA_PIX_FMT_YUYV:
        return V4L2_PIX_FMT_YUYV;

      case IMGDATA_PIX_FMT_UYVY:
        return V4L2_PIX_FMT_UYVY;

      case IMGDATA_PIX_FMT_NV12:
        return V4L2_PIX_FMT_NV12;

      case IMGDATA_PIX_FMT_JPEG:
        return V4L2_PIX_FMT_JPEG;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: usbhost_uvc_buildenum
 *
 * Description:
 *   Build the arrays that back VIDIOC_ENUM_FMT, VIDIOC_ENUM_FRAMESIZES and
 *   VIDIOC_ENUM_FRAMEINTERVALS from what the descriptors reported.
 *
 *   The framework indexes these arrays directly and does not filter frame
 *   sizes by format, so the frame sizes advertised are those of the default
 *   format, with the camera's own default frame first.  That first entry is
 *   also what the framework uses as the initial format of the device.  Any
 *   format and frame size combination the camera reports can still be
 *   selected with VIDIOC_S_FMT, which is validated against the full set.
 *
 ****************************************************************************/

static int usbhost_uvc_buildenum(FAR struct uvc_state_s *priv)
{
  FAR struct uvc_formatinfo_s *deffmt;
  FAR struct uvc_frameinfo_s *defframe;
  int i;
  int n;

  usbhost_uvc_defaults(priv, &deffmt, &defframe);

  priv->fmtdescs = kmm_zalloc(priv->nformats * sizeof(struct v4l2_fmtdesc));
  if (priv->fmtdescs == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < priv->nformats; i++)
    {
      FAR struct v4l2_fmtdesc *fd = &priv->fmtdescs[i];

      fd->index       = i;
      fd->type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fd->pixelformat = usbhost_uvc_v4l2fmt(priv->formats[i].pixfmt);

      strlcpy((FAR char *)fd->description,
              priv->formats[i].subtype == UVC_VS_FORMAT_MJPEG ?
              "Motion-JPEG" : "Uncompressed", sizeof(fd->description));
    }

  priv->fmtdescs_num = priv->nformats;

  priv->frmsizes = kmm_zalloc(deffmt->nframes *
                              sizeof(struct v4l2_frmsizeenum));
  if (priv->frmsizes == NULL)
    {
      return -ENOMEM;
    }

  /* The default frame goes first: it becomes the device's initial format */

  n = 0;
  for (i = -1; i < deffmt->nframes; i++)
    {
      FAR struct uvc_frameinfo_s *frame;

      if (i < 0)
        {
          frame = defframe;
        }
      else
        {
          frame = &deffmt->frames[i];
          if (frame == defframe)
            {
              continue;
            }
        }

      priv->frmsizes[n].index           = n;
      priv->frmsizes[n].pixel_format    =
        usbhost_uvc_v4l2fmt(deffmt->pixfmt);
      priv->frmsizes[n].type            = V4L2_FRMSIZE_TYPE_DISCRETE;
      priv->frmsizes[n].discrete.width  = frame->width;
      priv->frmsizes[n].discrete.height = frame->height;
      n++;
    }

  priv->frmsizes_num = n;

  priv->frmintervals = kmm_zalloc(sizeof(struct v4l2_frmivalenum));
  if (priv->frmintervals == NULL)
    {
      return -ENOMEM;
    }

  priv->frmintervals[0].index                 = 0;
  priv->frmintervals[0].pixel_format          =
    usbhost_uvc_v4l2fmt(deffmt->pixfmt);
  priv->frmintervals[0].width                 = defframe->width;
  priv->frmintervals[0].height                = defframe->height;
  priv->frmintervals[0].type                  = V4L2_FRMIVAL_TYPE_DISCRETE;
  priv->frmintervals[0].discrete.numerator    = defframe->definterval;
  priv->frmintervals[0].discrete.denominator  = 10000000;
  priv->frmintervals_num                      = 1;

  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_register
 *
 * Description:
 *   Publish the camera as an ordinary NuttX video device.
 *
 ****************************************************************************/

static int usbhost_uvc_register(FAR struct uvc_state_s *priv)
{
  FAR struct imgsensor_s *sensor = &priv->sensor;
  int ret;

  ret = usbhost_uvc_devno_alloc(priv);
  if (ret < 0)
    {
      uerr("ERROR: No free video device number: %d\n", ret);
      return ret;
    }

  snprintf(priv->devpath, sizeof(priv->devpath), UVC_DEVNAME_FMT,
           priv->devno);

  priv->sensor.ops          = &g_uvc_sensor_ops;
  priv->sensor.fmtdescs     = priv->fmtdescs;
  priv->sensor.fmtdescs_num = priv->fmtdescs_num;
  priv->sensor.frmsizes     = priv->frmsizes;
  priv->sensor.frmsizes_num = priv->frmsizes_num;
  priv->sensor.frmintervals = priv->frmintervals;
  priv->sensor.frmintervals_num = priv->frmintervals_num;
  priv->data.ops            = &g_uvc_data_ops;

  ret = capture_register(priv->devpath, &priv->data, &sensor, 1);
  if (ret < 0)
    {
      uerr("ERROR: capture_register(%s) failed: %d\n", priv->devpath, ret);
      usbhost_uvc_devno_free(priv);
      return ret;
    }

  uinfo("Registered %s\n", priv->devpath);
  return OK;
}

/****************************************************************************
 * Name: usbhost_uvc_destroy
 *
 * Description:
 *   Release the class instance and everything it owns.  Runs either directly
 *   from disconnected(), or from the work queue when disconnected() was
 *   called from an interrupt handler, because the teardown below needs to
 *   take blocking locks.
 *
 ****************************************************************************/

static void usbhost_uvc_destroy(FAR void *arg)
{
  FAR struct uvc_state_s *priv = (FAR struct uvc_state_s *)arg;
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  uinfo("crefs: %d\n", priv->crefs);

  if (priv->devno >= 0)
    {
      capture_unregister(priv->devpath);
      usbhost_uvc_devno_free(priv);
    }

  usbhost_uvc_freereqs(priv);
  usbhost_uvc_freebuffers(priv);
  usbhost_uvc_freeformats(priv);

  if (priv->fmtdescs != NULL)
    {
      kmm_free(priv->fmtdescs);
    }

  if (priv->frmsizes != NULL)
    {
      kmm_free(priv->frmsizes);
    }

  if (priv->frmintervals != NULL)
    {
      kmm_free(priv->frmintervals);
    }

  nxsem_destroy(&priv->donesem);
  nxmutex_destroy(&priv->lock);

  DRVR_DISCONNECT(hport->drvr, hport);
  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  kmm_free(priv);
}

/****************************************************************************
 * Name: usbhost_uvc_create
 *
 * Description:
 *   Create a class instance and bind it to a USB host driver session.  This
 *   is the create() method of the registry entry; see CLASS_CREATE().
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_uvc_create(FAR struct usbhost_hubport_s *hport,
                     FAR const struct usbhost_id_s *id)
{
  FAR struct uvc_state_s *priv;

  DEBUGASSERT(hport != NULL);

  priv = kmm_zalloc(sizeof(struct uvc_state_s));
  if (priv == NULL)
    {
      uerr("ERROR: Failed to allocate class instance\n");
      return NULL;
    }

  priv->usbclass.hport        = hport;
  priv->usbclass.connect      = usbhost_uvc_connect;
  priv->usbclass.disconnected = usbhost_uvc_disconnected;
  priv->crefs                 = 1;
  priv->devno                 = -1;

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->donesem, 0, 0);

  return &priv->usbclass;
}

/****************************************************************************
 * Name: usbhost_uvc_connect
 *
 * Description:
 *   This is the connect() method of struct usbhost_class_s.  Enumeration
 *   calls it with the configuration descriptor so that the class can
 *   configure itself.  On failure the instance stays valid and the caller
 *   is responsible for calling disconnected() to release it.
 *
 ****************************************************************************/

static int usbhost_uvc_connect(FAR struct usbhost_class_s *usbclass,
                               FAR const uint8_t *configdesc, int desclen)
{
  FAR struct uvc_state_s *priv = (FAR struct uvc_state_s *)usbclass;
  FAR struct uvc_formatinfo_s *fmt;
  FAR struct uvc_frameinfo_s *frame;
  int ret;

  DEBUGASSERT(priv != NULL && configdesc != NULL &&
              desclen >= (int)sizeof(struct usb_cfgdesc_s));

  ret = usbhost_uvc_parse(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: Failed to parse the UVC descriptors: %d\n", ret);
      return ret;
    }

  usbhost_uvc_report(priv);

  ret = usbhost_uvc_allocbuffers(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Probe the default configuration now, without committing it.  This
   * settles the frame and payload sizes the driver will have to buffer, and
   * it establishes at bind time, rather than at the first capture, that the
   * camera and the driver can agree on something at all.
   */

  usbhost_uvc_defaults(priv, &fmt, &frame);

  ret = usbhost_uvc_negotiate(priv, fmt, frame, frame->definterval, false);
  if (ret < 0)
    {
      uerr("ERROR: The camera accepted no default configuration: %d\n",
           ret);
      return ret;
    }

  ret = usbhost_uvc_buildenum(priv);
  if (ret < 0)
    {
      return ret;
    }

  return usbhost_uvc_register(priv);
}

/****************************************************************************
 * Name: usbhost_uvc_disconnected
 *
 * Description:
 *   This is the disconnected() method of struct usbhost_class_s.  The device
 *   is gone; nothing may be sent to it from here on.
 *
 ****************************************************************************/

static int usbhost_uvc_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct uvc_state_s *priv = (FAR struct uvc_state_s *)usbclass;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Mark the device gone before anything else, so that a transfer completing
   * concurrently does not resubmit.
   */

  flags = enter_critical_section();
  priv->disconnected = true;
  priv->streaming    = false;
  priv->framebuf     = NULL;
  leave_critical_section(flags);

  /* Report the loss to whoever is capturing, so that a blocked reader is
   * released rather than waiting for a frame that will never arrive.
   */

  if (priv->capture_cb != NULL)
    {
      priv->capture_cb(1, 0, NULL, priv->capture_arg);
    }

  flags = enter_critical_section();

  uinfo("crefs: %d\n", priv->crefs);

  if (priv->crefs == 1)
    {
      /* Nobody else holds a reference, so the instance can go.  The teardown
       * blocks, so defer it if we are in an interrupt handler.
       */

      if (up_interrupt_context())
        {
          leave_critical_section(flags);
          DEBUGASSERT(work_available(&priv->work));
          work_queue(LPWORK, &priv->work, usbhost_uvc_destroy, priv, 0);
          return OK;
        }

      leave_critical_section(flags);
      usbhost_uvc_destroy(priv);
      return OK;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_uvc_initialize
 *
 * Description:
 *   Register the USB Video Class host driver, so that a camera connected
 *   later is bound to it.
 *
 * Returned Value:
 *   Zero on success, or a negated errno value.
 *
 ****************************************************************************/

int usbhost_uvc_initialize(void)
{
  return usbhost_registerclass(&g_uvc);
}
