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
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/uvc.h>
#include <nuttx/video/imgdata.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A camera is not required to describe itself sanely.  These bound what we
 * are willing to allocate on its say-so.
 */

#define UVC_MAX_FORMATS         8
#define UVC_MAX_FRAMES          32
#define UVC_MAX_ALTSETTINGS     16

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
        priv->curinterval ? 10000000ul / priv->curinterval : 0,
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
                frame->mininterval ? 10000000ul / frame->mininterval : 0,
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

  usbhost_uvc_freebuffers(priv);
  usbhost_uvc_freeformats(priv);
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

  nxmutex_init(&priv->lock);

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

  return OK;
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
