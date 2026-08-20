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
