========================================
USB Video Class (UVC) Host-Side Driver
========================================

.. contents::
   :depth: 2
   :local:

Overview
========

``drivers/usbhost/usbhost_uvc.c`` is a NuttX USB *host* class driver for USB
Video Class (UVC) devices -- ordinary USB webcams.  It parses the UVC
descriptor hierarchy, performs the UVC ``PROBE``/``COMMIT`` streaming
negotiation, receives and re-assembles UVC payloads, and publishes the result
through the standard NuttX video capture framework, so that a camera behaves
like any other NuttX camera device (``/dev/video0``) and can be consumed by
``nxcamera`` without knowing that a USB device is involved.

The driver is host-controller agnostic: it uses only the interfaces published
by ``include/nuttx/usb/usbhost.h``.  On the NuttX simulator the underlying host
controller driver is ``arch/sim/src/sim/sim_usbhost.c``, whose host-side
back end (``arch/sim/src/sim/posix/sim_libusb.c``) reaches a *physical* USB
device on the Linux host through libusb.  This makes it possible to run a real,
generic webcam through the real NuttX UVC protocol implementation while
developing on a PC.

Data flow on the simulator::

    physical webcam
        |  USB
    Linux kernel usbfs  (the kernel uvcvideo driver is detached per interface)
        |  libusb
    arch/sim/src/sim/posix/sim_libusb.c      <- thin USB transport only
        |
    arch/sim/src/sim/sim_usbhost.c           <- NuttX host controller driver
        |  struct usbhost_driver_s
    drivers/usbhost/usbhost_uvc.c            <- all UVC protocol handling
        |  imgsensor_s / imgdata_s
    drivers/video/v4l2_cap.c                 <- /dev/video0
        |  V4L2 ioctls
    apps/system/nxcamera
        |  framebuffer
    arch/sim/src/sim/sim_framebuffer.c -> X11 window

The split is deliberate: the Linux side owns nothing but physical USB access
(device selection, kernel driver detach, interface claim, transfer submission).
Every byte of UVC protocol knowledge lives in ``usbhost_uvc.c``, which is
portable to any real NuttX target with a USB host controller.

Architecture investigation
==========================

This section records what already existed in NuttX before this driver was
written, because the design below is a direct consequence of it.

Existing UVC implementation (device side)
-----------------------------------------

NuttX already contains a UVC *gadget* (device-side) implementation:

* ``include/nuttx/usb/uvc.h`` -- UVC protocol constants: class-specific
  descriptor types (``UVC_CS_INTERFACE`` ...), video interface subclasses
  (``UVC_SC_VIDEOCONTROL``, ``UVC_SC_VIDEOSTREAMING``,
  ``UVC_SC_VIDEO_INTERFACE_COLLECTION``), VideoControl and VideoStreaming
  descriptor subtypes, terminal types, the UVC request codes
  (``UVC_SET_CUR``, ``UVC_GET_CUR``, ``UVC_GET_MIN``, ``UVC_GET_MAX``,
  ``UVC_GET_RES``, ``UVC_GET_DEF``, ``UVC_GET_LEN``, ``UVC_GET_INFO``), the
  VideoStreaming interface control selectors (``UVC_VS_PROBE_CONTROL``,
  ``UVC_VS_COMMIT_CONTROL``), the payload header bit definitions
  (``UVC_STREAM_FID``, ``UVC_STREAM_EOF`` ...), the YUY2 format GUID and
  ``struct uvc_streaming_control_s`` (the 26-byte probe/commit payload).
* ``drivers/usbdev/uvc.c`` -- the gadget itself: it answers probe/commit,
  builds descriptors and streams frames out of a bulk IN endpoint.

The header is protocol-side-neutral, so the host driver **reuses it as-is**
rather than duplicating the constants.  It is extended (additively) with the
packed descriptor layouts a *parser* needs and which the gadget, which only
ever emits hand-built byte arrays, does not define:
``struct uvc_vc_header_desc_s``, ``struct uvc_vs_input_header_desc_s``,
``struct uvc_format_uncompressed_desc_s``, ``struct uvc_frame_desc_s``,
``struct uvc_format_mjpeg_desc_s`` and the MJPEG/NV12 format GUIDs.

The gadget's *logic* is deliberately not reused: a device answers probe/commit
requests, a host issues them; the responsibilities are inverted.

Existing USB host framework
---------------------------

* Class drivers implement ``struct usbhost_class_s`` (``connect()`` /
  ``disconnected()``) and publish a ``struct usbhost_registry_s`` through
  ``usbhost_registerclass()``.  A registry entry may carry several
  ``struct usbhost_id_s`` matches (``nids``).
* ``usbhost_enumerate()`` (``drivers/usbhost/usbhost_enumerate.c``) reads the
  device and configuration descriptors, issues ``SET_CONFIGURATION`` and then
  binds a class:

  - if ``bDeviceClass == 0`` the configuration is walked and each interface is
    offered to the registry using the *interface* class/subclass/protocol;
  - otherwise, with ``CONFIG_USBHOST_COMPOSITE``, ``usbhost_composite()``
    splits the device into functions along Interface Association Descriptors
    and offers each function to the registry using the *IAD*
    class/subclass/protocol.

  Both paths matter for webcams: a single-function camera may report
  ``bDeviceClass == 0``, but any camera with a VideoControl + VideoStreaming
  pair (that is, essentially all of them) is required by the UVC specification
  to use an IAD and therefore reports ``bDeviceClass == 0xef`` (Miscellaneous /
  Interface Association).  The driver registers for both shapes.
* Transfers: ``DRVR_CTRLIN``/``DRVR_CTRLOUT`` (synchronous control),
  ``DRVR_TRANSFER`` (synchronous bulk/interrupt) and, with
  ``CONFIG_USBHOST_ASYNCH``, ``DRVR_ASYNCH``.  Endpoints are obtained with
  ``DRVR_EPALLOC`` from a ``struct usbhost_epdesc_s``, which already carries
  ``xfrtype``, so isochronous endpoints can be described, and released with
  ``DRVR_EPFREE``.
* Lifetime: the host controller driver calls ``CLASS_DISCONNECTED()`` on
  unplug; class drivers conventionally split interrupt-context work from
  blocking work using the work queue (see ``usbhost_storage.c``,
  ``usbhost_cdcacm.c``).

**Gap found:** the framework had no way for a class driver to learn *isochronous
packet boundaries*.  ``DRVR_TRANSFER``/``DRVR_ASYNCH`` return a single byte
count, and the only in-tree isochronous-capable host controller driver
(``usbhost_xhci.c``) follows that same flat model.  UVC cannot work that way:
every isochronous packet carries its own payload header, packets are routinely
short or empty, and a frame boundary is signalled by a bit inside one of those
headers.  Without per-packet lengths a receiver cannot find the next header.
See `Isochronous transfers`_ for how this was resolved.

Existing simulator USB support
------------------------------

The simulator already had a working USB host path, and it is reused rather than
replaced:

* ``arch/sim/src/sim/sim_usbhost.c`` -- a real ``struct usbhost_driver_s`` /
  ``struct usbhost_connection_s`` implementation with a root hub port, an
  enumeration task, and a work-queue poll (``CONFIG_SIM_LOOP_INTERVAL``) that
  drains completed transfers and detects connect/disconnect.
* ``arch/sim/src/sim/posix/sim_libusb.c`` -- the Linux back end
  (``CONFIG_SIM_LIBUSB``).  It already opened a device selected by
  ``CONFIG_SIM_USB_VID``/``CONFIG_SIM_USB_PID``, ran a libusb event thread, and
  submitted control, bulk, interrupt and isochronous transfers.
* ``arch/sim/src/sim/sim_usbhost.h`` -- the (simulator-internal) contract
  between the two: ``struct host_usb_ctrlreq_s``, ``struct host_usb_datareq_s``
  and the ``host_usbhost_*()`` entry points.

Reusing this was the single most important design decision: no new transport
mechanism was introduced.  The gaps that had to be closed in it are listed
under `Changes outside usbhost_uvc.c`_.

Existing camera framework
-------------------------

``drivers/video/v4l2_cap.c`` (``CONFIG_VIDEO_STREAM``) implements the
``/dev/videoN`` V4L2 capture character driver.  A camera is registered with::

    int capture_register(FAR const char *devpath,
                         FAR struct imgdata_s *data,
                         FAR struct imgsensor_s **sensors,
                         size_t sensor_num);

and is described by two small vtables:

* ``struct imgsensor_s`` (``include/nuttx/video/imgsensor.h``) -- the sensor:
  ``is_available()``, ``init()``, ``get_driver_name()``,
  ``validate_frame_setting()``, ``start_capture()``, ``stop_capture()``, plus
  the ``fmtdescs`` / ``frmsizes`` / ``frmintervals`` arrays that back
  ``VIDIOC_ENUM_FMT``, ``VIDIOC_ENUM_FRAMESIZES`` and
  ``VIDIOC_ENUM_FRAMEINTERVALS``.
* ``struct imgdata_s`` (``include/nuttx/video/imgdata.h``) -- the data path:
  ``set_buf()`` hands the driver the next capture buffer, ``start_capture()``
  passes a completion callback, and the driver reports each finished frame with
  ``callback(result, size, timestamp, arg)``.

``arch/sim/src/sim/sim_camera.c`` is the reference consumer of this API (it
bridges to the host's V4L2 devices, which this project must *not* use).  Its
registration and buffer handling pattern is what ``usbhost_uvc.c`` follows.

The UVC formats map onto existing pixel formats with no new definitions:
``IMGDATA_PIX_FMT_YUYV`` for YUY2 and ``IMGDATA_PIX_FMT_JPEG`` for MJPEG.

Existing X11 emulation
----------------------

The simulator's X11 support (``arch/sim/src/sim/sim_framebuffer.c`` and
``arch/sim/src/sim/posix/sim_x11framebuffer.c``, ``CONFIG_SIM_X11FB``) is a
*framebuffer* driver, not a camera abstraction.  ``nxcamera`` already dequeues
V4L2 buffers, converts them into the framebuffer pixel format with libyuv
(``CONFIG_LIBYUV``) and issues ``FBIOPAN_DISPLAY``.

Therefore no UVC code touches X11 at all, and the required dependency direction
falls out for free::

    usbhost_uvc.c -> v4l2_cap.c -> nxcamera -> framebuffer -> X11

The pre-existing ``sim:nxcamera`` board configuration already wires up
``CONFIG_SIM_X11FB``, ``CONFIG_VIDEO_STREAM``, ``CONFIG_LIBYUV`` and
``CONFIG_SYSTEM_NXCAMERA``; the new ``sim:uvcwebcam`` configuration is that
configuration plus the USB host stack.

Design decisions
================

Where the UVC protocol lives
----------------------------

All of it lives in ``drivers/usbhost/usbhost_uvc.c``: descriptor parsing,
format/frame enumeration, ``GET_*``/``SET_CUR`` requests, probe/commit
negotiation, alternate-setting selection, payload header handling and frame
assembly.  ``sim_libusb.c`` never inspects a video descriptor and has no
notion of a camera; it moves USB transfers.

Format selection
----------------

Both MJPEG and uncompressed YUY2 are parsed, enumerated to user space and can
be committed.  MJPEG frames are passed through the framework **compressed** --
they are never decoded and re-encoded inside the driver.

The default is YUY2 rather than MJPEG.  The reason is a property of the
*consumer*, not of the driver: ``nxcamera`` converts with libyuv, and libyuv
only decodes MJPEG when it is built against libjpeg, which the NuttX libyuv
package does not do.  YUY2 therefore renders in the X11 window out of the box,
while MJPEG reaches ``/dev/video0`` intact for applications that can handle it.
The default is a configuration choice (``CONFIG_USBHOST_UVC_PREFER_MJPEG``),
not a driver limitation.

Isochronous transfers
---------------------

The webcams this driver targets stream over isochronous endpoints; many offer
no bulk alternative at all.  Two problems had to be solved.

*Packet boundaries.*  As noted above, the flat byte-count transfer API cannot
express them.  Submitting one transfer per isochronous packet would preserve
boundaries within the existing API, but a high-speed endpoint produces 8000
packets per second, which neither the simulator's completion FIFO nor its
work-queue polling can sustain, and the resulting gaps between resubmissions
lose packets and tear frames.

The resolution is a small **additive** extension to
``include/nuttx/usb/usbhost.h``: an optional ``isocasynch()`` method (with a
``DRVR_ISOCASYNCH()`` accessor and a ``struct usbhost_isoc_s`` descriptor
carrying the packet count, the packet stride and an out array of per-packet
lengths), compiled only when isochronous support is enabled.  Host controller
drivers that do not implement it leave the pointer ``NULL`` and the accessor
returns ``-ENOSYS``; no existing driver, class driver or configuration changes
behaviour.  This mirrors what every real UVC host implementation needs (Linux
calls the same thing ``iso_frame_desc``).

*Continuity.*  The driver keeps several isochronous requests in flight and
resubmits each one from its completion handler, so the endpoint is never idle
between transfers.

Bulk streaming endpoints are supported through the ordinary ``DRVR_ASYNCH``
path, since a bulk UVC payload is self-delimiting (a short packet ends it).

Threading
---------

The driver adds no thread of its own.  Completions arrive on the host
controller driver's existing context and are handled in the callback; the only
deferred work is destruction on disconnect, which needs a blocking lock and is
therefore pushed to the low-priority work queue -- the same convention used by
``usbhost_storage.c`` and ``usbhost_cdcacm.c``.

Buffering
---------

Frames are assembled directly into the buffer that the video framework hands
the driver through ``IMGDATA_SET_BUF``; the driver keeps no frame queue of its
own.  The only driver-owned memory is the small pool of isochronous transfer
buffers, sized from the negotiated ``dwMaxPayloadTransferSize``.

Changes outside usbhost_uvc.c
=============================

Every modification outside the driver itself is listed here with its
justification, in keeping with the minimum-change principle.

``include/nuttx/usb/uvc.h``
    Added the packed UVC descriptor layouts and format GUIDs needed to *parse*
    descriptors.  Purely additive; the gadget is unaffected.  The alternative
    -- a second, private copy of the UVC constants in the host driver -- was
    rejected.

``include/nuttx/usb/usbhost.h``
    Added the optional isochronous transfer method described above.  Required
    because UVC cannot be implemented over an API that hides packet
    boundaries.  Additive and guarded by ``CONFIG_USBHOST_ISOC_DISABLE``.

``drivers/usbhost/usbhost_composite.c``
    Two fixes, both for composite devices in general rather than for UVC in
    particular.

    The per-function configuration descriptor handed to a member's
    ``connect()`` previously skipped interfaces whose alternate setting 0
    declares zero endpoints, and stopped after the first alternate setting's
    endpoints, which silently dropped both the UVC VideoStreaming
    class-specific descriptors (they hang off alternate setting 0, which has
    no endpoint) and every isochronous alternate setting.  All descriptors
    belonging to the interface are now copied, and the fixed-size 99 byte
    scratch buffer was replaced by one sized from the configuration
    descriptor itself, which is by construction an upper bound.

    A member returning an error from ``connect()`` used to fail the whole
    device, which then re-enumerated and failed again indefinitely.  A webcam
    that pairs a colour camera with an infrared one presents two video
    functions where the infrared one offers only a vendor pixel format, and
    refusing it should not cost the user the colour camera.  The failed
    member is now released and the rest kept, as already happens for a member
    with no registered driver at all.

``arch/sim/src/sim/sim_usbhost.c``, ``arch/sim/src/sim/sim_usbhost.h``
    Enlarged the descriptor buffer returned by ``DRVR_ALLOC`` (a UVC
    configuration descriptor is around 1 KiB; the previous 256 bytes made
    enumeration fail with ``-E2BIG``), implemented the isochronous method, and
    routed isochronous completions through the existing completion FIFO
    instead of calling back from the libusb event thread.

``arch/sim/src/sim/posix/sim_libusb.c``
    Device selection by command line rather than by compiled-in VID/PID;
    ``SET_INTERFACE`` support, mandatory for isochronous alternate settings;
    control transfers with a host-to-device data stage, which the UVC probe
    and commit requests are and which were previously dropped; claim and
    release of every interface of the selected configuration rather than
    interface 0 only; release of the device on every interceptable exit path,
    so the host camera comes back; and a fix for a request being both queued
    as a completion and freed when libusb refused it.

``arch/sim/Kconfig``, ``drivers/usbhost/Kconfig``, ``boards/sim/...``
    Configuration for the above.

``apps``
    None.  ``nxcamera`` is used unmodified, and no application-side change
    turned out to be necessary.

Build instructions, host permission requirements, webcam selection,
configuration options and troubleshooting are in
:doc:`usbhost_uvc_usage`.

Verified behaviour
==================

Measured on two cameras, a Logitech C920 (046d:082d) and the Microdia
0c45:672e built into a laptop, both reporting UVC 1.00:

* Both enumerate, and their descriptors are parsed into the formats and frame
  sizes they advertise: 19 uncompressed and 17 MJPEG frame sizes for the
  C920, 7 and 5 for the Microdia.  Formats neither camera describes in terms
  this driver knows are skipped without affecting the rest.
* The Microdia's second, infrared video function offers only an unsupported
  format.  It is refused and the colour camera still works.
* Uncompressed 640x480 at 30fps negotiates a 614400 byte frame on both.  The
  C920 asks 2688 bytes per microframe and the driver selects alternate
  setting 10; as MJPEG the same picture needs 640 bytes and it selects
  alternate setting 4, reserving a quarter of the bus bandwidth.
* nxcamera streams from ``/dev/video0`` and a live picture appears in the X11
  window, with no dropped or spoiled frames logged over 25 seconds.
* Stopping and restarting at another resolution renegotiates and moves to a
  different alternate setting.
* With no camera attached, or with a selector matching nothing, the simulator
  runs normally and no video device appears.
* An unsupported frame size or pixel format is refused with ``-EINVAL``.
* Killing the simulator mid-stream with SIGTERM or SIGINT returns the camera
  to Linux; with SIGKILL it does not, and the documented recovery is to run
  the simulator once more and exit it normally.
* A forced USB port reset under a running stream is detected: the driver
  abandons the stream and releases the application rather than retrying.
