=========================================
Using a USB Webcam on the NuttX Simulator
=========================================

.. contents::
   :depth: 2
   :local:

This page is the operating manual for the USB Video Class host driver on the
NuttX simulator: how to build it, what it needs from the Linux host, how to
choose a camera, and what to do when something does not work.  The design and
the reasoning behind it are in :doc:`usbhost_uvc`.

What this does
==============

A physical USB webcam plugged into the Linux host is driven by NuttX's own
UVC implementation, over raw USB, and appears inside the simulator as an
ordinary NuttX camera at ``/dev/video0``.  Linux's ``uvcvideo`` driver is not
involved in capturing: it is detached from that camera's interfaces for as
long as the simulator is using them, and re-attached afterwards.

This is not an emulated or synthetic camera.  Every descriptor is parsed,
every control request is sent, and every frame is reassembled by
``drivers/usbhost/usbhost_uvc.c``, the same code that would run on real
hardware with a real USB host controller.

Requirements
============

* Linux.  The simulator's USB host back end uses libusb, which on Linux
  reaches the device through usbfs.
* libusb 1.0 with its development headers::

    sudo apt install libusb-1.0-0-dev      # Debian, Ubuntu
    sudo dnf install libusb1-devel         # Fedora

* A USB Video Class webcam.  No vendor or model is assumed; any camera that
  offers MJPEG or an uncompressed YUY2, UYVY or NV12 format works.
* An X server, to see the picture.
* Permission to open the camera's usbfs node, described next.

Host permissions
================

The simulator runs as an ordinary user and opens the camera's usbfs node
directly, so that node has to be accessible.  By default it is owned by root
and a non-root process gets ``LIBUSB_ERROR_ACCESS``.

The repository ships udev rules that grant access to USB Video Class devices
only, matched by class rather than by any vendor or product ID::

    sudo install -m 0644 tools/sim/99-nuttx-uvc.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=change --subsystem-match=usb

The rules put webcam device nodes in the ``plugdev`` group.  Confirm that you
are in it, and check the result::

    id -nG | tr ' ' '\n' | grep plugdev
    ls -l /dev/bus/usb/003/004        # expect: crw-rw---- root plugdev

To undo them::

    sudo rm /etc/udev/rules.d/99-nuttx-uvc.rules
    sudo udevadm control --reload-rules

Running the simulator as root also works and needs no rules, but it is not
recommended: the X11 window then belongs to root as well.

Build and run
=============

::

    ./tools/configure.sh -l sim:uvcwebcam
    make -j$(nproc)
    ./nuttx

Then, at the NuttX shell::

    nsh> nxcamera
    nxcamera> input /dev/video0
    nxcamera> output /dev/fb0
    nxcamera> stream 640 480 30 YUYV

A live picture appears in the simulator's X11 window.  ``stop`` ends the
stream, ``quit`` leaves nxcamera, and ``poweroff`` shuts the simulator down.

Always leave with ``poweroff``, or with Ctrl-C, rather than killing the
process: both give the camera back to Linux.  See `Restoring the host
camera`_.

Choosing the camera
===================

The simulated host controller attaches to exactly one USB device, named by a
selector.  The ``sim:uvcwebcam`` configuration defaults to ``class=0e``, the
first USB Video Class device on the host, whichever it is.  Override it on
the command line::

    ./nuttx --sim-usb-device=class=0e      # first webcam, any vendor
    ./nuttx --sim-usb-device=046d:082d     # by vendor and product ID
    ./nuttx --sim-usb-device=3.34          # by bus and device address

``lsusb`` prints all three::

    $ lsusb
    Bus 003 Device 034: ID 046d:082d Logitech, Inc. HD Pro Webcam C920
        ^bus       ^address    ^VID  ^PID

Use ``bus.addr`` to pick one of several identical cameras.  Note that the
address changes when a device is replugged.

The build-time default is ``CONFIG_SIM_USB_DEVICE``; the command line
overrides it, so one build is not tied to one camera.

Formats
=======

The driver parses and can stream:

* **MJPEG**, passed through compressed.  It is never decoded inside the
  driver, so a JPEG frame reaches the application exactly as the camera sent
  it.
* **Uncompressed** YUY2, UYVY and NV12.

Uncompressed is the default.  This is a property of the *consumer*, not of the
driver: nxcamera converts frames for the framebuffer with libyuv, and the
NuttX libyuv package is not built with a JPEG decoder, so it cannot display
MJPEG.  Applications that store, forward, or decode JPEG themselves can select
MJPEG and get far better bandwidth: on a Logitech C920, 640x480 at 30fps needs
2688 bytes per microframe uncompressed and 640 as MJPEG.

To make MJPEG the default, set ``CONFIG_USBHOST_UVC_PREFER_MJPEG``.  Either
format can also be chosen at run time with ``VIDIOC_S_FMT``, or from
nxcamera::

    nxcamera> stream 640 480 30 MJPG

Any format the camera reports that the driver cannot name is skipped, and the
camera still works with the formats it can.

Configuration options
=====================

``CONFIG_USBHOST_UVC``
    Enable the driver.  Selects the video capture framework and asynchronous
    USB transfers.  Requires isochronous support, since most webcams offer no
    other streaming endpoint.

``CONFIG_USBHOST_UVC_NREQS`` (default 4)
    How many transfers are kept queued on the streaming endpoint.  An
    isochronous endpoint has no retry, so a gap between transfers loses
    packets outright and shows up as torn frames.  Raise this if frames tear
    under load; each one costs ``NPACKETS * (packet size)`` bytes.

``CONFIG_USBHOST_UVC_NPACKETS`` (default 32)
    Isochronous packets per transfer.  A high speed endpoint delivers one
    every 125us, so 32 covers 4ms of stream.

``CONFIG_USBHOST_UVC_PREFER_MJPEG`` (default off)
    Choose MJPEG rather than uncompressed when the camera offers both.

``CONFIG_SIM_USB_DEVICE``
    Default device selector, as described above.

``CONFIG_SIM_USB_BUFSIZE`` (default 4096)
    Descriptor buffer size.  Enumeration reads the whole configuration
    descriptor into one of these.  Webcams have large ones: 1047 bytes for a
    Microdia 0c45:672e, 3452 for a Logitech C920.

How the Linux driver detachment works
=====================================

This is the part that touches the host system, so it is worth being precise
about.

When the simulator selects a configuration on the camera, it claims every
interface of that configuration through libusb, with automatic kernel driver
detachment enabled.  Claiming an interface therefore detaches whatever kernel
driver held it, one interface at a time.  For a webcam that is ``uvcvideo``,
and for a camera with a microphone also ``snd-usb-audio``.

Consequences while the simulator is running:

* That camera's ``/dev/videoN`` nodes disappear from the host, and host
  applications cannot use it.
* **No kernel module is unloaded.**  ``uvcvideo`` stays loaded and every other
  camera on the machine keeps working normally.
* Nothing outside the selected device is touched.

When the simulator exits, the interfaces are released, which re-attaches the
kernel drivers, and the camera returns to normal Linux operation.  Interfaces
are released in descending order, because a driver that spans several
interfaces is offered the first one and claims the rest itself; releasing
upwards would offer it interface 0 while the rest were still taken and it
would refuse to bind.

Restoring the host camera
=========================

The cleanup runs on every exit path that the simulator can intercept:
``poweroff``, a normal exit, and the SIGINT, SIGTERM and SIGHUP signals, so
Ctrl-C is safe.

``kill -9`` is not interceptable.  If the simulator is killed that way, the
camera is left with no kernel driver: it will not appear in ``/dev/video*``
and Linux applications will not see it.  To recover, start the simulator once
more and leave it normally::

    ./nuttx --sim-usb-device=<same selector>
    nsh> poweroff

Unplugging and replugging the camera also restores it.  For a built-in camera
that cannot be unplugged, the simulator round trip above is the way back;
reloading the module (``sudo modprobe -r uvcvideo && sudo modprobe
uvcvideo``) also works but affects every camera on the machine, so it is a
last resort rather than the normal mechanism.

Troubleshooting
===============

**The host-side USB messages are not on the console.**  ``sim_libusb`` runs
outside NuttX and logs to the system log, as the simulator's other host code
does.  This is where permission errors, device selection and claim failures
are reported, so watch it while starting the simulator::

    journalctl -f | grep sim_libusb

For the NuttX side, build with ``CONFIG_DEBUG_USB_INFO``; the driver then logs
the camera it found, every format and frame size, and the negotiated stream.

**No /dev/video0 appears.**

* Check that the camera was found at all: ``sim_libusb: Selected USB device``
  should be in the system log.  If it is not, the selector matched nothing;
  check it against ``lsusb``.
* ``Access denied (insufficient permissions)`` means the udev rules are not in
  effect.  See `Host permissions`_.
* ``Cannot claim interface`` means something else still holds it.  Another
  copy of the simulator, or an application on the host holding the camera
  open, will do that.
* ``Configuration doesn't fit in buffer`` means the camera's descriptors are
  larger than ``CONFIG_SIM_USB_BUFSIZE``.  Raise it.
* ``Camera offers no format this driver knows`` means the camera offers only
  formats the driver cannot name, such as a vendor infrared format.  On a
  camera with several video functions this is normal for one of them and the
  others still work.

**The picture is torn or frames are missing.**  Raise
``CONFIG_USBHOST_UVC_NREQS``, or switch to MJPEG, which needs far less
bandwidth.  ``Frame exceeds ... bytes`` or ``Payload header claims ...`` in
the log point at a camera whose stream does not match what it negotiated.

**nxcamera says "No suitable Video Device found".**  It has no device set;
give it one with ``input /dev/video0``.

**The stream stops after a few seconds** with ``The stream stopped
answering``.  The camera stopped responding, typically because it was
unplugged or the bus was reset.  Restart the simulator.

**The host camera is missing after the simulator exits.**  See `Restoring the
host camera`_.

Known limitations
=================

* One camera at a time.  The simulated host controller has a single root port
  and attaches to one USB device.  The driver itself is not single-camera:
  it allocates a device number per instance and would publish
  ``/dev/video0``, ``/dev/video1`` and so on if the controller offered more
  ports.
* One streaming interface per camera.  A camera with several independent
  video streams is exposed as its first one.
* No camera controls.  Brightness, exposure, focus and the rest are not
  implemented; the camera runs with its own defaults.  The UVC request layer
  needed for them is in place.
* Still image capture and the interrupt status endpoint are not used.
* Bulk streaming treats one transfer as one payload, which suits the cameras
  that offer bulk endpoints but is not the full UVC bulk framing.
* MJPEG cannot be displayed by nxcamera, for the libyuv reason given under
  `Formats`_.  It reaches ``/dev/video0`` intact.
* A simulator killed with SIGKILL leaves the camera detached until it is
  replugged or the simulator is run once more.

Testing procedure
=================

To verify a working setup end to end:

1. ``lsusb`` and confirm the camera is listed.
2. ``ls -l /dev/bus/usb/<bus>/<addr>`` and confirm group ``plugdev``.
3. Build ``sim:uvcwebcam`` with ``CONFIG_DEBUG_USB_INFO`` enabled.
4. Start with ``journalctl -f | grep sim_libusb`` running alongside.
5. Confirm the driver's report in the NuttX log: the UVC release, the
   streaming endpoint, the alternate settings and the formats.
6. Confirm ``/dev/video0`` exists with ``ls /dev``.
7. Stream with nxcamera and confirm a live picture in the X11 window.
8. ``stop``, then ``stream`` again at a different resolution, and confirm the
   driver renegotiates and picks a different alternate setting.
9. ``poweroff``, then confirm on the host that ``/dev/video*`` and the
   ``uvcvideo`` binding are back::

     lsusb -t | grep -i video
