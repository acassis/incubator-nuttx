/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_at25.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "sam_spi.h"
#include "sam4e-ek.h"

#ifdef HAVE_AT25

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at25_automount
 *
 * Description:
 *   Initialize, configure, and mount the AT25 serial FLASH.  The FLASH will
 *   be mounted at /dev/at25.
 *
 ****************************************************************************/

int sam_at25_automount(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the SPI port driver */

      spi = sam_spibus_initialize(FLASH_CSNUM);
      if (!spi)
        {
          ferr("ERROR: Failed to initialize SPI port %d\n", FLASH_CSNUM);
          return -ENODEV;
        }

      /* Now bind the SPI interface to the AT25 SPI FLASH driver */

      mtd = at25_initialize(spi);
      if (!mtd)
        {
          ferr("ERROR: Failed to bind SPI port to the AT25 FLASH driver\n");
          return -ENODEV;
        }

#if defined(CONFIG_SAM4EEK_AT25_FTL)
      /* And finally, use the FTL layer to wrap the MTD driver as a block
       * driver at /dev/mtdblockN, where N=minor device number.
       */

      ret = ftl_initialize(minor, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAM4EEK_AT25_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at25 */

      ret = nx_mount(NULL, "/mnt/at25", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT25 */
