/****************************************************************************
 * drivers/sensors/max30100.c
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/max30100.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MAX30100)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MAX30100_I2C_FREQUENCY
#  define CONFIG_MAX30100_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct max30100_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address (could be hardcoded) */
  uint8_t sensor_mode;          /* Heart Rate only or SpO2 */
  uint8_t spo2_sample_rate;     /* SpO2 Sample Rate */
  uint8_t led_pw;               /* LED RED/IR Pulse Width */
  uint8_t led_curr;             /* LED RED/IR Current */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     max30100_write_reg(FAR struct max30100_dev_s *priv,
                                  FAR const uint8_t *buffer,
                                  const uint8_t value);
static int     max30100_read_reg(FAR struct max30100_dev_s *priv,
                                 FAR uint8_t *buffer,
                                 FAR uint8_t *value);
static int     max30100_readtemp(FAR struct max30100_dev_s *priv,
                                 FAR b16_t *temp);
static uint8_t max30100_read_partid(FAR struct max30100_dev_s *priv);

static int     max30100_writeconf(FAR struct max30100_dev_s *priv,
                                  uint8_t conf);

/* Character driver methods */

static int     max30100_open(FAR struct file *filep);
static int     max30100_close(FAR struct file *filep);
static ssize_t max30100_read(FAR struct file *filep,
                             FAR char *buffer,
                             size_t buflen);
static ssize_t max30100_write(FAR struct file *filep,
                              FAR const char *buffer,
                              size_t buflen);
static int     max30100_ioctl(FAR struct file *filep,
                              int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_max30100fops =
{
  max30100_open,
  max30100_close,
  max30100_read,
  max30100_write,
  NULL,
  max30100_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max30100_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int max30100_write_reg(FAR struct max30100_dev_s *priv,
                              FAR const uint8_t *reg, const uint8_t value)
{
  struct i2c_msg_s msg[2] =
  {
    {
      .frequency = CONFIG_MAX30100_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = I2C_M_NOSTOP,
      .buffer    = (FAR void *)reg,
      .length    = 1
    },
    {
      .frequency = CONFIG_MAX30100_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = 0,
      .buffer    = (FAR void *)&value,
      .length    = 1
    }
  };
  int ret;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: max30100_read_reg
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int max30100_read_reg(FAR struct max30100_dev_s *priv,
                             FAR uint8_t *reg, FAR uint8_t *value)
{
  struct i2c_msg_s msg[2] =
  {
    {
      .frequency = CONFIG_MAX30100_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = 0,
      .buffer    = (FAR void *)reg,
      .length    = 1
    },
    {
      .frequency = CONFIG_MAX30100_I2C_FREQUENCY,
      .addr      = priv->addr,
      .flags     = I2C_M_READ,
      .buffer    = (FAR void *)value,
      .length    = 1
    }
  };
  int ret;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: max30100_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling (MAX30100_TEMP_REG)
 *
 ****************************************************************************/

static int max30100_readtemp(FAR struct max30100_dev_s *priv, FAR b16_t *temp)
{
}

/****************************************************************************
 * Name: max30100_read_partid
 *
 * Description:
 *   Read the 8-bit MAX30100 PART ID register
 *
 ****************************************************************************/

static uint8_t max30100_read_partid(FAR struct max30100_dev_s *priv)
{
  int timeout = 0;
  int ret;
  uint8_t buffer;
  uint8_t value;

  /* Set PARTID Register */

  buffer = MAX30100_PART_ID;

  for (;;)
    {
      ret = max30100_read_reg(priv, &buffer, &value);
      if (value == MAX30100_ID)
        {
          break;
        }

      if (++timeout > 1000)
        {
          snerr("Fail to read PART ID\n");
	  break;
        }
    }

  return value;
}

/****************************************************************************
 * Name: max30100_writeconf
 *
 * Description:
 *   Write to a 8-bit MAX30100 configuration register.
 *
 ****************************************************************************/

static int max30100_writeconf(FAR struct max30100_dev_s *priv, uint8_t conf)
{
  return 0;
}

/****************************************************************************
 * Name: max30100_open
 *
 * Description:
 *   This function is called whenever the LM-75 device is opened.
 *
 ****************************************************************************/

static int max30100_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max30100_close
 *
 * Description:
 *   This routine is called when the LM-75 device is closed.
 *
 ****************************************************************************/

static int max30100_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: max30100_read
 ****************************************************************************/

static ssize_t max30100_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
#if 0
  FAR struct inode      *inode = filep->f_inode;
  FAR struct max30100_dev_s *priv   = inode->i_private;
  FAR b16_t             *ptr;
  ssize_t                nsamples;
  int                    i;
  int                    ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = max30100_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: max30100_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
#endif
  return 0;
}

/****************************************************************************
 * Name: max30100_write
 ****************************************************************************/

static ssize_t max30100_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: max30100_ioctl
 ****************************************************************************/

static int max30100_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
#if 0
  FAR struct inode      *inode = filep->f_inode;
  FAR struct max30100_dev_s *priv  = inode->i_private;
  int                    ret   = OK;

  switch (cmd)
    {
      /* Read from the configuration register. Arg: uint8_t* pointer */

      case SNIOC_READCONF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = max30100_readconf(priv, ptr);
          sninfo("conf: %02x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write to the configuration register. Arg:  uint8_t value */

      case SNIOC_WRITECONF:
        ret = max30100_writeconf(priv, (uint8_t)arg);
        sninfo("conf: %02x ret: %d\n", *(FAR uint8_t *)arg, ret);
        break;

      /* Shutdown the MAX30100, Arg: None */

      case SNIOC_SHUTDOWN:
        {
          uint8_t conf;
          ret = max30100_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = max30100_writeconf(priv, conf | MAX30100_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf | MAX30100_CONF_SHUTDOWN, ret);
        }
        break;

      /* Powerup the MAX30100, Arg: None */

      case SNIOC_POWERUP:
        {
          uint8_t conf;
          ret = max30100_readconf(priv, &conf);
          if (ret == OK)
            {
              ret = max30100_writeconf(priv, conf & ~MAX30100_CONF_SHUTDOWN);
            }

          sninfo("conf: %02x ret: %d\n", conf & ~MAX30100_CONF_SHUTDOWN, ret);
        }
        break;

      /* Report samples in Fahrenheit */

      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report Samples in Centigrade */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Centigrade\n");
        break;

      /* Read THYS temperature register.  Arg: b16_t* pointer */

      case SNIOC_READTHYS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = max30100_readb16(priv, MAX30100_THYS_REG, ptr);
          sninfo("THYS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write THYS temperature register. Arg: b16_t value */

      case SNIOC_WRITETHYS:
        ret = max30100_writeb16(priv, MAX30100_THYS_REG, (b16_t)arg);
        sninfo("THYS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      /* Read TOS (Over-temp Shutdown Threshold) Register. Arg: b16_t* pointer */

      case SNIOC_READTOS:
        {
          FAR b16_t *ptr = (FAR b16_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          ret = max30100_readb16(priv, MAX30100_TOS_REG, ptr);
          sninfo("TOS: %08x ret: %d\n", *ptr, ret);
        }
        break;

      /* Write TOS (Over-temp Shutdown Threshold) Register. Arg: b16_t value */

      case SNIOC_WRITETOS:
        ret = max30100_writeb16(priv, MAX30100_TOS_REG, (b16_t)arg);
        sninfo("TOS: %08x ret: %d\n", (b16_t)arg, ret);
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }
  return ret;
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max30100_register
 *
 * Description:
 *   Register the MAX30100 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/oxihr0"
 *   i2c - An instance of the I2C interface to use to communicate with MAX30100
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max30100_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct max30100_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the MAX30100 device structure */

  priv = (FAR struct max30100_dev_s *)
          kmm_malloc(sizeof(struct max30100_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c              = i2c;
  priv->addr             = 0x57;
  priv->sensor_mode      = MODE_SPO2;
  priv->spo2_sample_rate = SPO2_400SPS;  /* 400 samples per sec */
  priv->led_pw           = LED_PW_400US; /* LED Pulse Width = 400uS */
  priv->led_curr         = LED_11MA;     /* LED Current = 11mA */

  /* Probe the device to confirm is exist */

  ret = max30100_read_partid(priv);
  if (ret != MAX30100_ID)
    {
      snerr("ERROR: Wrong MAX30100 PART ID, expected: 0x%02x read: 0x%02x\n",
            MAX30100_ID, ret);
      kmm_free(priv);
      return -ENODEV;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_max30100fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_I2C && CONFIG_MAX30100_I2C */
