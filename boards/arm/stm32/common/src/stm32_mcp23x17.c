/****************************************************************************
 * boards/arm/stm32/common/src/stm32_mcp23x17.c
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
#include <nuttx/arch.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/mcp23x17.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mcp23x17_initialize
 *
 * Description:
 *   Initialize and register the BMP180 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mcp23x17_initialize(int busno)
{
  FAR struct i2c_master_s *i2c;
  FAR struct ioexpander_dev_s *ioe;
  FAR struct mcp23x17_config_s *config;
  int i;

  sninfo("Initializing MCP23X17!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Create a empty config */

  config = (FAR struct mcp23x17_config_s *)
    kmm_zalloc(sizeof(struct mcp23x17_config_s));
  if (!config)
    {
      return -ENOMEM;
    }

  config->frequency = 100000;
  config->address = 0x20;

  /* Get an instance of the simulated I/O expander */

  ioe = mcp23x17_initialize(i2c, config);
  if (ioe == NULL)
    {
      _err("ERROR: mcp23x17_initialize failed\n");
      return -ENOMEM;
    }

  /* Setup the first 8 pins as Input */

  for (i = 0; i < 8; i++)
    {
      /* Set pin as non-inverted, input pin */

      IOEXP_SETDIRECTION(ioe, i, IOEXPANDER_DIRECTION_IN);
      IOEXP_SETOPTION(ioe, i, IOEXPANDER_OPTION_INVERT,
                      (FAR void *)IOEXPANDER_VAL_NORMAL);
      IOEXP_SETOPTION(ioe, i, IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
      gpio_lower_half(ioe, i, GPIO_INPUT_PIN, i);
    }

  /* Setup the last 8 pins as Output */

  for (i = 8; i < 16; i++)
    {
      /* Pin 1: an non-inverted, output pin */

      IOEXP_SETDIRECTION(ioe, i, IOEXPANDER_DIRECTION_OUT);
      IOEXP_SETOPTION(ioe, i, IOEXPANDER_OPTION_INVERT,
                      (FAR void *)IOEXPANDER_VAL_NORMAL);
      IOEXP_SETOPTION(ioe, i, IOEXPANDER_OPTION_INTCFG,
                      (FAR void *)IOEXPANDER_VAL_DISABLE);
      gpio_lower_half(ioe, i, GPIO_OUTPUT_PIN, i);
    }

  return OK;
}

