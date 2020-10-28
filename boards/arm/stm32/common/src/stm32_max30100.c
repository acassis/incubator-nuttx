/****************************************************************************
 * boards/arm/stm32/common/src/stm32_max30100.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max30100.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MAX30100)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX30100_I2C_PORTNO 1   /* On I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_max30100_initialize
 *
 * Description:
 *   Initialize and register the MPL115A Pressure Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/oxihr0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_max30100_initialize(int devno, int busno)
{
  FAR struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  sninfo("Initializing MAX30100!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  snprintf(devpath, 12, "/dev/oxihr%d", devno);
  ret = max30100_register(devpath, i2c);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX30100\n");
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_MAX30100 && CONFIG_STM32_I2C1 */
