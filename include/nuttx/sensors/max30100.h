/****************************************************************************
 * include/nuttx/sensors/max30100.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MAX30100_H
#define __INCLUDE_NUTTX_SENSORS_MAX30100_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MAX30100)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_SENSOR_MAX30100 - Enables support for the MAX30100 driver
 */

#define MAX30100_ADDR 0x48

/* MAX30100 Register Definitions ********************************************/

/* MAX30100 Registers addresses */

#define MAX30100_INT_STAT          0x00     /* Interrupt Status Register */
#define MAX30100_INT_ENBL          0x01     /* Interrupt Enable Register */
#define MAX30100_FIFO_WR_PTR       0x02     /* FIFO Write Pointer Reg */
#define MAX30100_FIFO_OVR_FLOW_CNT 0x03     /* FIFO Overflow Counter Reg */
#define MAX30100_FIFO_RD_PTR       0x04     /* FIFO Read Pointer Reg */
#define MAX30100_FIFO_DATA_REG     0x05     /* FIFO Data Reg */
#define MAX30100_MODE_CONFIG       0x06     /* FIFO Mode Configuration Reg */
#define MAX30100_SPO2_CONFIG       0x07     /* FIFO SPO2 Configuration Reg */
                                            /* Position 0x08 Reserved */
#define MAX30100_LED_CONFIG        0x09     /* LED Configuration Reg */
                                            /* Posit. 0x0a-0x15 Reserved */
#define MAX30100_TEMP_INTEGER      0x16     /* Temperature Integer Reg */
#define MAX30100_TEMP_FRACTION     0x17     /* Temperature Fraction Reg */
                                            /* Position up to 0xfd Reserved */
#define MAX30100_REV_ID            0xfe     /* Revision ID Reg */
#define MAX30100_PART_ID           0xff     /* Part Number ID Reg */

/* STATUS - Configuration Register Bit Definitions */

/* Interrupt Status */

#define STATUS_PWR_RDY             (1 << 0) /* Bit 0: Power Ready */
#define STATUS_SPO2_RDY            (1 << 4) /* Bit 4: SPO2 Ready */
#define STATUS_HR_RDY              (1 << 5) /* Bit 5: Heart Rate Ready */
#define STATUS_TEMP_RDY            (1 << 6) /* Bit 6: Temperature Ready */
#define STATUS_A_FULL              (1 << 7) /* Bit 7: FIFO Almost Full */

/* Interrupt Enable */

#define INT_ENB_SPO2_RDY           (1 << 4) /* Bit 4: Enable SPO2_RDY */
#define INT_ENB_HR_RDY             (1 << 5) /* Bit 5: Enable HR_RDY */
#define INT_ENB_TEMP_RDY           (1 << 6) /* Bit 6: Enable TEMP_RDY */
#define INT_ENB_A_FULL             (1 << 7) /* Bit 7: Enable A_FULL */

/* FIFO - Configuration Register Bit Definitions */

#define FIFO_WR_PTR_SHIFT          0
#define FIFO_WR_PTR_MASK           (0xf << FIFO_WR_PTR_SHIFT)
#define FIFO_OVR_FLOW_CNT_SHIFT    0
#define FIFO_OVR_FLOW_CNT_MASK     (0xf << FIFO_OVR_FLOW_CNT_SHIFT)
#define FIFO_RD_PTR_SHIFT          0
#define FIFO_RD_PTR_MASK           (0xf << FIFO_RD_PTR_SHIFT)
#define FIFO_DATA_REG_SHIFT        0
#define FIFO_DATA_REG_MASK         (0xff << FIFO_RD_PTR_SHIFT)

/* CONFIGURATION - Configuration Register Bit Definitions */

/* Mode Configuration */

#define CFG_MODE_SHIFT             0
#define CFG_MODE_MASK              (7 << CFG_MODE_SHIFT)
#define CFG_MODE_TEMP_EN           (1 << 3)
#define CFG_MODE_RESET             (1 << 6)
#define CFG_MODE_SHDN              (1 << 7)

/* SPO2 Configuration */

#define CFG_SPO2_LED_PW_SHIFT      0
#define CFG_SPO2_LED_PW_MASK       (3 << CFG_SPO2_LED_PW_SHIFT)
#define CFG_SPO2_SR_SHIFT          2
#define CFG_SPO2_SR_MASK           (7 << CFG_SPO2_SR_SHIFT)
#define CFG_SPO2_HI_RES_EN         (1 << 6)

/* LED Configuration */

#define LED_IR_PA_SHIFT            0
#define LED_IR_PA_MASK             (0xf << LED_IR_PA_SHIFT)
#define LED_RED_PA_SHIFT           4
#define LED_RED_PA_MASK            (0xf << LED_RED_PA_SHIFT)

/* TEMPERATURE - Configuration Register Bit Definitions */

#define TEMP_TINT_SHIFT            0
#define TEMP_TINT_MASK             (0xff << TEMP_TINT_SHIFT)
#define TEMP_TFRAC_SHIFT           0
#define TEMP_TFRAC_MASK            (0xf << TEMP_TFRAC_SHIFT)

/* PART ID - Configuration Register Bit Definitions */

#define PART_REV_ID_SHIFT          0
#define PART_REV_ID_MASK           (0xff << PART_REV_ID_SHIFT)
#define PART_ID_SHIFT              0
#define PART_ID_MASK               (0xff << PART_ID_SHIFT)

/* MODES Definition */

#define MODE_HR_ONLY               2 /* Heart Rate only mode */
#define MODE_SPO2                  3 /* SpO2 mode */

/* SpO2 Sample Rate Definition */

#define SPO2_50SPS                 0 /* 50 Samples per Second */
#define SPO2_100SPS                1 /* 100 Samples per Second */
#define SPO2_167SPS                2 /* 167 Samples per Second */
#define SPO2_200SPS                3 /* 200 Samples per Second */
#define SPO2_400SPS                4 /* 400 Samples per Second */
#define SPO2_600SPS                5 /* 600 Samples per Second */
#define SPO2_800SPS                6 /* 800 Samples per Second */
#define SPO2_1KSPS                 7 /* 1000 Samples per Second */

/* LED Pulse Width Definition */

#define LED_PW_200US               0 /* 200uS @ 13-bit resolution */
#define LED_PW_400US               1 /* 400uS @ 14-bit resolution */
#define LED_PW_800US               2 /* 800uS @ 15-bit resolution */
#define LED_PW_1K6US               3 /* 1600uS @ 16-but resolution */

/* LED Current Definition */

#define LED_0MA                    0   /* 0.0 mA */
#define LED_4p4MA                  1   /* 4.4 mA */
#define LED_7p6MA                  2   /* 7.6 mA */
#define LED_11MA                   3   /* 11.0 mA */
#define LED_14p2MA                 4   /* 14.2 mA */
#define LED_17p4MA                 5   /* 17.4 mA */
#define LED_20p8MA                 6   /* 20.8 mA */
#define LED_24MA                   7   /* 24 mA */
#define LED_27p1MA                 8   /* 27.1 mA */
#define LED_30p6MA                 9   /* 30.6 mA */
#define LED_33p8MA                 0xa /* 33.8 mA */
#define LED_37MA                   0xb /* 37 mA */
#define LED_40p2MA                 0xc /* 40.2 mA */
#define LED_43p6MA                 0xd /* 43.6 mA */
#define LED_46p8MA                 0xe /* 46.8 mA */
#define LED_50MA                   0xf /* 50 mA */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: max30100_register
 *
 * Description:
 *   Register the MAX30100 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/oxihr0"
 *   i2c - An instance of the I2C interface to use to communicate w/ MAX30100
 *   addr - The I2C address of the MAX30100. The default I2C address of the
 *          MAX30100 is 0x57.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int max30100_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MAX30100 */
#endif /* __INCLUDE_NUTTX_SENSORS_MAX30100_H */
