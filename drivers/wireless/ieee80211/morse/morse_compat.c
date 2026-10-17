/****************************************************************************
 * drivers/wireless/ieee80211/morse/morse_compat.c
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

/* The pre-built libmorse is compiled against the newlib C library headers.
 * This file provides the few newlib specific symbols that it references
 * and that are not part of the NuttX C library.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* newlib character class bits, see newlib <ctype.h> */

#define _U  0x01  /* Upper case */
#define _L  0x02  /* Lower case */
#define _N  0x04  /* Digit */
#define _S  0x08  /* White space */
#define _P  0x10  /* Punctuation */
#define _C  0x20  /* Control */
#define _X  0x40  /* Hexadecimal digit */
#define _B  0x80  /* Blank */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* newlib character class table used by its <ctype.h> macros.  The first
 * entry is for EOF, the macros index it as (_ctype_ + 1)[c].  Characters
 * above 0x7f are not classified (C locale).
 */

const char _ctype_[1 + 256] =
{
  0,
  _C, _C, _C, _C,
  _C, _C, _C, _C,
  _C, _C | _S | _B, _C | _S, _C | _S,
  _C | _S, _C | _S, _C, _C,
  _C, _C, _C, _C,
  _C, _C, _C, _C,
  _C, _C, _C, _C,
  _C, _C, _C, _C,
  _S | _B, _P, _P, _P,
  _P, _P, _P, _P,
  _P, _P, _P, _P,
  _P, _P, _P, _P,
  _N, _N, _N, _N,
  _N, _N, _N, _N,
  _N, _N, _P, _P,
  _P, _P, _P, _P,
  _P, _U | _X, _U | _X, _U | _X,
  _U | _X, _U | _X, _U | _X, _U,
  _U, _U, _U, _U,
  _U, _U, _U, _U,
  _U, _U, _U, _U,
  _U, _U, _U, _U,
  _U, _U, _U, _P,
  _P, _P, _P, _P,
  _P, _L | _X, _L | _X, _L | _X,
  _L | _X, _L | _X, _L | _X, _L,
  _L, _L, _L, _L,
  _L, _L, _L, _L,
  _L, _L, _L, _L,
  _L, _L, _L, _L,
  _L, _L, _L, _P,
  _P, _P, _P, _C,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* None: this file only provides data */
