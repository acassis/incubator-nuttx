/****************************************************************************
 * libs/libc/modlib/modlib_load.c
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

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ELF_ALIGN_MASK   ((1 << CONFIG_MODLIB_ALIGN_LOG2) - 1)
#define ELF_ALIGNUP(a)   (((unsigned long)(a) + ELF_ALIGN_MASK) & ~ELF_ALIGN_MASK)
#define ELF_ALIGNDOWN(a) ((unsigned long)(a) & ~ELF_ALIGN_MASK)

#ifndef MAX
#  define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#  define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

/* _ALIGN_UP: 'a' is assumed to be a power of two */

#define _ALIGN_UP(v, a) (((v) + ((a) - 1)) & ~((a) - 1))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_elfsize
 *
 * Description:
 *   Calculate total memory allocation for the ELF file.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static void modlib_elfsize(struct mod_loadinfo_s *loadinfo)
{
  size_t textsize;
  size_t datasize;
  int i;

  /* Accumulate the size each section into memory that is marked SHF_ALLOC */

  textsize = 0;
  datasize = 0;

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      FAR Elf32_Phdr *phdr = &loadinfo->phdr[i];
      FAR void *textaddr = NULL;

      if (phdr->p_type == PT_LOAD)
	{
	  if (phdr->p_flags & PF_X) 
            {
	      textsize += phdr->p_memsz;
	      textaddr = (void *) phdr->p_vaddr;
            }
	  else
            {
	      datasize += phdr->p_memsz;
              loadinfo->datasec = phdr->p_vaddr;
              loadinfo->segpad  = phdr->p_vaddr - ((uintptr_t) textaddr + textsize);
            }
	}
    }

  /* Save the allocation size */

  loadinfo->textsize = textsize;
  loadinfo->datasize = datasize;
}

/****************************************************************************
 * Name: modlib_loadfile
 *
 * Description:
 *   Read the section data into memory. Section addresses in the shdr[] are
 *   updated to point to the corresponding position in the memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int modlib_loadfile(FAR struct mod_loadinfo_s *loadinfo)
{
  FAR uint8_t *text;
  FAR uint8_t *data;
  int ret;
  int i;

  /* Read each PT_LOAD area into memory */

  binfo("Loading sections - text: %p.%x data: %p.%x\n",
        (void *)loadinfo->textalloc,loadinfo->textsize,
	(void *)loadinfo->datastart,loadinfo->datasize);
  text = (FAR uint8_t *)loadinfo->textalloc;
  data = (FAR uint8_t *)loadinfo->datastart;

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      FAR Elf32_Phdr *phdr = &loadinfo->phdr[i];

      if (phdr->p_type == PT_LOAD)
        {
          if (phdr->p_flags & PF_X)
              ret = modlib_read(loadinfo, text, phdr->p_filesz, phdr->p_offset);
          else 
	    {
	      int bssSize = phdr->p_memsz - phdr->p_filesz;
              ret = modlib_read(loadinfo, data, phdr->p_filesz, phdr->p_offset);
	      memset((FAR void *)((uintptr_t) data + phdr->p_filesz), 0, bssSize);
	    }
          if (ret < 0)
            {
              berr("ERROR: Failed to read section %d: %d\n", i, ret);
              return ret;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_load
 *
 * Description:
 *   Loads the binary into memory, allocating memory, performing relocations
 *   and initializing the data and bss segments.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_load(FAR struct mod_loadinfo_s *loadinfo)
{
  int ret;

  binfo("loadinfo: %p\n", loadinfo);
  DEBUGASSERT(loadinfo && loadinfo->filfd >= 0);

  /* Load section and program headers into memory */

  ret = modlib_loadhdrs(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadhdrs failed: %d\n", ret);
      goto errout_with_buffers;
    }

  /* Determine total size to allocate */

  modlib_elfsize(loadinfo);

  /* Allocate (and zero) memory for the ELF file. */

  /* Allocate memory to hold the ELF image */

  if (loadinfo->textsize > 0)
    {
#if defined(CONFIG_ARCH_USE_TEXT_HEAP)
      loadinfo->textalloc = (uintptr_t)
                            up_textheap_memalign(loadinfo->textalign,
                                                 loadinfo->textsize + 
						 loadinfo->datasize + 
						 loadinfo->segpad);
#else
      loadinfo->textalloc = (uintptr_t)lib_memalign(loadinfo->textalign,
                                                    loadinfo->textsize + 
						    loadinfo->datasize + 
						    loadinfo->segpad);
#endif
      if (!loadinfo->textalloc)
        {
          berr("ERROR: Failed to allocate memory for the module text\n");
          ret = -ENOMEM;
          goto errout_with_buffers;
        }
    }

  if (loadinfo->datasize > 0)
    {
      loadinfo->datastart = loadinfo->textalloc + loadinfo->textsize + loadinfo->segpad;
    }

  /* Load ELF section data into memory */

  ret = modlib_loadfile(loadinfo);
  if (ret < 0)
    {
      berr("ERROR: modlib_loadfile failed: %d\n", ret);
      goto errout_with_buffers;
    }

  return OK;

  /* Error exits */

errout_with_buffers:
  modlib_unload(loadinfo);
  return ret;
}
