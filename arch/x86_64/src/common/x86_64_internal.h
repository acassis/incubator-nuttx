/****************************************************************************
 * arch/x86_64/src/common/x86_64_internal.h
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

#ifndef __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <nuttx/sched.h>
#  include <stdint.h>
#  include <arch/io.h>
#  include <arch/irq.h>
#  include <arch/multiboot2.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bring-up debug configurations.  These are here (vs defconfig) because
 * these should only be controlled during low level board bring-up and not
 * part of normal platform configuration.
 */

#undef  CONFIG_SUPPRESS_INTERRUPTS    /* DEFINED: Do not enable interrupts */
#undef  CONFIG_SUPPRESS_TIMER_INTS    /* DEFINED: No timer */
#undef  CONFIG_SUPPRESS_SERIAL_INTS   /* DEFINED: Console will poll */
#undef  CONFIG_SUPPRESS_UART_CONFIG   /* DEFINED: Do not reconfig UART */

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if !defined(CONFIG_DEV_CONSOLE)
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_16550_UART) && \
        !defined(CONFIG_16550_NO_SERIAL_CONSOLE)
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  elif defined(CONFIG_16550_PCI_UART) && \
        !defined(CONFIG_16550_PCI_NO_SERIAL_CONSOLE)
#    define USE_SERIALDRIVER 1
#    undef  USE_EARLYSERIALINIT
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/* Check if an interrupt stack size is configured */

#ifndef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK 0
#endif

/* The initial stack point is aligned at 16 bytes boundaries. If
 * necessary frame_size must be rounded up to the next boundary to retain
 * this alignment.
 */

#define STACK_ALIGNMENT     16

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (STACK_ALIGNMENT - 1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR         0xdeadbeef
#define INTSTACK_COLOR      0xdeadbeef
#define HEAP_COLOR          'h'

#define getreg8(p)          inb(p)
#define putreg8(v,p)        outb(v,p)
#define getreg16(p)         inw(p)
#define putreg16(v,p)       outw(v,p)
#define getreg32(p)         inl(p)
#define putreg32(v,p)       outl(v,p)

/* ISR/IRQ stack size */

#if CONFIG_ARCH_INTERRUPTSTACK == 0
#  define IRQ_STACK_SIZE 0x2000
#else
#  define IRQ_STACK_SIZE CONFIG_ARCH_INTERRUPTSTACK
#endif

/* Linker defined section addresses */

#define _START_TEXT  _stext
#define _END_TEXT    _etext
#define _START_BSS   _sbss
#define _END_BSS     _ebss
#define _DATA_INIT   _eronly
#define _START_DATA  _sdata
#define _END_DATA    _edata
#define _START_TDATA _stdata
#define _END_TDATA   _etdata
#define _START_TBSS  _stbss
#define _END_TBSS    _etbss

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*up_vector_t)(void);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the beginning of heap as provided from up_head.S. This is the
 * first address in DRAM after the loaded program+bss+idle stack.  The
 * end of the heap is CONFIG_RAM_END
 */

extern const uintptr_t g_idle_topstack[];

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
extern uint8_t g_intstackalloc[];
extern uint8_t g_isrstackalloc[];
#endif

/* These symbols are setup by the linker script. */

extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */
extern uint8_t _stdata[];          /* Start of .tdata */
extern uint8_t _etdata[];          /* End+1 of .tdata */
extern uint8_t _stbss[];           /* Start of .tbss */
extern uint8_t _etbss[];           /* End+1 of .tbss */
#endif

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void x86_64_cpuid(uint32_t leaf, uint32_t subleaf,
                                uint32_t *eax, uint32_t *ebx,
                                uint32_t *ecx, uint32_t *edx)
{
  __asm__ volatile("cpuid"
                   : "=a"(*eax), "=b"(*ebx), "=c"(*ecx), "=d"(*edx)
                   : "a" (leaf), "c" (subleaf)
                   : "memory");
}

#ifdef CONFIG_ARCH_KERNEL_STACK
static inline_function uint64_t *x86_64_get_ktopstk(void)
{
  uint64_t *ktopstk;
  __asm__ volatile("movq %%gs:(%c1), %0"
                   : "=rm" (ktopstk)
                   : "i" (offsetof(struct intel64_cpu_s, ktopstk)));
  return ktopstk;
}

static inline_function void x86_64_set_ktopstk(uint64_t *ktopstk)
{
  __asm__ volatile("movq %0, %%gs:(%c1)"
                   :: "r" (ktopstk), "i" (offsetof(struct intel64_cpu_s,
                                                   ktopstk)));
}
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/****************************************************************************
 * Name: x86_64_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory configs/<board-name>/src/.
 *
 ****************************************************************************/

void x86_64_boardinitialize(void);

/* Defined in files with the same name as the function */

void x86_64_copystate(uint64_t *dest, uint64_t *src);
void x86_64_decodeirq(uint64_t *regs);
#ifdef CONFIG_ARCH_DMA
void weak_function x86_64_dmainitialize(void);
#endif
void x86_64_fullcontextrestore(uint64_t *restoreregs) noreturn_function;
void x86_64_switchcontext(uint64_t *saveregs, uint64_t *restoreregs);
void x86_64_sigdeliver(void);
void x86_64_lowputc(char ch);
void x86_64_lowputs(const char *str);
void x86_64_restore_auxstate(struct tcb_s *rtcb);
void x86_64_checktasks(void);

uint64_t *x86_64_syscall(uint64_t *regs);

#ifdef CONFIG_ARCH_MULTIBOOT2
void x86_64_mb2_fbinitialize(struct multiboot_tag_framebuffer *tag);
void fb_putc(int ch);
#endif

/* Defined in up_allocateheap.c */

#if CONFIG_MM_REGIONS > 1
void x86_64_addregion(void);
#else
#  define x86_64_addregion()
#endif

/* Defined in xyz_serial.c */

void x86_64_earlyserialinit(void);
void x86_64_serialinit(void);

/* Defined in xyz_timerisr.c */

void x86_64_timer_calibrate_freq(void);
void x86_64_timer_initialize(void);

/* Defined in board/x86_64_network.c */

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void x86_64_netinitialize(void);
#else
#  define x86_64_netinitialize()
#endif

#ifdef CONFIG_USBDEV
void x86_64_usbinitialize(void);
void x86_64_usbuninitialize(void);
#else
#  define x86_64_usbinitialize()
#  define x86_64_usbuninitialize()
#endif

/* Defined in x86_64_pci.c */

#ifdef CONFIG_PCI
void x86_64_pci_init(void);
#endif

/* Defined in intel64_checkstack.c */

#ifdef CONFIG_STACK_COLORATION
size_t x86_64_stack_check(void *stackbase, size_t nbytes);
void x86_64_stack_color(void *stackbase, size_t nbytes);
#endif

/* TLB shootdown */

int x86_64_tlb_handler(int irq, void *c, void *arg);
void x86_64_tlb_shootdown(void);

#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_X86_64_SRC_COMMON_UP_INTERNAL_H */
