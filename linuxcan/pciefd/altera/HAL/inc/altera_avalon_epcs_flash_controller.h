/******************************************************************************
 *                                                                             *
 * License Agreement                                                           *
 *                                                                             *
 * Copyright (c) 2003 Altera Corporation, San Jose, California, USA.           *
 * All rights reserved.                                                        *
 *                                                                             *
 * Permission is hereby granted, free of charge, to any person obtaining a     *
 * copy of this software and associated documentation files (the "Software"),  *
 * to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
 * and/or sell copies of the Software, and to permit persons to whom the       *
 * Software is furnished to do so, subject to the following conditions:        *
 *                                                                             *
 * The above copyright notice and this permission notice shall be included in  *
 * all copies or substantial portions of the Software.                         *
 *                                                                             *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
 * DEALINGS IN THE SOFTWARE.                                                   *
 *                                                                             *
 * This agreement shall be governed in all respects by the laws of the State   *
 * of California and by the laws of the United States of America.              *
 *                                                                             *
 * Public interfaces to the Altera flash programming library                   *
 * Restrictions - For now this layer will program the sectors it needs         *
 * if it needs to erase a sector it will. If you wish to have all the contents *
 * of the sector preserved you the user need to be aware of this. The reasoning*
 * here is that sectors can be very large 64k which is a large buffer to tie   *
 * up in our programming library. Any filesystems you would want to use should *
 * have the capabilities to cope with this.                                    *
 *                                                                             *
 * Author Aaron Ferrucci, adapted from work by PRR                             *
 *                                                                             *
 ******************************************************************************/
#ifndef __ALT_EPCS_FLASH_H__
#define __ALT_EPCS_FLASH_H__
//#include "alt_types.h"
#include "inc/sys/alt_flash_dev.h"
//#include "sys/alt_llist.h"

#define SERIALFLASH_REGISTER_OFFSET (1024)

/******************************************************************************
 *                                                                             *
 * Structures                                                                  *
 *                                                                             *
 ******************************************************************************/

typedef struct alt_flash_epcs_dev alt_flash_epcs_dev;

/*
 *  Description of the flash
 *
 *  Contains the basic alt_flash_dev, plus
 *  epcs-specific parameters.
 *
 *  Every parameter that distinguishes this
 *  serial flash device from any other is
 *  encoded here.
 *
 *  Example:
 *  size_in_bytes: the number of bytes of
 *  storage in this chip.
 */
struct alt_flash_epcs_dev
{
  alt_flash_dev dev;

  volatile void * register_base;
  uint32_t size_in_bytes;
  uint32_t silicon_id;
  uint32_t page_size;
};

/*
 *   Macros used by alt_sys_init.c
 *
 * By default this driver is initialised in alt_sys_init. However if the
 * "small driver" feature has been selected, or if fast simulation has been requested
 * no initialisation is performed.
 *
 * This causes the driver to be excluded from the system (unless explicitly
 * initialised by the user from main()), and therefore reduces code footprint.
 *
 */
/*    ALT_LLIST_ENTRY,          \ */

/* #define ALTERA_AVALON_EPCS_FLASH_CONTROLLER_INSTANCE(name, dev) \ */
/* static alt_flash_epcs_dev dev =    \ */
/* {                                 \ */
/*   {                               \ */
/*     name##_NAME,                  \ */
/*     NULL,                         \ */
/*     NULL,                         \ */
/*     alt_epcs_flash_write,         \ */
/*     alt_epcs_flash_read,          \ */
/*     alt_epcs_flash_get_info,      \ */
/*     alt_epcs_flash_erase_block,   \ */
/*     alt_epcs_flash_write_block,   \ */
/*     ((void*)(name##_BASE))        \ */
/*   },                              \ */
/*   (name##_REGISTER_OFFSET + name##_BASE)          \ */
/* } */

/* #define ALTERA_AVALON_EPCS_FLASH_CONTROLLER_INIT(name, dev) \ */
/* alt_epcs_flash_init(&dev) */

/*
 *  The initialisation function which reads the EPCS table and fills out
 *  the appropriate sections of the the alt_flash_epcs_dev structure
 */
int alt_epcs_flash_init(alt_flash_epcs_dev* flash, volatile void * base);

/*
 *  Functions exported through the common Flash interface
 */

/*
 * Restrictions - For now this function will program the sectors it needs,
 * if it needs to erase a sector it will. If you wish to have all the contents
 * of the sector preserved you the user need to be aware of this and read out
 * the contents of that sector and add it to the data you wish to program.
 * The reasoning here is that sectors can be very large eg. 64k which is a
 * large buffer to tie up in our programming library, when not all users will
 * want that functionality.
 */

int alt_epcs_flash_memcmp( alt_flash_dev* flash_info,
                                  const void* src_buffer,
                                  int offset,
                                  size_t n );
 
int alt_epcs_flash_write(alt_flash_dev* flash_info, int offset,
                         const void* src_addr, int length);

int alt_epcs_flash_read(alt_flash_dev* flash_info, int offset,
                        void* dest_addr, int length);
int alt_epcs_flash_get_info(alt_flash_fd* fd, flash_region** info,
                            int*  );

int alt_epcs_flash_erase_block(alt_flash_dev* flash_info, int block_offset);

int alt_epcs_flash_write_block(alt_flash_dev* flash, int block_offset,
                               int data_offset, const void* data,
                               int length);

#endif /* __ALT_EPCS_FLASH_H__ */
