/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

// Linux USBcanII driver

#ifndef _USBCAN_HW_IF_H_
#define _USBCAN_HW_IF_H_



#include "helios_cmds.h"
#include "objbuf.h"

/*****************************************************************************/
/* defines */
/*****************************************************************************/

#define DEVICE_NAME_STRING              "usbcanII"
#define MAX_CARD_CHANNELS                        2
#define MAX_DRIVER_CHANNELS                    128
#define KV_USBCAN_MAIN_RCV_BUF_SIZE             16
#define KV_USBCAN_TX_CMD_BUF_SIZE               16
#define USBCAN_CMD_RESP_WAIT_TIME             1000
#define USBCANII_TICKS_PER_10US                  1


// Bits in the CxSTRH register in the M16C.
#define M16C_BUS_RESET    0x01    // Chip is in Reset state
#define M16C_BUS_ERROR    0x10    // Chip has seen a bus error
#define M16C_BUS_PASSIVE  0x20    // Chip is error passive
#define M16C_BUS_OFF      0x40    // Chip is bus off



#if DEBUG
#   define USBCAN_Q_CMD_WAIT_TIME                 800
#else
#   define USBCAN_Q_CMD_WAIT_TIME                 200
#endif



/* Channel specific data */
typedef struct UsbcanChanData
{
  /* These are the number of outgoing packets residing in the device */
  unsigned int outstanding_tx;
  spinlock_t   outTxLock;

  OBJECT_BUFFER *objbufs;

  CAN_MSG          current_tx_message[128];
} UsbcanChanData;



/*  Cards specific data */
typedef struct UsbcanCardData {


  unsigned int     max_outstanding_tx;
  int              autoTxBufferCount;
  int              autoTxBufferResolution;

  spinlock_t       timeHi_lock;

  spinlock_t       replyWaitListLock;
  struct list_head replyWaitList;

  /* Structure to hold all of our device specific stuff */
  struct workqueue_struct  *txTaskQ;
  struct work_struct       txWork;

  heliosCmd          txCmdBuffer[KV_USBCAN_TX_CMD_BUF_SIZE]; /* Control messages */
  Queue              txCmdQueue;

  // busparams
  unsigned long freq;
  unsigned char sjw;
  unsigned char tseg1;
  unsigned char tseg2;
  unsigned char samples;

  struct usb_device       *udev;               // save off the usb device pointer
  struct usb_interface    *interface;          // the interface for this device

  unsigned char *         bulk_in_buffer;      // the buffer to receive data
  size_t                  bulk_in_size;        // the size of the receive buffer
  __u8                    bulk_in_endpointAddr;// the address of the bulk in endpoint
  unsigned int            bulk_in_MaxPacketSize;

  unsigned char *         bulk_out_buffer;     // the buffer to send data
  size_t                  bulk_out_size;       // the size of the send buffer
  unsigned int            bulk_out_MaxPacketSize;

  struct urb *            write_urb;           // the urb used to send data
  struct urb *            read_urb;            // the urb used to receive data
  __u8                    bulk_out_endpointAddr;//the address of the bulk out endpoint
  struct completion       write_finished;       // wait for the write to finish

  VCanCardData           *vCard;
} UsbcanCardData;



#endif  /* _USBCAN_HW_IF_H_ */
