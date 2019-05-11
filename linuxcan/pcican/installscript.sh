#!/bin/sh

#
#             Copyright 2017 by Kvaser AB, Molndal, Sweden
#                         http://www.kvaser.com
#
#  This software is dual licensed under the following two licenses:
#  BSD-new and GPLv2. You may use either one. See the included
#  COPYING file for details.
#
#  License: BSD-new
#  ==============================================================================
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the <organization> nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
#  License: GPLv2
#  ==============================================================================
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
#
#
#  IMPORTANT NOTICE:
#  ==============================================================================
#  This source code is made available for free, as an open license, by Kvaser AB,
#  for use with its applications. Kvaser AB does not accept any liability
#  whatsoever for any third party patent or other immaterial property rights
#  violations that may result from any usage of this source code, regardless of
#  the combination of source code and various applications that it can be used
#  in, or with.
#
#  -----------------------------------------------------------------------------
#

MODNAME=kvpcican
DEPMOD=`which depmod`

install -d -m 755 /lib/modules/`uname -r`/kernel/drivers/char/
install -m 644 $MODNAME.ko /lib/modules/`uname -r`/kernel/drivers/char/
if [ "$?" -ne 0 ] ; then
  exit 1
fi
install -m 755 pcican.sh /usr/sbin/
if [ "$?" -ne 0 ] ; then
  exit 1
fi
/usr/sbin/pcican.sh stop 2>/dev/null

echo Checking for loaded SocketCAN driver for Kvaser PCI devices.
if lsmod | grep kvaser_pci ; then
  echo Unloading SocketCAN driver...
  modprobe -r kvaser_pci
  if lsmod | grep -q kvaser_pciefd ; then
    modprobe -r kvaser_pciefd
  fi
else
  echo SocketCAN driver not found.
fi

echo Blacklisting SocketCAN Kvaser PCI driver to prevent it from auto-loading.

if [ -f /etc/modprobe.conf ] ; then
  # CentOS/Redhat/RHEL/Fedora Linux...
  CONF=/etc/modprobe.conf
  BLACKLIST=$(printf "alias     kvaser_pci   /dev/null\nalias     kvaser_pciefd  /dev/null")
else
  # Debian/Ubuntu Linux
  CONF=/etc/modprobe.d/kvaser.conf
  BLACKLIST=$(printf "blacklist kvaser_pci\nblacklist kvaser_pciefd")
  if [ ! -f $CONF ] ; then
    touch $CONF
  fi
fi

# First, remove any old PCIcan settings.
# The space after pcican in the grep below is needed to not match pcicanII.
grep -v "pcican "        < $CONF                          > newconfx
grep -v "^${BLACKLIST}"  < newconfx                       > newconf
rm newconfx

# Add PCIcan.
echo "alias     pcican       $MODNAME"                  >> newconf
echo "install   $MODNAME     /usr/sbin/pcican.sh start" >> newconf
echo "remove    $MODNAME     /usr/sbin/pcican.sh stop"  >> newconf
# Since it conflicts with pcican, we disable kvaser_pci (SocketCAN).
echo "${BLACKLIST}"                                     >> newconf

cat newconf > $CONF
rm newconf

if [ "$#" -gt 0 ] && [ $1 = "develinstall" ] ; then
  echo "Ignoring $DEPMOD -a for now.."
else
  $DEPMOD -a
  if [ "$?" -ne 0 ] ; then
    echo Failed to execute $DEPMOD -a
  fi
fi

MODCONF=/etc/modules-load.d/kvaser.conf

if [ "$#" -gt 0 ] && [ $1 = "load" ] ; then
  /usr/sbin/pcican.sh start
  touch $MODCONF
  if [ -f $MODCONF ] ; then
    grep -v "$MODNAME" < $MODCONF    > newmodules
    echo "$MODNAME"                 >> newmodules
    cat newmodules > $MODCONF
    rm newmodules
  fi
fi