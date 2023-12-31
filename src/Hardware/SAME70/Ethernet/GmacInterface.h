/**
 *
 * \file
 *
 * \brief GMAC (Gigabit MAC) driver for lwIP.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef SAME70_ETHERNET_GMACINTERFACE_H_INCLUDED
#define SAME70_ETHERNET_GMACINTERFACE_H_INCLUDED

extern "C" {
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
}

#include <Platform/MessageType.h>

err_t ethernetif_init(struct netif *netif) noexcept;		// called by LwIP to initialise the interface

void ethernetif_terminate() noexcept;						// called when we shut down

bool ethernetif_input(struct netif *netif) noexcept;		// checks for a new packet and returns true if one was processed

void ethernetif_hardware_init() noexcept;					// initialises the low-level hardware interface

bool ethernetif_establish_link() noexcept;					// attempts to establish link and returns true on success

bool ethernetif_link_established() noexcept;				// asks the PHY if the link is still up

void ethernetif_set_mac_address(const uint8_t macAddress[]) noexcept;

void ethernetif_diagnostics(MessageType mtype) noexcept;

#endif /* SAME70_ETHERNET_GMACINTERFACE_H_INCLUDED */
