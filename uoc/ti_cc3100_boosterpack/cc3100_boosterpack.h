/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CC3100_BOOSTERPACK_H_
#define CC3100_BOOSTERPACK_H_

#include <stdint.h>

#include "netcfg.h"

#include "wifi_config.h"

int32_t wifi_init(void);
int32_t wifi_restore(void);

void wifi_sleep(void);
void wifi_wakeup(void);

int16_t wifi_tcp_client_open(SlSockAddrIn_t* socket_address);
int16_t wifi_tcp_client_send(int16_t socket_id, uint8_t* buffer, uint16_t length);
int16_t wifi_tcp_client_receive(int16_t socket_id, uint8_t* buffer, uint16_t length);

int16_t wifi_udp_client_open(SlSockAddrIn_t* socket_address);
int16_t wifi_udp_client_send(int16_t socket_id, SlSockAddrIn_t* socket_address, uint8_t* buffer, uint16_t length);
int16_t wifi_udp_client_receive(int16_t socket_id, SlSockAddrIn_t* socket_address, SlSocklen_t* socket_length, uint8_t* buffer, uint16_t length);

int16_t wifi_udp_server_open(int16_t socket_id, SlSockAddrIn_t* socket_address);

int16_t wifi_client_close(int16_t socket_id);

int16_t wifi_get_host_by_name(uint8_t* server_name, uint16_t length, uint32_t* ip_address);
void wifi_set_socket_address(SlSockAddrIn_t* socket, uint32_t address, uint16_t port, bool swap);


#endif /* CC3100_BOOSTERPACK_H_ */
