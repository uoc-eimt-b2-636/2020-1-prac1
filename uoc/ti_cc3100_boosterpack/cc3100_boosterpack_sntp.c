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

/*----------------------------------------------------------------------------*/
#include "cc3100_boosterpack_sntp.h"
#include "cc3100_boosterpack.h"
/*----------------------------------------------------------------------------*/
#define SUCCESS             ( 0 )
#define MAX_BUF_SIZE        ( 48 )

#define SMTP_UDP_PORT       ( 123 )

#define SECONDS_FROM_2013   ( 3565987200L )       /* 43 years + 11 days of leap years */

#define SEC_IN_MIN          ( 60 )
#define SEC_IN_HOUR         ( 3600 )
#define SEC_IN_DAY          ( 86400 )

#define GMT_TIME_ZONE_HR    ( 2 )
#define GMT_TIME_ZONE_MIN   ( 0 )
/*----------------------------------------------------------------------------*/
static const uint8_t sntp_server_name[30] = "time-c.nist.gov";
static uint16_t sntp_server_length = 0;
static uint32_t sntp_server_address = 0;
static uint16_t sntp_server_port = SMTP_UDP_PORT;
/*----------------------------------------------------------------------------*/
static uint8_t tx_buffer[MAX_BUF_SIZE];
static uint8_t rx_buffer[MAX_BUF_SIZE];
/*----------------------------------------------------------------------------*/
int32_t cc3100_boosterpack_sntp_get(sntp_time_t* sntp_time)
{
    SlSockAddrIn_t tx_socket, rx_socket;
    int16_t socket_id;
    int16_t status;
    int16_t rx_bytes;

    memset(tx_buffer, 0, sizeof(tx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));

    /* Resolve SNTP server address from name */
    sntp_server_length = strlen((const char *)sntp_server_name);
    status = wifi_get_host_by_name((uint8_t *)sntp_server_name, sntp_server_length, &sntp_server_address);
    if (status < 0)
    {
        while(1)
            ;
    }

    /* Fill in SNTP request */
    tx_buffer[0] = 0x1B;

    /* Open UDP socket to send SNTP request */
    wifi_set_socket_address(&tx_socket, sntp_server_address, sntp_server_port, true);
    socket_id = wifi_udp_client_open(&tx_socket);
    if (socket_id < 0)
    {
        while(1)
            ;
    }

    /* Send SNTP request over UDP socket */
    status = wifi_udp_client_send(socket_id, &tx_socket, tx_buffer, sizeof(tx_buffer));
    if (status < 0)
    {
        while(1)
            ;
    }

    /* Open UDP socket to receive SNTP response */
    wifi_set_socket_address(&rx_socket, 0, 0, false);
    status = wifi_udp_server_open(socket_id, &rx_socket);
    if (status < 0)
    {
        while(1)
            ;
    }

    /* Receive SNTP response from socket */
    rx_bytes = sizeof(SlSockAddrIn_t);
    status = wifi_udp_client_receive(socket_id, (SlSockAddrIn_t *)&rx_socket, (SlSocklen_t *)&rx_bytes, rx_buffer, sizeof(rx_buffer));
    if (status < 0)
    {
        while(1)
            ;
    }

    /* Parse SNTP response */
    if ((rx_buffer[0] & 0x7) == 4)
    {
        uint32_t seconds = 0;

        /* Recover time from receive buffer */
        seconds = rx_buffer[40];
        seconds <<= 8;
        seconds += rx_buffer[41];
        seconds <<= 8;
        seconds += rx_buffer[42];
        seconds <<= 8;
        seconds += rx_buffer[43];

        /* Subtract time from 2013 */
        seconds = seconds - SECONDS_FROM_2013;

        /* Add GMT correction for hours and minutes */
        seconds += GMT_TIME_ZONE_HR * SEC_IN_HOUR;
        seconds += GMT_TIME_ZONE_MIN * SEC_IN_MIN;

        /* Seconds from start of day */
        seconds = seconds % SEC_IN_DAY;

        /* Hours */
        sntp_time->hour = seconds / SEC_IN_HOUR;

        /* Minutes */
        seconds = seconds % SEC_IN_HOUR;
        sntp_time->minute = seconds / SEC_IN_MIN;

        /* Seconds */
        seconds = seconds % SEC_IN_MIN;
        sntp_time->second = seconds;
    }
    else
    {
        while(1)
            ;
    }

    return SUCCESS;
}
/*----------------------------------------------------------------------------*/
