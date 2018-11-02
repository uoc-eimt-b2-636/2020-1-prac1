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

#include "msp432_launchpad_board.h"
#include "cc3100_boosterpack.h"

#include "simplelink.h"
#include "driverlib.h"
#include "sl_common.h"
#include "osi.h"
#include "cli_uart.h"
#include "netcfg.h"

/*----------------------------------------------------------------------------*/

#define SL_STOP_TIMEOUT         ( 0xFF )

/*----------------------------------------------------------------------------*/

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,
    BSD_UDP_CLIENT_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

/*----------------------------------------------------------------------------*/

static int32_t establishConnectionWithAP(void);
static int32_t configureSimpleLinkToDefaultState(void);

static uint32_t g_Status;

/*----------------------------------------------------------------------------*/

int32_t wifi_init(void)
{
    int32_t retVal = 0;
    uint8_t RxAggrEnable;

    /* Configure command line interface */
    CLI_Configure();

    /* Start the SimpleLink */
    retVal = sl_Start(NULL, NULL, NULL);
    if ((retVal < 0) || (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        return retVal;
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if (retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        return retVal;
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    /* Disable packet aggregation */
    sl_NetCfgSet(SL_SET_HOST_RX_AGGR, 0, sizeof(RxAggrEnable), (_u8 *) &RxAggrEnable);

    return retVal;
}

/*----------------------------------------------------------------------------*/

int32_t wifi_restore(void)
{
    int32_t retVal = 0;

    /* Start the SimpleLink Host */
    retVal = configureSimpleLinkToDefaultState();
    if (retVal < 0)
    {
       if (DEVICE_NOT_IN_STATION_MODE == retVal)
       {
           CLI_Write(" Failed to configure the device in its default state \n\r");
       }

       return retVal;
    }

    CLI_Write(" Device is configured in default state \n\r");

    return retVal;
}

/*----------------------------------------------------------------------------*/

void wifi_sleep(void)
{
    /* Go to sleep */
    sl_Stop(SL_STOP_TIMEOUT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*----------------------------------------------------------------------------*/

void wifi_wakeup(void)
{
    int32_t retVal = 0;
    g_Status = 0;

    /* Re-start */
    sl_Start(NULL, NULL, NULL);
    if ((retVal < 0) || (ROLE_STA != retVal) )
    {
        led_red_on();
        while(1);
    }

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if (retVal < 0)
    {
        led_red_on();
        while(1);
    }

    /* Wait until connected to Wi-Fi and IP address acquired */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status)));
}

/*----------------------------------------------------------------------------*/

int16_t wifi_tcp_client_open(SlSockAddrIn_t* socket_address)
{
    int16_t status;
    int16_t socket_id;

    /* Create TCP socket */
    socket_id = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
    if (socket_id < 0)
    {
        status = socket_id;
        return status;
    }

    /* Connect TCP socket */
    status = sl_Connect(socket_id, (SlSockAddr_t *)socket_address, sizeof(SlSockAddrIn_t));
    if (status < 0)
    {
        /* Close TCP socket */
        sl_Close(socket_id);
        return status;
    }

    SlSockNonblocking_t enableOption = {.NonblockingEnabled = true,};
    sl_SetSockOpt(socket_id, SOL_SOCKET, SL_SO_NONBLOCKING, &enableOption, sizeof(enableOption));

    return socket_id;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_tcp_client_send(int16_t socket_id, uint8_t* buffer, uint16_t length)
{
    int16_t status;

    /* Send TCP packet */
    status = sl_Send(socket_id, buffer, length, 0);
    if (status != length)
    {
        /* Close TCP socket */
        sl_Close(socket_id);
        return -1;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_tcp_client_receive(int16_t socket_id, uint8_t* buffer, uint16_t length)
{
    int16_t status;

    /* Receive from TCP socket */
    status = sl_Recv(socket_id, buffer, length, 0);

    if (status < 0 && status != SL_EAGAIN) {
        sl_Close(socket_id);
    }

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_udp_client_open(SlSockAddrIn_t* socket_address)
{
    int16_t socket_id;

    /* Create UDP socket */
    socket_id = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, IPPROTO_UDP);

    return socket_id;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_udp_server_open(int16_t socket_id, SlSockAddrIn_t* socket_address)
{
    int16_t status;

    /* Create UDP socket */
    status = sl_Bind(socket_id,(SlSockAddr_t *)socket_address, sizeof(SlSockAddrIn_t));

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_udp_client_send(int16_t socket_id, SlSockAddrIn_t* socket_address, uint8_t* buffer, uint16_t length)
{
    int16_t status;

    /* Send UDP packet */
    status = sl_SendTo(socket_id, buffer, length, 0, (SlSockAddr_t *)socket_address, sizeof(SlSockAddrIn_t));

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_udp_client_receive(int16_t socket_id, SlSockAddrIn_t* socket_address, SlSocklen_t* socket_length, uint8_t* buffer, uint16_t length)
{
    int16_t status;

    status = sl_RecvFrom(socket_id, buffer, length, 0, (SlSockAddr_t *) socket_address, (SlSocklen_t*) socket_length);

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_client_close(int16_t socket_id)
{
    int16_t status;

    /* Close UDP socket */
    status = sl_Close(socket_id);

    return status;
}

/*----------------------------------------------------------------------------*/

int16_t wifi_get_host_by_name(uint8_t* server_name, uint16_t length, uint32_t* ip_address)
{
    int16_t status;

    status = sl_NetAppDnsGetHostByName((int8_t *) server_name, length, (unsigned long *) ip_address, SL_AF_INET);

    return status;
}

/*----------------------------------------------------------------------------*/

void wifi_set_socket_address(SlSockAddrIn_t* socket, uint32_t address, uint16_t port, bool swap)
{
    socket->sin_family = SL_AF_INET;

    if (swap) {
        socket->sin_addr.s_addr = sl_Htonl((uint32_t)address);
    } else {
        socket->sin_addr.s_addr = address;
    }

    socket->sin_port = sl_Htons((uint16_t)port);
}

/*----------------------------------------------------------------------------*/

static int32_t configureSimpleLinkToDefaultState(void)
{
    SlVersionFull ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    uint8_t val = 1;
    uint8_t configOpt = 0;
    uint8_t configLen = 0;
    uint8_t power = 0;
    int32_t retVal = -1;
    int32_t mode = -1;

    /* Initializing the Wi-Fi subsystem as a WLAN station */
    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status));
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    // retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 1, 0, 0, 0), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status));
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE, 1, 1, &val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 15;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM, SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    /* Success */
    return retVal;
}

/*----------------------------------------------------------------------------*/

static int32_t establishConnectionWithAP(void)
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key    = SEC_KEY;
    secParams.KeyLen = pal_Strlen(SEC_KEY);
    secParams.Type   = SEC_TYPE;

    retVal = sl_WlanConnect((_i8 *)SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status)));

    return SUCCESS;
}

/*----------------------------------------------------------------------------*/

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if (pWlanEvent == NULL)
    {
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
    }

    switch (pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*----------------------------------------------------------------------------*/

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if (pNetAppEvent == NULL)
    {
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
    }

    switch (pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*----------------------------------------------------------------------------*/

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse)
{
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*----------------------------------------------------------------------------*/

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*----------------------------------------------------------------------------*/

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if (pSock == NULL)
        {
            CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
            return;
        }

        switch (pSock->Event)
        {
            case SL_SOCKET_TX_FAILED_EVENT:
                switch (pSock->socketAsyncEvent.SockTxFailData.status)
                {
                    case SL_ECLOSE:
                        CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                        break;
                    default:
                        CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                        break;
                }
                break;

            default:
                CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
        }
}

/*----------------------------------------------------------------------------*/
