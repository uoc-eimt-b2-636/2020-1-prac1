/*
 * spi_cc3100.c
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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
 *    Neither the name of Texas Instruments Incorporated nor the names of
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

#include <msp432.h>

#include "simplelink.h"
#include "spi.h"
#include "board.h"

/*----------------------------------------------------------------------------*/

#define SPI_BASE                    ( EUSCI_B0_BASE )

#define SPI_SYSTEM_CLK              ( 40000000 )
#define SPI_PERIPH_CLK              ( 20000000 )

#define SPI_MOSI_PORT               ( GPIO_PORT_P1 )
#define SPI_MOSI_PIN                ( GPIO_PIN6 )
#define SPI_MISO_PORT               ( GPIO_PORT_P1 )
#define SPI_MISO_PIN                ( GPIO_PIN7 )
#define SPI_CLK_PORT                ( GPIO_PORT_P1 )
#define SPI_CLK_PIN                 ( GPIO_PIN5 )
#define SPI_CS_PORT                 ( GPIO_PORT_P3 )
#define SPI_CS_PIN                  ( GPIO_PIN0 )

#define CC3100_IRQ_PORT             ( GPIO_PORT_P2 )
#define CC3100_IRQ_PIN              ( GPIO_PIN5 )
#define CC3100_IRQ_INT              ( INT_PORT2 )

#define CC3100_nHIB_PORT            ( GPIO_PORT_P4 )
#define CC3100_nHIB_PIN             ( GPIO_PIN1 )

#define DMA_TX_CHANNEL              ( DMA_CH0_EUSCIB0TX0 )
#define DMA_TX_NUMBER               ( 0 )
#define DMA_RX_CHANNEL              ( DMA_CH1_EUSCIB0RX0 )
#define DMA_RX_NUMBER               ( 1 )

#define DMA_BUFF_SIZE_MIN           ( 100 )
#define DMA_MAX_TRANSACTION_SIZE    ( 1024 )

/*----------------------------------------------------------------------------*/

static void spi_assert_cs(void);
static void spi_deassert_cs(void);
static void dma_setup_write(unsigned char *pBuff, int len);
static void dma_setup_read(unsigned char *pBuff, int len);

/*----------------------------------------------------------------------------*/

#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable1, 1024)
DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable1[32];

/* SPI Master Configuration Parameter */
static const eUSCI_SPI_MasterConfig spiMasterConfig = {
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                           // SMCLK Clock Source
        SPI_SYSTEM_CLK,                                          // SMCLK = 48 MHz
        SPI_PERIPH_CLK,                                          // SPICLK = 16 MHz
        EUSCI_B_SPI_MSB_FIRST,                                   // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT, // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                // Low polarity
        EUSCI_B_SPI_3PIN                                         // 3Wire SPI Mode
};

/*----------------------------------------------------------------------------*/

Fd_t spi_Open(char *ifName, unsigned long flags)
{
    /* Select the MSP432 SPI lines: MOSI/MISO on P1.6,7 CLK on P3.2 */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            SPI_CLK_PORT, SPI_CLK_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            SPI_MISO_PORT, SPI_MISO_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring SPI in 3wire master mode */
    MAP_SPI_initMaster(SPI_BASE, &spiMasterConfig);

    /* Enable SPI module */
    MAP_SPI_enableModule(SPI_BASE);

    MAP_GPIO_setOutputLowOnPin(CC3100_nHIB_PORT, CC3100_nHIB_PIN);
    MAP_GPIO_setAsOutputPin(CC3100_nHIB_PORT, CC3100_nHIB_PIN);

    /* Configure SPI IRQ line on P2.5 on MSP432 */
    MAP_GPIO_setAsInputPinWithPullDownResistor(CC3100_IRQ_PORT, CC3100_IRQ_PIN);

    /* Configure the SPI CS to be on P3.0 on MSP432*/
    MAP_GPIO_setOutputHighOnPin(SPI_CS_PORT, SPI_CS_PIN);
    MAP_GPIO_setAsOutputPin(SPI_CS_PORT, SPI_CS_PIN);

    /* 50 ms delay */
    Delay(50);

    /* Enable WLAN interrupt */
    CC3100_InterruptEnable();

#ifdef SL_PLATFORM_MULTI_THREADED
    return OSI_OK;
#else
    return NONOS_RET_OK;
#endif
}

/*----------------------------------------------------------------------------*/

int spi_Close(Fd_t fd)
{
    /* Disable WLAN interrupt */
    CC3100_InterruptDisable();

#ifdef SL_PLATFORM_MULTI_THREADED
    return OSI_OK;
#else
    return NONOS_RET_OK;
#endif
}

/*----------------------------------------------------------------------------*/

int spi_Write(Fd_t fd, unsigned char *pBuff, int len)
{
    int len_to_return = 0;
    int read_size = 0;

    /* Check if USCI_B0 is busy */
    while (UCB0STATW & UCBUSY);

    /* Assert the CS pin */
    spi_assert_cs();

    /* Use DMA if size is above minimum to minimize delay, polling otherwise */
    if (len > DMA_BUFF_SIZE_MIN)
    {
        /* Repeat while more bytes are pending */
        while (len > 0)
        {
            if (len < DMA_MAX_TRANSACTION_SIZE)
            {
                dma_setup_write(&pBuff[read_size], len);
                read_size += len;
                len = 0;
            }
            else
            {
                dma_setup_write(&pBuff[read_size], DMA_MAX_TRANSACTION_SIZE);
                read_size += DMA_MAX_TRANSACTION_SIZE;
                len -= DMA_MAX_TRANSACTION_SIZE;
            }
        }

        /* Return bytes that have been transmitted */
        len_to_return = read_size;
    }
    else
    {
        /* Return bytes that have been transmitted */
        len_to_return = len;

        /* While there are bytes pending */
        while (len)
        {
            /* Wait while USCI_B0 is not transmitting */
            while (!(UCB0IFG & UCTXIFG));

            /* Put data in the USCI_B0 transmit buffer */
            UCB0TXBUF = *pBuff;

            /* Wait while USCI_B0 is not receiving */
            while (!(UCB0IFG & UCRXIFG));

            /* Read the USCI_B0 receive buffer */
            UCB0RXBUF;

            /* Decrease pending bytes */
            len --;

            /* Increase data pointer */
            pBuff++;
        }
    }

    /* De-assert the CS pin */
    spi_deassert_cs();

    return len_to_return;
}

/*----------------------------------------------------------------------------*/

int spi_Read(Fd_t fd, unsigned char *pBuff, int len)
{
    uint16_t i = 0;
    uint16_t read_size = 0;

    /* Check if USCI_B0 is busy */
    while (UCB0STATW & UCBUSY);

    /* Assert the CS pin */
    spi_assert_cs();

    /* Use DMA if size is above minimum to minimize delay, polling otherwise */
    if (len > DMA_BUFF_SIZE_MIN)
    {
        while (len > 0)
        {
            if (len < DMA_MAX_TRANSACTION_SIZE)
            {
                dma_setup_read(&pBuff[read_size], len);
                read_size += len;
                len = 0;
            }
            else
            {
                dma_setup_read(&pBuff[read_size], DMA_MAX_TRANSACTION_SIZE);
                read_size += DMA_MAX_TRANSACTION_SIZE;
                len -= DMA_MAX_TRANSACTION_SIZE;
            }
        }

        /* Return bytes that have been received */
        len = read_size;
    }
    else
    {
        /* Repeat for all bytes to be received */
        for (i = 0; i < len; i ++)
        {
            /* Wait while USCI_B0 is not transmitting */
            while (!(UCB0IFG & UCTXIFG));

            /* Put dummy data in the USCI_B0 transmit buffer */
            UCB0TXBUF = 0xFF;

            /* Wait while USCI_B0 is not receiving */
            while (!(UCB0IFG & UCRXIFG));

            /* Read the USCI_B0 receive buffer */
            pBuff[i] = UCB0RXBUF;
        }
    }

    /* De-assert the CS pin */
    spi_deassert_cs();

    return len;
}

/*----------------------------------------------------------------------------*/

static void spi_assert_cs(void)
{
    /* Put down the CS pin to active the SPI slave */
    MAP_GPIO_setOutputLowOnPin(SPI_CS_PORT, SPI_CS_PIN);
}

/*----------------------------------------------------------------------------*/

static void spi_deassert_cs(void)
{
    /* Put up the CS pin to deactivate the SPI slave */
    MAP_GPIO_setOutputHighOnPin(SPI_CS_PORT, SPI_CS_PIN);
}

/*----------------------------------------------------------------------------*/

static void dma_setup_write(unsigned char *pBuff, int len) {
    /* Configuring DMA module */
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable1);

    /* Assign DMA TX channel */
    MAP_DMA_assignChannel(DMA_TX_CHANNEL);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelControl(DMA_TX_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(DMA_TX_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC, pBuff, (void *) MAP_SPI_getTransmitBufferAddressForDMA(SPI_BASE), len);

    /* Enable DMA channel to start SPI transaction */
    MAP_DMA_enableChannel(DMA_TX_NUMBER);

    /* Wait until SPI transaction is complete */
    while(DMA_isChannelEnabled(DMA_TX_NUMBER))
       ;
}

/*----------------------------------------------------------------------------*/

static void dma_setup_read(unsigned char *pBuff, int len)
{
    uint8_t dummy = 0xFF;

    /* Configuring DMA module */
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable1);

    /* Assign DMA TX and RX channels */
    MAP_DMA_assignChannel(DMA_TX_CHANNEL);
    MAP_DMA_assignChannel(DMA_RX_CHANNEL);

    /* Setup the TX transfer characteristics & buffers */
    MAP_DMA_setChannelControl(DMA_TX_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_DST_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(DMA_TX_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC, &dummy, (void *) MAP_SPI_getTransmitBufferAddressForDMA(SPI_BASE), len);

    /* Setup the RX transfer characteristics & buffers */
    MAP_DMA_setChannelControl(DMA_RX_CHANNEL | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(DMA_RX_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *) MAP_SPI_getReceiveBufferAddressForDMA(SPI_BASE), pBuff, len);

    /* Enable DMA channels to start SPI transaction */
    MAP_DMA_enableChannel(DMA_RX_NUMBER);
    MAP_DMA_enableChannel(DMA_TX_NUMBER);

    /* Wait until SPI transaction is complete */
    while(DMA_isChannelEnabled(DMA_RX_NUMBER))
       ;
}
