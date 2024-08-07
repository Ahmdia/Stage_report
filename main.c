//###########################################################################
//
// FILE:   enet_lwip.c
//
// TITLE:  lwIP based Ethernet Example.
//
// Example to demonstrate UDP socket (for daikin customer)
// buf_rx,buf_tx are the watch variables which can be used or updated in the
// main application based on the requirement.
//
// Test setup
//
// F2838x Control Card connected to a PC/laptop  on the Ethernet port.
// PC/laptop runs the SocketTest/â€™Packet Senderâ€™ software, configured for the IP Address and port . \
// Keywords â€˜STARTâ€™ and â€˜STOPâ€™ are used to send and stop receiving messages respectively
// Received data is stored in â€˜buf_rxâ€™ array and data to be transmitted is stored in â€˜buf_txâ€™ array on CM core of F2838x device
// Examples is Interrupt based with a callback function â€˜udp_rx_callbackâ€™ which handles the received data from the SocketTest/Packet Sender software.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################


#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_emac.h"

#include "driverlib_cm/ethernet.h"
#include "driverlib_cm/gpio.h"
#include "driverlib_cm/interrupt.h"
#include "driverlib_cm/flash.h"

#include "driverlib_cm/sysctl.h"
#include "driverlib_cm/systick.h"

#include "utils/lwiplib.h"
#include "board_drivers/pinout.h"


#include "lwipopts.h"



#define PAYLOAD 1088

volatile uint32_t msTime=0;


volatile bool flag_TX_frame_UDP=true;


#define MAKE_IP_ADDRESS(a0,a1,a2,a3) (((a0<<24) & 0xFF000000) | ((a1<<16) & 0x00FF0000) | ((a2<<8)  & 0x0000FF00) | (a3 & 0x000000FF) )

bool Connected_udp_28000 = false;

uint16_t cnt_Connected_udp_28000 = 0;

//
// Defines
//
#define IPC_CMD_READ_MEM   0x1001
#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA

//
// Defines
//
#define PACKET_LENGTH 132

struct udp_pcb *g_upcb;



u8_t buf_rx[PAYLOAD];
uint32_t buf_tx[PAYLOAD];
char * buf_tx_start_msg = "Something to show UDP is working \n";
uint32_t buf_tx_start_msg_count = 35;

uint16_t cont_rx_udp = 0;

struct pbuf *pbuf1_tx;


//*****************************************************************************
//
//! \addtogroup master_example_list
//! <h1>Ethernet with lwIP (enet_lwip)</h1>
//!
//! This example application demonstrates the operation of the F2838x
//! microcontroller Ethernet controller using the lwIP TCP/IP Stack. Once
//! programmed, the device sits endlessly waiting for ICMP ping requests. It
//! has a static IP address. To ping the device, the sender has to be in the
//! same network. The stack also supports ARP.
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

// These are defined by the linker (see device linker command file)
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;

extern uint16_t constLoadStart;
extern uint16_t constLoadEnd;
extern uint16_t constLoadSize;
extern uint16_t constRunStart;
extern uint16_t constRunEnd;
extern uint16_t constRunSize;

#define DEVICE_FLASH_WAITSTATES 2

//*****************************************************************************
//
// Driver specific initialization code and macro.
//
//*****************************************************************************

#define ETHERNET_NO_OF_RX_PACKETS   2U
#define ETHERNET_MAX_PACKET_LENGTH 1538U

#define NUM_PACKET_DESC_RX_APPLICATION PBUF_POOL_SIZE //8 - same as PBUF_POOL_SIZE


Ethernet_Handle emac_handle;
Ethernet_InitConfig *pInitCfg;
uint32_t Ethernet_numRxCallbackCustom = 0;
uint32_t releaseTxCount = 0;
uint32_t genericISRCustomcount = 0;
uint32_t genericISRCustomRBUcount = 0;
uint32_t genericISRCustomROVcount = 0;
uint32_t genericISRCustomRIcount = 0;

uint32_t systickPeriodValue = 125000; //15000000;
Ethernet_Pkt_Desc  pktDescriptorRXCustom[NUM_PACKET_DESC_RX_APPLICATION];
extern uint32_t Ethernet_numGetPacketBufferCallback;
extern Ethernet_Device Ethernet_device_struct;
uint8_t Ethernet_rxBuffer[ETHERNET_NO_OF_RX_PACKETS *
                          ETHERNET_MAX_PACKET_LENGTH];

uint32_t sendPacketFailedCount = 0;

uint8_t mac_custom[6] = {0xA8, 0x63, 0xF2, 0x00, 0x1D, 0x98};

extern Ethernet_Pkt_Desc*
lwIPEthernetIntHandler(Ethernet_Pkt_Desc *pPacket);

void CM_init(void)
{
    //
    // Disable the watchdog
    //
    SysCtl_disableWatchdog();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    // Html pages are also being copied from flash to ram.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    memcpy(&constRunStart, &constLoadStart, (size_t)&constLoadSize);
    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    //
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    //
    // Sets the NVIC vector table offset address.
    //
#ifdef _FLASH
    Interrupt_setVectorTableOffset((uint32_t)vectorTableFlash);
#else
    Interrupt_setVectorTableOffset((uint32_t)vectorTableRAM);
#endif

}
//*****************************************************************************
//
// HTTP Webserver related callbacks and definitions.
//
//*****************************************************************************
//
// Currently, this implemented as a pointer to function which is called when
// corresponding query is received by the HTTP webserver daemon. When more
// features are needed to be added, it should be implemented as a separate
// interface.
//
void httpLEDToggle(void);
void(*ledtoggleFuncPtr)(void) = &httpLEDToggle;

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
 //   lwIPTimer(systickPeriodValue);
       lwIPTimer(1);
}

//*****************************************************************************
//
//  This function is a callback function called by the example to
//  get a Packet Buffer. Has to return a ETHERNET_Pkt_Desc Structure.
//  Rewrite this API for custom use case.
//
//*****************************************************************************
Ethernet_Pkt_Desc* Ethernet_getPacketBufferCustom(void)
{
    //
    // Get the next packet descriptor from the descriptor pool
    //
    uint32_t shortIndex = (Ethernet_numGetPacketBufferCallback + 3)
                % NUM_PACKET_DESC_RX_APPLICATION;

    //
    // Increment the book-keeping pointer which acts as a head pointer
    // to the circular array of packet descriptor pool.
    //
    Ethernet_numGetPacketBufferCallback++;

    //
    // Update buffer length information to the newly procured packet
    // descriptor.
    //
    pktDescriptorRXCustom[shortIndex].bufferLength =
                                  ETHERNET_MAX_PACKET_LENGTH;

    //
    // Update the receive buffer address in the packer descriptor.
    //
    pktDescriptorRXCustom[shortIndex].dataBuffer =
                                      &Ethernet_device_struct.rxBuffer [ \
               (ETHERNET_MAX_PACKET_LENGTH*Ethernet_device_struct.rxBuffIndex)];

    //
    // Update the receive buffer pool index.
    //
    Ethernet_device_struct.rxBuffIndex += 1U;
    Ethernet_device_struct.rxBuffIndex  = \
    (Ethernet_device_struct.rxBuffIndex%ETHERNET_NO_OF_RX_PACKETS);

    //
    // Receive buffer is usable from Address 0
    //
    pktDescriptorRXCustom[shortIndex].dataOffset = 0U;

    //
    // Return this new descriptor to the driver.
    //
    return (&(pktDescriptorRXCustom[shortIndex]));
}

//*****************************************************************************
//
//  This is a hook function and called by the driver when it receives a
//  packet. Application is expected to replenish the buffer after consuming it.
//  Has to return a ETHERNET_Pkt_Desc Structure.
//  Rewrite this API for custom use case.
//
//*****************************************************************************
Ethernet_Pkt_Desc* Ethernet_receivePacketCallbackCustom(
        Ethernet_Handle handleApplication,
        Ethernet_Pkt_Desc *pPacket)
{

    Ethernet_Pkt_Desc* temp_eth_pkt;
    //
    // Book-keeping to maintain number of callbacks received.
    //
#ifdef ETHERNET_DEBUG
    Ethernet_numRxCallbackCustom++;
#endif



      Ethernet_disableRxDMAReception(EMAC_BASE,0);


    //
    // This is a placeholder for Application specific handling
    // We are replenishing the buffer received with another buffer
    //
  //  return lwIPEthernetIntHandler(pPacket);

      temp_eth_pkt=lwIPEthernetIntHandler(pPacket);


      Ethernet_enableRxDMAReception(EMAC_BASE,0);

      return temp_eth_pkt;
}

void Ethernet_releaseTxPacketBufferCustom(
        Ethernet_Handle handleApplication,
        Ethernet_Pkt_Desc *pPacket)
{
    //
    // Once the packet is sent, reuse the packet memory to avoid
    // memory leaks. Call this interrupt handler function which will take care
    // of freeing the memory used by the packet descriptor.
    //
    lwIPEthernetIntHandler(pPacket);

    //
    // Increment the book-keeping counter.
    //
#ifdef ETHERNET_DEBUG
    releaseTxCount++;
#endif
}

Ethernet_Pkt_Desc *Ethernet_performPopOnPacketQueueCustom(
            Ethernet_PKT_Queue_T *pktQueuePtr)
{
    Ethernet_Pkt_Desc *pktDescHdrPtr;

    pktDescHdrPtr = pktQueuePtr->head;

    if(0U != pktDescHdrPtr)
    {
        pktQueuePtr->head = pktDescHdrPtr->nextPacketDesc;
        pktQueuePtr->count--;
    }

    return(pktDescHdrPtr);
}
void Ethernet_performPushOnPacketQueueCustom(
        Ethernet_PKT_Queue_T *pktQueuePtr,
        Ethernet_Pkt_Desc *pktDescHdrPtr)
{
    pktDescHdrPtr->nextPacketDesc = 0U;

    if(0U == pktQueuePtr->head)
    {
        //
        // Queue is empty - Initialize it with this one packet
        //
        pktQueuePtr->head = pktDescHdrPtr;
        pktQueuePtr->tail = pktDescHdrPtr;
    }
    else
    {
        //
        // Queue is not empty - Push onto END
        //
        pktQueuePtr->tail->nextPacketDesc = pktDescHdrPtr;
        pktQueuePtr->tail        = pktDescHdrPtr;
    }
    pktQueuePtr->count++;
}
void Ethernet_setMACConfigurationCustom(uint32_t base, uint32_t flags)
{
    HWREG(base + ETHERNET_O_MAC_CONFIGURATION) |= flags;
}
void Ethernet_clearMACConfigurationCustom(uint32_t base, uint32_t flags)
{
    HWREG(base + ETHERNET_O_MAC_CONFIGURATION) &= ~flags;

}

interrupt void Ethernet_genericISRCustom(void)
{
    genericISRCustomcount++;
    Ethernet_RxChDesc *rxChan;
    Ethernet_TxChDesc *txChan;
    Ethernet_HW_descriptor    *descPtr;
    Ethernet_HW_descriptor    *tailPtr;
    uint16_t i=0;
    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_TE);
    for(i = 0U;i < Ethernet_device_struct.initConfig.numChannels;i++)
     {
         Ethernet_disableRxDMAReception(
               Ethernet_device_struct.baseAddresses.enet_base,
               i);
     }
    if(((ETHERNET_DMA_CH0_STATUS_AIS |
                         ETHERNET_DMA_CH0_STATUS_RBU) ==
                       (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                              ETHERNET_O_DMA_CH0_STATUS) &
                              (uint32_t)(ETHERNET_DMA_CH0_STATUS_AIS |
                                         ETHERNET_DMA_CH0_STATUS_RBU))) ||
          (ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS) ==
                                   (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                          ETHERNET_O_MTL_Q0_INTERRUPT_CONTROL_STATUS) &
                                          (uint32_t)(ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS
                                                     )))
      {
          if((ETHERNET_DMA_CH0_STATUS_AIS |
                             ETHERNET_DMA_CH0_STATUS_RBU) ==
                           (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                  ETHERNET_O_DMA_CH0_STATUS) &
                                  (uint32_t)(ETHERNET_DMA_CH0_STATUS_AIS |
                                             ETHERNET_DMA_CH0_STATUS_RBU)))
          {
          genericISRCustomRBUcount++;
          }
          if((ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS) ==
                  (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                         ETHERNET_O_MTL_Q0_INTERRUPT_CONTROL_STATUS) &
                         (uint32_t)(ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS
                                    )))
          {
              genericISRCustomROVcount++;
              Ethernet_enableMTLInterrupt(Ethernet_device_struct.baseAddresses.enet_base,0,
                                          ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS);
          }

        /*
             * Clear the AIS and RBU status bit. These MUST be
             * cleared together!
             */
            Ethernet_clearDMAChannelInterrupt(
                    Ethernet_device_struct.baseAddresses.enet_base,
                    ETHERNET_DMA_CHANNEL_NUM_0,
                    ETHERNET_DMA_CH0_STATUS_AIS |
                    ETHERNET_DMA_CH0_STATUS_RBU);

            /*
           *Recover from Receive Buffer Unavailable (and hung DMA)
         *
         * All descriptor buffers are owned by the application, and
         * in result the DMA cannot transfer incoming frames to the
         * buffers (RBU condition). DMA has also entered suspend
         * mode at this point, too.
         *
         * Drain the RX queues
         */

            /* Upon RBU error, discard all previously received packets */
            if(Ethernet_device_struct.initConfig.pfcbDeletePackets != NULL)
                (*Ethernet_device_struct.initConfig.pfcbDeletePackets)();

            rxChan =
               &Ethernet_device_struct.dmaObj.rxDma[ETHERNET_DMA_CHANNEL_NUM_0];
            txChan=
               &Ethernet_device_struct.dmaObj.txDma[ETHERNET_DMA_CHANNEL_NUM_0];

    /*
     * Need to disable multiple interrupts, so protect the code to do so within
     * a global disable block (to prevent getting interrupted in between)
     */

            if(NULL!= Ethernet_device_struct.ptrPlatformInterruptDisable)
            {
                (*Ethernet_device_struct.ptrPlatformInterruptDisable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_RX_INTR_CH0 + rxChan->chInfo->chNum]);

                (*Ethernet_device_struct.ptrPlatformInterruptDisable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_GENERIC_INTERRUPT]);
            }
            /* verify we have full capacity in the descriptor queue */
            if(rxChan->descQueue.count < rxChan->descMax) {
              /* The queue is not at full capacity due to OOM errors.
              Try to fill it again */
                Ethernet_addPacketsIntoRxQueue(rxChan);
            }
            Ethernet_initRxChannel(
                    &Ethernet_device_struct.initConfig.chInfo[ETHERNET_CH_DIR_RX][0]);

            Ethernet_writeRxDescTailPointer(
                Ethernet_device_struct.baseAddresses.enet_base,
                0,
                (&Ethernet_device_struct.rxDesc[
                 ((uint32_t)ETHERNET_DESCRIPTORS_NUM_RX_PER_CHANNEL) *
                  (0 + (uint32_t)1U)]));

            if(NULL!= Ethernet_device_struct.ptrPlatformInterruptEnable)
            {
                (*Ethernet_device_struct.ptrPlatformInterruptEnable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_RX_INTR_CH0 + rxChan->chInfo->chNum]);
                (*Ethernet_device_struct.ptrPlatformInterruptEnable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_GENERIC_INTERRUPT]);
            }


    }
    if(0U != (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                 ETHERNET_O_DMA_CH0_STATUS) &
                           (uint32_t) ETHERNET_DMA_CH0_STATUS_RI))
    {
        genericISRCustomRIcount++;
        Ethernet_clearDMAChannelInterrupt(
                        Ethernet_device_struct.baseAddresses.enet_base,
                        ETHERNET_DMA_CHANNEL_NUM_0,
                        ETHERNET_DMA_CH0_STATUS_NIS | ETHERNET_DMA_CH0_STATUS_RI);
    }

    for(i = 0U;i < Ethernet_device_struct.initConfig.numChannels;i++)
     {
         Ethernet_enableRxDMAReception(
               Ethernet_device_struct.baseAddresses.enet_base,
               i);
     }
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_TE);
}

void
Ethernet_init(const unsigned char *mac)
{
    Ethernet_InitInterfaceConfig initInterfaceConfig;
    uint32_t macLower;
    uint32_t macHigher;
    uint8_t *temp;

    initInterfaceConfig.ssbase = EMAC_SS_BASE;
    initInterfaceConfig.enet_base = EMAC_BASE;
    initInterfaceConfig.phyMode = ETHERNET_SS_PHY_INTF_SEL_MII;

    //
    // Assign SoC specific functions for Enabling,Disabling interrupts
    // and for enabling the Peripheral at system level
    //
    initInterfaceConfig.ptrPlatformInterruptDisable =
                                                    &Platform_disableInterrupt;
    initInterfaceConfig.ptrPlatformInterruptEnable =
                                                     &Platform_enableInterrupt;
    initInterfaceConfig.ptrPlatformPeripheralEnable =
                                                    &Platform_enablePeripheral;
    initInterfaceConfig.ptrPlatformPeripheralReset =
                                                     &Platform_resetPeripheral;

    //
    // Assign the peripheral number at the SoC
    //
    initInterfaceConfig.peripheralNum = SYSCTL_PERIPH_CLK_ENET;

    //
    // Assign the default SoC specific interrupt numbers of Ethernet interrupts
    //
    initInterfaceConfig.interruptNum[0] = INT_EMAC;
    initInterfaceConfig.interruptNum[1] = INT_EMAC_TX0;
    initInterfaceConfig.interruptNum[2] = INT_EMAC_TX1;
    initInterfaceConfig.interruptNum[3] = INT_EMAC_RX0;
    initInterfaceConfig.interruptNum[4] = INT_EMAC_RX1;

    pInitCfg = Ethernet_initInterface(initInterfaceConfig);

    Ethernet_getInitConfig(pInitCfg);
    pInitCfg->dmaMode.InterruptMode = ETHERNET_DMA_MODE_INTM_MODE2;

    //
    // Assign the callbacks for Getting packet buffer when needed
    // Releasing the TxPacketBuffer on Transmit interrupt callbacks
    // Receive packet callback on Receive packet completion interrupt
    //
    pInitCfg->pfcbRxPacket = &Ethernet_receivePacketCallbackCustom;
    pInitCfg->pfcbGetPacket = &Ethernet_getPacketBuffer;    //custom
    pInitCfg->pfcbFreePacket = &Ethernet_releaseTxPacketBufferCustom;

    //
    //Assign the Buffer to be used by the Low level driver for receiving
    //Packets. This should be accessible by the Ethernet DMA
    //
    pInitCfg->rxBuffer = Ethernet_rxBuffer;

    //
    // The Application handle is not used by this application
    // Hence using a dummy value of 1
    //
    Ethernet_getHandle((Ethernet_Handle)1, pInitCfg , &emac_handle);

    //
    // Disable transmit buffer unavailable and normal interrupt which
    // are enabled by default in Ethernet_getHandle.
    //
    Ethernet_disableDmaInterrupt(Ethernet_device_struct.baseAddresses.enet_base,
                                 0, (ETHERNET_DMA_CH0_INTERRUPT_ENABLE_TBUE |
                                     ETHERNET_DMA_CH0_INTERRUPT_ENABLE_NIE));

    //
    // Enable the MTL interrupt to service the receive FIFO overflow
    // condition in the Ethernet module.
    //
    Ethernet_enableMTLInterrupt(Ethernet_device_struct.baseAddresses.enet_base,0,
                                ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOIE);

    //
    // Disable the MAC Management counter interrupts as they are not used
    // in this application.
    //
    HWREG(Ethernet_device_struct.baseAddresses.enet_base + ETHERNET_O_MMC_RX_INTERRUPT_MASK) = 0xFFFFFFFF;
    HWREG(Ethernet_device_struct.baseAddresses.enet_base + ETHERNET_O_MMC_IPC_RX_INTERRUPT_MASK) = 0xFFFFFFFF;
    HWREG(Ethernet_device_struct.baseAddresses.enet_base + ETHERNET_O_MMC_TX_INTERRUPT_MASK) = 0xFFFFFFFF;
    //
    //Do global Interrupt Enable
    //
    (void)Interrupt_enableInProcessor();

    //
    //Assign default ISRs
    //
    Interrupt_registerHandler(INT_EMAC_TX0, Ethernet_transmitISR);
    Interrupt_registerHandler(INT_EMAC_RX0, Ethernet_receiveISR);
    Interrupt_registerHandler(INT_EMAC, Ethernet_genericISRCustom);

    //
    // Convert the mac address string into the 32/16 split variables format
    // that is required by the driver to program into hardware registers.
    // Note: This step is done after the Ethernet_getHandle function because
    //       a dummy MAC address is programmed in that function.
    //
    temp = (uint8_t *)&macLower;
    temp[0] = mac[0];
    temp[1] = mac[1];
    temp[2] = mac[2];
    temp[3] = mac[3];

    temp = (uint8_t *)&macHigher;
    temp[0] = mac[4];
    temp[1] = mac[5];

    //
    // Program the unicast mac address.
    //
    Ethernet_setMACAddr(EMAC_BASE,
                        0,
                        macHigher,
                        macLower,
                        ETHERNET_CHANNEL_0);

    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
}




void udp_rx_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port)

{
    char *cad_rx;
    uint16_t long_actual = 0;
    uint16_t long_UDP_complete = 0;
    uint16_t long_total = 0;
    uint8_t cnt_lee = 0;

    httpLEDToggle();


    memset(buf_rx, 0x00, 50);


    long_total = p->tot_len;

    cont_rx_udp++;



    while ((long_UDP_complete < long_total)
            && (long_UDP_complete < 1800) || (cnt_lee == 0))
    {
        cnt_lee++;
        long_actual = p->len;

        cad_rx = p->payload;

        int i = 0;
        for (i = 0; i < long_actual; i++)
        {

            buf_rx[0 + i + long_UDP_complete] = *cad_rx++;
        }
        long_UDP_complete = long_actual + long_UDP_complete;
        buf_rx[long_UDP_complete + 1] = '\n';

        if (long_UDP_complete == long_total)
        {
            pbuf_free(p); /* don't leak the pbuf!*/
        }
        else
        {
            if ( p->next != NULL)
            p = p->next;
            //  pbuf_free(a);                                   /* don't leak the pbuf!*/
        }

        if ((buf_rx[0] == 'S') && (buf_rx[1] == 'T') && (buf_rx[2] == 'O') && (buf_rx[3] == 'P'))
        {                    //Disconnect
                             //udp_disconnect(upcb);

            Connected_udp_28000 = false;
            cnt_Connected_udp_28000 = 0;
        }
        else if ((buf_rx[0] == 'S') && (buf_rx[1] == 'T') && (buf_rx[2] == 'A') && (buf_rx[3] == 'R') && (buf_rx[4] == 'T'))
        {                //Connect

                    Connected_udp_28000 = true;
                    cnt_Connected_udp_28000 = 0;

                   /* process the payload in p->payload */
                   udp_connect(upcb, addr, port); /* connect to the remote host */

        }
    }

    pbuf_free(p);

    if (!Connected_udp_28000)
    {
        udp_disconnect(upcb);
    }

}

/* UDP initialization ......................................................*/
void my_udp_init(void)
{

    g_upcb = udp_new();
    udp_bind(g_upcb, IP_ADDR_ANY, 28000);
    udp_recv(g_upcb, &udp_rx_callback, (void*) 0);


}


//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************

unsigned long IPAddr =  0xC0A80004; // 0xC0A80004; //192.168.0.4
unsigned long NetMask = 0xFFFFFF00;
unsigned long GWAddr = 0xC0A8000A;

#define IPC_DATA_SIZE 100 // Taille des données transférées via IPC
#define DATALOAD 500
uint32_t buf_kx[DATALOAD];

int
main(void)
{
    unsigned long ulUser0, ulUser1;
    unsigned char pucMACArray[8];

    int i=0;


    //  ////////////////////////////////////////
    // Initializing the CM. Loading the required functions to SRAM.
    //
    CM_init();

    SYSTICK_setPeriod(systickPeriodValue);
    SYSTICK_enableCounter();
    SYSTICK_registerInterruptHandler(SysTickIntHandler);
    SYSTICK_enableInterrupt();

    //
    // Enable processor interrupts.
    //
    Interrupt_enableInProcessor();

    // Set user/company specific MAC octets
    // (for this code we are using A8-63-F2-00-00-80)
    // 0x00 MACOCT3 MACOCT2 MACOCT1
    ulUser0 = 0x00F263A8;

    // 0x00 MACOCT6 MACOCT5 MACOCT4
    ulUser1 = 0x00800000;

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //
    pucMACArray[0] = ((ulUser0 >>  0) & 0xff);
    pucMACArray[1] = ((ulUser0 >>  8) & 0xff);
    pucMACArray[2] = ((ulUser0 >> 16) & 0xff);
    pucMACArray[3] = ((ulUser1 >>  0) & 0xff);
    pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
    pucMACArray[5] = ((ulUser1 >> 16) & 0xff);

    //
    // Initialize ethernet module.
    //
    Ethernet_init(pucMACArray);

    //
    // Initialze the lwIP library, using DHCP.
    //
    lwIPInit(0, pucMACArray, IPAddr, NetMask, GWAddr, IPADDR_USE_STATIC);

    // Initialize the UDP server
    //
    my_udp_init();


    //
    // Loop forever. All the work is done in interrupt handlers.
    //
     uint32_t cnt_FFT = 0;

    Interrupt_setPriority(INT_EMAC_TX0, 2);
    Interrupt_setPriority(INT_EMAC_RX0, 1);
    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);
    Interrupt_enable(INT_EMAC);

    IPC_init(IPC_CM_L_CPU1_R);

    IPC_sync(IPC_CM_L_CPU1_R, IPC_FLAG31);

    volatile int cnt_total_loop=0;
    volatile int cnt_total_mqtt_pub=0;
    int j=0 ;

    while (1)
    {
        if (IPC_isFlagBusyRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0))
            {
                uint32_t cmd, addr, data;

                IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG0,IPC_ADDR_CORRECTION_DISABLE, &cmd, &addr, &data);
                if (cmd == IPC_CMD_READ_MEM)
                {

                 /*   uint32_t* source = (uint32_t*)addr;
                   memcpy(buf_kx, source, DATALOAD * sizeof(buf_kx));*/
                    memcpy(&buf_kx[j], &addr,sizeof(data));
                    j++;
                }

               IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);


                if(j>=DATALOAD){j=0;}
            }


        cnt_total_loop++;

        if (Connected_udp_28000)
        {
            //UDP TX block

            if (flag_TX_frame_UDP)
            {
                cnt_FFT++;

                //flag_TX_frame_UDP = false;

                /////////////////////////////////////////////
                for (i = 0; i < PAYLOAD; i++)
                {
                    if(i < buf_tx_start_msg_count)
                    {
                        buf_tx[i] = (uint8_t)buf_tx_start_msg[i];
                    }
                    else if (i - buf_tx_start_msg_count < sizeof(buf_kx)) {
                      buf_tx[i] = buf_kx[i - buf_tx_start_msg_count];
                   } else {
                   buf_tx[i] = 0; // Valeur par défaut si les données reçues sont plus petites que prévu
                                                                     }

                }

                ////////////////////////////////////////////

                pbuf1_tx = pbuf_alloc(PBUF_TRANSPORT, PAYLOAD, PBUF_RAM);
                if (pbuf1_tx!= NULL)
                {
                pbuf1_tx->payload = (void*) buf_tx;
                pbuf1_tx->tot_len = PAYLOAD;  //17        // long_UDP_complete+4;
                pbuf1_tx->len = PAYLOAD;   //17       // long_UDP_complete+4;

                    udp_send(g_upcb, pbuf1_tx);

                }


                cnt_Connected_udp_28000++;


                if (pbuf1_tx!= NULL)
                pbuf_free(pbuf1_tx);

            }

        }


        SysCtl_delay(6000000);//aprox 1 sec

        sys_check_timeouts();

    }
}

//*****************************************************************************
//
// Called by lwIP Library. Toggles the led when a command is received by the
// HTTP webserver.
//
//*****************************************************************************
void httpLEDToggle(void)
{
    //
    // Toggle the LED D1 on the control card.
    //
    GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
}


//*****************************************************************************
//
// Called by lwIP Library. Could be used for periodic custom tasks.
//
//*****************************************************************************

uint32_t cnt_ms_lwip_Htimer=0;
uint32_t cnt_ms_TX_Htimer=0;
void lwIPHostTimerHandler(void)
{
//  msTime++;

    cnt_ms_lwip_Htimer++;
}
/*  int j=0;
                   // Stocker les données dans buf_tx
                   buf_kx[j]= data;
                   j++;*/
                //   memcpy(&buf_kx[j], &addr,sizeof(buf_kx));
                //   memcpy(&buf_kx, (uint32_t *)&addr, sizeof(buf_kx));
                /*   for(j=0;j<PAYLOAD;j++)
                   {
                       memcpy(&buf_kx, *((uint32_t *)addr+j) , sizeof(data));
                   }*/
                  // j++;
