//#############################################################################
//
// FILE:   cm_common_config_c28x.c
//
// TITLE:  C28x Common Configurations to be used for the CM Side.
//
//! \addtogroup driver_example_list
//! <h1>C28x Common Configurations</h1>
//!
//! This example configures the GPIOs and Allocates the shared peripherals
//! according to the defines selected by the users.
//!
//
//#############################################################################
//
//
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
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "c2000ware_libraries.h"
#include "math.h"
#include "lab_shared.h"
#include "board.h"

uint16_t   Shift=0;
uint16_t   Base_periode = 500;
uint16_t   Angle=0;

int32_t  Test;
int32_t  Test1;
int32_t  Test3=0;
int32_t   Shift_ADC=0;
int32_t   Base_origine = 2048;
int32_t   Angle_ADC=0;


float  Udc_pr;
float  Idc_pr;
float  Udc_sec;
float  Idc_sec;

uint16_t  Test4=0;
uint16_t  Test5=0;


uint16_t Number=0;
uint16_t Somme_x=0;
uint16_t Somme_y=0;
uint16_t Somme_produit ;
uint16_t Somme_carre;
uint16_t i;
uint16_t j;
uint16_t k=0;
uint16_t buffer_index;
bool send=false;

float buffer_udc_primary[32][2];
float buffer_Idc_primary[32][2];
float buffer_udc_secondary[32][2];
float buffer_Idc_secondary[32][2];

volatile uint16_t buffer_ADC_Based_Cla[5][100];
volatile uint16_t buffer_ADC_Based_CPU[5][100];
volatile uint16_t temp[500];

float A_Idr_pr =   0.023559919f ;
float B_Idr_pr = -44.42353565f;

float A_Idr_sec =   0.02166155f ;
float B_Idr_sec = -43.65283635f;

float A_Udr_pr =   0.249326597f ;
float B_Udr_pr = -57.13262122f;

float A_Udr_sec =   0.243145165f ;
float B_Udr_sec = -57.29941968f;


uint16_t Udc_prmax;
uint16_t Idc_prmax;
uint16_t Udc_secmax;
uint16_t Idc_secmax;
uint16_t avg_idc_prim;


#define BUF_BITS    7                           // Buffer bits <= 16.
#define BUF_LEN     (1 << BUF_BITS)             // Buffer length.
#define BUF_MASK    ((uint16_t)(BUF_LEN - 1))   // Buffer mask.

float       ClaBuf[BUF_LEN];                    // Buffer to store filtered samples.
float       AdcBuf[BUF_LEN];                    // Buffer for un-filtered samples.
uint16_t    ClaBufIdx   = 0;                    // Buffer index for ClaBuf.
uint16_t    AdcBufIdx   = 0;                    // Buffer index for AdcBufIdx.
uint16_t    LedCtr      = 0;


#pragma DATA_SECTION(filter_out,"Cla1ToCpuMsgRAM");
float filter_out;

#pragma DATA_SECTION(filter_in,"Cla1ToCpuMsgRAM");
float filter_in;

//**************************************TPC**********************************//
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001

#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA


#pragma DATA_SECTION(readData, "MSGRAM_CPU_TO_CM")
uint16_t readData;

uint16_t pass;

//********************************************************************//



//
//ADC_Interupt
//


//
// Main
//

void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    // Initialize the PIE module and vector table.
      Interrupt_initModule();
      Interrupt_initVectorTable();

      Board_init();

    //
    // Boot CM core
    //
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

#ifdef ETHERNET
    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_110_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(108, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(108, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(108,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(119, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(119, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(119,1);
#endif

#ifdef MCAN
    //
    // Setting the MCAN Clock.
    //
    SysCtl_setMCANClk(SYSCTL_MCANCLK_DIV_4);

    //
    // Configuring the GPIOs for MCAN.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_MCANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_MCANTXA);
#endif

#ifdef CANA
    //
    // Configuring the GPIOs for CAN A.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);

    //
    // Allocate Shared Peripheral CAN A to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_CAN_A,0x1U);
#endif

#ifdef CANB
    //
    // Configuring the GPIOs for CAN B.
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

    //
    // Allocate Shared Peripheral CAN B to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_CAN_B,0x1U);
#endif

#ifdef UART
    //
    // Configure GPIO85 as the UART Rx pin.
    //
    GPIO_setPinConfig(GPIO_85_UARTA_RX);
    GPIO_setDirectionMode(85, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(85, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(85, GPIO_QUAL_ASYNC);

    //
    // Configure GPIO84 as the UART Tx pin.
    //
    GPIO_setPinConfig(GPIO_84_UARTA_TX);
    GPIO_setDirectionMode(84, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(84, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(84, GPIO_QUAL_ASYNC);
#endif

#ifdef USB
#ifdef USE_20MHZ_XTAL
    //
    // Set up the auxiliary PLL so a 60 MHz output clock is provided to the USB module.
    // This fixed frequency is required for all USB operations.
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(48) |
                       SYSCTL_REFDIV(2U) | SYSCTL_ODIV(4U) |
                       SYSCTL_AUXPLL_DIV_2 |
                       SYSCTL_AUXPLL_ENABLE |
                       SYSCTL_DCC_BASE_0);
#else
    //
    // Set up the auxiliary PLL so a 60 MHz output clock is provided to the USB module.
    // This fixed frequency is required for all USB operations.
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(48) |
                       SYSCTL_REFDIV(2U) | SYSCTL_ODIV(5U) |
                       SYSCTL_AUXPLL_DIV_2 |
                       SYSCTL_AUXPLL_ENABLE |
                       SYSCTL_DCC_BASE_0);
#endif

    //
    // Allocate Shared Peripheral USB to the CM Side.
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_USBA, 1);

    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(0, GPIO_CORE_CM);

    //
    // Set the master core of GPIOs to CM.
    //
    GPIO_setMasterCore(42, GPIO_CORE_CM);
    GPIO_setMasterCore(43, GPIO_CORE_CM);
    GPIO_setMasterCore(46, GPIO_CORE_CM);
    GPIO_setMasterCore(47, GPIO_CORE_CM);
    GPIO_setMasterCore(120, GPIO_CORE_CM);
    GPIO_setMasterCore(121, GPIO_CORE_CM);

    //
    // Set the USB DM and DP GPIOs.
    //
    GPIO_setAnalogMode(42, GPIO_ANALOG_ENABLED);
    GPIO_setAnalogMode(43, GPIO_ANALOG_ENABLED);

    //
    // Set the direction for VBUS and ID.
    //
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);

    //
    // Configure the Power Fault.
    //
    GPIO_setMasterCore(120, GPIO_CORE_CM);
    GPIO_setDirectionMode(120, GPIO_DIR_MODE_IN);

    //
    // Configure the External Power Signal Enable.
    //
    GPIO_setMasterCore(121, GPIO_CORE_CM);
	GPIO_setDirectionMode(121, GPIO_DIR_MODE_OUT);
	GPIO_writePin(121, 1);

    //
    // Set the CM Clock to run at 120MHz.
    // The CM Clock is a fractional multiple of the AUXPLL Clock (120 Mhz) from
    // which the USB Clock (60 MHz) is derived.
    //
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_1, SYSCTL_SOURCE_AUXPLL);
#endif

    // Enable global interrupts.
       EINT;

       // Enable real-time debug.
       ERTM;

       IPC_init(IPC_CPU1_L_CM_R);

              IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);

       for(;;) {
               //       NOP; Do nothing.
                   if(buffer_index==100) {
                        buffer_index=0;

                        for(i=0; i<5; i++)
                                          {
                                          for(j=0; j<100; j++)
                                           {
                                              buffer_ADC_Based_CPU[i][j]= buffer_ADC_Based_Cla[i][j];
                                                      if(send==true){
                                                          k=0;
                                                            for(i=0; i<5; i++)
                                                               {
                                                               for(j=0; j<100; j++)
                                                                    {
                                                             temp[k]=buffer_ADC_Based_CPU[i][j];
                                                             k++;
                                                                    }
                                                                }

                                                            send=false;
                                                            k=0;
                                                               }

                                                 }
                                          }


                        readData = temp[k];
                      if (!IPC_isFlagBusyLtoR(IPC_CPU1_L_CM_R, IPC_FLAG0)) {
                             IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_DISABLE,
                              IPC_CMD_READ_MEM, IPC_Instance[IPC_CPU1_L_CM_R].IPC_SendCmd_Reg->IPC_SENDDATA , readData);
                             k++;

                             if(k==500){ send=true;  k=0; }
                                             }
                         while(IPC_isFlagBusyRtoL(IPC_CPU1_L_CM_R, IPC_FLAG0)) {
                                         // Boucle d'attente
                                                                      }
                           IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG0);


                    }

               Test5 ++;
           }

}



__interrupt void cla1Isr1(void)
{

      Somme_x=0;
          Somme_y=0;
          Number=0;
    Test3++;

    if(buffer_index<100)
     {

                 buffer_ADC_Based_Cla[0][buffer_index]=buffer_index;
                 buffer_ADC_Based_Cla[1][buffer_index]=A_Udr_pr*(float)ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC2) + B_Udr_pr;
                 buffer_ADC_Based_Cla[2][buffer_index]= A_Idr_pr*(float)ADC_readResult(myADC1_RESULT_BASE,myADC1_SOC2) + B_Idr_pr;
                 buffer_ADC_Based_Cla[3][buffer_index]=  A_Udr_pr*(float)ADC_readResult(myADC2_RESULT_BASE,myADC2_SOC2) + B_Udr_sec;
                 buffer_ADC_Based_Cla[4][buffer_index]=     A_Idr_sec*(float)ADC_readResult(myADC3_RESULT_BASE,myADC3_SOC2) + B_Idr_sec;
                 buffer_index++;
                 /*
                 buffer_ADC_Based_Cla[1][buffer_index]=ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC2);
                 buffer_ADC_Based_Cla[2][buffer_index]=ADC_readResult(myADC1_RESULT_BASE,myADC1_SOC2);
                 buffer_ADC_Based_Cla[3][buffer_index]=ADC_readResult(myADC2_RESULT_BASE,myADC2_SOC2);
                 buffer_ADC_Based_Cla[4][buffer_index]=ADC_readResult(myADC3_RESULT_BASE,myADC3_SOC2);
*/

     }
    ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
        Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);


        // Store raw ADC sample in AdcBuf.
           AdcBuf[AdcBufIdx++] = filter_in;
           AdcBufIdx &= BUF_MASK;

           // Store filtered output in ClaBuf.
           ClaBuf[ClaBufIdx++] = filter_out;
           ClaBufIdx &= BUF_MASK;

           // Toggle LED1 at a rate of 1Hz.
           if (LedCtr++ >= 8000) {
              GPIO_togglePin(myBoardLED0_GPIO);

               LedCtr = 0;
           }

}

__interrupt void INT_myADC0_1_ISR(void)
{

    //  volatile uint16_t  PieCtrlRegs  = PIECTRL_BASE;


    Test1++;
    // Test= ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5 );
       Test = ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC5);
/*
   if(Test<=2048)
        {
               if(Test==2048)
               {
                   Angle_ADC=180;

               }else{
                   Angle_ADC=(Test*180)/2048;

               }
               Shift_ADC=(Base_origine*Angle_ADC)/180 ;

        }else  if(Test>2048)
        {
            Angle_ADC=((Test)*180)/2048;
            Shift_ADC=(Base_origine*Angle_ADC)/180 ;
        }
*/
    if(Test<=2048)
           {
        Shift_ADC=250;
           }
    else
           {
        Shift_ADC=-250;
           }
    Shift_ADC+=(Test*500)/4096;

     EPWM_setPhaseShift(myEPWM2_BASE, Shift_ADC);



       ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
    //   Interrupt_clearACKGroup(ADC_INT_NUMBER3);
        Interrupt_clearACKGroup(INT_myADC0_1_INTERRUPT_ACK_GROUP);
      //  Test1++;
}

__interrupt void INT_Phase_shift_ISR(void)
{
    Angle++;
   Shift=-(Base_periode*Angle)/360 ;

   //  EPWM_setPhaseShift(myEPWM1_BASE, Shift);
   if(Angle==180)
   {
       Angle=0;
   }
   Interrupt_clearACKGroup(INT_Phase_shift_INTERRUPT_ACK_GROUP);
}
/*
 *
 */

