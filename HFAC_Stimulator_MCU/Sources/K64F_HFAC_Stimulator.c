/*
 * K64F_HFAC_Stimulator.c
 *
 *  Created on: 21 Jan 2019
 *      Author: Adrien Rapeaux
 *
 *  Licensing:
 *  This file is licensed under the GNU GPL v3 license.
 *  You can access the license at https://www.gnu.org/licenses/gpl-3.0.html
 */


// Define SEMIHOSTING in preprocessor variables to enable semihosting (only recommended for debug)
#ifdef SEMIHOSTING
    #include "stdio.h" //This allows printing to/receiving input from the console during debugging
#endif


#include "fsl_gpio.h" //GPIO drivers to toggle the LEDs
#include "fsl_dspi.h" //SPI drivers
#include "fsl_uart.h" //UART drivers
#include "fsl_edma.h" //eDMA driver
#include "fsl_dmamux.h" //DMA Trigger Source Multiplexer driver
#include "fsl_pit.h" //Periodic Interrupt Timer driver
#include "fsl_ftm.h" //Flex Timer Module driver

#include "../Sources/board.h"
#include "../Sources/clock_config.h"
#include "../Sources/pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk)/4)
#define TCD_QUEUE_SIZE 1
#define STIMULATOR_NUMBER_OF_CHANNELS 4
#define SPI_TRANSFER_BAUDRATE 4000000U
  //SPI baudrate currently at 4 Mbaud
#define SPI_DUMMYCOMMAND 0x0000BEEF
#define STIM_PROTOCOL_COUNTDOWN_USEC 50U
  //hardcoded delay between connection of stimulator output to electrodes and start of stimulation command and actual start of stimulation

//The following defines are to set the bitshifts for the SPI command sent in the two daisychained SPI switches
//The numbers are defined by hardware connections between the switch inputs and outputs
//Currently switch0-0 and switch0-1 are connected at input to channel 0. Outputs to dummy and electrode respectively
//Same for switch0-2 to switch0-7 for channels 1-3/
//Switch1-0 and switch1-1 inputs (route to dummy) is connected to switch0-0,2,4,6 output. Outputs are resistive and resistive-capacitive dummy loads
//Switch1-2 and Switch1-3 inputs are connected to Switch1-0 and Switch1-1 outputs (measure dummy load voltage)
//Switch1-4 through to Switch1-7 are connected to electrode outputs connected to Switch0 outputs (measure electrode voltage)
//Switch1-2 through to Switch1-7 outputs are all connected together to the calibration circuit (and so only one input may be connected at a time)
#define SPISWITCH_CHAN0_BITADJUST 0
#define SPISWITCH_CHAN1_BITADJUST 2
#define SPISWITCH_CHAN2_BITADJUST 4
#define SPISWITCH_CHAN3_BITADJUST 6
#define SPISWITCH_DUMMY_BITADJUST 8
#define SPISWITCH_CALIB_BITADJUST 8
#define SPISWITCH_CALIBRATIONSRC_BITADJUST 14
  //Due to how enum calib is built, the first value after 0 is 2 not 1, therefore even if 10 is expected here it should be 9

//GPIO Special functions
//Digital Isolator Power on PORTA
#define BOARD_DIGISO_PWR GPIOA
#define SPIDIGISO_PWR_PIN 1u
#define UARTDIGISO_PWR_PIN 2u


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
edma_handle_t g_EDMA_Handle[STIMULATOR_NUMBER_OF_CHANNELS*3+2];

uint32_t timerPeriods[2] = {0};
uint32_t spiDummyArray[10] = {0};
uint8_t DMADummyTransfer[32000] = {0};
volatile uint32_t debugValue = 0;

volatile uint32_t test;
volatile uint32_t test2;

//Enum definitions
enum route {channelRoute_off = 0, channelRoute_dummy, channelRoute_electrode, channelRoute_dummyAndelectrode, routenumvalues};
enum dummy {dummyRoute_rescapload = 0, dummyRoute_resload, dummynumvalues};
enum state {channelState_OFF = 0, channelState_ONConventional, channelState_ONBlock};
enum calib {calibOff = 0, calibDummy, calibChan0, calibChan1, calibChan2, calibChan3, calibnumValues};
enum refsource {refsource_GND = 0, refsource_AGCL, refsourcenumValues};

//Variables common to all channels
uint16_t extADCzeropoint = 31684;
float extADCgain = 10000/17568;
uint16_t zeropoint[STIMULATOR_NUMBER_OF_CHANNELS] = {2048, 2048, 2048, 2048};
float gain[STIMULATOR_NUMBER_OF_CHANNELS] = {(float)10000/(float)2048, (float)10000/(float)2048, (float)10000/(float)2048, (float)10000/(float)2048}; //pre-calibration conversion factor between microamperes and 12-bit DAC code
enum state channelState[STIMULATOR_NUMBER_OF_CHANNELS] = {0};
uint16_t dacChannelAddress[STIMULATOR_NUMBER_OF_CHANNELS] = {0b0010000000000000, 0b0101000000000000, 0b1000000000000000, 0b1011000000000000};
uint16_t channelRoute[STIMULATOR_NUMBER_OF_CHANNELS] = {channelRoute_off};
uint16_t dummyRoute = dummyRoute_rescapload;
uint16_t calibrationRoute = calibOff;
uint16_t calibrationReference = refsource_GND;
uint8_t electrodeRoutingPath[STIMULATOR_NUMBER_OF_CHANNELS] = {1,1,2,2};
uint8_t dummyRoutingPath[STIMULATOR_NUMBER_OF_CHANNELS] = {2,2,1,1};
uint8_t electrodeBitsRoutingPosition[STIMULATOR_NUMBER_OF_CHANNELS] = {SPISWITCH_CHAN0_BITADJUST, SPISWITCH_CHAN1_BITADJUST,
                                                                      SPISWITCH_CHAN2_BITADJUST, SPISWITCH_CHAN3_BITADJUST};

//Variables for conventional stim channels
uint32_t stimDACArray[STIMULATOR_NUMBER_OF_CHANNELS][500] = {0};
//uint32_t stimDAC1Array[500] = {0};
//uint32_t stimDAC2Array[500] = {0};
//uint32_t stimDAC3Array[500] = {0};

uint32_t stimPITArray[STIMULATOR_NUMBER_OF_CHANNELS][500] = {0};
//uint32_t stimPIT1Array[500] = {0};
//uint32_t stimPIT2Array[500] = {0};
//uint32_t stimPIT3Array[500] = {0};

uint32_t stimEventNumber[STIMULATOR_NUMBER_OF_CHANNELS] = {0};

uint32_t stimGPIOArray[STIMULATOR_NUMBER_OF_CHANNELS] = {0};
uint32_t GPIOStimTriggerPins[4] = {5, 7, 0, 9}; //Identifies which pins on the C GPIO port we'll use for stim trigger signals.

//Variables for exploratory stimulation
volatile uint16_t exStimAmp = 0; //Amplitude of pulses in exploratory stim
volatile uint32_t exStimPW = 0; //Pulse width
volatile uint32_t exStimIPD = 0; //Inter Pulse Delay
volatile int exStimState = 0; //State of stim i.e. active or off so that the PIT interrupt knows what to do
volatile uint8_t exStimChannel = 0; //Channel used for exploratory stimulation
volatile int exStimFlag = 0;

//DMA TCDs for event-driven stimulation
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t stimTcdDAC[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t stimTcdPIT[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t stimTcdGPIO[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));

//Variables for blocking stim channels
uint32_t blockDACArray[STIMULATOR_NUMBER_OF_CHANNELS][512] = {0};
uint32_t blockDACArrayStop[STIMULATOR_NUMBER_OF_CHANNELS][2] = {0};

uint32_t blockPITArray[STIMULATOR_NUMBER_OF_CHANNELS][6] = {0};
//Value 1: time to start blocking waveform from start of protocol
//Value 2: half-period time at start of block to comply with triphasic stimulation
//Value 3: full period time defining the frequency of the blocking signal
//Value 4: half-period time at end of block to comply with triphasic stimulation

volatile uint32_t blockNumberOfTransfers[STIMULATOR_NUMBER_OF_CHANNELS] = {0};
volatile edma_tcd_t* blockTcdDACPenUltPtr[STIMULATOR_NUMBER_OF_CHANNELS] = {0};

//DMA TCDs for continuous (neural block) stimulation
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdDACStart[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdDACMain[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdDACPenUlt[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdDACLast[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdPITStart[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdPITMain[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdPITPenUlt[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t blockTcdPITLast[STIMULATOR_NUMBER_OF_CHANNELS], sizeof(edma_tcd_t));

//DMA TCDs for external ADC interfacing
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t extADCReadTcd, sizeof(edma_tcd_t));
AT_NONCACHEABLE_SECTION_ALIGN(edma_tcd_t extADCWriteTcd, sizeof(edma_tcd_t));

volatile bool g_Transfer_Done = false;

// SPI0 Setup variables
volatile uint32_t spiToDACCommand;
volatile uint32_t spiToSwitchCommand;
volatile uint32_t spiToADCCommand;

// UART4 Variables
volatile uint8_t command = 0;
volatile uint32_t uartCommandCounter = 0;
volatile uint32_t uartCommandFlag = 0;
volatile uint8_t uartCommandArray[8] = {0}; //Array used to store incoming UART data. Commands are 8-bytes long.

//External ADC Variables
uint16_t ADCCommand = 0; //Dummy SPI word to send out on MOSI as it is not connected to external ADC
uint16_t ADCData[512] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
/* User callback functions for EDMA transfer.
 * note that a callback function should be created for every primary DMA channel used
 * this is hardware dependent as in this program the maximum number of DMA channels
 * that can be used as primaries i.e. that are triggered by the Periodic Interrupt Timer
 * are 4 on the K64F. Other microcontrollers may have more or less.
 *TODO Use preprocessor commands to determine whether to compile a certain number of
 *interrupts in order to make this code more portable */
void EDMA_Callback0(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
  //This callback should trigger when a stimulation channel has finished its preprogrammed protocol
  //This callback should:
  //Stop the corresponding PIT timer
  PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
  //Set the stim trigger GPIO to low
  GPIOC->PCOR |= (1<<GPIOStimTriggerPins[0]);
  //Clear DONE and interrupt bits in the channel
  EDMA_ClearChannelStatusFlags(DMA0, 0, kEDMA_DoneFlag);
  EDMA_ClearChannelStatusFlags(DMA0, 0, kEDMA_InterruptFlag);
  //Clear DONE bit in the linked channel
  EDMA_ClearChannelStatusFlags(DMA0, 4, kEDMA_DoneFlag);
    if (transferDone)
    {
        g_Transfer_Done = true;
        #ifdef SEMIHOSTING
            puts("DMA Channel 0 Transfer Done!\r\n"); //For use with semihosting to confirm the DMA transfer has completed.
        #endif


    }
    //channelState[0] = channelState_OFF; //Can test whether multiple stimulation runs can be done after just configuring once
  //At this point DMA Channel 0 and linked channel 4 should be ready to receive a new stimulation configuration
  //sent with ChannelSetup, followed by the stimulation start command to carry out another stimulation protocol
}

void EDMA_Callback1(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
  //This callback should trigger when a stimulation channel has finished its preprogrammed protocol
  //This callback should:
  //Stop the corresponding PIT timer
    PIT->CHANNEL[1].TCTRL &= ~PIT_TCTRL_TEN_MASK;
    //Clear DONE and interrupt bits in the channel
  //Set the stim trigger GPIO to low
  GPIOC->PCOR |= (1<<GPIOStimTriggerPins[1]);
    EDMA_ClearChannelStatusFlags(DMA0, 1, kEDMA_DoneFlag);
    EDMA_ClearChannelStatusFlags(DMA0, 1, kEDMA_InterruptFlag);
    //Clear DONE bit in the linked channel
    EDMA_ClearChannelStatusFlags(DMA0, 5, kEDMA_DoneFlag);
  //Reset both its corresponding and linked DMA channels to be ready for another protocol start command
    if (transferDone)
    {
        g_Transfer_Done = true;
        #ifdef SEMIHOSTING
            puts("DMA Channel 1 Transfer Done!\r\n"); //For use with semihosting to confirm the DMA transfer has completed.
        #endif


    }
}

void EDMA_Callback2(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
  //This callback should trigger when a stimulation channel has finished its preprogrammed protocol
  //This callback should:
  //Stop the corresponding PIT timer
    PIT->CHANNEL[2].TCTRL &= ~PIT_TCTRL_TEN_MASK;
    //Clear DONE and interrupt bits in the channel
  //Set the stim trigger GPIO to low
    GPIOC->PCOR |= (1<<GPIOStimTriggerPins[2]);
    EDMA_ClearChannelStatusFlags(DMA0, 2, kEDMA_DoneFlag);
    EDMA_ClearChannelStatusFlags(DMA0, 2, kEDMA_InterruptFlag);
    //Clear DONE bit in the linked channel
    EDMA_ClearChannelStatusFlags(DMA0, 6, kEDMA_DoneFlag);
  //Reset both its corresponding and linked DMA channels to be ready for another protocol start command
    if (transferDone)
    {
        g_Transfer_Done = true;
        #ifdef SEMIHOSTING
            puts("DMA Channel 2 Transfer Done!\r\n"); //For use with semihosting to confirm the DMA transfer has completed.
        #endif


    }
}

void EDMA_Callback3(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
  //This callback should trigger when a stimulation channel has finished its preprogrammed protocol
  //This callback should:
  //Stop the corresponding PIT timer
  PIT->CHANNEL[3].TCTRL &= ~PIT_TCTRL_TEN_MASK;
  //Clear DONE and interrupt bits in the channel
  //Set the stim trigger GPIO to low
  GPIOC->PCOR |= (1<<GPIOStimTriggerPins[3]);
    EDMA_ClearChannelStatusFlags(DMA0, 3, kEDMA_DoneFlag);
    EDMA_ClearChannelStatusFlags(DMA0, 3, kEDMA_InterruptFlag);
    //Clear DONE bit in the linked channel
    EDMA_ClearChannelStatusFlags(DMA0, 8, kEDMA_DoneFlag);
  //Reset both its corresponding and linked DMA channels to be ready for another protocol start command
    if (transferDone)
    {
        g_Transfer_Done = true;
        #ifdef SEMIHOSTING
            puts("DMA Channel 3 Transfer Done!\r\n"); //For use with semihosting to confirm the DMA transfer has completed.
        #endif


    }
}

void EDMA_Callback12(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
  //This callback should trigger when the sampling cycle from the external ADC has finished (512 samples)
  //This callback should:
  //Stop the TPM/FTM timer used to trigger SPI commands to the external DAC
  FTM_StopTimer(FTM0);
  //Wait until the last SPI transfer has finished before reading the last sample (check SPI flags)
  ADCData[511] = SPI0->POPR;

  //Clear DONE and interrupt bits in the channel
    EDMA_ClearChannelStatusFlags(DMA0, 13, kEDMA_DoneFlag);
    EDMA_ClearChannelStatusFlags(DMA0, 13, kEDMA_InterruptFlag);
    //Clear DONE bit in channel 12 which was the triggered channel
    EDMA_ClearChannelStatusFlags(DMA0, 12, kEDMA_DoneFlag);
  //Reset both its corresponding and linked DMA channels to be ready for another protocol start command

    //Setup External ADC DMAs again
        EDMA_InstallTCD(DMA0, 12, &extADCReadTcd);
        EDMA_StartTransfer(&g_EDMA_Handle[12]); //Arm DMA channel 12, starting TPM/FTM now will begin a cycle of external ADC sampling
        EDMA_InstallTCD(DMA0, 13, &extADCWriteTcd);

    if (transferDone)
    {
        g_Transfer_Done = true;
        #ifdef SEMIHOSTING
            puts("External ADC Sampling Cycle Complete!\r\n"); //For use with semihosting to confirm the DMA transfer has completed.
        #endif


    }
}

void UART4_RX_TX_IRQHandler (void)
{
  uint32_t uartStatusFlags;
  uint8_t byte;
  uartStatusFlags =  UART_GetStatusFlags(UART4); //For now we read the status flags but don't actually do anything with them
                                       //TODO later: react to error flags
  byte = UART_ReadByte(UART4);
  if (uartCommandFlag == 0) {
    uartCommandArray[uartCommandCounter] = byte;
    uartCommandCounter++;
    if (uartCommandCounter > 7) {
          uartCommandFlag = 1;
          uartCommandCounter = 0;
    }
  }
  else { //if the command flag is already active it hasn't been serviced by the MCU yet.
        //this is an error so we turn the red led on to warn the user. A reset and debug may be required.
      LED_RED_ON();
      uartCommandCounter = 0;
      uartCommandFlag = 0;

  }


}

void PIT0_IRQHandler(void) {
  PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); //Clear flags so that interrupt only runs once per trigger
//  uint32_t temp = USEC_TO_COUNT(exStimPW, PIT_SOURCE_CLOCK);
  if(exStimState == 0) { //If we're not currently stimulating
      SetExternalDACOutput(exStimAmp, exStimChannel, zeropoint, gain); //Set output to the current exStimAmp set output
      PIT->CHANNEL[0].LDVAL = (USEC_TO_COUNT(exStimIPD, PIT_SOURCE_CLOCK));
      GPIOC->PSOR |= (1<<GPIOStimTriggerPins[exStimChannel]);
      exStimState = 1;

  }
  else {
      SetExternalDACOutput(0, exStimChannel, zeropoint, gain);
      PIT->CHANNEL[0].LDVAL = (USEC_TO_COUNT(exStimPW, PIT_SOURCE_CLOCK));
      GPIOC->PCOR |= (1<<GPIOStimTriggerPins[exStimChannel]);
      exStimState = 0;
  }
}

void SendSPIData(uint32_t spiCommand){
  //This function manually sends commands through SPI
  //The chip select which is used for the transaction is determined by bits in spiCommand
  //spiCommand must be appropriately formatted by FormatSwitchCommand or FormatDacCommand
  DSPI_ClearStatusFlags(SPI0, (uint32_t)kDSPI_AllStatusFlag);
  SPI0->PUSHR = spiCommand;
  while (!(SPI0->SR & SPI_SR_TCF_MASK))
    {
        //DSPI_ClearStatusFlags(SPI0, (uint32_t)kDSPI_AllStatusFlag);
    }
}

uint32_t FormatDacCommand(int16_t inVal, uint8_t channel, uint16_t* zeropoint, float* gain)
{
  //Formats a 16-bit SPI frame from a 12-bit DAC target output value
  //and a 4-bit command value to be sent to the SPI external DAC

  uint32_t outCommand = 0;
  float tempVal;
  uint16_t outVal;

  if (inVal<0){
      inVal = -inVal;
      tempVal = (float)zeropoint[channel]-(float)inVal/gain[channel];
      outVal = zeropoint[channel] - ((uint16_t)tempVal-zeropoint[channel]);
  }
  else {
      tempVal = (float)zeropoint[channel]-(float)inVal/gain[channel];
      outVal = (uint16_t)tempVal;
  }

  //Maximum value of outVal: 4095. Setting boundaries below
  if (outVal>4095) {
      outVal = 4095;
  }
  outCommand = ((uint32_t)outVal + (uint32_t)dacChannelAddress[channel]) | spiToDACCommand;
  return outCommand;
}

void UpdateChannelRouting(uint16_t* channelRoute, uint16_t dummyRoute, uint16_t calibrationRoute)
{
  int i;
  //This function takes in a vector of
  //Formats a 16-bit SPI frame to set the two SPI external octal switches to
  //connect stimulator output channels, electrode contacts, dummy loads, and the calibration circuit input together
  //An output channel may be connected to:
  //an electrode contact
  //a dummy load (resistive or series resistive/capacitive) referred to system ground
  //the input to a voltage monitor/calibration module
  //At any time only ONE of the following may be connected to the calibration module:
  //The dummy load
  //an output channel from 0-3
  //Futhermore ONE of two references can be used for the calibration module:
  //System Ground
  //AgCL Reference (through external connection to AgCl electrode)
  //All channels are switched in and out simultaneously
  uint32_t command = 0;
  //TODO finish writing command translation routine for switches here
  for (i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
      switch (channelRoute[i]) {
        case channelRoute_off: {
          command = command|channelRoute[i]<<electrodeBitsRoutingPosition[i];
          break;
        }
        case channelRoute_dummy: {
          command = command|dummyRoutingPath[i]<<electrodeBitsRoutingPosition[i];

          break;
        }
        case channelRoute_electrode: {
          command = command|electrodeRoutingPath[i]<<electrodeBitsRoutingPosition[i];
          break;
        }
        default: {
          command = command|channelRoute[i]<<electrodeBitsRoutingPosition[i];
          break;
        }
      }
  }
  command = command|dummyRoute<<SPISWITCH_DUMMY_BITADJUST;
  if(calibrationRoute){
      command = command|1<<SPISWITCH_CALIB_BITADJUST+calibrationRoute; //TODO complete this command, likely cannot use normal structure because number of possible connections is greater than 2.
      command = command|1<<SPISWITCH_CALIBRATIONSRC_BITADJUST+calibrationReference;
  }
  SendSPIData(command | spiToSwitchCommand);
}



uint32_t ConvertFrequencyToPitTicks(uint32_t frequency){
  float usec = 1000000/frequency;
  return USEC_TO_COUNT(usec, PIT_SOURCE_CLOCK);
}

void KIN1_SoftwareReset(void)
{
  /* Generic way to request a reset from software for ARM Cortex */
  /* See https://community.freescale.com/thread/99740
     To write to this register, you must write 0x5FA to the VECTKEY field, otherwise the processor ignores the write.
     SYSRESETREQ will cause a system reset asynchronously, so need to wait afterwards.
   */
  SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
  for(;;) {
    /* wait until reset */
  }
}

void ChannelSetup(uint8_t channel, uint8_t mode) {
  //This channel sets up the DMA and PIT channels needed for a stimulation channel according to 3 possible modes
  //Possible modes are 0: OFF, 1: conventional event-driven stimulation, 2: continuous stimulation (generally for block)
  edma_transfer_config_t transferConfig;
  //edma_config_t userConfig;
  switch (mode) {
    case 1:{ //This mode corresponds to a conventional stimulation protocol of discrete stimulation events in time
      //First set up the DMA channel which will transfer DAC value data to SPI for transmission to the external DAC
      EDMA_PrepareTransfer(&transferConfig,
                           &stimDACArray[channel][0],
                           sizeof(stimDACArray[channel][0]),
                           &(SPI0->PUSHR),
                           sizeof(SPI0->PUSHR),
                           sizeof(stimDACArray[channel][0]),
                           stimEventNumber[channel]*sizeof(stimDACArray[channel][0]),
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&stimTcdDAC[channel], &transferConfig, NULL);
        //This sets up a TCD for the corresponding DMA channel, with an optional link to
        //a subsequent TCD for scatter/gather. Since this is conventional stimulation
        //We don't need this function a priori and the pointer to the next TCD is NULL
      EDMA_TcdSetChannelLink(&stimTcdDAC[channel], kEDMA_MinorLink, channel+4);
        //channels 4-7 are linked to to change the PIT period between each event
      EDMA_TcdEnableInterrupts(&stimTcdDAC[channel], kEDMA_MajorInterruptEnable);
        //As this TCD is the only TCD for a protocol in this mode it must be able to
        //call the DMA channel interrupt to stop the PIT timer and reset the channel
        //at the end of the protocol
      EDMA_InstallTCD(DMA0, channel, &stimTcdDAC[channel]);
      EDMA_StartTransfer(&g_EDMA_Handle[channel]); //This enables the DMA channel to accept transfer requests
        //NOTE only need to do this for gated channels as linked channels will be made active by their caller

      //Next set up the DMA channel linked to by the first which changes the period of the PIT timer channel
      //triggering the first DMA channel.
      //We have to preload the first value in the PITArray to LDVAL after we start the timer so the DMA takes
      //care of transfers to LDVAL from the second element onwards (hence why we start at stimPITArray[channel][1])
      EDMA_PrepareTransfer(&transferConfig,
                           &stimPITArray[channel][1],
                           sizeof(stimPITArray[channel][1]),
                           &(PIT->CHANNEL[channel].LDVAL),
                           sizeof(PIT->CHANNEL[channel].LDVAL),
                           sizeof(stimPITArray[channel][1]),
                           (stimEventNumber[channel]-1)*sizeof(stimPITArray[channel][1]),
                             //Assumes there will at least be two events in any stimulation protocol. The number of transfers is equal to the number of events minus 1.
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&stimTcdPIT[channel], &transferConfig, NULL);
      EDMA_TcdSetChannelLink(&stimTcdPIT[channel], kEDMA_MinorMajorLink, channel+8); //For conventional stim, set up a third DMA channel linked to a GPIO to keep track of when we're stimulating.
        //channels 8-11 are linked to toggle stim trigger GPIO to trigger external devices when stimulating
      EDMA_InstallTCD(DMA0, channel+4, &stimTcdPIT[channel]);

      //Set up the GPIO toggling DMA channel
      EDMA_PrepareTransfer(&transferConfig,
                           &stimGPIOArray[channel],
                           sizeof(stimGPIOArray[channel]),
                           &(GPIOC->PTOR),
                           sizeof(GPIOC->PTOR),
                           sizeof(stimGPIOArray[channel]),
                           (stimEventNumber[channel])*sizeof(stimGPIOArray[channel]),
                             //Assumes there will at least be two events in any stimulation protocol. The number of transfers is equal to the number of events minus 1.
                           kEDMA_PeripheralToPeripheral);

      EDMA_TcdSetTransferConfig(&stimTcdGPIO[channel], &transferConfig, NULL);
      EDMA_InstallTCD(DMA0, channel+8, &stimTcdGPIO[channel]);

      break;

    }
    case 2:{ //This mode corresponds to continuous stimulation generally used for HFAC Block
      //It relies on PIT-triggered DMA transfers to SPI to send output value data to the external DAC
      //A series of DMA Transfer Control Descriptors needs to be used in order for the output waveform
      //to comply with the triphasic stimulation standard
      //Set up blockTCDDACStart[channel]


      //Startup stage setup: SPI DMA
      EDMA_PrepareTransfer(&transferConfig,
                           &blockDACArray[channel][1],
                           sizeof(blockDACArray[channel][1]),
                           &(SPI0->PUSHR),
                           sizeof(SPI0->PUSHR),
                           sizeof(blockDACArray[channel][1]),
                           sizeof(blockDACArray[channel][1]), //The start TCD only has one transfer
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdDACStart[channel], &transferConfig, &blockTcdDACMain[channel]);
        //Make this TCD link to the Main TCD
      EDMA_TcdSetChannelLink(&blockTcdDACStart[channel], kEDMA_MajorLink, channel+4);
        //Set the channel link on major loop completion to change the PIT period from a half period to the full period (for startup)
      EDMA_InstallTCD(DMA0, channel, &blockTcdDACStart[channel]); //Install this TCD to run first. No interrupts needed here.

      EDMA_StartTransfer(&g_EDMA_Handle[channel]); //This enables the DMA channel to accept transfer requests
        //The protocol will only start when the PIT triggers so starting the PIT starts the protocol

      //Startup stage setup: PIT DMA
      //The goal of this TCD is to set the PIT period to that of the normal blocking signal
      //The initial PIT period change to the initial half-period will have been set up by the processor upon starting the protocol
      //This is because there are no SPI DMA transfers initially which can trigger this channel to do this
      EDMA_PrepareTransfer(&transferConfig,
                           &blockPITArray[channel][2],
                           //The second element is the one corresponding to the frequency of the block signal
                           sizeof(blockPITArray[channel][2]),
                           &(PIT->CHANNEL[channel].LDVAL),
                           sizeof(PIT->CHANNEL[channel].LDVAL),
                           sizeof(blockPITArray[channel][2]),
                           sizeof(blockPITArray[channel][2]), //The start TCD only has one transfer
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdPITStart[channel], &transferConfig, &blockTcdPITMain[channel]);
        //Make this TCD link to the main TCD
      EDMA_TcdSetChannelLink(&blockTcdPITStart[channel], kEDMA_MajorLink, channel+8);
        //Make this channel toggle the stim trigger signal when it fires
      EDMA_InstallTCD(DMA0, channel+4, &blockTcdPITStart[channel]);

      //Set up the GPIO toggling DMA channel for the block stim trigger signal
      EDMA_PrepareTransfer(&transferConfig,
                           &stimGPIOArray[channel],
                           sizeof(stimGPIOArray[channel]),
                           &(GPIOC->PTOR),
                           sizeof(GPIOC->PTOR),
                           sizeof(stimGPIOArray[channel]),
                           sizeof(stimGPIOArray[channel]),
                             //Total bytes transferred is the size of the destination as this channel will only fire once
                           kEDMA_PeripheralToPeripheral);

      EDMA_TcdSetTransferConfig(&stimTcdGPIO[channel], &transferConfig, NULL);
      EDMA_InstallTCD(DMA0, channel+8, &stimTcdGPIO[channel]);


      //Main stage setup: here we must figure out how many TCDs worth of transfers the SPI DMA will carry out
      //If the number of transfers is inferior to 1001 we are in a 'shortstim' case
      //In this case the SPI DMA TCDs are chained similarly to conventional stimulation
      //There is a slight difference between cases where transfers are < 500 and cases >=500

      //If the number of transfers is over 1000 we are in the 'longstim' case
      //In this case the 'Main' TCD is repeated until the number of transfers remaining can be fit into the Penultimate TCD
      //Main -> Main x N -> Penultimate -> Final
      //To count the number of 'Main' TCDs loaded by the SPI DMA we use the PIT DMA major loop iteration counter
      //(up to 32767 SPI DMA 'Main' TCDs can be counted in this way with a single PIT DMA TCD, using a major loop link
      //In this way we avoid having to store SPI DMA TCDs in a chain in memory or having to interrupt the processor
      //The PIT DMA loads a new TCD to change the SPI DMA TCD in memory to the penultimate version
      //when the appropriate number of SPI DMA TCDs has been expended. The updated TCD is subsequently loaded by the SPI DMA.

      //Initial variables:
      uint32_t blockNumberOfTcds = blockNumberOfTransfers[channel]/500;
      uint32_t blockFinalTransfers = blockNumberOfTransfers[channel]%500;
      if (blockFinalTransfers == 0){
          blockNumberOfTcds--;
          blockFinalTransfers = 500;
      }
      if (blockNumberOfTcds>32000) {
          blockNumberOfTcds = 32000;
          //Dealing with a higher number than this is not yet implemented. TODO with chains of PIT DMA TCDs
          //Maximum time for 50 kHz stimulation: 320 seconds.
          LED_RED_ON();
          break;
      }
      switch (blockNumberOfTcds) {
        case 0: { //Less than 500 transfers

          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               (blockFinalTransfers)*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD link to the last TCD
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to change the PIT period for the last SPI transfer to be a half-period.

//          //shortstim case 1 - SPI DMA Final
//          //the goal of this TCD is to set the channel current output level to 0
//          EDMA_PrepareTransfer(&transferConfig,
//                               &blockDACArrayStop[channel][0], //This value corresponds to zero current output for the channel
//                               sizeof(blockDACArrayStop[channel][0]),
//                               &(SPI0->PUSHR),
//                               sizeof(SPI0->PUSHR),
//                               sizeof(blockDACArrayStop[channel][0]),
//                               2*sizeof(blockDACArrayStop[channel][0]), //The final TCD only stores one transfer
//                               kEDMA_MemoryToPeripheral);
//
//          EDMA_TcdSetTransferConfig(&blockTcdDACLast[channel], &transferConfig, NULL);
//          //This TCD is the last in the protocol and so doesn't need to lead anywhere
//
//          EDMA_TcdEnableInterrupts(&stimTcdDAC[channel], kEDMA_MajorInterruptEnable);
//          //Make the SPI DMA interrupt to enable the processor to reset all the appropriate values at the end of the protocol



          //shortstim case 1 - PIT DMA
          //In this case the Main PIT DMA TCD acts like the final TCD
          //The purpose of this TCD is to change the PIT period to half of what it should be at the end of the blocking waveform
          //for the purpose of charge balance in triphasic block
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][5],
                               sizeof(blockPITArray[channel][5]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][5]),
                               sizeof(blockPITArray[channel][5]), //The 'Main' PIT TCD in this case only has one transfer
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        case 1: { //Between 501 and 1000 transfers

          //In this case Main -> Penultimate -> Last
          //shortstim case 2 - SPI DMA Main
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               500*sizeof(blockDACArray[channel][0]),
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACPenUlt[channel]);
            //Make this TCD link to the penultimate TCD

          //shortstim case 2 - SPI DMA PenUlt
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               blockFinalTransfers*sizeof(blockDACArray[channel][0]),
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD link to the last TCD in the protocol
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to change the PIT period for the last SPI transfer to be a half-period.
            //This is primarily what sets Main and PenUlt appart.

          //shortstim case 2 - PIT DMA
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][5],
                               sizeof(blockPITArray[channel][5]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][5]),
                               sizeof(blockPITArray[channel][5]), //The 'Main' PIT TCD in this case only has one transfer
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        case 2: {
            //Go to case 3
        }
        case 3: { //up to 2000 transfers

          //longstim - SPI DMA Main
          //Due to how scatter gather works we can't use TCDs with 500 transfers in; we need to ensure at least 4 TCDs are used,
          //including the penultimate TCD
          test = ((blockNumberOfTransfers[channel]-2)/4);
          test2 = ((blockNumberOfTransfers[channel]-2)/4+2+(blockNumberOfTransfers[channel]-2)%4);

          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               ((blockNumberOfTransfers[channel]-2)/4)*sizeof(blockDACArray[channel][0]),
                                 //Due to how scatter-gather works we have to make it so that more TCDs are used to
                                 //cover for cases between 1001 and 1500 transfers otherwise. Should not affect
                                 //the program negatively.
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACMain[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA decrement its major loop iteration counter
            //once for each time the SPI DMA completes a TCD's worth of transfers (500)


          //longstim - SPI DMA Penultimate
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               ((blockNumberOfTransfers[channel]-2)/4+2+(blockNumberOfTransfers[channel]-2)%4)*sizeof(blockDACArray[channel][0]),
                                 //We must ensure that there will always be at least one transfer inside this TCD, hence the +2
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA change the PIT period to half for
            //Charge balancing the waveform. This assumes that by the time the SPI DMA carries out this transfer
            //the PIT DMA will have changed its TCD after counting SPI DMA TCDs back to changing the PIT period.

          //longstim - PIT DMA Main
          //In this case the PIT DMA Main acts like the penultimate, which is replaced. The Main TCD links directly to the Last.
          EDMA_PrepareTransfer(&transferConfig,
                               &blockTcdDACPenUltPtr[channel],
                               sizeof(blockTcdDACPenUltPtr[channel]),
                               &(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA), //The PenUlt PIT TCD in this case only has one transfer
                               kEDMA_MemoryToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, &blockTcdPITLast[channel]);
            //Make this TCD link to the last PIT TCD for the protocol

          //longstim - PIT DMA Last
          //The purpose of this TCD is to change the PIT period to half for the charge balancing half-phase at the end
          //of the continuous stimulation protocol
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][3],
                               sizeof(blockPITArray[channel][3]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][3]),
                               3*sizeof(blockPITArray[channel][3]),
                                 //This TCD has three transfers, the first two being dummy transfers due to how the TCDs are chained
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITLast[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        default: { //More than 2000 transfers

          //longstim - SPI DMA Main
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               500*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACMain[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA decrement its major loop iteration counter
            //once for each time the SPI DMA completes a TCD's worth of transfers (500)


          //longstim - SPI DMA Penultimate
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               (blockFinalTransfers)*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA change the PIT period to half for
            //Charge balancing the waveform. This assumes that by the time the SPI DMA carries out this transfer
            //the PIT DMA will have changed its TCD after counting SPI DMA TCDs back to changing the PIT period.

          //longstim - PIT DMA Main
          //The purpose of this TCD is to use the major loop iteration counter of the PIT DMA to count the number
          //of 'Main' TCDs the SPI DMA has gone through
          EDMA_PrepareTransfer(&transferConfig,
                               &DMADummyTransfer[0],
                               sizeof(DMADummyTransfer[0]),
                               &DMADummyTransfer[1],
                               sizeof(DMADummyTransfer[1]),
                               sizeof(DMADummyTransfer[0]),
                               (blockNumberOfTcds-3)*sizeof(DMADummyTransfer[0]),
                               kEDMA_PeripheralToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, &blockTcdPITPenUlt[channel]);
            //Make this TCD link to the PIT DMA Penultimate TCD

          //longstim - PIT DMA PenUlt
          //The purpose of this TCD is to have the PIT DMA change the nextTCD pointer in the SPI DMA TCD in memory to point
          //to the penultimate TCD when the configured number of loops through the main TCD have been expended. The stimulation
          //then terminates as in the previous cases
          EDMA_PrepareTransfer(&transferConfig,
                               &blockTcdDACPenUltPtr[channel],
                               sizeof(blockTcdDACPenUltPtr[channel]),
                               &(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA), //The PenUlt PIT TCD in this case only has one transfer
                               kEDMA_MemoryToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITPenUlt[channel], &transferConfig, &blockTcdPITLast[channel]);
            //Make this TCD link to the last PIT TCD for the protocol

          //longstim - PIT DMA Last
          //The purpose of this TCD is to change the PIT period to half for the charge balancing half-phase at the end
          //of the continuous stimulation protocol
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][3],
                               sizeof(blockPITArray[channel][3]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][3]),
                               3*sizeof(blockPITArray[channel][3]),
                                 //This TCD has three transfers, the first two being dummy transfers due to how the TCDs are chained
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITLast[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel
          break;
        }
      }



      //All cases - SPI DMA Last
      //the goal of this TCD is to set the channel current output level to 0 after a charge-balancing final half-phase
      EDMA_PrepareTransfer(&transferConfig,
                           &blockDACArrayStop[channel][0], //This value corresponds to zero current output for the channel
                           sizeof(blockDACArrayStop[channel][0]),
                           &(SPI0->PUSHR),
                           sizeof(SPI0->PUSHR),
                           sizeof(blockDACArrayStop[channel][0]),
                           2*sizeof(blockDACArrayStop[channel][0]), //The final TCD stores two transfers: the first to complete the final half-phase
                             //for charge balancing, the second to set the current output of the channel to zero
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdDACLast[channel], &transferConfig, NULL);
        //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

      EDMA_TcdEnableInterrupts(&blockTcdDACLast[channel], kEDMA_MajorInterruptEnable);
      //Make the DMA interrupt to enable the processor to reset all the appropriate values at the end of the protocol


      break;
    }
    case 3:{//This mode corresponds to continuous biphasic stimulation used for experimental purposes
      //It relies on PIT-triggered DMA transfers to SPI to send output value data to the external DAC
      //A series of DMA Transfer Control Descriptors handle all switching and chnages in output from the DAC
      //This protocol is identical to the triphasic mode however the values of PIT DMA vector have been changed
      //to ensure that the start and end of the waveform use full length half-phases instead of half-length
      //as it is in triphasic mode
      //Biphasic block shouldn't be used by default since it causes large DC-like current transients to occur
      //at the electrode-electrolyte interface at the start and end of stimulation.


      //Set up blockTCDDACStart[channel]
      //Startup stage setup: SPI DMA
      EDMA_PrepareTransfer(&transferConfig,
                           &blockDACArray[channel][1],
                           sizeof(blockDACArray[channel][1]),
                           &(SPI0->PUSHR),
                           sizeof(SPI0->PUSHR),
                           sizeof(blockDACArray[channel][1]),
                           sizeof(blockDACArray[channel][1]), //The start TCD only has one transfer
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdDACStart[channel], &transferConfig, &blockTcdDACMain[channel]);
        //Make this TCD link to the Main TCD
      EDMA_TcdSetChannelLink(&blockTcdDACStart[channel], kEDMA_MajorLink, channel+4);
        //Set the channel link on major loop completion to change the PIT period from a half period to the full period (for startup)
      EDMA_InstallTCD(DMA0, channel, &blockTcdDACStart[channel]); //Install this TCD to run first. No interrupts needed here.

      EDMA_StartTransfer(&g_EDMA_Handle[channel]); //This enables the DMA channel to accept transfer requests
        //The protocol will only start when the PIT triggers so starting the PIT starts the protocol

      //Startup stage setup: PIT DMA
      //The goal of this TCD is to set the PIT period to that of the normal blocking signal
      //The initial PIT period change to the initial half-period will have been set up by the processor upon starting the protocol
      //This is because there are no SPI DMA transfers initially which can trigger this channel to do this
      EDMA_PrepareTransfer(&transferConfig,
                           &blockPITArray[channel][2],
                           //The second element is the one corresponding to the frequency of the block signal
                           sizeof(blockPITArray[channel][2]),
                           &(PIT->CHANNEL[channel].LDVAL),
                           sizeof(PIT->CHANNEL[channel].LDVAL),
                           sizeof(blockPITArray[channel][2]),
                           sizeof(blockPITArray[channel][2]), //The start TCD only has one transfer
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdPITStart[channel], &transferConfig, &blockTcdPITMain[channel]);
        //Make this TCD link to the main TCD
      EDMA_TcdSetChannelLink(&blockTcdPITStart[channel], kEDMA_MajorLink, channel+8);
        //Make this channel toggle the stim trigger signal when it fires
      EDMA_InstallTCD(DMA0, channel+4, &blockTcdPITStart[channel]);

      //Set up the GPIO toggling DMA channel for the block stim trigger signal
      EDMA_PrepareTransfer(&transferConfig,
                           &stimGPIOArray[channel],
                           sizeof(stimGPIOArray[channel]),
                           &(GPIOC->PTOR),
                           sizeof(GPIOC->PTOR),
                           sizeof(stimGPIOArray[channel]),
                           sizeof(stimGPIOArray[channel]),
                             //Total bytes transferred is the size of the destination as this channel will only fire once
                           kEDMA_PeripheralToPeripheral);

      EDMA_TcdSetTransferConfig(&stimTcdGPIO[channel], &transferConfig, NULL);
      EDMA_InstallTCD(DMA0, channel+8, &stimTcdGPIO[channel]);


      //Main stage setup: here we must figure out how many TCDs worth of transfers the SPI DMA will carry out
      //If the number of transfers is inferior to 1001 we are in a 'shortstim' case
      //In this case the SPI DMA TCDs are chained similarly to conventional stimulation
      //There is a slight difference between cases where transfers are < 500 and cases >=500

      //If the number of transfers is over 1000 we are in the 'longstim' case
      //In this case the 'Main' TCD is repeated until the number of transfers remaining can be fit into the Penultimate TCD
      //Main -> Main x N -> Penultimate -> Final
      //To count the number of 'Main' TCDs loaded by the SPI DMA we use the PIT DMA major loop iteration counter
      //(up to 32767 SPI DMA 'Main' TCDs can be counted in this way with a single PIT DMA TCD, using a major loop link
      //In this way we avoid having to store SPI DMA TCDs in a chain in memory or having to interrupt the processor
      //The PIT DMA loads a new TCD to change the SPI DMA TCD in memory to the penultimate version
      //when the appropriate number of SPI DMA TCDs has been expended. The updated TCD is subsequently loaded by the SPI DMA.

      //Initial variables:
      uint32_t blockNumberOfTcds = blockNumberOfTransfers[channel]/500;
      uint32_t blockFinalTransfers = blockNumberOfTransfers[channel]%500;
      if (blockFinalTransfers == 0){
          blockNumberOfTcds--;
          blockFinalTransfers = 500;
      }
      if (blockNumberOfTcds>32000) {
          blockNumberOfTcds = 32000;
          //Dealing with a higher number than this is not yet implemented. TODO with chains of PIT DMA TCDs
          //Maximum time for 50 kHz stimulation: 320 seconds.
          LED_RED_ON();
          break;
      }
      switch (blockNumberOfTcds) {
        case 0: { //Less than 500 transfers

          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               (blockFinalTransfers)*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD link to the last TCD
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to change the PIT period for the last SPI transfer to be a half-period.

//          //shortstim case 1 - SPI DMA Final
//          //the goal of this TCD is to set the channel current output level to 0
//          EDMA_PrepareTransfer(&transferConfig,
//                               &blockDACArrayStop[channel][0], //This value corresponds to zero current output for the channel
//                               sizeof(blockDACArrayStop[channel][0]),
//                               &(SPI0->PUSHR),
//                               sizeof(SPI0->PUSHR),
//                               sizeof(blockDACArrayStop[channel][0]),
//                               2*sizeof(blockDACArrayStop[channel][0]), //The final TCD only stores one transfer
//                               kEDMA_MemoryToPeripheral);
//
//          EDMA_TcdSetTransferConfig(&blockTcdDACLast[channel], &transferConfig, NULL);
//          //This TCD is the last in the protocol and so doesn't need to lead anywhere
//
//          EDMA_TcdEnableInterrupts(&stimTcdDAC[channel], kEDMA_MajorInterruptEnable);
//          //Make the SPI DMA interrupt to enable the processor to reset all the appropriate values at the end of the protocol



          //shortstim case 1 - PIT DMA
          //In this case the Main PIT DMA TCD acts like the final TCD
          //The purpose of this TCD is to change the PIT period to half of what it should be at the end of the blocking waveform
          //for the purpose of charge balance in triphasic block
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][5],
                               sizeof(blockPITArray[channel][5]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][5]),
                               sizeof(blockPITArray[channel][5]), //The 'Main' PIT TCD in this case only has one transfer
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        case 1: { //Between 501 and 1000 transfers

          //In this case Main -> Penultimate -> Last
          //shortstim case 2 - SPI DMA Main
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               500*sizeof(blockDACArray[channel][0]),
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACPenUlt[channel]);
            //Make this TCD link to the penultimate TCD

          //shortstim case 2 - SPI DMA PenUlt
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               blockFinalTransfers*sizeof(blockDACArray[channel][0]),
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD link to the last TCD in the protocol
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to change the PIT period for the last SPI transfer to be a half-period.
            //This is primarily what sets Main and PenUlt appart.

          //shortstim case 2 - PIT DMA
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][5],
                               sizeof(blockPITArray[channel][5]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][5]),
                               sizeof(blockPITArray[channel][5]), //The 'Main' PIT TCD in this case only has one transfer
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        case 2: {
            //Go to case 3
        }
        case 3: { //up to 2000 transfers

          //longstim - SPI DMA Main
          //Due to how scatter gather works we can't use TCDs with 500 transfers in; we need to ensure at least 4 TCDs are used,
          //including the penultimate TCD
          test = ((blockNumberOfTransfers[channel]-2)/4);
          test2 = ((blockNumberOfTransfers[channel]-2)/4+2+(blockNumberOfTransfers[channel]-2)%4);

          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               ((blockNumberOfTransfers[channel]-2)/4)*sizeof(blockDACArray[channel][0]),
                                 //Due to how scatter-gather works we have to make it so that more TCDs are used to
                                 //cover for cases between 1001 and 1500 transfers otherwise. Should not affect
                                 //the program negatively.
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACMain[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA decrement its major loop iteration counter
            //once for each time the SPI DMA completes a TCD's worth of transfers (500)


          //longstim - SPI DMA Penultimate
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               ((blockNumberOfTransfers[channel]-2)/4+2+(blockNumberOfTransfers[channel]-2)%4)*sizeof(blockDACArray[channel][0]),
                                 //We must ensure that there will always be at least one transfer inside this TCD, hence the +2
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA change the PIT period to half for
            //Charge balancing the waveform. This assumes that by the time the SPI DMA carries out this transfer
            //the PIT DMA will have changed its TCD after counting SPI DMA TCDs back to changing the PIT period.

          //longstim - PIT DMA Main
          //In this case the PIT DMA Main acts like the penultimate, which is replaced. The Main TCD links directly to the Last.
          EDMA_PrepareTransfer(&transferConfig,
                               &blockTcdDACPenUltPtr[channel],
                               sizeof(blockTcdDACPenUltPtr[channel]),
                               &(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA), //The PenUlt PIT TCD in this case only has one transfer
                               kEDMA_MemoryToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, &blockTcdPITLast[channel]);
            //Make this TCD link to the last PIT TCD for the protocol

          //longstim - PIT DMA Last
          //The purpose of this TCD is to change the PIT period to half for the charge balancing half-phase at the end
          //of the continuous stimulation protocol
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][3],
                               sizeof(blockPITArray[channel][3]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][3]),
                               3*sizeof(blockPITArray[channel][3]),
                                 //This TCD has three transfers, the first two being dummy transfers due to how the TCDs are chained
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITLast[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

          break;
        }
        default: { //More than 2000 transfers

          //longstim - SPI DMA Main
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               500*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACMain[channel], &transferConfig, &blockTcdDACMain[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACMain[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA decrement its major loop iteration counter
            //once for each time the SPI DMA completes a TCD's worth of transfers (500)


          //longstim - SPI DMA Penultimate
          EDMA_PrepareTransfer(&transferConfig,
                               &blockDACArray[channel][0],
                               sizeof(blockDACArray[channel][0]),
                               &(SPI0->PUSHR),
                               sizeof(SPI0->PUSHR),
                               sizeof(blockDACArray[channel][0]),
                               (blockFinalTransfers)*sizeof(blockDACArray[channel][0]), //
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdDACPenUlt[channel], &transferConfig, &blockTcdDACLast[channel]);
            //Make this TCD point to itself for scatter-gather
          EDMA_TcdSetChannelLink(&blockTcdDACPenUlt[channel], kEDMA_MajorLink, channel+4);
            //Set the channel link on major loop completion to have the PIT DMA change the PIT period to half for
            //Charge balancing the waveform. This assumes that by the time the SPI DMA carries out this transfer
            //the PIT DMA will have changed its TCD after counting SPI DMA TCDs back to changing the PIT period.

          //longstim - PIT DMA Main
          //The purpose of this TCD is to use the major loop iteration counter of the PIT DMA to count the number
          //of 'Main' TCDs the SPI DMA has gone through
          EDMA_PrepareTransfer(&transferConfig,
                               &DMADummyTransfer[0],
                               sizeof(DMADummyTransfer[0]),
                               &DMADummyTransfer[1],
                               sizeof(DMADummyTransfer[1]),
                               sizeof(DMADummyTransfer[0]),
                               (blockNumberOfTcds-3)*sizeof(DMADummyTransfer[0]),
                               kEDMA_PeripheralToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITMain[channel], &transferConfig, &blockTcdPITPenUlt[channel]);
            //Make this TCD link to the PIT DMA Penultimate TCD

          //longstim - PIT DMA PenUlt
          //The purpose of this TCD is to have the PIT DMA change the nextTCD pointer in the SPI DMA TCD in memory to point
          //to the penultimate TCD when the configured number of loops through the main TCD have been expended. The stimulation
          //then terminates as in the previous cases
          EDMA_PrepareTransfer(&transferConfig,
                               &blockTcdDACPenUltPtr[channel],
                               sizeof(blockTcdDACPenUltPtr[channel]),
                               &(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA),
                               sizeof(blockTcdDACMain[channel].DLAST_SGA), //The PenUlt PIT TCD in this case only has one transfer
                               kEDMA_MemoryToMemory);

          EDMA_TcdSetTransferConfig(&blockTcdPITPenUlt[channel], &transferConfig, &blockTcdPITLast[channel]);
            //Make this TCD link to the last PIT TCD for the protocol

          //longstim - PIT DMA Last
          //The purpose of this TCD is to change the PIT period to half for the charge balancing half-phase at the end
          //of the continuous stimulation protocol
          EDMA_PrepareTransfer(&transferConfig,
                               &blockPITArray[channel][3],
                               sizeof(blockPITArray[channel][3]),
                               &(PIT->CHANNEL[channel].LDVAL),
                               sizeof(PIT->CHANNEL[channel].LDVAL),
                               sizeof(blockPITArray[channel][3]),
                               3*sizeof(blockPITArray[channel][3]),
                                 //This TCD has three transfers, the first two being dummy transfers due to how the TCDs are chained
                               kEDMA_MemoryToPeripheral);

          EDMA_TcdSetTransferConfig(&blockTcdPITLast[channel], &transferConfig, NULL);
            //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel
          break;
        }
      }



      //All cases - SPI DMA Last
      //the goal of this TCD is to set the channel current output level to 0 after a charge-balancing final half-phase
      EDMA_PrepareTransfer(&transferConfig,
                           &blockDACArrayStop[channel][0], //This value corresponds to zero current output for the channel
                           sizeof(blockDACArrayStop[channel][0]),
                           &(SPI0->PUSHR),
                           sizeof(SPI0->PUSHR),
                           sizeof(blockDACArrayStop[channel][0]),
                           2*sizeof(blockDACArrayStop[channel][0]), //The final TCD stores two transfers: the first to complete the final half-phase
                             //for charge balancing, the second to set the current output of the channel to zero
                           kEDMA_MemoryToPeripheral);

      EDMA_TcdSetTransferConfig(&blockTcdDACLast[channel], &transferConfig, NULL);
        //Make this TCD link to nothing as the transfer it describes will be the last in the protocol for that channel

      EDMA_TcdEnableInterrupts(&blockTcdDACLast[channel], kEDMA_MajorInterruptEnable);
      //Make the DMA interrupt to enable the processor to reset all the appropriate values at the end of the protocol


      break;
    }
    default: {
      break; //If value for mode other than 1 or 2 then do nothing
    }
  }
}

void SetExternalDACOutput(uint16_t value, uint8_t channel, uint16_t* zeropoint, float* gain){
  //This function uses the SPI manually to set the output of a channel of the external DAC
  //the transfers are blocking and should not be used within an interrupt
  //Note value is signed 16-bit in microamperes (converted to unsigned 12-bit binary code for the DAC)
  SPI0->PUSHR = FormatDacCommand(value,channel,zeropoint,gain);
  while (!(DSPI_GetStatusFlags(SPI0) & kDSPI_TxFifoFillRequestFlag))
    {
        DSPI_ClearStatusFlags(SPI0, kDSPI_TxFifoFillRequestFlag);
    }

}

float extADC_512Avg(){
  float temp = 0.0;
  int i;

  for(i=0;i<512;i++){
          SendSPIData((uint32_t)(ADCCommand | spiToADCCommand));
          ADCData[i] = SPI0->POPR; //Read back data sent to SPI from external ADC
          if(i>1) temp = temp + (float)ADCData[i]; //discard first two samples
  }
  temp = temp/510;
  return temp;
}

int main(void)
{
  //Local Variables
    pit_config_t pitConfig;
    ftm_config_t ftmInfo;
    uint32_t i = 0, j = 0;
    uint8_t channel;
    uint32_t time;
    uint16_t halfcycle_us;
    int16_t value;
    int16_t top;
    int16_t bottom;
    float temp;
    float temp2;

    //DAC channel variables (adjustable by calibration)


    //DMA local variables
    edma_transfer_config_t transferConfig;
    edma_config_t userConfig;

    //SPI local variables
    uint32_t srcClock_Hz;
    srcClock_Hz = CLOCK_GetFreq(DSPI0_CLK_SRC);

    dspi_command_data_config_t DACSPIConfig;
    DACSPIConfig.isPcsContinuous = false;
    DACSPIConfig.whichCtar = kDSPI_Ctar0;
    DACSPIConfig.whichPcs = kDSPI_Pcs1;
    DACSPIConfig.isEndOfQueue = false;
    DACSPIConfig.clearTransferCount = false;

    dspi_command_data_config_t SwitchSPIConfig;
    SwitchSPIConfig.isPcsContinuous = false;
    SwitchSPIConfig.whichCtar = kDSPI_Ctar0;
    SwitchSPIConfig.whichPcs = kDSPI_Pcs0;
    SwitchSPIConfig.isEndOfQueue = false;
    SwitchSPIConfig.clearTransferCount = false;

    dspi_command_data_config_t ADCSPIConfig;
    ADCSPIConfig.isPcsContinuous = false;
    ADCSPIConfig.whichCtar = kDSPI_Ctar0;
    ADCSPIConfig.whichPcs = kDSPI_Pcs2;
    ADCSPIConfig.isEndOfQueue = false;
    ADCSPIConfig.clearTransferCount = false;

    dspi_master_config_t masterConfig;
    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = SPI_TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 16;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / SPI_TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / SPI_TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / SPI_TRANSFER_BAUDRATE;
    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    //NVIC Setup
    EnableIRQ(SPI0_IRQn); //Enable interrupts from SPI in NVIC
    EnableIRQ(UART4_RX_TX_IRQn); //Enable interrupt from UART4 in NVIC
    EnableIRQ(PIT0_IRQn); //Enable interrupt from PIT0 in NVIC

    /* Board pin init */
    BOARD_InitPins(); //Refer to pin_mux
    BOARD_BootClockRUN();

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Init output LED GPIOs. */
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

    /* Initialize LEDs to OFF */
    LED_RED_OFF();
    LED_GREEN_OFF();
    LED_BLUE_OFF();

    /* Define the init structure for the GPIO power pins to digital isolators*/
    gpio_pin_config_t gpiopwr_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Init Digital Isolator Power Pins (Direct through PTA1 and PTA2 GPIO)*/
    PORTA->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOA->PDDR |= (1<<2);
    GPIOA->PSOR |= (1<<2);

    PORTA->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOA->PDDR |= (1<<1);
    GPIOA->PSOR |= (1<<1);

    /* Init Digital pins to send trigger signals to external devices when the device is stimulating. */
    /* Pins used: PORTC pins 5,7,0,9 (in order of channel to which the signal corresponds) */
    /* Make sure pins are set as GPIO outputs (initialized as gpios in pin_mux.c) */
    GPIOC->PDDR |= (1<<5);
    GPIOC->PDDR |= (1<<7);
    GPIOC->PDDR |= (1<<0);
    GPIOC->PDDR |= (1<<9);


    /* Setup UART4 */
    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    UART_Init(UART4, &config, CLOCK_GetFreq(UART4_CLK_SRC));
    UART_EnableInterrupts(UART4, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);


    /* Setup SPI0 */
    DSPI_MasterInit(SPI0, &masterConfig, srcClock_Hz);
    SPI0->MCR |= SPI_MCR_PCSIS(kDSPI_Pcs1); //Ensure Pcs1 is active low like Pcs0 manually
    SPI0->MCR |= SPI_MCR_PCSIS(kDSPI_Pcs2); //Ensure Pcs2 is active low like Pcs0 manually
    DSPI_SetFifoEnable(SPI0, false, false);
    spiToDACCommand = DSPI_MasterGetFormattedCommand(&DACSPIConfig); //translate DACSPIConfig to hardware format
    spiToSwitchCommand = DSPI_MasterGetFormattedCommand(&SwitchSPIConfig); //translate DACSPIConfig to hardware format
    spiToADCCommand = DSPI_MasterGetFormattedCommand(&ADCSPIConfig);
      //OR commands with any SPI data to get the full 32-bit SPI transfer descriptor to be sent to SPI0
      //via DMA
//    for(i=0;i<10;i++)
//      {
//        spiDummyArray[i] = masterCommand | SPI_DUMMYCOMMAND; //create 10 dummy SPI transfer descriptors for testing
//
//      }


    /* Setup PIT */
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    timerPeriods[0] = USEC_TO_COUNT(1000000U, PIT_SOURCE_CLOCK);
    timerPeriods[1] = USEC_TO_COUNT(500000U, PIT_SOURCE_CLOCK);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, timerPeriods[1]); //initial 0.5s interval (60Mhz count rate)
    //Do not need to enable PIT interrupts as the LED will be toggled by the DMA

    /* Setup TPM0/FTM0 */
    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_4;
    FTM_Init(FTM0, &ftmInfo);
    FTM_SetTimerPeriod(FTM0, USEC_TO_COUNT(1000U, FTM_SOURCE_CLOCK)); //Set timer period to 1 ms (frequency 1 kHz)
    //FTM_StartTimer(FTM0, kFTM_SystemClock); //Command to start FTM


    //DMAMUX Setup
    DMAMUX_Init(DMAMUX0);
    //Setup DMA Channels 0-11 as always on as they are either periodically triggered or triggered by other DMA channels
    #if defined(FSL_FEATURE_DMAMUX_HAS_A_ON) && FSL_FEATURE_DMAMUX_HAS_A_ON
      for(i=0;i<12;i++){
          DMAMUX_EnableAlwaysOn(DMAMUX0, i, true); //channel 0-11 always on
      }
    #else
      for(i=0;i<12;i++){
          DMAMUX_SetSource(DMAMUX0, i, 63);
      }
    #endif /* FSL_FEATURE_DMAMUX_HAS_A_ON We want an aways on source for the channel.
    We can use the always on feature if enabled or source 63 if it is not */


    #if defined(FSL_FEATURE_DMAMUX_HAS_TRIG) && FSL_FEATURE_DMAMUX_HAS_TRIG > 0U
      //NOTE: If we don't have this the program won't work as of now as another periodic triggering source will be needed
      for(i=0;i<4;i++){
          DMAMUX_EnablePeriodTrigger(DMAMUX0, i);
            //Pit triggering for channels 0-3
            //We don't need triggers for channels 4-7 since they are linked to by channels 0-3
      }

    #endif

    //Set up DMA Channel 12 as being triggered from TPM0 (TPM0 triggers sampling from external ADC)
    DMAMUX_SetSource(DMAMUX0, 12, 20);
    //Set up DMA Channel 13 as being always on (will be triggered by channel 12)
    DMAMUX_SetSource(DMAMUX0, 13, 63);


    for(i=0;i<14;i++){
        DMAMUX_EnableChannel(DMAMUX0, i); //Enable DMA channels in the MUX
    }

    //DMA General Config
    EDMA_GetDefaultConfig(&userConfig); //Start from the default working configuration for the DMA and store it in userConfig
    EDMA_Init(DMA0, &userConfig);

    //DMA Channels Setup
    //SPI DMAs
    EDMA_CreateHandle(&g_EDMA_Handle[0], DMA0, 0);
    EDMA_SetCallback(&g_EDMA_Handle[0], EDMA_Callback0, NULL);
    EDMA_CreateHandle(&g_EDMA_Handle[1], DMA0, 1);
    EDMA_SetCallback(&g_EDMA_Handle[1], EDMA_Callback1, NULL);
    EDMA_CreateHandle(&g_EDMA_Handle[2], DMA0, 2);
    EDMA_SetCallback(&g_EDMA_Handle[2], EDMA_Callback2, NULL);
    EDMA_CreateHandle(&g_EDMA_Handle[3], DMA0, 3);
    EDMA_SetCallback(&g_EDMA_Handle[3], EDMA_Callback3, NULL);

    //PIT DMAs
    EDMA_CreateHandle(&g_EDMA_Handle[4], DMA0, 4);
    EDMA_CreateHandle(&g_EDMA_Handle[5], DMA0, 5);
    EDMA_CreateHandle(&g_EDMA_Handle[6], DMA0, 6);
    EDMA_CreateHandle(&g_EDMA_Handle[7], DMA0, 7);

    //GPIO DMAs
    EDMA_CreateHandle(&g_EDMA_Handle[8], DMA0, 8);
    EDMA_CreateHandle(&g_EDMA_Handle[9], DMA0, 9);
    EDMA_CreateHandle(&g_EDMA_Handle[10], DMA0, 10);
    EDMA_CreateHandle(&g_EDMA_Handle[11], DMA0, 11);

    //External ADC DMAs
    EDMA_CreateHandle(&g_EDMA_Handle[12], DMA0, 12);
    EDMA_SetCallback(&g_EDMA_Handle[12], EDMA_Callback12, NULL);
    EDMA_CreateHandle(&g_EDMA_Handle[13], DMA0, 13);


    //DMA PenUlt TCD pointer setup (used in continuous stimulation for block)
    for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++) {
        blockTcdDACPenUltPtr[i] = &blockTcdDACPenUlt[i];
    }

    //DMA Dummy Transfer Setup
    DMADummyTransfer[0] = 0xFF;

    //Octal SPI Switch Daisy Chain Setup
    //DSPI_MasterWriteDataBlocking(SPI0, spiToSwitchCommand, (uint16_t)0x2500);
    SendSPIData((uint32_t)((uint16_t)0x2500 | spiToSwitchCommand));
      //Special SPI command used at boot-time to make the octal switches enter daisy-chain mode

    //Set up stim-time GPIO toggle bit arrays for use by DMA to drive the external stim signal (toggle pin)
    for (i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++) {
        stimGPIOArray[i] = 1<<GPIOStimTriggerPins[i];
    }

    //Setup External ADC 32-bit SPI command (16 bit config, 16 bit data)
    //Data is 0 as we are sending dummy data to get a sample out of the external ADC
    ADCCommand = (uint32_t)((uint16_t)0x0000 | spiToADCCommand);

    //Setup External ADC DMAs
    //Channel 12: read from SPI receive buffer
    //triggered by FTM/TPM
    //links to channel 13 for every minor loop complete
    //Fills ADC Data with a major loop
    EDMA_PrepareTransfer(&transferConfig,
                         &(SPI0->POPR),
                         sizeof(SPI0->POPR),
                         &(ADCData[0]),
                         sizeof(ADCData[0]),
                         sizeof(ADCData[0]),
                         sizeof(ADCData),
                         kEDMA_PeripheralToMemory);

    EDMA_TcdSetTransferConfig(&extADCReadTcd, &transferConfig, NULL);
      //This sets up a TCD for the corresponding DMA channel, with an optional link to
      //a subsequent TCD for scatter/gather. The pointer to the next TCD is NULL
    EDMA_TcdSetChannelLink(&extADCReadTcd, kEDMA_MinorLink, 13);
      //Trigger a new sample by sending another SPI word (using DMA channel 13) at every minor loop iteration
    EDMA_TcdEnableInterrupts(&extADCReadTcd, kEDMA_MajorInterruptEnable);
      //call the DMA channel interrupt to stop the TPM/FTM timer and reset the channel
      //when sampling has finished (after obtaining 512 samples back from the ADC)
    EDMA_InstallTCD(DMA0, 12, &extADCReadTcd);
    EDMA_StartTransfer(&g_EDMA_Handle[12]); //Arm DMA channel 12, starting TPM/FTM now will begin a cycle of external ADC sampling


    //Channel 13: send dummy data to external ADC to obtain a new sample
    //Triggered by channel 12
    //interrupt on major loop complete to reset channels and disable TPM/FTM triggering channel 12
    //transfers equal to number of entries in ADCData -1
    EDMA_PrepareTransfer(&transferConfig,
                         &ADCCommand,
                         sizeof(ADCCommand),
                         &(SPI0->PUSHR),
                         sizeof(SPI0->PUSHR),
                         sizeof(ADCCommand),
                         2*(sizeof(ADCData)-sizeof(ADCData[0])), //2*sizeof(ADCData) because we sent 32 bits by DMA for every 16-bit ADC sample we want
                         kEDMA_PeripheralToPeripheral);

    EDMA_TcdSetTransferConfig(&extADCWriteTcd, &transferConfig, NULL);
      //This sets up a TCD for the corresponding DMA channel, with an optional link to
      //a subsequent TCD for scatter/gather. Since this is conventional stimulation
      //We don't need this function a priori and the pointer to the next TCD is NULL

    EDMA_InstallTCD(DMA0, 13, &extADCWriteTcd);


    //DEBUG: connect current output channel 0 to electrode
    //SendSPIData((uint16_t)(routeInfo[0].switchOne[channelRoute_resload] | routeInfo[0].switchTwo[channelRoute_resload]<<8) | spiToSwitchCommand);

    //Self-calibration feature
    //TODO
    //Channel 0 calibration

    for(j=0;j<4;j++) {

        channelRoute[j] = channelRoute_dummy;
        dummyRoute = dummyRoute_resload;
        calibrationRoute = calibDummy;
        calibrationReference = refsource_GND;

        value = 2048; //start value for determining zeropoint
        top = 0;
        bottom = 4095;
        UpdateChannelRouting(channelRoute,dummyRoute,calibrationRoute); //Set up connections for self-calibration
        SendSPIData((uint32_t)((dacChannelAddress[j] + value) | spiToDACCommand));

        for(i=0;i<256;i++){
          temp = extADC_512Avg();
          if(temp-extADCzeropoint < 0){
              top = value;
              value = (value+bottom)/2;
              SendSPIData((uint32_t)((dacChannelAddress[j] + value) | spiToDACCommand));
          }
          else{
              bottom = value;
              value = (value + top)/2;
              SendSPIData((uint32_t)((dacChannelAddress[j] + value) | spiToDACCommand));
          }
        }
        if(bottom-top<=1){
            SendSPIData((uint32_t)((dacChannelAddress[j] + top) | spiToDACCommand));
            temp = extADC_512Avg();
            SendSPIData((uint32_t)((dacChannelAddress[j] + bottom) | spiToDACCommand));
            temp2 = extADC_512Avg();
            if(abs(temp-extADCzeropoint)<=abs(temp2-extADCzeropoint)){
                zeropoint[j] = top;
            }
            else{
                zeropoint[j] = bottom;
            }
        }
        SendSPIData((uint32_t)((dacChannelAddress[j] + zeropoint[j]) | spiToDACCommand));
        for(i=0;i<4;i++) {
                channelRoute[i] = channelRoute_off;
        }
        UpdateChannelRouting(channelRoute,dummyRoute,calibrationRoute); //disconnect all channels at the end of each channel calibration
    }

    //For each current output channel
    //Route output to resistive only
    //Output current  and current full-scale output (be wary of not saturating the current pump)
    //Feed back measured voltage to adjust zeropoint and gain for each channel

    //Charge balance calibration
    //Set up calibration stimulation protocol
    //Route output to ResCap
    //Integrate zerocurrent to determine drift
    //Perform biphasic/triphasic pulse and measure residual voltage on capacitor

    while (1)
    {
        if (uartCommandFlag) {
            //This flag is set by the UART interrupt whenever 8 bytes of input information/commands have been stored
            //Ensure that the flag is set back to zero to allow the UART to receive additional commands
            //TODO: as of now there is no UART ping-pong buffer so operation may be incorrect if commands
            //take too much time to be executed - if this is the case implement a ping-pong or circular buffer with 8-byte
            //elements
            switch (uartCommandArray[0]) {
              case 90: { //Input new value-time event for conventional stimulation
                //convert rest of data in 8-byte package into channel, value, time
                //8-bit channel
                channel = uartCommandArray[1];
                //16-bit signed value in microamperes (to be converted to DAC binary code by FormatDacCommand)
                value = uartCommandArray[2] + (uartCommandArray[3]<<8);
                //32-bit time value in microseconds
                time = uartCommandArray[4] + (uartCommandArray[5]<<8) + (uartCommandArray[6]<<16) + (uartCommandArray[7]<<24);
                if (channel<STIMULATOR_NUMBER_OF_CHANNELS){
                    if (stimEventNumber[channel]<500) {
                        stimDACArray[channel][stimEventNumber[channel]] = FormatDacCommand(value,channel,zeropoint,gain);
                        stimPITArray[channel][stimEventNumber[channel]] = USEC_TO_COUNT(time, PIT_SOURCE_CLOCK);
                        stimEventNumber[channel] = stimEventNumber[channel] + 1;
                    }
                }
                break;
              }
              case 91: { //Input new set of values for continuous stimulation part 1
                //8-bit channel
                channel = uartCommandArray[1];
                //16-bit frequency (Hz) -> to be converted to a 32-bit value for input into the PIT
                halfcycle_us = uartCommandArray[2] + (uartCommandArray[3]<<8);
                //signed 16-bit value in microamps
                value = uartCommandArray[4] + (uartCommandArray[5]<<8);
                if (channel<STIMULATOR_NUMBER_OF_CHANNELS){
                    for(i=0;i<512;i++) {
                        if(i%2) {
                            blockDACArray[channel][i] = FormatDacCommand(-value,channel,zeropoint,gain);
                        }
                        else {
                            blockDACArray[channel][i] = FormatDacCommand(value,channel,zeropoint,gain);
                        }

                    }
                    blockDACArrayStop[channel][0] = blockDACArray[channel][0];
                    blockDACArrayStop[channel][1] = FormatDacCommand(0,channel,zeropoint,gain);
                      //We multiply the frequency by 2 as there needs to be two SPI transfers to the DAC per waveform cycle
                      //For triphasic stimulation we start and end with half-cycles of half the normal duration hence we need
                      //to multiply by 4
                    blockPITArray[channel][1] = (uint32_t)USEC_TO_COUNT(halfcycle_us/2, PIT_SOURCE_CLOCK);
                      //Here we multiply the frequency by 2 to halve the PIT trigger time.
                      //This allows us to stimulate with a half-phase at the start of block.
                      //That makes the block waveform comply with the triphasic stimulation method.
                    blockPITArray[channel][2] = (uint32_t)USEC_TO_COUNT(halfcycle_us, PIT_SOURCE_CLOCK);
                    blockPITArray[channel][3] = (uint32_t)USEC_TO_COUNT(halfcycle_us, PIT_SOURCE_CLOCK);
                    blockPITArray[channel][4] = (uint32_t)USEC_TO_COUNT(halfcycle_us, PIT_SOURCE_CLOCK);
                    blockPITArray[channel][5] = (uint32_t)USEC_TO_COUNT(halfcycle_us/2, PIT_SOURCE_CLOCK);
                }
                break;
              }
              case 92: { //Input new set of values for continuous stimulation part 2
                //8-bit channel
                channel = uartCommandArray[1];
                //32-bit start time in microseconds
                time = uartCommandArray[2] + (uartCommandArray[3]<<8) + (uartCommandArray[4]<<16) + (uartCommandArray[5]<<24);
                if (channel<STIMULATOR_NUMBER_OF_CHANNELS){
                    blockPITArray[channel][0] = USEC_TO_COUNT(time, PIT_SOURCE_CLOCK);
                }
                break;
              }
              case 93: { //Input new set of values for continuous stimulation part 3
                //8-bit channel
                channel = uartCommandArray[1];
                //32-bit number of cycles of block to deliver (one cycle = one positive and one negative phase)
                //Time(blocking signal duration) can be converted to half-cycles (transfers) by the program that talks to the stimulator from the PC

                blockNumberOfTransfers[channel] = uartCommandArray[2] + (uartCommandArray[3]<<8) + (uartCommandArray[4]<<16) + (uartCommandArray[5]<<24);
                break;
              }
              case 94: { //Command to set up stimulation with already uploaded data
                //the rest of the command data specifies which mode each channel is in:
                //0: channel OFF
                //1: channel ON for conventional stimulation
                //2: channel ON for blocking stimulation, triphasic mode (use this by default)
                //3: channel ON for blocking stimulation, biphasic mode
                for(i=0;i<(STIMULATOR_NUMBER_OF_CHANNELS);i++) {
                    channelState[i] = uartCommandArray[i+1];
                    ChannelSetup(i,uartCommandArray[i+1]);
                }
                break;
              }
              case 95: { //Start stimulation protocol command
                //Discussion: should this function access a variable asserting which PIT timer channels to start?
                for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
                    if(channelState[i]>0){
                        SendSPIData(FormatDacCommand(0, i, zeropoint, gain));
                        //channelRoute[i] = channelRoute_electrode;
                    }
                }
                //UpdateChannelRouting(channelRoute);
                for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
                    switch (channelState[i]) {
                      case 1:{
                        //debugValue = (uint32_t)USEC_TO_COUNT(STIM_PROTOCOL_COUNTDOWN_USEC, PIT_SOURCE_CLOCK);
                        PIT->CHANNEL[i].LDVAL = USEC_TO_COUNT(STIM_PROTOCOL_COUNTDOWN_USEC, PIT_SOURCE_CLOCK); //Set PIT trigger time as countdown before start of stimulation protocol
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of this
                          //Won't be triggered before then
                        break;
                      }
                      case 2:{
                        //debugValue = (uint32_t)USEC_TO_COUNT(STIM_PROTOCOL_COUNTDOWN_USEC, PIT_SOURCE_CLOCK)+blockPITArray[i][0];
                        PIT->CHANNEL[i].LDVAL = (uint32_t)USEC_TO_COUNT(STIM_PROTOCOL_COUNTDOWN_USEC, PIT_SOURCE_CLOCK)+blockPITArray[i][0];
                          //Set PIT trigger time as countdown before start of stimulation protocol + the block start delay
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of changing LDVAL
                          //won't be triggered before then
                        break;
                      }
                      case 3:{ //Biphasic mode for block: just change the blockPITArray[i][1,5] to have the length in time of full half-phases
                        blockPITArray[i][1] = (uint32_t)USEC_TO_COUNT(halfcycle_us, PIT_SOURCE_CLOCK);
                        blockPITArray[i][5] = (uint32_t)USEC_TO_COUNT(halfcycle_us, PIT_SOURCE_CLOCK);
                        PIT->CHANNEL[i].LDVAL = (uint32_t)USEC_TO_COUNT(STIM_PROTOCOL_COUNTDOWN_USEC, PIT_SOURCE_CLOCK)+blockPITArray[i][0];
                          //Set PIT trigger time as countdown before start of stimulation protocol + the block start delay
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of changing LDVAL
                          //won't be triggered before then
                        break;
                      }
                      default: { //default value e.g. 0: do nothing (i.e. don't start the PIT timer) as the channel isn't active
                        break;
                      }
                    }

                }
                //This second step attempts to activate the different timers as close to the same time as possible
                for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
                    switch (channelState[i]) {
                      case 1:{
                        //In this second step, activate the PIT and quickly update the LDVAL so that the PIT has the correct timing when triggering for the first time
                        PIT->CHANNEL[i].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                        PIT->CHANNEL[i].LDVAL = stimPITArray[i][0];
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of this
                          //Won't be triggered before then
                        break;
                      }
                      case 2:{
                        //In this second step, activate the PIT and quickly update the LDVAL so that the PIT has the correct timing when triggering for the first time
                        PIT->CHANNEL[i].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                        PIT->CHANNEL[i].LDVAL = blockPITArray[i][1];
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of changing LDVAL
                          //won't be triggered before then
                        break;
                      }
                      case 3:{//Biphasic mode for block, identical to case 2 here
                        //In this second step, activate the PIT and quickly update the LDVAL so that the PIT has the correct timing when triggering for the first time
                        PIT->CHANNEL[i].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                        PIT->CHANNEL[i].LDVAL = blockPITArray[i][1];
                          //Preload LDVAL on the PIT for the first trigger as the DMA normally in charge of changing LDVAL
                          //won't be triggered before then
                        break;
                      }
                      default: { //default value e.g. 0: do nothing (i.e. don't start the PIT timer) as the channel isn't active
                        break;
                      }
                    }

                }
                break;
              }
              case 96: { //Delete stored stimulation waveforms both for event-driven and continuous modes
                //This allows new ones to be stored in order to define a different protocol
                //TODO
                for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
                  for(j=0;j<500;j++){
                      stimDACArray[i][j] = 0;
                      stimPITArray[i][j] = 0;

                  }
                  for(j=0;j<4;j++){
                      blockDACArray[i][j] = 0;
                      blockPITArray[i][j] = 0;

                  }
                  stimEventNumber[i] = 0;
                  blockNumberOfTransfers[i] = 0;
                  channelState[i] = 0;
                }
                break;
              }

              case 100: { //Set DAC output level command
                channel = uartCommandArray[1];
                value = uartCommandArray[2] + (uartCommandArray[3]<<8);
                SetExternalDACOutput(value, channel, zeropoint, gain);
                break;
              }
              case 101: { //Set zeropoint command
                channel = uartCommandArray[1];
                zeropoint[channel] = uartCommandArray[2] + (uartCommandArray[3]<<8);
                SetExternalDACOutput(0, channel, zeropoint, gain);
                break;
              }
              case 102: { //Set gain command
                channel = uartCommandArray[1];
                //use i to temporarily store the value to be cast to float
                i = uartCommandArray[2] + (uartCommandArray[3]<<8) + (uartCommandArray[4]<<16) + (uartCommandArray[5]<<24);

                gain[channel] = *((float*)&i);
                break;
              }
              case 110: { //Change switch states command
                //TODO can do iterative or complete change version of the command depending on value of array[1]
                for(i=0;i<STIMULATOR_NUMBER_OF_CHANNELS;i++){
                    channelRoute[i] = uartCommandArray[i+2];
                }
                dummyRoute = uartCommandArray[6];
                calibrationRoute = uartCommandArray[7];
                UpdateChannelRouting(channelRoute,dummyRoute,calibrationRoute);
                break;
              }
              case 120: { //Exploratory stim start command
                channel = uartCommandArray[1];
                exStimFlag = 1;
                exStimState = 1;
                DMAMUX_DisableChannel(DMAMUX0, 0);
                PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); //Clear flags so that the timer interrupt will not trigger immediately
                PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable); //Turn on PIT interrupts
                PIT->CHANNEL[0].LDVAL = (USEC_TO_COUNT(100, PIT_SOURCE_CLOCK)); //Load Timer delay as pre-stim delay
                PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                break;
              }
              case 121: { //Exploratory stimulation parameter change command
                if(exStimFlag == 1) {
                    PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; //Stop the PIT channel
                    PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable); //Disable interrupts
                    SetExternalDACOutput(0, exStimChannel, zeropoint, gain); //Set output to zero in case we stopped mid-pulse
                    exStimState = 1;
                }
                exStimChannel = uartCommandArray[1];
                exStimAmp = uartCommandArray[2] + (uartCommandArray[3]<<8);
                //32-bit time value in microseconds
                exStimPW = uartCommandArray[4] + (uartCommandArray[5]<<8) + (uartCommandArray[6]<<16) + (uartCommandArray[7]<<24);
                if(exStimFlag == 1){ //If we're actively stimulating, start the PIT timer again
                    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable); //Turn on PIT interrupts
                    PIT->CHANNEL[0].LDVAL = (USEC_TO_COUNT(100, PIT_SOURCE_CLOCK)); //Load Timer delay as pre-stim delay
                    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                }
                break;
              }
              case 122: { //Exploratory stimulation inter pulse delay change command
                exStimIPD = uartCommandArray[4] + (uartCommandArray[5]<<8) + (uartCommandArray[6]<<16) + (uartCommandArray[7]<<24);
                break;
              }
              case 123: { //Exploratory stimulation stop command
                PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; //Stop the PIT channel
                PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable); //Disable interrupts
                DMAMUX_EnableChannel(DMAMUX0, 0);
                SetExternalDACOutput(0, channel, zeropoint, gain); //Set output to zero in case we stopped mid-pulse
                exStimState = 0;
                exStimFlag = 0;
                break;
              }
              case 130: { //DEBUG: force external ADC sample (x1). Use debug peripheral view to check result.
                //DSPI_FlushFifo(SPI0, false, true);




                temp = 0.0;
                for(i=0;i<512;i++) {
                    SendSPIData((uint32_t)(ADCCommand | spiToADCCommand));
                    ADCData[i] = SPI0->POPR; //Read back data sent to SPI from external ADC
                    if(i>1) temp = temp + (float)ADCData[i]; //discard first two samples
                }
                temp = temp/510.0; //finish average
                break;
              }
              case 135: { //DEBUG: Set DAC output level without gain/zeropoint
                channel = uartCommandArray[1];
                value = uartCommandArray[2] + (uartCommandArray[3]<<8); //here expects only positive values!!
                if (value<0 || value>4095) { //sanitise value as dac is 12 bit
                      value = 4095;
                }
                switch (channel){
                    case 1:{
                      SendSPIData((uint32_t)((0b0101000000000000 + value) | spiToDACCommand));
                      break;
                    }
                    case 2: {
                      SendSPIData((uint32_t)((0b1000000000000000 + value) | spiToDACCommand));
                      break;
                    }
                    case 3: {
                      SendSPIData((uint32_t)((0b1011000000000000 + value) | spiToDACCommand));
                      break;
                    }
                    default: {
                      SendSPIData((uint32_t)((0b0010000000000000 + value) | spiToDACCommand));
                      break;
                    }

                  }
                break;
              }
              case 254: { //DEBUG: infinite loop function with 1s per loop
                PIT->CHANNEL[0].LDVAL = (USEC_TO_COUNT(1000000, PIT_SOURCE_CLOCK));
                PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK; //start the channel's PIT timer
                //channelRoute[0] = channelRoute_dummy;
                calibrationRoute = calibOff;
                for(i=0;i<10;i++){
                  dummyRoute = dummyRoute_resload;
                  UpdateChannelRouting(channelRoute,dummyRoute,calibrationRoute);
                  while(!(PIT->CHANNEL[0].TFLG&&PIT_TFLG_TIF_MASK)){}
                  PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,PIT_TFLG_TIF_MASK);
                  dummyRoute = dummyRoute_rescapload;
                  UpdateChannelRouting(channelRoute,dummyRoute,calibrationRoute);
                  while(!(PIT->CHANNEL[0].TFLG&&PIT_TFLG_TIF_MASK)){}
                  PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,PIT_TFLG_TIF_MASK);
                }
                PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; //Stop the PIT channel
                break;
              }
              case 255: {
                KIN1_SoftwareReset();
                break;
              }
              default: { //Unknown command: flush uartCommandArray by resetting uartCommandCounter to 0
                uartCommandCounter = 0;
                break;
              }
            }
            uartCommandFlag = 0;
        }
    }
}
