/*
             LUFA Library
     Copyright (C) Dean Camera, 2015.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2015  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"

/** Current firmware mode, making the device behave as either a programmer or a USART bridge */
bool CurrentFirmwareMode = MODE_USART_BRIDGE;

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[128];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	uint16_t counter = 0;

	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	{
		RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
		RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	}
	else
	{
		V2Protocol_Init();
	}
	
	GlobalInterruptEnable();

	for (;;)
	{
		if (CurrentFirmwareMode == MODE_USART_BRIDGE)
		{
			/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
			if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
			{
				int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

				/* Store received byte into the USART transmit buffer */
				if (!(ReceivedByte < 0))
				{
					#if (BOARD == BOARD_GSCHEIDUINO)
						LEDS_PORT &= ~(LEDMASK_RX);
					#else
						LEDS_PORT |= (LEDMASK_RX);
					#endif
					counter = 0;
					RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
				}
			}

			uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
			if (BufferCount)
			{
				#if (BOARD == BOARD_GSCHEIDUINO)
				LEDS_PORT &= ~(LEDMASK_TX);
				#else
				LEDS_PORT |= (LEDMASK_TX);
				#endif
				counter = 0;
				Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

				/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
				 * until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
				if (Endpoint_IsINReady())
				{
					/* Never send more than one bank size less one byte to the host at a time, so that we don't block
					 * while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
					uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

					/* Read bytes from the USART receive buffer into the USB IN endpoint */
					while (BytesToSend--)
					{
						
						/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
						if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
												RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
						{
							break;
						}

						/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
						RingBuffer_Remove(&USARTtoUSB_Buffer);
					}
				}
			}

			/* Load the next byte from the USART transmit buffer into the USART if transmit buffer space is available */
			if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(&USBtoUSART_Buffer)))
			  Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));

			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		}
		else
		{
			AVRISP_Task();
		}
		
		// Check SCK Pin and activate PRG-LED (to handle as L-LED from original Arduino)
		#if (BOARD == BOARD_GSCHEIDUINO)
		if(PINB & (1<<PINB1))
			LEDS_PORT &= ~(LEDS_LED1);
		else
			LEDS_PORT |= (LEDS_LED1);
		#endif
		
		USB_USBTask();
		
		counter++;
		if(counter == 1000)
		{
			#if (BOARD == BOARD_GSCHEIDUINO)
				LEDS_PORT |= (LEDMASK_TX | LEDMASK_RX);
			#else
				LEDS_PORT &= ~(LEDMASK_TX | LEDMASK_RX);
			#endif
			counter = 0;
		}
	}
}

/** Processes incoming V2 Protocol commands from the host, returning a response when required. */
void AVRISP_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	//V2Params_UpdateParamValues();

	Endpoint_SelectEndpoint(AVRISP_DATA_OUT_EPADDR);

	/* Check to see if a V2 Protocol command has been received */
	if (Endpoint_IsOUTReceived())
	{
		#if (BOARD == BOARD_GSCHEIDUINO)
			LEDS_PORT &= ~(LEDS_LED1);
		#else
			LEDS_PORT |= (LEDS_LED1);
		#endif

		/* Pass off processing of the V2 Protocol command to the V2 Protocol handler */
		V2Protocol_ProcessCommand();

		#if (BOARD == BOARD_GSCHEIDUINO)
			LEDS_PORT |= (LEDS_LED1);
		#else
			LEDS_PORT &= ~(LEDS_LED1);
		#endif
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	//clock_prescale_set(clock_div_1);

	CLKSEL0 |= (1<<EXTE) | (1<<CLKS);
	CLKPR = 1<<CLKPCE;
	CLKPR = 0;
	//CLKPR = 0;
	//CLKPR = (1<<CLKPCE);
	//	CLKPR &= ~((1<<CLKPS3) | (1<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0));
#endif

	
	#if (BOARD == BOARD_GSCHEIDUINO)
		DDRC &= ~(1<<PC5);
		PORTC |= (1<<PC5);
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		CurrentFirmwareMode = (PINC & (1 << PC5)) ? MODE_USART_BRIDGE : MODE_PDI_PROGRAMMER;
		/* Pull target /RESET line high */
		AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
		AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
		// Set SPI-Pins as inputs:
		DDRB &= ~((1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4));
	#else
		DDRD &= ~(1<<PD0);
		PORTD |= (1<<PD0);
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		CurrentFirmwareMode = (PIND & (1 << PD0)) ? MODE_USART_BRIDGE : MODE_PDI_PROGRAMMER;
	#endif
	
	/* Hardware Initialization */
	LEDS_DDR  |=  LEDS_ALL_LEDS;
	LEDS_PORT |=  LEDS_ALL_LEDS;
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	#if (BOARD == BOARD_GSCHEIDUINO)
		LEDS_PORT |= (LEDS_LED1);
	#else
		LEDS_PORT &= ~(LEDS_LED1);
	#endif
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	#if (BOARD == BOARD_GSCHEIDUINO)
		LEDS_PORT &= ~(LEDS_LED1);
	#else
		LEDS_PORT |= (LEDS_LED1);
	#endif
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	{
		ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

		/* Configure the UART flush timer - run at Fcpu/1024 for maximum interval before overflow */
		//TCCR0B = ((1 << CS02) | (1 << CS00));

		/* Initialize ring buffers used to hold serial data between USB and software UART interfaces */
		RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
		RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	}
	else
	{
		/* Setup AVRISP Data OUT endpoint */
		ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_OUT_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);

		/* Setup AVRISP Data IN endpoint if it is using a physically different endpoint */
		if ((AVRISP_DATA_IN_EPADDR & ENDPOINT_EPNUM_MASK) != (AVRISP_DATA_OUT_EPADDR & ENDPOINT_EPNUM_MASK))
		  ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_IN_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);
	}

	if(ConfigSuccess == true)
	{
		#if (BOARD == BOARD_GSCHEIDUINO)
			LEDS_PORT |= (LEDS_LED1);
		#else
			LEDS_PORT &= ~(LEDS_LED1);
		#endif
	}
	else
	{
		#if (BOARD == BOARD_GSCHEIDUINO)
			LEDS_PORT &= ~(LEDS_LED1);
		#else
			LEDS_PORT |= (LEDS_LED1);
		#endif
	}
	
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	if (CurrentDTRState)
	  AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	else
	  AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
		CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&USARTtoUSB_Buffer)))
	  RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Set the new baud rate before configuring the USART */
	UBRR1  = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 *
 *  \param[in]  wValue                 Descriptor type and index to retrieve
 *  \param[in]  wIndex                 Sub-index to retrieve (such as a localized string language)
 *  \param[out] DescriptorAddress      Address of the retrieved descriptor
 *  \param[out] DescriptorMemorySpace  Memory space that the descriptor is stored in
 *
 *  \return Length of the retrieved descriptor in bytes, or NO_DESCRIPTOR if the descriptor was not found
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress,
                                    uint8_t* DescriptorMemorySpace)
{
	/* Return the correct descriptors based on the selected mode */
	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	  return USART_GetDescriptor(wValue, wIndex, DescriptorAddress, DescriptorMemorySpace);
	else
	  return AVRISP_GetDescriptor(wValue, wIndex, DescriptorAddress, DescriptorMemorySpace);
}
