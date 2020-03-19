/**
* @file main.c
*
* This file contains code to complete Milestone 4.
*
* The goal of this milestone is to continuously read packets and print the
* contents to the screen.
*
* Testing will be to send a packet to the NIC100 then read the packet's data
* from the NIC100 into a buffer.  Each packet will be printed to the screen.
*
*
*<pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- ---------------------------------------------------------
* 1.0   PS   03/14/20 First release
*</pre>
******************************************************************************/
#include "xparameters.h"
#include "xspi.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "sleep.h"
#include <stdlib.h>

/************************** Constant Definitions *****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define SPI_DEVICE_ID		XPAR_SPI_0_DEVICE_ID
#define INTC			XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler
#define INTC_DEVICE_ID	XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SPI_IRPT_INTR	XPAR_FABRIC_SPI_0_VEC_ID

/*
 *  This is the size of the buffer to be transmitted/received in this example.
 */
#define BUFFER_SIZE		12

#define ERXST_ADDR		0x5340
#define ERXTAIL_ADDR	0x5FFE

/************************** Function Prototypes ******************************/

int SpiReceivePackets(INTC *IntcInstancePtr, XSpi *SpiInstancePtr,
		u16 SpiDeviceId, u16
		SpiIntrId);

void SpiIntrHandler(void *CallBackRef, u32 StatusEvent, u32 ByteCount);

static int SpiSetupIntrSystem(INTC *IntcInstancePtr, XSpi *SpiInstancePtr,
		u16 SpiIntrId);

static void SpiDisableIntrSystem(INTC *IntcInstancePtr, u16 SpiIntrId);

int SpiTransferData(XSpi* SpiInstancePtr, u8* SendBufPtr, u8* RecvBufPtr,
		unsigned int ByteCount);

void ReadPackets(XSpi* SpiInstancePtr);
/************************** Variable Definitions *****************************/

/*
 * The instances to support the device drivers are global such that the
 * are initialized to zero each time the program runs.
 */
static INTC Intc;	 /* The instance of the Interrupt Controller */
static XSpi  SpiInstance;	 /* The instance of the SPI device */

/*
 * The following variables are shared between non-interrupt processing and
 * interrupt processing such that they must be global.
 */
volatile int TransferInProgress;

/*
 * The following variable tracks any errors that occur during interrupt
 * processing
 */
int Error;

/*
 * The following variables are used to read and write to the SPI device, they
 * are global to avoid having large buffers on the stack.
 */
u8 ReadBuffer[BUFFER_SIZE];
u8 WriteBuffer[BUFFER_SIZE];

/*
 * NextPacketPtr contains the address of the next packet in the NIC100's receive
 * buffer.
 */
u16 NextPacketPtr;

/*
 * Total number of packets read.
 */
u32 PacketCount = 1;

/*
 *
 */
u8 ReceivePacketsCmd[1519] = { 0x2C, 0x00};
u8 ReceivePacketsBuf[1519] = { 0x00 };

/*****************************************************************************/
/**
*
* Main function to call the Spi interrupt example.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int main(void)
{
	int Status;
	xil_printf("Starting SpiReceivePackets\r\n");

	Status = SpiReceivePackets(&Intc, &SpiInstance, SPI_DEVICE_ID, SPI_IRPT_INTR);

	if (Status != XST_SUCCESS) {
		xil_printf("SPI interrupt Example Failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran SpiReceivePackets\r\n");
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function does a minimal test on the Spi device and driver as a
* design example. The purpose of this function is to illustrate how to use
* the XSpi component using the interrupt mode.
*
* This function sends data and expects to receive the same data.
*
*
* @param	IntcInstancePtr is a pointer to the instance of the INTC
* 		component.
* @param	SpiInstancePtr is a pointer to the instance of Spi component.
* @param	SpiDeviceId is the Device ID of the Spi Device and is the
*		XPAR_<SPI_instance>_DEVICE_ID value from xparameters.h.
* @param	SpiIntrId is the interrupt Id and is typically
*		XPAR_<INTC_instance>_<SPI_instance>_VEC_ID value from
*		xparameters.h .
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note
*
* This function contains an infinite loop such that if interrupts are not
* working it may never return.
*
******************************************************************************/
int SpiReceivePackets(INTC *IntcInstancePtr, XSpi *SpiInstancePtr, u16 SpiDeviceId, u16 SpiIntrId)
{
	int Status;
	XSpi_Config *ConfigPtr;	/* Pointer to Configuration data */

	// Disable the instruction caches.
	void Xil_ICacheDisable(void);

	// Disable level 1 the Data cache.
	void Xil_L1DCacheDisable(void);

	// Disable level 1 the instruction cache.
	void Xil_L1ICacheDisable(void);

	// Disable the L2 cache.
	void Xil_L2CacheDisable(void);

	/*
	 * Initialize the SPI driver so that it is  ready to use.
	 */
	ConfigPtr = XSpi_LookupConfig(SpiDeviceId);
	if (ConfigPtr == NULL) {
		return XST_DEVICE_NOT_FOUND;
	}

	Status = XSpi_CfgInitialize(SpiInstancePtr, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the SPI device to the interrupt subsystem such that
	 * interrupts can occur. This function is application specific.
	 */
	Status = SpiSetupIntrSystem(IntcInstancePtr, SpiInstancePtr, SpiIntrId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the SPI that will be called from the interrupt
	 * context when an SPI status occurs, specify a pointer to the SPI
	 * driver instance as the callback reference so the handler is able to
	 * access the instance data.
	 */
	XSpi_SetStatusHandler(SpiInstancePtr, SpiInstancePtr, (XSpi_StatusHandler) SpiIntrHandler);

	/*
	 * Set the SPI device as a master.
	 */
	Status = XSpi_SetOptions(SpiInstancePtr, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XSpi_Start(SpiInstancePtr);

	/*
	 * Contains the commands that are sent to the NIC100 during initialization
	 */
	u8 CommandBuffer[][4] = {
			{ 0x56, 0x34, 0x12, 0x00 }, // Write 0x1234 to EUDAST
			{ 0x16, 0x00, 0x00, 0x00 }, // Read EUDAST
			{ 0x1B, 0x00, 0x00, 0x00 }, // Read ESTAT
			{ 0xCA, 0x00, 0x00, 0x00 }, // System Reset
			{ 0x22, 0x42, 0x77, 0x00 }, // Unbanked Write MACON2
			{ 0x22, 0x40, 0x0D, 0x00 }, // Unbanked Write MACON1
			{ 0x22, 0x04, 0x40, 0x53 }, // Unbanked Write ERXST
			{ 0x22, 0x06, 0xFE, 0x5F }, // Unbanked Write ERXTAIL
			{ 0x22, 0x34, 0x5E, 0x00 }, // Unbanked Write ERXFCON
			{ 0x22, 0x1E, 0x01, 0x00 }, // Unbanked Write ECON1, RXEN to start transmission
			{ 0x20, 0x1A, 0x00, 0x00 }, // Unbanked Read PCKCNT
			{ 0x22, 0x8A, 0x40, 0x53 }, // Unbanked Write ERXRDPT
			{ 0x2C, 0x00, 0x00, 0x00 }, // Read from SRAM
	};

	/*
	 * Contains the length in number of bytes for each command sent to NIC100
	 */
	u8 CommandLength[] = {3, 3, 2, 1, 4, 4, 4, 4, 4, 4, 4, 4};

	// Wait until power on reset (POR) is completed
	do {
		SpiTransferData(SpiInstancePtr, CommandBuffer[0], NULL, CommandLength[0]);
		ReadBuffer[0] = 0x00;
		ReadBuffer[1] = 0x00;
		ReadBuffer[2] = 0x00;
		SpiTransferData(SpiInstancePtr, CommandBuffer[1], ReadBuffer, CommandLength[1]);
	} while (ReadBuffer[1] != 0x34 && ReadBuffer[2] != 0x12);

	// Pol ClkRdy
	do {
		ReadBuffer[0] = 0x00;
		ReadBuffer[1] = 0x00;
		SpiTransferData(SpiInstancePtr, CommandBuffer[2], ReadBuffer, CommandLength[2]);
	} while ((ReadBuffer[1] & 0x10) == 0x00);

	// Issue System Reset
	SpiTransferData(SpiInstancePtr, CommandBuffer[3], NULL, CommandLength[3]);
	usleep(25);

	// Confirm EUDAST is 0x0000
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	ReadBuffer[2] = 0x00;
	SpiTransferData(SpiInstancePtr, CommandBuffer[2], ReadBuffer, CommandLength[2]);
	if (ReadBuffer[1] != 0x00 && ReadBuffer[2] != 0x00) {
		xil_printf("EUDAST was not reset to 0000h");
		return XST_FAILURE;
	}
	usleep(256);

	// System has finished reset
	// Set MACON2
	SpiTransferData(SpiInstancePtr, CommandBuffer[4], NULL, CommandLength[5]);

	/*
	 * Setup MACON1:
	 */
	SpiTransferData(SpiInstancePtr, CommandBuffer[5], NULL, CommandLength[6]);

	/*
	 * Program ERXST pointer, to first address used by receive filter
	 */
	SpiTransferData(SpiInstancePtr, CommandBuffer[6], NULL, CommandLength[6]);
	NextPacketPtr = ERXST_ADDR;

	/*
	 * Program ERXTAIL pointer, to last even address of buffer
	 */
	SpiTransferData(SpiInstancePtr, CommandBuffer[7], NULL, CommandLength[7]);

	/*
	 * Receive all valid packets by enabling CRCEN, RUNEN, UCEN, NOTMEEN, MCEN, filters and
	 * disabling other filters. 0000_0000_0101_1110
	 */
	SpiTransferData(SpiInstancePtr, CommandBuffer[8], NULL, CommandLength[8]);

	// Set ECON1 RXEN
	SpiTransferData(SpiInstancePtr, CommandBuffer[9], NULL, CommandLength[9]);

	while(1) {
		ReadPackets(SpiInstancePtr);
	}

	// Done
	XSpi_Stop(SpiInstancePtr);

	// Disable the SPI interrupt.
	SpiDisableIntrSystem(IntcInstancePtr, SpiIntrId);

	return XST_SUCCESS;
}

/*
 * Reads a packet from the Ethernet Card and prints it to the screen.
 */
void ReadPackets(XSpi* SpiInstancePtr) {
	u8 Commands[][4] = {
			{ 0x20, 0x1A, 0x00, 0x00 }, // PKTCNT
			{ 0x22, 0x1E, 0x01, 0x01 },  //
			{ 0x22, 0x06, 0x00, 0x00 } // Write ERXTAIL
		};
	u8 CmdLength[] = { 3, 4, 4 };
	u8 WriteRxReadPtr[] = { 0x22, 0x8A, 0x00, 0x00 };
	u8 Buffer[4] = { 0x00 };

	WriteRxReadPtr[2] = (NextPacketPtr & 0x00FF);
	WriteRxReadPtr[3] = (NextPacketPtr & 0xFF00) >> 8;

	// Wait for a packet
	do {
		SpiTransferData(SpiInstancePtr, Commands[0], Buffer, CmdLength[0]);
		usleep(1000);
	} while(Buffer[2] == 0);

	// Write to ERXRDPT
	SpiTransferData(SpiInstancePtr, WriteRxReadPtr, NULL, 4);

	SpiTransferData(SpiInstancePtr, ReceivePacketsCmd, ReceivePacketsBuf, 9);

	NextPacketPtr = (ReceivePacketsBuf[2] << 8) + ReceivePacketsBuf[1];
	u16 FrameSize = (ReceivePacketsBuf[4] << 8) + ReceivePacketsBuf[3];

	SpiTransferData(SpiInstancePtr, ReceivePacketsCmd, ReceivePacketsBuf, FrameSize + 1);

	// Update ERXTAIL
	u16 TailPtr = NextPacketPtr - 2;
	Commands[2][2] = (TailPtr & 0x00FF);
	Commands[2][3] = (TailPtr & 0xFF00) >> 8;
	SpiTransferData(SpiInstancePtr, Commands[2], NULL, CmdLength[2]);

	// Decrement PCKCNT
	SpiTransferData(SpiInstancePtr, Commands[1], NULL, CmdLength[1]);
	SpiTransferData(SpiInstancePtr, Commands[0], Buffer, CmdLength[0]);

	// Print out data
	xil_printf("Packets Remaining: %d\r\n", Buffer[3]);
	xil_printf("Packet Number: %d\r\n", PacketCount++);
	xil_printf("Packet Size: %d\r\n", FrameSize);
	xil_printf("Next Packet at: %04X%\r\n", NextPacketPtr);
	// Print the frame to terminal
	for (int i = 1; i <= FrameSize; i++) {
		xil_printf("%02X ", ReceivePacketsBuf[i]);
		if (i % 8 == 0) {
			xil_printf("  ");
		}
		if (i % 16 == 0) {
			xil_printf("\r\n");
		}
	}
	xil_printf("\r\n\r\n");
}


int SpiTransferData(XSpi* SpiInstancePtr, u8* SendBufPtr, u8* RecvBufPtr, unsigned int ByteCount) {
	int Rtn;
	int Status;

	Status = XSpi_SetSlaveSelect(SpiInstancePtr, 1);

	TransferInProgress = TRUE;
	Rtn = XSpi_Transfer(SpiInstancePtr, SendBufPtr, RecvBufPtr, ByteCount);
	if (Rtn != XST_SUCCESS) {
		xil_printf("Spi READ transfer failed\r\n");
		return XST_FAILURE;
	}

	while (TransferInProgress);
	Status = XSpi_SetSlaveSelect(SpiInstancePtr, 0);

	return Status;
}


/*****************************************************************************/
/**
*
* This function is the handler which performs processing for the SPI driver.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized. It is called when a transfer of SPI data
* completes or an error occurs.
*
* This handler provides an example of how to handle SPI interrupts and
* is application specific.
*
* @param	CallBackRef is the upper layer callback reference passed back
*		when the callback function is invoked.
* @param	StatusEvent is the event that just occurred.
* @param	ByteCount is the number of bytes transferred up until the event
*		occurred.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void SpiIntrHandler(void *CallBackRef, u32 StatusEvent, u32 ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event.
	 */
	TransferInProgress = FALSE;

	/*
	 * If the event was not transfer done, then track it as an error.
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {
		Error++;
	}
}


/*****************************************************************************/
/**
*
* This function setups the interrupt system such that interrupts can occur
* for the Spi device. This function is application specific since the actual
* system may or may not have an interrupt controller. The Spi device could be
* directly connected to a processor without an interrupt controller.  The
* user should modify this function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the instance of the Intc device.
* @param	SpiInstancePtr is a pointer to the instance of the Spi device.
* @param	SpiIntrId is the interrupt Id and is typically
*		XPAR_<INTC_instance>_<SPI_instance>_VEC_ID value from
*		xparameters.h
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
static int SpiSetupIntrSystem(INTC *IntcInstancePtr, XSpi *SpiInstancePtr,
					 u16 SpiIntrId)
{
	int Status;

#ifdef XPAR_INTC_0_DEVICE_ID
#ifndef TESTAPP_GEN
	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	Status = XIntc_Initialize(IntcInstancePtr, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device.
	 */
	Status = XIntc_Connect(IntcInstancePtr, SpiIntrId,
	 			(XInterruptHandler) XSpi_InterruptHandler,
				(void *)SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

#ifndef TESTAPP_GEN
	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specific real mode so that
	 * the SPI can cause interrupts through the interrupt controller.
	 */
	Status = XIntc_Start(IntcInstancePtr, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
 	   return XST_FAILURE;
	}
#endif

	/*
	 * Enable the interrupt for the SPI device.
	 */
	XIntc_Enable(IntcInstancePtr, SpiIntrId);
#else

	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
#ifndef TESTAPP_GEN
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
				IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	XScuGic_SetPriorityTriggerType(IntcInstancePtr, SpiIntrId, 0xA0, 0x3);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, SpiIntrId,
				(Xil_InterruptHandler)XSpi_InterruptHandler,
				SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	XScuGic_Enable(IntcInstancePtr, SpiIntrId);
#endif

#ifndef TESTAPP_GEN

	/* Enable interrupts from the hardware */
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)INTC_HANDLER, (void *)IntcInstancePtr);

	Xil_ExceptionEnable();

#endif /* TESTAPP_GEN */

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function disables the interrupts that occur for the Spi device.
*
* @param	IntcInstancePtr is the pointer to the instance of the INTC
*		component.
* @param	SpiIntrId is the interrupt Id and is typically
*		XPAR_<INTC_instance>_<SPI_instance>_VEC_ID value from
*		xparameters.h
*
* @return	None
*
* @note		None
*
******************************************************************************/
static void SpiDisableIntrSystem(INTC *IntcInstancePtr, u16 SpiIntrId)
{
	/*
	 * Disconnect and disable the interrupt for the Spi device.
	 */
#ifdef XPAR_INTC_0_DEVICE_ID
	/* Disconnect the interrupts for the Master complete and error */
	XIntc_Disconnect(IntcInstancePtr, SpiIntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, SpiIntrId);
#endif
}
