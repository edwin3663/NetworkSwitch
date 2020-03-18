# Network Switch Project

### Goal:
The goal of this project is to learn about computer networking by building an unmanaged network switch.

### Design:
The network switch I will design will have four ports.  Each port will be capable of 10/100 Mbs speed.

### Hardware:
* 1 x Zynq-7000 SoC (ARM Cortex A9 processesor mated with an Artix-7 base FPGA)
* 4 x Microchip Stand-Alone 10/100 Ethernet Controllers with SPI Interface

### Software:
* Xilinx Vivado 2018.3
* Xilinx Software Development Kit

### Development Process:

I will complete this project by breaking it down into small pieces.  Each piece will contain the solution for one part of the design.

## Milestones:

### Milestone 1: Communicating with the Ethernet controller
Date Completed: March 11, 2020

Goal: Connect a single Ethernet controller to the ARM processor using the Serial Peripheral Interface.

1. Create the block diagram in Vivado
2. Learn how to use the SPI protocol to send and receive data

Testing:  Used the Vivado signal analyzer to examine the signals sent to and from the SPI

### Milestone 2: Initialize the Ethernet controller
Date Completed: March 13, 2020

Goal: Send packets to the Ethernet controller and count the number received

1. Initialize the Ethernet controller by following the steps laid out in the controller's manual
2. Set the Ethernet controller to promiscous mode to capture any packets
3. Send X number of packets to the Ethernet controller then read the packet count register.

Testing:  Used wireshark to count the number of packets sent from the host.  Compared the number sent to the number recorded in the Ethernet controller's count register.

### Milestone 3: Read a single packet
Date Completed: March 14, 2020

Goal: Read a single packet store in the Ethernet controller and print it to screen

1. Access the Ethernet controller's SRAM to read a packet sent to the controller.

Testing:  Used wireshark to compare the packet sent from the host to the Ethernet controller.

### Milestone 4: Continuously read packets
Date Completed: March 16, 2020

Goal: Continously read packets sent to the Enternet controller

1. Learn how to use the Ethernet controller's circular buffer
2. Continously read packets sent to the controller and print them to the screen

Testing:  Used wireshark to compare the packets sent from the host to the Ethernet controller.



