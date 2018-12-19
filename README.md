# Arduino asynchronous Dynamixel library

This library implements a half-duplex communication with a Robotis Dynamixel with **no need of additional hardware**, and efficient use of the ATMega hardware, making it possible to perform all communication in **parallel** with the main routine, without calling any blocking functions. It achieves this with interrupts, both from an USART module and a timer module.

## Brief Working explanation
A circular buffer is statically allocated to hold commands to be sent to the servo. Each buffer entry can contain a command of up to 16 bytes to be sent, in addition to the number of bytes to be sent, the number of bytes already sent, and the address to a callback function. The buffer is allocated with one extra position besides the desired length for holding a default command in case there are no more commands to be sent(currently the default command retrieves all information on the servo state).

The main routine may call the function `queueInstruction` to add commands to this buffer, and then return as soon as the command is succesfully added or a failure occurs(if the buffer is full). This function **will not** block until this command is sent. Instead, normal code will resume and the command will be sent as soon as possible.

The command is retrieved from the buffer via the ISR functions, tha govern all communication with the dynamixel. There are for interrupts configured:
 * The USART interrupt for transmission buffer empty is called whenever we can add a new byte to be sent. Everytime this interrupt happens, we load the next byte to be sent, or, if there are no more bytes to be sent, we disable this interrupt temporarilly.
 * The USART interrupt for transmission complete, which occurs when the last byte was already shifted out, signals that the desired command has been entirely transmitted, so we should disable the transmitter function of the hardware so the dynamixel can take control of the bus, and enable the receiver to receive the dynamixel response.
 * The USART interrupt for receibed byte, which will be called everytime the arduino receives a new byte from the dynamixel, simply receives this byte, checks for obvious errors, and saves it in a receiving buffer. Using the robotis protocol, we can predict how many bytes should be received, so if we receive the amount of bytes we have been expecting, we call the callback function(**do notice**: this function will be called **inside** the ISR, so it **must** be short) registered the moment the command was added to the buffer and begin procedure to send the next command.
 * An interrupt for match from a timer. This interrupt happens when the timer counter achieves the value we configured. On every receiving byte, we set this value to be slightly higher than the current value of the counter, so in normal receiving operation, this interrupt won't happen, but in case something goes wrong and we do not received the expected bytes, we perform a timeout procedure in this interrupt, for trying to send the last command again. When all bytes are received as expected, we also set the value for an interrupt in the receiving interrupt, so that we only start sending the next command after allowing some time to the Dynamixel to process the last command, release the BUS, etc. If this interrupt happens in this situation, we simply turn on transmission interrupts back on, decide what is the next command to be sent(next in the buffer, or the default command if the buffer is empty), turn off the receiver and turn on the transmitter again to begin shifting out the next command.

## Warnings:
This library uses Timer3 Output Compare mode to check for communication timeouts and inserting intervals between transmissions asynchronously. PWMs generated from Timer3 will not work. If you were using PWM in pins that use this timer, you will need to change pins, or change which counter is used by the library.

## TODO:
 * Export everything to an arduino library, for easy of use
 * Create high level functions for setting position, speed and some basic configuration with no need to know details on the robotis communication protocol
 * Support for multiple devices in the same bus
 * Remove `sent` variable from struct `tx_msg`

 ## Related content
 This project was started for the conclusion project of a Computer engineering undergraduate. The following video is the presentation of the work, including operation of the dynamixel asynchronously: https://www.youtube.com/watch?v=ybda-4s1Lig
