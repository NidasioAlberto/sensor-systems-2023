# Half-duplex link with infrared led and photodiode

Objective of the project is to implement an half-duplex link to enable communication between multiple boards, using the infrared led and photodiode available on the development board.

Steps:
1. Implement transmission via the infrared led
2. Configure the USART1 peripheral to receive data from the photodiode

## How the IR link works

On the development board there are:
- An infrared LED, controlled by `PB10` via a "class B" amplifier
- An infrared photodiode, that embeds a demodulator and a band pass filter connected to `PA10`

The idea is that the led sends an UART signal with a carrier frequency of 38KHz and the photodiode filters the signal (to prevent interferences from other IR sources) and demodulates it back into a UART signal.

The characteristics of the IR signal (carrier frequency and baudrate) comes primarily from the photodiode, specifically a `BC817-25`. In its datasheet we can find two key parameters:
- The carrier frequency at 38KHz
- Minimum burst length of 10 carrier cycles for each burst/symbol

This imposes a maximum baudrate of $38KHz/10 = 3800Hz$.

## Development board hardware

The main difference between the transmitter (the led) and the receiver (the photodiode) is that the transmitter is connected to a simple timer while the receiver to an UART Rx GPIO.

While on the receiver the data readout is simple, since the photodiode provides a clean signal that the UART peripheral can read, the transmitter is a bit more complicated. This is because the timer's pin of the microcontroller needs to generate the carrier frequency and modulate it. This results in configuring the timer pin in PWM mode to generate a square wave at 38KHz and an interrupt routine (generate by another timer) to switch on or off the PWM depending on which bit needs to be transmitted at any given time, thus manually generating the UART signal.

This would have been much easier if on the transmitter side two GPIO, connected with an AND gate, where used:
- One timer pin to generate the carrier frequency
- An UART Tx pin to generate the UART signal

## Implementation

We ended up creating a single project showcasing both the transmission and reception since the two parts a independed.

To transmit data we configured:
- TIM4 to generate an interrupt at 2KHz with a prescaler of $8400-1$ and a counter period of $5-1$. We lowered the baudrate from the suggested 2400bps to 2000bps because the communication was more stable.
- TIM2 to generate a PWM signal at 38KHz

The interrupt uses a control struct where there are memorized 4 things:
- A pointer to the data that need to be transmitted
- The length of the data
- The index of the current byte being transmitted
- The index of the next bit to be transmitted

This way we follow, for each byte until the end of the buffer, a state simple state machine:
- If the next bit to be transmitted is -1 we send the start signal
- If the next bit is from 0 to 7 we send a bit of the current byte
- If the next bit is 8 we send the end signal (actually we send two stop bits because we found the link to be more reliable)

The receiver part is instead much simpler. We just setup the USART1 peripheral with the same baudrate.

We setup both the USART1 and USART2 in interrupt mode. When a byte is read, the interrupt either reads from USART1 and sends the byte on UART2 or reads from UART2 (the terminal) and configures the control truct to send one bit and starts TIM4.

This way if we connect to the serial terminal of one board, everything that we type gets sent over the IR link and everything received from the IR link gets printed on the terminal.