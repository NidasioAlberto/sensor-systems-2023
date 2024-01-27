# Homeworks Recap

## 1: Detect a noise and trigger the LED

Objective: Detect a loud noise with the microphone and make the LED blink

### Report

To detect when the snap you use the pin connected to the microphone as interrupt. When the microphone detects a loud sound the interrupt will be triggered.

The led is made blinking with TIM2.

### Professor notes

Note: In Part a did you noticed any bouncing problems? Try to solve it.

We could solve it by limiting the trigger frequency. This can be done by keeping track of the last time we toggled the LED and checking if the current interrupt is being triggered too soon.

```cpp
void pin_irq() {
    currentTime = HAL_GetTick();

    if(currentTime - previousTime > TRIGGER_PERIOD) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        previousTime = currentTime;
    }
}
```

## 2: Play a song

Objective: Play a song using the speaker, without using the `HAL_Delay` function

### Report

We created two functions:
- `playNote` setups the timer to produce the note's frequency
- `playSong` loops trough the song's notes, uses `playNote` and then `HAL_Delay`

The microphone interrupt sets a flag that allows the main loop to trigger the start of the song.

To avoid `HAL_Delay` we used TIM2 as a counter to schedule the next change of note after the duration of the current note.

### Professor notes

Note: Strange behavior with the interrupt priority

`HAL_Delay` is not meant to be used inside an interrupt. So the solution we used was correct (a flag to trigger the main loop).

## 3. Use UART with DMA / Write text on the LCD

Objectives:
1. Send a string every second to you PC with UART using DMA
2. Write on the LCD text that changes every second

### Report

To send data over UART once a second, we used TIM2 to create a periodic interrupt, which triggered the UART transmission with DMA.

We used TIM2 to generate a periodic interrupt, which triggered the update of the LCD.

## 4. Read the potentiometer with the ADC and show the value via UART or LCD

Objective: Read the potentiometer voltage using a timer to trigger a conversion at a regular conversion rate of 1Hz and show it first via UART and then via LCD

### Report

We need to trigger the ADC conversion with a timer. TIM2 is setup to run at 1Hz

### Professor notes

Notes: Use `HAL_TIM_Base_Start` instead of `HAL_TIM_Base_Start_IT`

Ok

## 5. ADC scan with DMA

Objectives:
1. Read 3 voltages (potentiometer, temperature sensor and Vref) every second and send them over UART. Data must be saved into the microcontroller with DMA
2. Read the LDR resistance value every millisecond and send the average value over UART every second

### Report

Setup the ADC in "regular conversion mode" and select the channels you want to convert. Enable the DMA request to make the ADC store each sample automatically into memory.

### Professor notes

Note 1: `while(!resultValid)` is blocking, use a more efficient way

Instead of using a while loop in the main with the `HAL_Delay` function, use a timer to start the ADC and in the main loop check if the flag has been set. This way you are not blocking the loop when nothing needs to be done. The ADC will need to be configured in "external trigger mode" to be triggered by the timer.

Note 2: To optimize the project you should have used the ADC with DMA to fill the 1000 element buffer (thus the conversion complete callback is called only once all the 1000 elements are converted)

Ok

## 6a: Read the temperature from the LM75 sensor with I2C

## 6b: Read the accelerometer

Objective: Read the acceleration measured by the accelerometer and send it via UART every second with DMA for both I2C and UART

## 7: Write symbols on the LED matrix

## 8: Read a quadrature encoder

Objective: Read a quadrature encoder, using the specific mode of STM32 timers, in order to provide the rotation frequency (expressed in rpm) and direction (“+” for clockwise and “-” for counterclockwise). 

## 9: Read keypad inputs

## 10: Half-duplex communication between boards

Objective: Implement an half-duplex link to enable communication between multiple boards, using the infrared led and photodiode available on the development board. 