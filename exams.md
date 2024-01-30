Modify homework 2: stop the song if the blue button is pressed

You need to configure pin PC13 with GPIO_EXTI13. In the NVIC enable the EXTI line[15:10] interrupt. In the code when the interrupt is triggered stop the song.

Modify homework 3: continuously show the names until you snap your finger, then only show the last name blinking at 1Hz

You need to set PA8 as GPIO_EXTI8. In the NVIC enable the EXTI line[9:5] interrupt. In the code, when the microphone interrupt fires, change a flag to tell the LCD loop what to show.

Modify homework 4: use the voltage of the potentiometer to regulate the intensity of light of the LED

The LED is connected to TIM2 CH1 and to make it blink we need to produce a PWM signal. TIM2 is used to trigger the ADC, so first you need to use another timer (TIM3) for the ADC. Then you can configure TIM2 with the correct timing and enable PWM mode for CH1. In the main you do `HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)` and in the ADC interrupt you update the CCR1 register.

Modify homework 6b: send to the terminal the average of 1000 values along the 3 axes every second

