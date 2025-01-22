# stm32_neopixel

A simple code example for controlling WS2812 LED strip with stm32 controller.

I've tested this on STM32 F303RE Nucleo board with 10 LED strip.

I've used CubeMX to setup configuration and CubeIDE to write the code. The setup is very simple.

## CubeMX settings

First thing you need to do - is to look up the timer frequency. You can use any timer you want, they all have the same frequency - notated as APB1 (timer clocks). In my case it was 72 MHz.

![frequency](https://github.com/KLelkov/stm32_neopixel/blob/master/Images/frequency.png?raw=true)

Next you need to configure the timer. Once again, you can pick any - they all are linked to different GPIO pins. In my case I used TIM1 channel one, which is linked to PC0 pin.

You need to change the following settings:

- set clock source to *Internal Clock*
- set desired channel to *PWM Generation*
- set prescaler to *0*
- set pulse to *0*
- set CH polarity to *High*
- set counter period (ARR). This one a little tricky. For WS2812 you want 1.25 microseconds pulse width. To achieve this you need `ARR = 1.25 * timer_freq - 1`. Where timer_freq is the exact frequency (in MHz) you looked up on the first step. In my case `ARR = 1.25 * 72 - 1 = 89`.

![timer](https://github.com/KLelkov/stm32_neopixel/blob/master/Images/timer.png?raw=true)

Setup GPIO parameters for timer pin according to the picture below

![gpio](https://github.com/KLelkov/stm32_neopixel/blob/master/Images/gpio.png?raw=true)

Setup DMA parameters as follows

![dma.png](https://github.com/KLelkov/stm32_neopixel/blob/master/Images/dma.png?raw=true)

That's it for CubeMX config, now lets write (or copy) some code :)

## Code

All the code is located inside `main.c` file - it should be easy to follow.

First we define the structure to store color values for each LED

```/* USER CODE BEGIN PTD */
/* USER CODE BEGIN PTD */
// This structure is used to conveniently form DMA buffer from RGB values
typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

/* USER CODE END PTD */
```

Then we setup LED strip parameters. Notice that the `NEOPIXEL_ZERO` and `NEOPIXEL_ONE` are calculated manually based on your `ARR` value.

```
/* USER CODE BEGIN PD */
// LOW and HIGH values for pixels are easily calculated from the Counter Period (ARR) from CubeMX
#define NEOPIXEL_ZERO 29  // (ARR + 1) * 0.32
#define NEOPIXEL_ONE 58   // (ARR + 1) * 0.64
						  // round everything to the nearest integer
#define NUM_PIXELS 10  // Number of LEDs
#define DMA_BUFF_SIZE (NUM_PIXELS * 24) + 1

/* USER CODE END PD */
```

After that we define a couple of global arrays so we can control the LEDs individually (so when we change the color of one LED the rest of them stays the same)

```
/* USER CODE BEGIN PV */
PixelRGB_t pixel[NUM_PIXELS] = {0};  // Used to store color values of each led
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};  // Apparently DMA buffer needs to be incremental,
										  // so you cant just create a new one on each
										  // function call
/* USER CODE END PV */
```

And finally we define these three functions:

```
/* USER CODE BEGIN 0 */

// Needed to break pwm cycle (to stop changing leds, because led adresses start acting funny)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
}

void set_led(int id, int red, int green, int blue)
{
	uint32_t *pBuff;
	pixel[id].color.g = green;
	pixel[id].color.r = red;
	pixel[id].color.b = blue;

	pBuff = dmaBuffer;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
	 for (int j = 23; j >= 0; j--)
	 {
	   if ((pixel[i].data >> j) & 0x01)
	   {
		 *pBuff = NEOPIXEL_ONE;
	   }
	   else
	   {
		 *pBuff = NEOPIXEL_ZERO;
	   }
	   pBuff++;
	 }
	}
	dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
}

void clear_leds()
{
	uint32_t *pBuff;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
		pixel[i].color.g = 0;
		pixel[i].color.r = 0;
		pixel[i].color.b = 0;
	}

	pBuff = dmaBuffer;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
	 for (int j = 23; j >= 0; j--)
	 {
	   if ((pixel[i].data >> j) & 0x01)
	   {
		 *pBuff = NEOPIXEL_ONE;
	   }
	   else
	   {
		 *pBuff = NEOPIXEL_ZERO;
	   }
	   pBuff++;
	 }
	}
	dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
}

/* USER CODE END 0 */
```

The first one `HAL_TIM_PWM_PulseFinishedCallback` is a service function, you don't need to worry about it. The second one `set_led` we will be actually using to control LEDs. And the third one `clear_leds` will be used to clear (turn off) all the LEDs at once.

In `HAL_TIM_PWM_PulseFinishedCallback` replace the `TIM_CHANNEL_1` with whatever channel you are using (remember, Im using `TIM1 CH1`)

In `set_led` and `clear_leds` replace `&htim1` and `TIM_CHANNEL_1` with whatever timer and channels you are using.

**That's it!** Now you can easily control your LED strip with these two functions just like this inside your `main()`function:

```
/* USER CODE BEGIN 2 */
  clear_leds();
  set_led(8, 100, 0, 0); // Yup, it is that easy
  set_led(9, 100, 0, 0);

  /* USER CODE END 2 */
```

This is the basics. You can build up from there and create more complex LED strip color effects :)





