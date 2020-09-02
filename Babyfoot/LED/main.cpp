
/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "WS2812.h"
#include "PixelArray.h"

/* Defines -------------------------------------------------------------------*/
#define NUM_COLORS 		3
#define WS2812_BUF 		10

/* Objects -------------------------------------------------------------------*/
Serial pc (USBTX, USBRX, 115200);
PixelArray px_b(WS2812_BUF);
// PixelArray px_r(WS2812_BUF);
WS2812 ws_b(D10, WS2812_BUF, 0, 5, 5, 0);
// WS2812 ws_r(D9, WS2812_BUF, 0, 5, 5, 0);


/********************** Prototypes **********************/
void led_set_buffer (void);
void led_set_intensity (uint8_t led_intensity);
void led_display_italian_flag (void);
void led_black (void);

/* Variables -----------------------------------------------------------------*/

/* Define colors to be used, here we are using red, blue, black */
int colorbuf[NUM_COLORS] = {0x00002f, 0x002f00, 0x000000};

int main()
{
	ws_b.useII(WS2812::PER_PIXEL);
	// ws_r.useII(WS2812::PER_PIXEL);
	led_set_buffer();
	/* Low intensity, better for the video */
	led_set_intensity(100);
	thread_sleep_for(100);
	led_black();
	thread_sleep_for(100);
	while(1) {
		/* Display leds & play music */
		led_display_italian_flag();
		thread_sleep_for(1000);
		led_black();
		thread_sleep_for(1000);
	}
}

/* Functions definition ------------------------------------------------------*/
void led_set_buffer ()
{
	/* Red when score */
	for (int i = 0; i < WS2812_BUF / 2; i++)
	{
		px_b.Set(i, colorbuf[0]);
		// px_r.Set(i, colorbuf[0]);
	}
	/* Black to stop using leds */
	for (int i = WS2812_BUF / 2; i < WS2812_BUF; i++) {
		px_b.Set(i, colorbuf[1]);
		// px_r.Set(i, colorbuf[4]);
	}
}

void led_set_intensity (uint8_t led_intensity)
{
	/* Define the same intensity for green & red leds */
	for (int j = 0; j < WS2812_BUF; j++) {
		px_b.SetI(j, led_intensity);
		// px_r.SetI(j, led_intensity);
	}
}

void led_display_italian_flag ()
{
	/* Display italian flag */
	ws_b.write(px_b.getBuf());

	// ws_r.write(px_r.getBuf());
}

void led_black ()
{
	/* Display the 6 leds in black = leds off */
	ws_b.write_offsets(px_b.getBuf(), 0, 0, 0);
	// ws_r.write_offsets(px_r.getBuf(), WS2812_BUF / 2, WS2812_BUF / 2, WS2812_BUF / 2);
}
