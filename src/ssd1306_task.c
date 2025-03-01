/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ssd1306_task.c
 * @defgroup 
 * @{
 * 
 * image to byte converter: https://mischianti.org/images-to-byte-array-online-converter-cpp-arduino/
 * 
 * https://www.mischianti.org/2021/07/14/ssd1306-oled-display-draw-images-splash-and-animations-2/
 * https://iitestudent.blogspot.com/2013/01/displaying-bitmap-on-graphic-lcd.html
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>


#include <ssd1306_task.h>
#include <font_latin_8x8.h>
#include <bitmap_icon.h>



static void ssd1306_full_demo(ssd1306_handle_t handle) {
	int center = 1, top = 1, bottom = 4;
	char lineChar[16];
	uint8_t image[24];

	ESP_LOGI(APP_TAG, "Full Demo");

	// Display x3 text
	ESP_LOGI(APP_TAG, "Display x3 Text");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text_x3(handle, 0, "Hello", 6, false);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Display bitmap icons
	ESP_LOGI(APP_TAG, "Display bitmap icons");
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, 31, 0, data_rx_icon_32x32, 32, 32, false);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	ssd1306_display_bitmap(handle, 31, 0, data_tx_icon_32x32, 32, 32, false);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Display x2 text
	ESP_LOGI(APP_TAG, "Display x2 Text");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text_x2(handle, 0, "{xTEXTx}", 9, false);
	ssd1306_display_text_x2(handle, 2, " X2-X2", 7, false);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Display text
	ESP_LOGI(APP_TAG, "Display Text");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, 0, "SSD1306 128x64", 15, false);
	ssd1306_display_text(handle, 1, "Hello World!!", 14, false);
	ssd1306_display_text(handle, 2, "SSD1306 128x64", 15, true);
	ssd1306_display_text(handle, 3, "Hello World!!", 14, true);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	// Display Count Down
	ESP_LOGI(APP_TAG, "Display Count Down");
	memset(image, 0, sizeof(image));
	ssd1306_display_image(handle, top, (6*8-1), image, sizeof(image));
	ssd1306_display_image(handle, top+1, (6*8-1), image, sizeof(image));
	ssd1306_display_image(handle, top+2, (6*8-1), image, sizeof(image));
	for(int font = 0x39; font > 0x30; font--) {
		memset(image, 0, sizeof(image));
		ssd1306_display_image(handle, top+1, (7*8-1), image, 8);
		memcpy(image, font8x8_latin_tr[font], 8);
		if (handle->dev_config.flip_enabled) ssd1306_flip_buffer(image, 8);
		ssd1306_display_image(handle, top+1, (7*8-1), image, 8);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
	// Scroll Up
	ESP_LOGI(APP_TAG, "Scroll Up");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, 0, "---Scroll UP---", 16, true);
	ssd1306_set_software_scroll(handle, (handle->pages - 1), 1);
	for (int line = 0; line < bottom+10; line++) {
		lineChar[0] = 0x01;
		sprintf(&lineChar[1], " Line %02d", line);
		ssd1306_display_scroll_text(handle, lineChar, strlen(lineChar), false);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	// Scroll Down
	ESP_LOGI(APP_TAG, "Scroll Down");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, 0, "--Scroll DOWN--", 16, true);
	ssd1306_set_software_scroll(handle, 1, (handle->pages - 1) );
	for (int page = 0; page < bottom+10; page++) {
		lineChar[0] = 0x02;
		sprintf(&lineChar[1], " Line %02d", page);
		ssd1306_display_scroll_text(handle, lineChar, strlen(lineChar), false);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Page Down
	ESP_LOGI(APP_TAG, "Page Down");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, 0, "---Page	DOWN---", 16, true);
	ssd1306_set_software_scroll(handle, 1, (handle->pages-1) );
	for (int page = 0; page < bottom+10; page++) {
		if ( (page % (handle->pages-1)) == 0) ssd1306_clear_scroll_display(handle);
		lineChar[0] = 0x02;
		sprintf(&lineChar[1], " Line %02d", page);
		ssd1306_display_scroll_text(handle, lineChar, strlen(lineChar), false);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ssd1306_clear_scroll_display(handle);

	// Horizontal Scroll
	ESP_LOGI(APP_TAG, "Horizontal Scroll");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, center, "Horizontal", 11, false);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_RIGHT, SSD1306_SCROLL_2_FRAMES);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_LEFT, SSD1306_SCROLL_2_FRAMES);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_STOP, SSD1306_SCROLL_2_FRAMES);
	
	// Vertical Scroll
	ESP_LOGI(APP_TAG, "Vertical Scroll");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, center, "Vertical", 9, false);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_DOWN, SSD1306_SCROLL_2_FRAMES);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_UP, SSD1306_SCROLL_2_FRAMES);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	ssd1306_set_hardware_scroll(handle, SSD1306_SCROLL_STOP, SSD1306_SCROLL_2_FRAMES);

	// Bitmaps
	ESP_LOGI(APP_TAG, "Bitmaps");
	ssd1306_clear_display(handle, false);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, 1, "BATMAN", 7, false);
	int bitmap_width = 4*8;
	int xpos = (handle->width/2) - (bitmap_width/2); // center of width
	int ypos = 20;
	ESP_LOGD(APP_TAG, "width=%d xpos=%d", handle->width, xpos);
	ssd1306_display_bitmap(handle, xpos, ypos, batman_icon_32x13, 32, 13, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	for(int i=0;i<128;i++) {
		ssd1306_set_display_wrap_around(handle, SSD1306_SCROLL_RIGHT, 2, 3, 0);
	}
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, ((handle->width/2)-(64/2)), 0, radioactive_icon_64x64, 64, 64, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, ((handle->width/2)-(70/2)), 0, biohazard_icon_70x64, 70, 64, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, ((handle->width/2)-(50/2)), 0, skull_icon_50x64, 50, 64, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, ((handle->width/2)-(64/2)), 0, proton_icon_64x64, 64, 64, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	ssd1306_clear_display(handle, false);
	ssd1306_display_bitmap(handle, ((handle->width/2)-(64/2)), 0, molecule_icon_64x64, 64, 64, false);
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	// Invert
	ESP_LOGI(APP_TAG, "Invert");
	ssd1306_clear_display(handle, true);
	ssd1306_set_display_contrast(handle, 0xff);
	ssd1306_display_text(handle, center, "  Good Bye!!", 13, true);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Fade Out
	ESP_LOGI(APP_TAG, "Fade Out");
	ssd1306_fadeout_display(handle);
}

static void ssd1306_contrast_demo(ssd1306_handle_t handle) {
	const uint8_t iterations = 255;

	char lineChar[16];
	uint8_t contrast = 0;
	uint8_t direction = 0;

	ESP_LOGI(APP_TAG, "Contrast Demo");

	ssd1306_enable_display(handle);

	for(uint8_t i = 0; i < iterations; i+=5) {
		ssd1306_set_display_contrast(handle, contrast);

		sprintf(lineChar, "Contrast %02d", i);
		ssd1306_display_text(handle, 1, lineChar, strlen(lineChar), true);

		if (contrast == 0xff) direction = -1;
		if (contrast == 0x00) direction = 1;

		contrast += direction;

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	ssd1306_clear_display(handle, false);

	vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void i2c0_ssd1306_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ssd1306_config_t dev_cfg         = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ssd1306_handle_t dev_hdl;
    //
    // init device
    ssd1306_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ssd1306 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SSD1306 - START #########################");
		//
		// panel size
		if(dev_hdl->dev_config.panel_size == SSD1306_PANEL_128x32) {
			ESP_LOGI(APP_TAG, "Display Panel: 128x32");
		} else if(dev_hdl->dev_config.panel_size == SSD1306_PANEL_128x64) {
			ESP_LOGI(APP_TAG, "Display Panel: 128x64");
		} else {
			ESP_LOGI(APP_TAG, "Display Panel: 128x128");
		}
        //
		// full demo
		ssd1306_full_demo(dev_hdl);
		//
		// contrast demo
		ssd1306_contrast_demo(dev_hdl);
        //
        ESP_LOGI(APP_TAG, "######################## SSD1306 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE + 50 );
    }
    //
    // free resources
    ssd1306_delete( dev_hdl );
    vTaskDelete( NULL );
}