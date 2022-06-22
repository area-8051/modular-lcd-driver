/*
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * Copyright (c) 2022 Vincent DEFERT. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "project-defs.h"
#include "lcd-graphics.h"
#include "lcd-controller.h"

/**
 * @file lcd-graphics.c
 * 
 * Graphics operations for LCD devices: implementation.
 */

static uint8_t reverseBits(uint8_t byte) {
	uint8_t result = 0;
	uint8_t mask = 0x80;
	
	for (uint8_t i = 8; i; i--) {
		result = (result >> 1) | ((byte & mask) ? 0x80 : 0);
		mask = mask >> 1;
	}
	
	return result;
}

static void __lcd_gfxSetGraphicsDisplayAddress(LCD_Device *device, uint16_t pixelX, uint16_t pixelY) {
	if (device->displayBuffer) {
		device->__displayAddressX = pixelX & ~7;
		device->__displayAddressY = pixelY;
	} else {
		lcd_setGraphicsDisplayAddress(device, pixelX, pixelY);
	}
}

static void __lcd_gfxUpdateExtents(LCD_Device *device) {
	if (device->__displayAddressX < device->__minExtentX) {
		device->__minExtentX = device->__displayAddressX;
	}
	
	if (device->__displayAddressX > device->__maxExtentX) {
		device->__maxExtentX = device->__displayAddressX;
	}
	
	if (device->__displayAddressY < device->__minExtentY) {
		device->__minExtentY = device->__displayAddressY;
	}
	
	if (device->__displayAddressY > device->__maxExtentY) {
		device->__maxExtentY = device->__displayAddressY;
	}
}

static uint16_t __lcd_gfxGetAndAutoIncrementByteOffset(LCD_Device *device) {
	uint16_t result = (device->__displayAddressY * device->__bytesWidth) + (device->__displayAddressX / 8);
	
	// For consistency, we reproduce the behaviour of the underlying 
	// LCD controller when working with the buffer.
	device->__displayAddressX += 8;
	
	if (device->__displayAddressX >= device->pixelWidth) {
		device->__displayAddressX = 0;
	}
	
	return result;
}

static uint8_t __lcd_gfxReadByte(LCD_Device *device) {
	uint8_t result = 0;
	
	if (device->displayBuffer) {
		result = device->displayBuffer[__lcd_gfxGetAndAutoIncrementByteOffset(device)];
	} else {
		result = lcd_readByte(device);
	}
	
	return result;
}

static void __lcd_gfxWriteByte(LCD_Device *device, uint8_t byte) {
	if (device->displayBuffer) {
		__lcd_gfxUpdateExtents(device);
		device->displayBuffer[__lcd_gfxGetAndAutoIncrementByteOffset(device)] = byte;
	} else {
		lcd_writeByte(device, byte);
	}
}

static void __lcd_gfxBeginOperation(LCD_Device *device) {
	if (device->displayBuffer && !device->__batchStarted) {
		device->__batchStarted = 1;
		device->__minExtentX = device->pixelWidth;
		device->__minExtentY = device->pixelHeight;
		device->__maxExtentX = 0;
		device->__maxExtentY = 0;
	}
}

static void __lcd_gfxEndOperation(LCD_Device *device) {
	if (device->displayBuffer && device->__autoUpdate) {
		lcd_gfxUpdateDisplay(device);
	}
}

void lcd_gfxInitialiseDisplayMode(LCD_Device *device) {
	device->__autoUpdate = 0;
	device->__batchStarted = 0;
	lcd_gfxClear(device);
}

void lcd_gfxEnableAutoUpdate(LCD_Device *device) {
	// A graphics display buffer must be available to use batch updates.
	if (device->displayBuffer) {
		device->__autoUpdate = 1;
	}
}

void lcd_gfxDisableAutoUpdate(LCD_Device *device) {
	lcd_gfxUpdateDisplay(device);
	device->__autoUpdate = 0;
}

void lcd_gfxUpdateDisplay(LCD_Device *device) {
	if (device->displayBuffer && device->__batchStarted) {
		device->__batchStarted = 0;
		
		for (uint16_t y = device->__minExtentY; y <= device->__maxExtentY; y++) {
			lcd_setGraphicsDisplayAddress(device, device->__minExtentX, y);
			__lcd_gfxSetGraphicsDisplayAddress(device, device->__minExtentX, y);
			
			for (uint16_t x = device->__minExtentX; x <= device->__maxExtentX; x += 8) {
				lcd_writeByte(device, device->displayBuffer[__lcd_gfxGetAndAutoIncrementByteOffset(device)]);
			}
		}
	}
}

void lcd_gfxClear(LCD_Device *device) {
	// This is the fastest way to clear the graphics display (< 2 ms)
	lcd_disableGraphicsDisplay(device);
	lcd_clearTextDisplay(device);
	lcd_enableGraphicsDisplay(device);
	
	// But don't forget to update our buffer if we're using one.
	if (device->displayBuffer) {
		for (uint16_t y = 0; y < device->pixelHeight; y++) {
			__lcd_gfxSetGraphicsDisplayAddress(device, 0, y);
			
			for (uint16_t col = 0; col < device->__bytesWidth; col++) {
				__lcd_gfxWriteByte(device, 0);
			}
		}
		
		device->__batchStarted = 0;
	}
}

void lcd_gfxPoint(LCD_Device *device, uint16_t x, uint16_t y, uint8_t colour) {
	__lcd_gfxBeginOperation(device);
	
	__lcd_gfxSetGraphicsDisplayAddress(device, x, y);
	uint16_t word = (__lcd_gfxReadByte(device) << 8) | __lcd_gfxReadByte(device);
	uint16_t bitMask = 1 << (15 - x % 16);
	
	// Set the bit according to the requested "colour"
	if (colour) {
		word |= bitMask;
	} else {
		word &= ~bitMask;
	}
	
	__lcd_gfxSetGraphicsDisplayAddress(device, x, y);
	__lcd_gfxWriteByte(device, word >> 8);
	__lcd_gfxWriteByte(device, word);
	
	__lcd_gfxEndOperation(device);
}

void lcd_gfxXbmImage(LCD_Device *device, 
	uint16_t imageWidth, uint16_t imageHeight, uint8_t *imageBits, 
	LCD_Alignment alignment, uint16_t positionX, uint16_t positionY) {
	__lcd_gfxBeginOperation(device);
	
	// XBM format: if imageWidth is not a multiple of 8, unused bits in the last byte are ignored.
	uint16_t bytesPerImageRow = imageWidth / 8 + ((imageWidth % 8) ? 1 : 0);
	// The LCD display expects to receive pairs of bytes.
	uint16_t bytesPerDisplayRow = bytesPerImageRow + (bytesPerImageRow % 2);
	uint16_t offsetX = 0;
	uint16_t offsetY = 0;
	
	switch (alignment) {
	case LCD_ALIGN_TOP_LEFT:
	case LCD_ALIGN_BOTTOM_LEFT:
	case LCD_ALIGN_MIDDLE_LEFT:
		offsetX = 0;
		break;
	case LCD_ALIGN_TOP_RIGHT:
	case LCD_ALIGN_BOTTOM_RIGHT:
	case LCD_ALIGN_MIDDLE_RIGHT:
		offsetX = (device->pixelWidth > imageWidth) ? (device->pixelWidth - imageWidth) : 0;
		break;
	case LCD_ALIGN_TOP_CENTER:
	case LCD_ALIGN_BOTTOM_CENTER:
	case LCD_ALIGN_MIDDLE_CENTER:
		offsetX = (device->pixelWidth > imageWidth) ? ((device->pixelWidth - imageWidth) / 2) : 0;
		break;
	case LCD_ALIGN_CUSTOM_OFFSET:
		offsetX = positionX;
		break;
	}
	
	switch (alignment) {
	case LCD_ALIGN_TOP_LEFT:
	case LCD_ALIGN_TOP_RIGHT:
	case LCD_ALIGN_TOP_CENTER:
		offsetY = 0;
		break;
	case LCD_ALIGN_BOTTOM_LEFT:
	case LCD_ALIGN_BOTTOM_RIGHT:
	case LCD_ALIGN_BOTTOM_CENTER:
		offsetY = (device->pixelHeight > imageHeight) ? (device->pixelHeight - imageHeight) : 0;
		break;
	case LCD_ALIGN_MIDDLE_LEFT:
	case LCD_ALIGN_MIDDLE_RIGHT:
	case LCD_ALIGN_MIDDLE_CENTER:
		offsetY = (device->pixelHeight > imageHeight) ? ((device->pixelHeight - imageHeight) / 2) : 0;
		break;
	case LCD_ALIGN_CUSTOM_OFFSET:
		offsetY = positionY;
		break;
	}
	
	for (uint16_t row = 0; row < imageHeight; row++) {
		__lcd_gfxSetGraphicsDisplayAddress(device, offsetX, offsetY + row);
		
		for (uint16_t col = 0; col < bytesPerDisplayRow; ) {
			uint8_t msByte = imageBits[row * bytesPerImageRow + col];
			col++;
			uint8_t lsByte = (col < bytesPerImageRow) ? imageBits[row * bytesPerImageRow + col] : 0;
			col++;
			// XBM format: the leftmost pixel is represented by the least significant bit,
			// so XBM bytes need to be reversed before being sent to the display.
			__lcd_gfxWriteByte(device, reverseBits(msByte));
			__lcd_gfxWriteByte(device, reverseBits(lsByte));
		}
	}
	
	__lcd_gfxEndOperation(device);
}
