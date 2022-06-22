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

/**
 * @file ST7920.c
 * 
 * Driver for graphics-capable LCD devices using an ST7920 controller.
 * 
 * The ST7920 is found on 12864 LCD devices; these can be used in text 
 * mode (4 lines of 16 latin characters, or 8 Chinese characters), or 
 * in 128x64 pixels graphics mode.
 * 
 * This file implements the abstraction layer described in 
 * lcd-controller.h, which is why it doesn't have its own header file. 
 * This design decision was driven by MCS-51/SDCC restrictions on 
 * function calls made through pointers. As a consequence, if the 
 * application uses several LCD devices, ALL of them **MUST** be based 
 * on the same controller model.
 * 
 * **Pin assignments - PARALLEL interface**
 * 
 * Signal	Description
 * ---------------------------------------------------------------------
 * VSS		Connect to GND
 * VDD		Connect to +5V
 * VO		Contrast adjustment. Connect to +5V for maximum contrast.
 * RS		Register Select. 0 = Instruction, 1 = Data.
 * RW		Transfer direction. 0 = Write, 1 = Read.
 * E		I/O enable. Enables data out when high. Latches data in on falling edge.
 * D0		Data bus (bidirectional).
 * D1
 * D2
 * D3
 * D4
 * D5
 * D6
 * D7
 * BLA		Backlight anode. Connect to +5V.
 * BLK		Backlight cathode. Connect to GND.
 * PSB		Parallel Serial Bit. 0 = Serial, 1 = Parallel. Connect to +5V.
 * 
 * **Pin assignments - SERIAL interface**
 * 
 * Signal	Description
 * ---------------------------------------------------------------------
 * VSS		Connect to GND
 * VDD		Connect to +5V
 * VO		Contrast adjustment. Connect to +5V for maximum contrast.
 * RS		Chip Select (CS)
 * RW		Serial Data (SID)
 * E		Serial Clock (SCLK)
 * BLA		Backlight anode. Connect to +5V.
 * BLK		Backlight cathode. Connect to GND.
 * PSB		Parallel Serial Bit. 0 = Serial, 1 = Parallel. Connect to GND.
 * 
 * **Notes**
 * 
 * - **IMPORTANT**: read operations are not supported in serial mode.
 * This has implications if your application needs to read the display's
 * DDRAM/CGRAM/GDRAM (you need to manage a display buffer, so your MCU 
 * have enough RAM for this).
 * 
 * - If you find backlight too bright, you may connect A to +5V through 
 * a resistor of, say 150 or 180 Ohm.
 * 
 * - If contrast is too weak with VO connected to +5V (text only visible 
 * when viewed under a small angle), check your power supply voltage 
 * (must be greater than 4.5V).
 * 
 * - In 4-bit parallel mode, only D7..D4 are used, and most significant 
 * nibble is always transfered first.
 */

#include "project-defs.h"
#include "lcd-controller.h"
#include "delay.h"

#define WRITE_DELAY 7

static void __waitWhileBusy(LCD_Interface *interface) {
	while (lcd_readStatus(interface) & 0x80) delay10us(1);
}

static void __sendCommand(LCD_Device *device, uint8_t command) {
	__waitWhileBusy(device->interface);
	lcd_sendCommand(device->interface, command);
	device->__status.setAddressInvoked = 0;
	// All instructions except Clear Display take 72us
	delay10us(WRITE_DELAY);
}

static void __sendData(LCD_Interface *interface, uint8_t data) {
	__waitWhileBusy(interface);
	lcd_sendData(interface, data);
	// All data write operations take 72us
	delay10us(WRITE_DELAY);
}

static uint8_t __readData(LCD_Interface *interface) {
	__waitWhileBusy(interface);
	uint8_t result = lcd_readData(interface);
	
	return result;
}

static void __displayControl(LCD_Device *device, uint8_t displayOn, uint8_t cursorOn, uint8_t blinkCursor) {
	// Set entire display on/off
	uint8_t d = displayOn ? 4 : 0;
	// Set cursor on/off
	uint8_t c = cursorOn ? 2 : 0;
	// Cursor blink on/off
	uint8_t s = blinkCursor ? 1 : 0;
	__sendCommand(device, 0x08 | d | c | s);
}

static void __clearTextDisplay(LCD_Device *device) {
	__sendCommand(device, 0x01);
	// Clear Display takes 1.6ms but we already waited for WRITE_DELAY in sendCommand()
	delay10us((uint8_t) (160 - WRITE_DELAY));
}

static void __setEntryMode(LCD_Device *device, uint8_t textDirection, uint8_t shiftDisplay) {
	// 1 = shift display when cursor goes beyond display edge
	// 0 = don't shift
	uint8_t s = shiftDisplay ? 1 : 0;
	// 1 = increment address counter => move cursor right (LTR text) or shift display left, 
	// 0 = decrement address counter => move cursor left (RTL text) or shift display right
	uint8_t id = textDirection ? 2 : 0;
	__sendCommand(device, 0x04 | id | s);
}

void lcd_initialiseController(LCD_Device *device) {
    lcd_linkConfigurationBegins(device->interface);
	
	// Data sheet says to wait for > 40ms
	delay1ms(45);

	if (device->interface->linkType == LCD_ParallelLink) {
		// Configure parallel interface mode (Function Set command)
		if (lcd_getLinkWidth(device->interface) == 8) {
			// 8-bit mode
			__sendCommand(device, 0x30);
			// Data sheet says > 100us and we already waited for WRITE_DELAY
			delay10us(10 - WRITE_DELAY);
			__sendCommand(device, 0x30);
		} else {
			// 4-bit mode
			__sendCommand(device, 0x20);
			// Data sheet says > 100us and we already waited for WRITE_DELAY
			delay10us(10 - WRITE_DELAY);
			__sendCommand(device, 0x20);
			// Data sheet says > 100us and we already waited for WRITE_DELAY
			delay10us(10 - WRITE_DELAY);
		}
	}
    
    lcd_linkConfigurationComplete(device->interface);

	// Turn display on, no cursor, no blinking.
	__displayControl(device, 1, 0, 0);

	// Clear display
    __clearTextDisplay(device);

	// Set Entry Mode (required, even if we want to use the device
	// in graphics mode only).
	__setEntryMode(device, 1, 0);
}

static void __functionSet(LCD_Device *device) {
	uint8_t dl = lcd_getDataLinkWidth(device->interface) == 8 ? 0x10 : 0;
	uint8_t re = device->__status.extendedInstructionSet ? 4 : 0;
	uint8_t g = device->__status.graphicsMode ? 2 : 0;
	__sendCommand(device, 0x20 | dl | re | g);
}

static void __selectExtendedInstructionSet(LCD_Device *device) {
	if (!device->__status.extendedInstructionSet) {
		device->__status.extendedInstructionSet = 1;
		__functionSet(device);
	}
}

static void __selectBasicInstructionSet(LCD_Device *device) {
	if (device->__status.extendedInstructionSet) {
		device->__status.extendedInstructionSet = 0;
		__functionSet(device);
	}
}

static void __setVerticalScrollMode(LCD_Device *device, uint8_t sr) {
	if (device->__status.verticalScroll != sr) {
		__selectExtendedInstructionSet(device);
		// sr == 1 => enable vertical scroll mode
		// sr == 0 => enable "Set CGRAM Address" instruction (basic instruction set)
		device->__status.verticalScroll = sr;
		__sendCommand(device, 0x02 | sr);
	}
}

void lcd_writeByte(LCD_Device *device, uint8_t byte) {
	__sendData(device->interface, byte);
	device->__status.setAddressInvoked = 0;
}

uint8_t lcd_readByte(LCD_Device *device)  {
	// A "dummy" read MUST be performed immediately after 
	// a setAddress command, before reading data bytes.
	if (device->__status.setAddressInvoked) {
		__readData(device->interface);
		device->__status.setAddressInvoked = 0;
	}
	
	return __readData(device->interface);
}

uint8_t lcd_readBusyFlagAndAddress(LCD_Device *device)  {
	return lcd_readStatus(device->interface);
}

void lcd_clearTextDisplay(LCD_Device *device)  {
	__selectBasicInstructionSet(device);
	__clearTextDisplay(device);
}

void lcd_returnHome(LCD_Device *device)  {
	__selectBasicInstructionSet(device);
	__sendCommand(device, 0x02);
}

void lcd_setEntryMode(LCD_Device *device, uint8_t textDirection, uint8_t shiftDisplay)  {
	__selectBasicInstructionSet(device);
	__setEntryMode(device, textDirection, shiftDisplay);
}

void lcd_displayControl(LCD_Device *device, uint8_t displayOn, uint8_t cursorOn, uint8_t blinkCursor)  {
	__selectBasicInstructionSet(device);
	__displayControl(device, displayOn, cursorOn, blinkCursor);
}

void lcd_cursorDisplayShiftControl(LCD_Device *device, uint8_t shiftDisplay, uint8_t shiftRight)  {
	__selectBasicInstructionSet(device);
	// shiftDisplay == 1 && shiftRight == 1 => display shifts right, cursor follows, address counter does not change
	// shiftDisplay == 1 && shiftRight == 0 => display shifts left,  cursor follows, address counter does not change
	// shiftDisplay == 0 && shiftRight == 1 => cursor moves right, address counter is incremented
	// shiftDisplay == 0 && shiftRight == 0 => cursor moves left,  address counter is decremented
	// 1 = Shift display, 0 = Move cursor
	uint8_t sc = shiftDisplay ? 8 : 0;
	// 1 = Shift/move to the right, 0 = Shift/move to the left
	uint8_t rl = shiftRight ? 4 : 0;
	__sendCommand(device, 0x10 | sc | rl);
}

void lcd_enableGraphicsDisplay(LCD_Device *device)  {
	if (!device->__status.graphicsMode) {
		__selectExtendedInstructionSet(device);
		device->__status.graphicsMode = 1;
		__functionSet(device);
	}
}

void lcd_disableGraphicsDisplay(LCD_Device *device)  {
	if (device->__status.graphicsMode) {
		__selectExtendedInstructionSet(device);
		device->__status.graphicsMode = 0;
		__functionSet(device);
	}
}

void lcd_setCharacterGeneratorAddress(LCD_Device *device, uint8_t address)  {
	__setVerticalScrollMode(device, 0); // Required (see datasheet)
	__selectBasicInstructionSet(device);
	__sendCommand(device, 0x40 | (address & 0x3F));
	device->__status.setAddressInvoked = 1;
}

void lcd_setTextDisplayAddress(LCD_Device *device, uint8_t address) {
	__selectBasicInstructionSet(device);
	__sendCommand(device, 0x80 | (address & 0x3f));
	device->__status.setAddressInvoked = 1;
}

void lcd_setTextDisplayPosition(LCD_Device *device, uint8_t row, uint8_t column)  {
	lcd_setTextDisplayAddress(device, ((row & 0x01) << 4) | ((row & 0x02) ? 0x08 : 0x00) | ((column >> 1) & 0x07));
}

void lcd_setGraphicsDisplayAddress(LCD_Device *device, uint16_t pixelX, uint16_t pixelY) {
	__selectExtendedInstructionSet(device);
	uint8_t colAddr = pixelX >> 4;
	uint8_t verticalAddr = pixelY & 0x1f;
	uint8_t horizontalAddr = (colAddr & 0x07) | ((pixelY & 0x20) ? 0x08 : 0);
	__sendCommand(device, 0x80 | verticalAddr);
	__sendCommand(device, 0x80 | horizontalAddr);
	device->__status.setAddressInvoked = 1;
}

void lcd_enterStandbyMode(LCD_Device *device)  {
	__selectExtendedInstructionSet(device);
	__sendCommand(device, 0x01);
	// Any other subsequent command terminates standby mode,
	// so there's no need for a leaveStandbyMode() function.
}

void lcd_reverseRow(LCD_Device *device, uint8_t row)  {
	__selectExtendedInstructionSet(device);
	// Toggles the appearance of the given row between normal and reverse mode.
	__sendCommand(device, 0x04 | (row & 3));
}

void lcd_setScrollAddress(LCD_Device *device, uint8_t address)  {
	__selectExtendedInstructionSet(device);
	__sendCommand(device, 0x40 | (address & 0x3f));
}

void lcd_enableVerticalScroll(LCD_Device *device) {
	__setVerticalScrollMode(device, 1);
}

void lcd_disableVerticalScroll(LCD_Device *device) {
	__setVerticalScrollMode(device, 0);
}
