/*******************************************************************************
* File Name: app_hid_codes.h
*
* Description: This file consists of the function definitions that are
*              necessary for developing Keyscan/keyboard use cases.
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#define HID_NONE    0x00 /* No key pressed */
#define HID_ERR_OVF 0x01 /* Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")*/

#define HID_A 0x04 /* Keyboard a and A */
#define HID_B 0x05 /* Keyboard b and B */
#define HID_C 0x06 /* Keyboard c and C */
#define HID_D 0x07 /* Keyboard d and D */
#define HID_E 0x08 /* Keyboard e and E */
#define HID_F 0x09 /* Keyboard f and F */
#define HID_G 0x0a /* Keyboard g and G */
#define HID_H 0x0b /* Keyboard h and H */
#define HID_I 0x0c /* Keyboard i and I */
#define HID_J 0x0d /* Keyboard j and J */
#define HID_K 0x0e /* Keyboard k and K */
#define HID_L 0x0f /* Keyboard l and L */
#define HID_M 0x10 /* Keyboard m and M */
#define HID_N 0x11 /* Keyboard n and N */
#define HID_O 0x12 /* Keyboard o and O */
#define HID_P 0x13 /* Keyboard p and P */
#define HID_Q 0x14 /* Keyboard q and Q */
#define HID_R 0x15 /* Keyboard r and R */
#define HID_S 0x16 /* Keyboard s and S */
#define HID_T 0x17 /* Keyboard t and T */
#define HID_U 0x18 /* Keyboard u and U */
#define HID_V 0x19 /* Keyboard v and V */
#define HID_W 0x1a /* Keyboard w and W */
#define HID_X 0x1b /* Keyboard x and X */
#define HID_Y 0x1c /* Keyboard y and Y */
#define HID_Z 0x1d /* Keyboard z and Z */

#define HID_1 0x1e /* Keyboard 1 and ! */
#define HID_2 0x1f /* Keyboard 2 and @ */
#define HID_3 0x20 /* Keyboard 3 and # */
#define HID_4 0x21 /* Keyboard 4 and $ */
#define HID_5 0x22 /* Keyboard 5 and % */
#define HID_6 0x23 /* Keyboard 6 and ^ */
#define HID_7 0x24 /* Keyboard 7 and & */
#define HID_8 0x25 /* Keyboard 8 and * */
#define HID_9 0x26 /* Keyboard 9 and ( */
#define HID_0 0x27 /* Keyboard 0 and ) */

#define HID_ENTER 0x28 /* Keyboard Return (ENTER) */
#define HID_ESC 0x29 /* Keyboard ESCAPE */
#define HID_BACKSPACE 0x2a /* Keyboard DELETE (Backspace) */
#define HID_TAB 0x2b /* Keyboard Tab */
#define HID_SPACE 0x2c /* Keyboard Spacebar */
#define HID_MINUS 0x2d /* Keyboard - and _ */
#define HID_EQUAL 0x2e /* Keyboard = and + */
#define HID_LEFTBRACE 0x2f /* Keyboard [ and { */
#define HID_RIGHTBRACE 0x30 /* Keyboard ] and } */
#define HID_BACKSLASH 0x31 /* Keyboard \ and | */
#define HID_HASHTILDE 0x32 /* Keyboard Non-US # and ~ */
#define HID_SEMICOLON 0x33 /* Keyboard ; and : */
#define HID_APOSTROPHE 0x34 /* Keyboard ' and " */
#define HID_GRAVE 0x35 /* Keyboard ` and ~ */
#define HID_COMMA 0x36 /* Keyboard , and < */
#define HID_DOT 0x37 /* Keyboard . and > */
#define HID_SLASH 0x38 /* Keyboard / and ? */
#define HID_CAPSLOCK 0x39 /* Keyboard Caps Lock */

#define HID_F1 0x3a /* Keyboard F1 */
#define HID_F2 0x3b /* Keyboard F2 */
#define HID_F3 0x3c /* Keyboard F3 */
#define HID_F4 0x3d /* Keyboard F4 */
#define HID_F5 0x3e /* Keyboard F5 */
#define HID_F6 0x3f /* Keyboard F6 */
#define HID_F7 0x40 /* Keyboard F7 */
#define HID_F8 0x41 /* Keyboard F8 */
#define HID_F9 0x42 /* Keyboard F9 */
#define HID_F10 0x43 /* Keyboard F10 */
#define HID_F11 0x44 /* Keyboard F11 */
#define HID_F12 0x45 /* Keyboard F12 */

#define HID_SYSRQ 0x46 /* Keyboard Print Screen */
#define HID_SCROLLLOCK 0x47 /* Keyboard Scroll Lock */
#define HID_PAUSE 0x48 /* Keyboard Pause */
#define HID_INSERT 0x49 /* Keyboard Insert */
#define HID_HOME 0x4a /* Keyboard Home */
#define HID_PAGEUP 0x4b /* Keyboard Page Up */
#define HID_DELETE 0x4c /* Keyboard Delete Forward */
#define HID_END 0x4d /* Keyboard End */
#define HID_PAGEDOWN 0x4e /* Keyboard Page Down */
#define HID_RIGHT 0x4f /* Keyboard Right Arrow */
#define HID_LEFT 0x50 /* Keyboard Left Arrow */
#define HID_DOWN 0x51 /* Keyboard Down Arrow */
#define HID_UP 0x52 /* Keyboard Up Arrow */

#define HID_NUMLOCK 0x53 /* Keyboard Num Lock and Clear */
#define HID_KPSLASH 0x54 /* Keypad / */
#define HID_KPASTERISK 0x55 /* Keypad * */
#define HID_KPMINUS 0x56 /* Keypad - */
#define HID_KPPLUS 0x57 /* Keypad + */
#define HID_KPENTER 0x58 /* Keypad ENTER */
#define HID_KP1 0x59 /* Keypad 1 and End */
#define HID_KP2 0x5a /* Keypad 2 and Down Arrow */
#define HID_KP3 0x5b /* Keypad 3 and PageDn */
#define HID_KP4 0x5c /* Keypad 4 and Left Arrow */
#define HID_KP5 0x5d /* Keypad 5 */
#define HID_KP6 0x5e /* Keypad 6 and Right Arrow */
#define HID_KP7 0x5f /* Keypad 7 and Home */
#define HID_KP8 0x60 /* Keypad 8 and Up Arrow */
#define HID_KP9 0x61 /* Keypad 9 and Page Up */
#define HID_KP0 0x62 /* Keypad 0 and Insert */
#define HID_KPDOT 0x63 /* Keypad . and Delete */

#define HID_102ND 0x64 /* Keyboard Non-US \ and | */
#define HID_COMPOSE 0x65 /* Keyboard Application */
#define HID_POWER 0x66 /* Keyboard Power */
#define HID_KPEQUAL 0x67 /* Keypad = */

#define HID_F13 0x68 /* Keyboard F13 */
#define HID_F14 0x69 /* Keyboard F14 */
#define HID_F15 0x6a /* Keyboard F15 */
#define HID_F16 0x6b /* Keyboard F16 */
#define HID_F17 0x6c /* Keyboard F17 */
#define HID_F18 0x6d /* Keyboard F18 */
#define HID_F19 0x6e /* Keyboard F19 */
#define HID_F20 0x6f /* Keyboard F20 */
#define HID_F21 0x70 /* Keyboard F21 */
#define HID_F22 0x71 /* Keyboard F22 */
#define HID_F23 0x72 /* Keyboard F23 */
#define HID_F24 0x73 /* Keyboard F24 */

#define HID_OPEN 0x74 /* Keyboard Execute */
#define HID_HELP 0x75 /* Keyboard Help */
#define HID_PROPS 0x76 /* Keyboard Menu */
#define HID_FRONT 0x77 /* Keyboard Select */
#define HID_STOP 0x78 /* Keyboard Stop */
#define HID_AGAIN 0x79 /* Keyboard Again */
#define HID_UNDO 0x7a /* Keyboard Undo */
#define HID_CUT 0x7b /* Keyboard Cut */
#define HID_COPY 0x7c /* Keyboard Copy */
#define HID_PASTE 0x7d /* Keyboard Paste */
#define HID_FIND 0x7e /* Keyboard Find */
#define HID_MUTE 0x7f /* Keyboard Mute */
#define HID_VOLUMEUP 0x80 /* Keyboard Volume Up */
#define HID_VOLUMEDOWN 0x81 /* Keyboard Volume Down */

#define HID_RETURN 0x9e  /* keyboard return */
#define HID_KPLEFTPAREN 0xb6 /* Keypad ( */
#define HID_KPRIGHTPAREN 0xb7 /* Keypad ) */

#define HID_LEFTCTRL 0xe0 /* Keyboard Left Control */
#define HID_LEFTSHIFT 0xe1 /* Keyboard Left Shift */
#define HID_LEFTALT 0xe2 /* Keyboard Left Alt */
#define HID_LEFTMETA 0xe3 /* Keyboard Left GUI */
#define HID_RIGHTCTRL 0xe4 /* Keyboard Right Control */
#define HID_RIGHTSHIFT 0xe5 /* Keyboard Right Shift */
#define HID_RIGHTALT 0xe6 /* Keyboard Right Alt */
#define HID_RIGHTMETA 0xe7 /* Keyboard Right GUI */

#define HID_FN    0xffff
#define HID_INVALID    0xffff

#define HID_MEDIA_PLAYPAUSE 0xe8
#define HID_MEDIA_STOPCD 0xe9
#define HID_MEDIA_PREVIOUSSONG 0xea
#define HID_MEDIA_NEXTSONG 0xeb
#define HID_MEDIA_EJECTCD 0xec
#define HID_MEDIA_VOLUMEUP 0xed
#define HID_MEDIA_VOLUMEDOWN 0xee
#define HID_MEDIA_MUTE 0xef
#define HID_MEDIA_WWW 0xf0
#define HID_MEDIA_BACK 0xf1
#define HID_MEDIA_FORWARD 0xf2
#define HID_MEDIA_STOP 0xf3
#define HID_MEDIA_FIND 0xf4
#define HID_MEDIA_SCROLLUP 0xf5
#define HID_MEDIA_SCROLLDOWN 0xf6
#define HID_MEDIA_EDIT 0xf7
#define HID_MEDIA_SLEEP 0xf8
#define HID_MEDIA_COFFEE 0xf9
#define HID_MEDIA_REFRESH 0xfa
#define HID_MEDIA_CALC 0xfb

/* Consumer option HID Codes */
/* Order of consumer key functionality - make sure the order is same as in device configurator */
#define HID_CC_HOME                     (uint8_t)(1)
#define HID_CC_BROWSER                  (uint8_t)(2)
#define HID_CC_EMAIL                    (uint8_t)(3)
#define HID_CC_MENU                     (uint8_t)(4)
#define HID_CC_SCREEN_CAPTURE           (uint8_t)(5)
#define HID_CC_MEDIA_PLAYER             (uint8_t)(6)
#define HID_CC_SEARCH                   (uint8_t)(7)
#define HID_CC_PREVIOUS_TRACK           (uint8_t)(8)
#define HID_CC_PLAY_PAUSE               (uint8_t)(9)
#define HID_CC_NEXT_TRACK               (uint8_t)(10)
#define HID_CC_MUTE                     (uint8_t)(11)
#define HID_CC_VOLUME_DOWN              (uint8_t)(12)
#define HID_CC_VOLUME_UP                (uint8_t)(13)
#define HID_CC_SCREEN_LOCK              (uint8_t)(14)
#define HID_CC_BRIGHTNESS_DOWN          (uint8_t)(15)
#define HID_CC_BRIGHTNESS_UP            (uint8_t)(16)
#define HID_CC_RETURN                   (uint8_t)(17)
#define HID_CC_VIRTUAL_KEYBOARD         (uint8_t)(18) 
#define HID_CC_RIGHT_CLICK              (uint8_t)(19) 
#define HID_CC_LANGUAGE_EXCHANGE        (uint8_t)(20)