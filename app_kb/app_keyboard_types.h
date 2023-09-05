/*******************************************************************************
* File Name: app_keyboard_types.h
*
* Description: This file consists of the function prototypes that are
*              necessary for developing Keyscan use cases.
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
#include "app_keyscan.h"

#define SPL_FN_ALL_KEYS_SIZE        ((uint8_t)18)
#define CHANNEL_SELECT_KEYS_SIZE    ((uint8_t)3)
#define OSMODE_SELECT_KEYS_SIZE     ((uint8_t)3)

// #define CHANNEL_SELECT_KEYS_SIZE    (sizeof(device_channel_map) / sizeof(channel_t))
// #define OSMODE_SELECT_KEYS_SIZE     (sizeof(os_mode_map) / sizeof(osMode_t))

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

cy_stc_keyscan_context_t context;

bool events_pending;
volatile bool key_press_detected = false;

/* enum for os types */
typedef enum Mode
{
    WINDOWS,
    IOS,
    ANDROID,
} os_mode_t;

/* enum for device channels */
typedef enum Channel
{
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
} ble_channel_t;

typedef struct special_fuction
{
    bool fn_pressed;
    bool channel_key_pressed;
    os_mode_t os_mode;
    ble_channel_t ble_channel;
    uint16_t report_type;
} spl_fun_struct_t;

/* this enum captures all the expected key codes expected in the keyscan */
typedef enum
{
    KEYCODE_Q = 8,
    KEYCODE_TAB = 9,
    KEYCODE_A = 10,
    KEYCODE_ESC = 11,
    KEYCODE_Z = 12,
    KEYCODE_GRAVE = 13,
    KEYCODE_TILDE = 14,
    KEYCODE_1 = 15,
    KEYCODE_W = 16,
    KEYCODE_CAPSLOCK = 17,
    KEYCODE_S = 18,
    KEYCODE_X = 20,
    KEYCODE_F1 = 22,
    KEYCODE_2 = 23,
    KEYCODE_E = 24,
    KEYCODE_F3 = 25,
    KEYCODE_D = 26,
    KEYCODE_F4 = 27,
    KEYCODE_C = 28,
    KEYCODE_F2 = 30,
    KEYCODE_3 = 31,
    KEYCODE_R = 32,
    KEYCODE_T = 33,
    KEYCODE_F = 34,
    KEYCODE_G = 35,
    KEYCODE_V = 36,
    KEYCODE_B = 37,
    KEYCODE_5 = 38,
    KEYCODE_4 = 39,
    KEYCODE_U = 40,
    KEYCODE_Y = 41,
    KEYCODE_J = 42,
    KEYCODE_H = 43,
    KEYCODE_M = 44,
    KEYCODE_N = 45,
    KEYCODE_6 = 46,
    KEYCODE_7 = 47,
    KEYCODE_I = 48,
    KEYCODE_RIGHTBRACE = 49,
    KEYCODE_K = 50,
    KEYCODE_F6 = 51,
    KEYCODE_COMMA = 52,
    KEYCODE_EQUAL = 54,
    KEYCODE_8 = 55,
    KEYCODE_O = 56,
    KEYCODE_F7 = 57,
    KEYCODE_L = 58,
    KEYCODE_DOT = 60,
    KEYCODE_F8 = 62,
    KEYCODE_9 = 63,
    KEYCODE_UP = 115,
    KEYCODE_LEFT = 117,
    KEYCODE_RIGHT = 141,
    KEYCODE_LOPTION = 129,
    KEYCODE_ROPTION = 4,
    KEYCODE_LEFTCTRL = 74,
    KEYCODE_F5 = 7,
    KEYCODE_FN = 6,
    KEYCODE_LCOMMAND = 75,
    KEYCODE_RCOMMAND = 77,
    KEYCODE_F12 = 79,
    KEYCODE_P = 64,
    KEYCODE_LEFTBRACE = 65,
    KEYCODE_SEMICOLON = 66,
    KEYCODE_APOSTROPHE = 67,
    KEYCODE_SLASH = 69,
    KEYCODE_MINUS = 70,
    KEYCODE_0 = 71,
    KEYCODE_BACKSPACE = 81,
    KEYCODE_BACKSLASH = 82,
    KEYCODE_ENTER = 84,
    KEYCODE_F9 = 86,
    KEYCODE_F10 = 87,
    KEYCODE_LEFTSHIFT = 121,
    KEYCODE_RIGHTSHIFT = 122,
    KEYCODE_SPACE = 107,
    KEYCODE_F11 = 108,
    KEYCODE_DOWN = 109,
    KEYCODE_DELETE = 110,
} eKeycode_t;

/* data type for key code to hid code table */
typedef struct Keymap
{
    eKeycode_t key_code;
    uint16_t hid_code;
}KeycodeMap_t;

/* data type qused in special fn mapping table */
typedef struct mode_report
{
    uint16_t hid_code;
    uint16_t report_type;
}mode_report_t;

/* data type for channel switching */
typedef struct channel
{
    eKeycode_t key_code;
    ble_channel_t channel;
}channel_t;
channel_t keycodeChannel;

/* key code to os mode data type */
typedef struct osMode
{
    eKeycode_t key_code;
    os_mode_t os_mode;
}osMode_t;

/* data type for special function mapping table */
typedef struct Key_SplFn_map
{
    eKeycode_t key_code;
    mode_report_t os_mode_report[OSMODE_SELECT_KEYS_SIZE];
}Keycode_SplFn_Map_t;

/* key code to hid code mapping table for keyboard keys */
KeycodeMap_t KeyCode_HID_Map[]=
{
    /* key code                 HID code */
    {KEYCODE_Q,                 HID_Q,},
    {KEYCODE_TAB,               HID_TAB,},
    {KEYCODE_A,                 HID_A,},
    {KEYCODE_ESC,               HID_ESC,},
    {KEYCODE_Z,                 HID_Z,},
    {KEYCODE_TILDE,             HID_GRAVE,},
    {KEYCODE_1,                 HID_1,},
    {KEYCODE_W,                 HID_W,},
    {KEYCODE_CAPSLOCK,          HID_CAPSLOCK,},
    {KEYCODE_S,                 HID_S,},
    {KEYCODE_X,                 HID_X,},
    {KEYCODE_F1,                HID_F1,},
    {KEYCODE_2,                 HID_2,},
    {KEYCODE_E,                 HID_E,},
    {KEYCODE_F3,                HID_F3,},
    {KEYCODE_D,                 HID_D,},
    {KEYCODE_F4,                HID_F4,},
    {KEYCODE_C,                 HID_C,},
    {KEYCODE_F2,                HID_F2,},
    {KEYCODE_3,                 HID_3,},
    {KEYCODE_R,                 HID_R,},
    {KEYCODE_T,                 HID_T,},
    {KEYCODE_F,                 HID_F,},
    {KEYCODE_G,                 HID_G,},
    {KEYCODE_V,                 HID_V,},
    {KEYCODE_B,                 HID_B,},
    {KEYCODE_5,                 HID_5,},
    {KEYCODE_4,                 HID_4,},
    {KEYCODE_U,                 HID_U,},
    {KEYCODE_Y,                 HID_Y,},
    {KEYCODE_J,                 HID_J,},
    {KEYCODE_H,                 HID_H,},
    {KEYCODE_M,                 HID_M,},
    {KEYCODE_N,                 HID_N,},
    {KEYCODE_6,                 HID_6,},
    {KEYCODE_7,                 HID_7,},
    {KEYCODE_I,                 HID_I,},
    {KEYCODE_RIGHTBRACE,        HID_RIGHTBRACE,},
    {KEYCODE_K,                 HID_K,},
    {KEYCODE_F6,                HID_F6,},
    {KEYCODE_COMMA,             HID_COMMA,},
    {KEYCODE_EQUAL,             HID_EQUAL,},
    {KEYCODE_8,                 HID_8,},
    {KEYCODE_O,                 HID_O,},
    {KEYCODE_F7,                HID_F7,},
    {KEYCODE_L,                 HID_L,},
    {KEYCODE_DOT,               HID_DOT,},
    {KEYCODE_F8,                HID_F8,},
    {KEYCODE_9,                 HID_9,},
    {KEYCODE_UP,                HID_UP,},
    {KEYCODE_LEFT,              HID_LEFT,},
    {KEYCODE_RIGHT,             HID_RIGHT,},
    {KEYCODE_LOPTION,           HID_LEFTALT,},
    {KEYCODE_ROPTION,           HID_RIGHTALT,},
    {KEYCODE_FN,                HID_FN,},
    {KEYCODE_F5,                HID_F5,},
    {KEYCODE_LEFTCTRL,          HID_LEFTCTRL,},
    {KEYCODE_LCOMMAND,          HID_LEFTMETA,},
    {KEYCODE_RCOMMAND,          HID_RIGHTMETA,},
    {KEYCODE_F12,               HID_F12,},
    {KEYCODE_P,                 HID_P,},
    {KEYCODE_LEFTBRACE,         HID_LEFTBRACE,},
    {KEYCODE_SEMICOLON,         HID_SEMICOLON,},
    {KEYCODE_APOSTROPHE,        HID_APOSTROPHE,},
    {KEYCODE_SLASH,             HID_SLASH,},
    {KEYCODE_MINUS,             HID_MINUS,},
    {KEYCODE_0,                 HID_0,},
    {KEYCODE_BACKSPACE,         HID_BACKSPACE,},
    {KEYCODE_BACKSLASH,         HID_BACKSLASH,},
    {KEYCODE_ENTER,             HID_ENTER,},
    {KEYCODE_F9,                HID_F9,},
    {KEYCODE_F10,               HID_F10,},
    {KEYCODE_LEFTSHIFT,         HID_LEFTSHIFT,},
    {KEYCODE_RIGHTSHIFT,        HID_RIGHTSHIFT,},
    {KEYCODE_SPACE,             HID_SPACE,},
    {KEYCODE_F11,               HID_F11,},
    {KEYCODE_DOWN,              HID_DOWN,},
    {KEYCODE_DELETE,            HID_DELETE,},

};

/* key code to hid code mapping table for function key combinations */
Keycode_SplFn_Map_t KeyCode_Special_Function_Map[]=
{
    /*Key Code             windows HID                    windows report type                  ios HID                                ios report type              Android HID                             Android report type  */ 
    {KEYCODE_ESC ,       {{ HID_CC_HOME ,                 HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_HOME ,                  HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_HOME ,                  HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F1 ,        {{ HID_RETURN ,                  HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_CC_BRIGHTNESS_DOWN ,       HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_RETURN ,                HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F2 ,        {{ HID_CC_EMAIL ,                HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_BRIGHTNESS_UP ,         HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_EMAIL ,                 HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F3 ,        {{ HID_COMPOSE ,                 HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_CC_VIRTUAL_KEYBOARD ,      HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_MENU ,                  HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F4 ,        {{ HID_CC_MEDIA_PLAYER ,         HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_SYSRQ  ,                   HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_CC_MEDIA_PLAYER ,          HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F5 ,        {{ HID_CC_SEARCH ,               HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_SEARCH ,                HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_SEARCH ,                HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F6 ,        {{ HID_CC_LANGUAGE_EXCHANGE ,    HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_LANGUAGE_EXCHANGE ,     HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_LANGUAGE_EXCHANGE ,     HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F7 ,        {{ HID_CC_PREVIOUS_TRACK ,       HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_PREVIOUS_TRACK ,        HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_PREVIOUS_TRACK ,        HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F8 ,        {{ HID_CC_PLAY_PAUSE ,           HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_PLAY_PAUSE ,            HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_PLAY_PAUSE ,            HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F9 ,        {{ HID_CC_NEXT_TRACK ,           HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_NEXT_TRACK ,            HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_NEXT_TRACK ,            HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F10 ,       {{ HID_CC_MUTE ,                 HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_MUTE ,                  HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_MUTE ,                  HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F11 ,       {{ HID_CC_VOLUME_DOWN ,          HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_VOLUME_DOWN ,           HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_VOLUME_DOWN ,           HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_F12 ,       {{ HID_CC_VOLUME_UP ,            HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_VOLUME_UP ,             HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_VOLUME_UP ,             HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_DELETE ,    {{ HID_CC_SCREEN_LOCK ,          HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_SCREEN_LOCK ,           HDLC_HIDS_CC_IN_REPORT_VALUE} ,     {HID_CC_SCREEN_LOCK ,           HDLC_HIDS_CC_IN_REPORT_VALUE}} ,},
    {KEYCODE_LEFT ,      {{ HID_HOME ,                    HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_HOME ,                     HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_HOME ,                     HDLC_HIDS_KBD_IN_REPORT_VALUE}} ,},
    {KEYCODE_RIGHT ,     {{ HID_END ,                     HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_END ,                      HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_END ,                      HDLC_HIDS_KBD_IN_REPORT_VALUE}} ,},
    {KEYCODE_UP ,        {{ HID_PAGEUP ,                  HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_PAGEUP ,                   HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_PAGEUP ,                   HDLC_HIDS_KBD_IN_REPORT_VALUE}} ,},
    {KEYCODE_DOWN ,      {{ HID_PAGEDOWN ,                HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_PAGEDOWN ,                 HDLC_HIDS_KBD_IN_REPORT_VALUE} ,    {HID_PAGEDOWN ,                 HDLC_HIDS_KBD_IN_REPORT_VALUE}} ,},
};

/* key code to device channel */
channel_t device_channel_map[] =
{
    {KEYCODE_1 , CHANNEL_1},
    {KEYCODE_2 , CHANNEL_2},
    {KEYCODE_3 , CHANNEL_3},
};

/* key code to os type */
osMode_t os_mode_map[] =
{
    {KEYCODE_Q , IOS},
    {KEYCODE_W , ANDROID},
    {KEYCODE_E , WINDOWS},
};