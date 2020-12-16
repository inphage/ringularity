/**
 * Copyright (c) 2020 Open Ring Project, All rights reserved
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without 
 * restriction, including without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef __UI_H__
#define __UI_H__

#include "bsp.h"

#define DEFAULT_UI_ENABLED 1

// replace functions with alternate definitions if needed
#define UI_INDICATION_SET default_ui$indication_set
#define UI_ON_BSP_EVENT_SLEEP default_ui$on_bsp_event_sleep
#define UI_ON_BSP_EVENT_DISCONNECT default_ui$on_bsp_event_disconnect
#define UI_ON_BSP_EVENT_WHITELIST_OFF default_ui$on_bsp_event_whitelist_off
#define UI_ON_BSP_EVENT_KEY_0 default_ui$on_bsp_event_key_0
#define UI_INIT default_ui$init
#define UI_ON_BSP_EVENT default_ui$on_bsp_evt
#define UI_SLEEP_MODE_PREPARE default_ui$sleep_mode_prepare

void default_ui$indication_set(bsp_indication_t indication);
void default_ui$init(bool * p_erase_bonds);
void default_ui$sleep_mode_prepare();

typedef void (*bsp_evt_handler_t)(bsp_event_t event);
void advertising$on_bsp_event(bsp_event_t event);
void ble$on_bsp_event(bsp_event_t event);
void srvcon$health_thermometer_service$on_bsp_event(bsp_event_t event);
void srvcon$current_time_service_client$on_bsp_event(bsp_event_t event);
void srvcon$pulse_oximeter_service$on_bsp_event(bsp_event_t event);
#define BSP_EVT_HANDLERS(handlers) bsp_evt_handler_t handlers[] = {\
        srvcon$current_time_service_client$on_bsp_event,\
        srvcon$pulse_oximeter_service$on_bsp_event,\
        srvcon$health_thermometer_service$on_bsp_event,\
        advertising$on_bsp_event,\
        ble$on_bsp_event\
    }


#endif