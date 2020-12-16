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
#ifndef __PULSE_OXIMETER_SERVICE_CONTROLLER_H__
#define __PULSE_OXIMETER_SERVICE_CONTROLLER_H__

void srvcon$pulse_oximeter_service$init();
void srvcon$pulse_oximeter_service$start();

void srvcon$pulse_oximeter_service$on_pm_evt(pm_evt_t const * p_evt);
void srvcon$pulse_oximeter_service$on_bsp_event(bsp_event_t event);
void srvcon$pulse_oximeter_service$on_ble_evt(ble_evt_t const * p_ble_evt);

#endif //__PULSE_OXIMETER_SERVICE_CONTROLLER_H__