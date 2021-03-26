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
#ifndef __ATOMIX_H__
#define __ATOMIX_H__
//not exactly a multi-threaded environment, basically one main thread and many ISRs (including the timed events), 
// but methods of synchronization/signaling are still helpful

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_atomic.h"
#include "nrf_assert.h"

#define ATOMIX_EVT_SET      1
#define ATOMIX_EVT_CLEAR    0

typedef nrf_atomic_u32_t atomix_evt_t;

__STATIC_INLINE void atomix_evt_init(atomix_evt_t * p_evt);

__STATIC_INLINE void atomix_evt_destroy(atomix_evt_t * p_evt);

__STATIC_INLINE bool atomix_evt_set(nrf_mtx_t * p_evt);

__STATIC_INLINE void atomix_evt_clear(atomix_evt_t * p_evt);

__STATIC_INLINE void atomix_evt_check(atomix_evt_t * p_evt);

#ifndef SUPPRESS_INLINE_IMPLEMENTATION

__STATIC_INLINE void atomix_evt_init(atomix_evt_t * p_evt)
{
    ASSERT(p_evt != NULL);

    *p_evt = ATOMIX_EVT_CLEAR;
    __DMB();
}

__STATIC_INLINE void atomix_evt_destroy(atomix_evt_t * p_evt)
{
    ASSERT(p_evt  != NULL);

    // Add memory barrier to ensure that any memory operations protected by the event complete
    // before the mutex is destroyed.
    __DMB();

    *p_evt = ATOMIX_EVT_CLEAR;
}

__STATIC_INLINE void atomix_evt_set(atomix_evt_t * p_evt)
{
    ASSERT(p_evt  != NULL);

    *p_evt = NRF_EVT_SET;

    // Add memory barrier to ensure that the mutex is locked before any memory operations protected
    // by the mutex are started.
    __DMB();
    __SEV();
}

__STATIC_INLINE void atomix_evt_clear(atomix_evt_t *p_evt)
{
    ASSERT(p_evt  != NULL);
    ASSERT(*p_evt == ATOMIX_EVT_SET);

    // Add memory barrier to ensure that any memory operations protected by the mutex complete
    // before the mutex is unlocked.
    __DMB();

    *p_evt = ATOMIX_EVT_CLEAR;
}

__STATIC_INLINE bool atomix_evt_check(atomix_evt_t *p_evt) {
    __DMB();
    return (*p_evt == ATOMIX_EVT_SET);
}

void nil_proc(void *) {}
__STATIC_INLINE bool atomix_evt_wait() {
    atomic_evt_idle_wait(nil_proc, NULL);
}

//calls idle function while waiting for event to get set
void (*pfn_idle_t)(void *p_param);
__STATIC_INLINE bool atomix_evt_idle_wait(atomix_evt_t *p_evt, pfn_idle_t pfn_idle, void *p_param) {
    ASSERT(p_evt  != NULL);
    ASSERT(pfn_idle  != NULL);
    while(!atomix_evt_check(p_evt)) {
        (*pfn_idle)(p_param);
        __WFE();
        (*pfn_idle)(p_param);
    }
}

#endif //SUPPRESS_INLINE_IMPLEMENTATION

#endif // __ATOMIX_H__
