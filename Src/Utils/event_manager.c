/*
 * event_manager.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


#include "event_manager.h"
#include <string.h>

#define EVENT_QUEUE_SIZE 16

static Event_t event_queue[EVENT_QUEUE_SIZE];
static volatile uint8_t head = 0;
static volatile uint8_t tail = 0;

void event_manager_init(void) {
    head = 0;
    tail = 0;
    memset(event_queue, 0, sizeof(event_queue));
}

bool push_event(Event_t event) {
    uint8_t next_head = (head + 1) % EVENT_QUEUE_SIZE;
    if (next_head == tail) {
        // Queue full
        return false;
    }
    event_queue[head] = event;
    head = next_head;
    return true;
}

Event_t pop_event(void) {
    if (head == tail) {
        return EVENT_NONE;  // Queue empty
    }
    Event_t event = event_queue[tail];
    tail = (tail + 1) % EVENT_QUEUE_SIZE;
    return event;
}
