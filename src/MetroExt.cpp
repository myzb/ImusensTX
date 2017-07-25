/*
 * MetroExt.cpp
 *
 *  Created on: 22 Jul 2017
 *      Author: may
 *
 *  Notice:
 *    Based on the original Metro library by Thomas Ouellet Fredericks. Adjusted to query for
 *    high resolution clock.
 */

#include <Arduino.h>

#include "MetroExt.h"

MetroExt::MetroExt(unsigned int interval_micros)
{
    this->autoreset = 0;
    interval(interval_micros);
    reset();
}

// New creator so I can use either the original check behavior or benjamin.soelberg's
// suggested one (see below).
// autoreset = 0 is benjamin.soelberg's check behavior
// autoreset != 0 is the original behavior

MetroExt::MetroExt(unsigned int interval_micros, uint8_t autoreset)
{
    this->autoreset = autoreset; // Fix by Paul Bouchier
    interval(interval_micros);
    reset();
}

void MetroExt::interval(unsigned int interval_micros)
{
    this->interval_micros = interval_micros;
}

// Benjamin.soelberg's check behavior:
// When a check is true, add the interval to the internal counter.
// This should guarantee a better overall stability.

// Original check behavior:
// When a check is true, add the interval to the current micros() counter.
// This method can add a certain offset over time.

char MetroExt::check()
{
    if (micros() - this->previous_micros >= this->interval_micros) {
        // Backup prevouis micros for requeue
        this->previous_micros_old = this->previous_micros;
        // As suggested by benjamin.soelberg@gmail.com, the following line
        // this->previous_micros = micros();
        // was changed to
        // this->previous_micros += this->interval_micros;

        // If the interval is set to 0 we revert to the original behavior
        if (this->interval_micros <= 0 || this->autoreset) {
            this->previous_micros = micros();
        } else {
            this->previous_micros += this->interval_micros;
        }
        return 1;
    }
    return 0;
}

void MetroExt::reset()
{
    this->previous_micros = micros();
}

// Should be called after a check to assert the next check as true
void MetroExt::requeue()
{
    // Set the updated previous_micros to the old previous_micros
    // This allows the next check() to instantly return true
    this->previous_micros = this->previous_micros_old;
}
