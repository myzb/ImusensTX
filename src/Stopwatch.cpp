/*
 * Stopwatch.cpp
 *
 *  Created on: 22 Jul 2017
 *      Author: may
 */

#include <Arduino.h>

#include "Stopwatch.h"

stopwatch::stopwatch()
{
    _previous_micros = _start_micros = micros();
    _previous_millis = _start_millis = millis();
    _dt_us = 0;
    _dt_ms = 0;
}

float stopwatch::split()
{
    _dt_us = micros() - _previous_micros;
    _previous_micros = micros();

    return (float)_dt_us / 1000000.0f;
}

float stopwatch::peek()
{
    return (float)(micros() - _start_micros) / 1000000.0f;
}

float stopwatch::split_lp()
{
    _dt_ms = millis() - _previous_millis;
    _previous_millis = millis();

    return (float)_dt_ms / 1000.0f;
}

float stopwatch::peek_lp()
{
    return (float)(millis() - _start_millis) / 1000.0f;
}

void stopwatch::reset()
{
    _previous_micros = _start_micros = micros();
    _previous_millis = _start_millis = millis();
}
