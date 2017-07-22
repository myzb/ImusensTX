/*
 * Stopwatch.cpp
 *
 *  Created on: 22 Jul 2017
 *      Author: may
 */

#include <Arduino.h>

#include "Stopwatch.h"

Stopwatch::Stopwatch()
{
    this->_previous_micros = micros();
    _dt = 0;
}

float Stopwatch::Split()
{
    _dt = micros() - this->_previous_micros;
    this->_previous_micros = micros();

    return (float)_dt / 1000000.0f;
}

void Stopwatch::Reset()
{
  this->_previous_micros = micros();
}


