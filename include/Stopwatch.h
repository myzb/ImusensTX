/*
 * Stopwatch.h
 *
 *  Created on: 22 Jul 2017
 *      Author: matt
 */

#ifndef _STOPWATCH_H_
#define _STOPWATCH_H_

class Stopwatch {

public:
    Stopwatch();
    void Reset();
    float Split();

private:
    uint32_t  _previous_micros, _dt;

};

#endif /* _STOPWATCH_H_ */
