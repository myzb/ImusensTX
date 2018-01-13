/*
 * Stopwatch.h
 *
 *  Created on: 22 Jul 2017
 *      Author: may
 */

#ifndef _STOPWATCH_H_
#define _STOPWATCH_H_

class stopwatch {

public:
    stopwatch();
    void reset();
    float split();
    float peek();
    float split_lp();
    float peek_lp();

private:
    uint32_t  _previous_micros, _start_micros, _dt_us;
    uint32_t  _previous_millis, _start_millis, _dt_ms;

};

#endif /* _STOPWATCH_H_ */
