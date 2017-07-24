/*
 * MetroExt.h
 *
 *  Created on: 22 Jul 2017
 *      Author: may
 *
 * Based on the original Metro library by Thomas Ouellet Fredericks, with additional support
 * for high resolution clock.
 */

#ifndef _METROEXT_H_
#define _METROEXT_H_

#include <inttypes.h>

class MetroExt {

public:
    MetroExt(unsigned int interval_micros);
    MetroExt(unsigned int interval_micros, uint8_t autoreset);
    void interval(unsigned int interval_micros);
    char check();
    void reset();

private:
    uint8_t autoreset;
    unsigned int  previous_micros, interval_micros;

};

#endif /* _METROEXT_H_ */


