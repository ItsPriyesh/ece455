/********************************************************************
 * 
 *  Author: Carlos Moreno (cmoreno@uwaterloo.ca)
 *
 *  Copyright info:
 *      The use of this file is restricted to the Lab component of 
 *      the ECE-455 course at the University of Waterloo.  Usage 
 *      is restricted to students attending the course.
 *
 *      Redistribution of this file in any form is not permitted.
 *
 *  Usage:
 *      Add a line with  #include "fault_injection.h"  in any of 
 *      your C source files where this functionality is used and 
 *      include fault_injection.c in your project so that it gets 
 *      compiled together with your code.
 *
 ********************************************************************/

#ifndef FAULT_INJECTION_H
#define FAULT_INJECTION_H

#define RANDOM_FAULT    0
#define STUCK_AT_FAULT  1

void fault_injection_reset(void);
int faulty_int (int original_value, int fault_type);
double faulty_double (double original_value, int fault_type);

#endif

