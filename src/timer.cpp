/*                                                                         80->|
 * timer.cpp
 *
 * Modifications: James William Dunn
 *          Date: June 15, 2017
 */

#include "timer.h"

Timer1::Timer1() {
  start();
}
Timer1::~Timer1() {}

void Timer1::start() {
  start_ = high_resolution_clock::now();
}

Timer1::ms Timer1::duration() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  Timer1::ms durat = std::chrono::duration_cast<ms>(now - start_);
  start_ = now;
  return durat;
}