/*                                                                         80->|
 * timer.h
 *
 * Modifications: James William Dunn
 *          Date: June 15, 2017
 */

#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer1 {
 public:
  using high_resolution_clock = std::chrono::high_resolution_clock;
  using ms = std::chrono::milliseconds;
  Timer1();
  ~Timer1();
  void start();
  ms duration();

 private:
  high_resolution_clock::time_point start_;
};
#endif /* TIMER_H */