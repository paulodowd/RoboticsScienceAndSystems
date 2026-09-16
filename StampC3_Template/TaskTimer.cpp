#include "TaskTimer.h"

TaskTimer_c::TaskTimer_c(unsigned long interval_ms)
  : interval_ms(interval_ms), last_reset_ms(0) {
}

void TaskTimer_c::setIntervalMS(unsigned long new_interval_ms) {
  interval_ms = new_interval_ms;
}

bool TaskTimer_c::isReady(unsigned long now_ms) const {
  return now_ms - last_reset_ms >= interval_ms;
}

bool TaskTimer_c::isReady() const {
  return isReady(millis());
}

void TaskTimer_c::resetTimer(unsigned long now_ms) {
  last_reset_ms = now_ms;
}

void TaskTimer_c::resetTimer() {
  resetTimer(millis());
}
