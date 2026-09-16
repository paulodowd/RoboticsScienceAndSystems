#ifndef TASK_TIMER_H
#define TASK_TIMER_H

#include <Arduino.h>

/**
 * @brief A small polling timer for scheduling loose, non-blocking tasks.
 *
 * Call isReady() regularly from loop() or a controller update method. The
 * timer never calls a function itself and never waits: the caller decides what
 * to do when the interval has elapsed, then calls resetTimer().
 */
class TaskTimer_c {
  public:
    /** @brief Creates a timer with an interval in milliseconds. */
    TaskTimer_c(unsigned long interval_ms = 0);

    /** @brief Changes the interval used by later readiness checks. */
    void setIntervalMS(unsigned long new_interval_ms);

    /**
     * @brief Returns true when the interval has elapsed since the last reset.
     * @param now_ms Time from the caller's clock, in milliseconds.
     * @note This method does not reset the timer. Unsigned subtraction remains
     * correct when the millisecond clock wraps around.
     */
    bool isReady(unsigned long now_ms) const;

    /** @brief Convenience overload using the Arduino millisecond clock. */
    bool isReady() const;

    /** @brief Starts the next interval from a caller-supplied time. */
    void resetTimer(unsigned long now_ms);

    /** @brief Convenience overload using the Arduino millisecond clock. */
    void resetTimer();

  private:
    unsigned long interval_ms;
    unsigned long last_reset_ms;
};

#endif
