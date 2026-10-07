/**
 * @brief A small polling timer for scheduling loose, non-blocking tasks.
 *
 * Call isReady() regularly from loop() or a controller update method. The
 * timer never calls a function itself and never waits: the caller decides what
 * to do when the interval has elapsed, then calls resetTimer().
 */
class TaskTimer_c {
  
   long interval_ms;
   long last_reset_ms;
  
  /** @brief Creates a timer with an interval in milliseconds. */
   TaskTimer_c( long interval_ms ) {
      this.interval_ms = interval_ms;
      this.last_reset_ms = 0;
   }
   /** @brief Changes the interval used by later readiness checks. */
   void setIntervalMS( long new_interval_ms ) {
     this.interval_ms = new_interval_ms;
   }
   /**
     * @brief Returns true when the interval has elapsed since the last reset.
     * @param now_ms Time from the caller's clock, in milliseconds.
     * @note This method does not reset the timer. 
     */
   boolean isReady( long now_ms ) {
      return now_ms - last_reset_ms >= interval_ms;
   }
   /** @brief Convenience overload using the Arduino millisecond clock. */
   boolean isReady() {
      return isReady( millis() ); 
   }
   /** @brief Starts the next interval from a caller-supplied time. */
   void resetTimer( long now_ms ) {
     last_reset_ms = now_ms;
     
   }
   /** @brief Convenience overload using the Arduino millisecond clock. */
   void resetTimer() {
     resetTimer( millis() );
   } 
}
