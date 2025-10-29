#pragma once

#include <chrono>
#include <string>
#include <utility>

/// \brief A timer class for measuring elapsed time.
class Timer {
 public:
  /// \brief Constructs a new Timer object.
  Timer() = default;

  /// \brief Starts the performance timer.
  /// \details This function starts the performance timer, which can be used to
  /// measure the execution time of a block of code. Call End() to stop the
  /// timer and get the elapsed time.
  /// \note This function is not thread-safe.
  void Start();

  /// \brief Stops the performance timer and returns the elapsed time.
  /// \details This function stops the performance timer and returns the elapsed
  /// time in milliseconds. It also outputs the stdout with a prefix "TIMER". It
  /// will automatically start a new timer.
  /// \param msg The message prefix to output.
  /// \return The elapsed time in milliseconds.
  /// \note This function is not thread-safe.
  uint64_t End(const std::string &msg);

  /// \brief Deleted copy constructor.
  Timer(const Timer &) = delete;
  /// \brief Deleted copy assignment operator.
  Timer &operator=(const Timer &) = delete;

 private:
  // in ms.
  // uint64_t start_time_;
  // uint64_t end_time_;

  /// \brief The start time of the timer in nanoseconds.
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  /// \brief The end time of the timer in nanoseconds.
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time_;
};

/// \brief A wrapper class for Timer.
class TimerWrapper {
 public:
  /// \brief Constructs a new TimerWrapper object.
  /// \param msg The message prefix to output.
  explicit TimerWrapper(std::string msg) : msg_(std::move(msg)) {
    timer_.Start();
  }

  /// \brief Deconstructs the TimerWrapper object.
  ~TimerWrapper() { timer_.End(msg_); }

  /// \brief Deleted copy constructor.
  TimerWrapper(const TimerWrapper &) = delete;
  /// \brief Deleted copy assignment operator.
  TimerWrapper &operator=(const TimerWrapper &) = delete;

 private:
  /// \brief The timer object.
  Timer timer_;
  /// \brief The message prefix to output.
  std::string msg_;
};

/// \brief Gets the full name of a function.
/// \details This function gets the full name of a function. The full name
/// includes name within the '(' and ')' characters.
/// \param full_name The full name of the function.
/// \return The full name of the function.
inline std::string get_full_name(const std::string &full_name) {
  std::size_t end = full_name.find('(');

  if (end == std::string::npos) {
    return full_name;
  }

  std::string new_str = full_name.substr(0, end);
  std::size_t start = new_str.rfind(' ');

  if (start == std::string::npos) {
    return full_name;
  }

  return new_str.substr(start + 1);
}

/// \brief Gets the full name of a function with an indicator.
/// \param full_name The full name of the function.
/// \param indicator The indicator to add to the full name.
/// \return The full name of the function with the indicator.
inline std::string get_full_name(const std::string &full_name,
                                 const std::string &indicator) {
  return indicator + "_" + get_full_name(full_name);
}

/// \brief 使用宏定义巧妙的禁用计时测试
#ifdef DISABLE_PERF

// disable macros.
#define MFLA_TIMER_FUNCTION()

#define MFLA_TIMER_FUNCTION_WITH_INDICATOR(indicator)

#define MFLA_TIMER_BLOCK_START()

#define MFLA_TIMER_BLOCK_END(msg)

#define MFLA_TIMER_BLOCK_END_WITH_INDICATOR(indicator, msg)

#else

/// \brief Macro to initialize a timer and start it.
#define MFLA_TIMER_FUNCTION() \
  TimerWrapper _timer_wrapper_(get_full_name(__PRETTY_FUNCTION__))

/// \brief Macro to initialize a timer and start it with an indicator.
#define MFLA_TIMER_FUNCTION_WITH_INDICATOR(indicator) \
  TimerWrapper _timer_wrapper_(get_full_name(__PRETTY_FUNCTION__, indicator))

/// \brief Macro to start a timer.
#define MFLA_TIMER_BLOCK_START() \
  Timer _timer_;                 \
  _timer_.Start()

/// \brief Macro to stop a timer and output the elapsed time.
#define MFLA_TIMER_BLOCK_END(msg) _timer_.End(msg)

/// \brief Macro to stop a timer and output the elapsed time with an indicator.
#define MFLA_TIMER_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(indicator + "_" + msg)

#endif
