#ifndef domabot_exception_h
#define domabot_exception_h

#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Domabot {

namespace Exception {

  std::runtime_error backTrack(
    const std::string& file,
    const int& line,
    const std::string& func,
    const std::exception& child,
    const std::string& msg = ""
  );

  template <typename Last>
  void create(std::stringstream &stream, Last&& last) {
    stream << std::forward<Last>(last);
  }

  template <typename First, typename... Args>
  void create(std::stringstream& stream, First&& first, Args&&... args) {
    stream << std::forward<First>(first);
    create(stream, std::forward<Args>(args)...);
  }

  template <typename... Args>
  std::runtime_error create(Args&&... args) {
    return std::runtime_error([&args...]() {
      std::stringstream stream;
      create(stream, std::forward<Args>(args)...);
      return stream.str();
    }());
  }

  #define BacktrackMsg(child, msg) backTrack(__FILE__, __LINE__, __func__, child, msg)
  #define Backtrack(child) backTrack(__FILE__, __LINE__, __func__, child)

} // Exception

#define defaultCatch catch (const std::exception& e) { throw Domabot::Exception::Backtrack(e); }

} // Domabot

#endif
