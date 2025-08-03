#ifndef Domabot_Exception_h
#define Domabot_Exception_h

#include <functional>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Domabot {

class Exception : public std::runtime_error {
  protected:
    using Childs =  std::list<std::string>;

    Childs m_childs = {};

    template <typename Last>
    static void createMsg(std::stringstream &stream, Last&& last) {
      stream << std::forward<Last>(last);
    }

    template <typename First, typename... Args>
    static void createMsg(std::stringstream& stream, First&& first, Args&&... args) {
      stream << std::forward<First>(first);
      createMsg(stream, std::forward<Args>(args)...);
    }

  public:
    Exception() = delete;

    Exception(const std::string& msg = "");
    Exception(const std::exception& e);

    Exception(const Exception&) = default;
    Exception(Exception&&) = default;
    Exception& operator=(const Exception&) = default;
    Exception& operator=(Exception&&) = default;

    virtual ~Exception() = default;

    bool isChildsEmpty() const noexcept;
    void add(const std::exception& child);
    void add(const std::string& child);

    void checkSelf();

    std::string toString() const;
    std::runtime_error toRuntimeError() const;

    static std::runtime_error backTrack(
      const std::string& file,
      const int& line,
      const std::string& func,
      const std::exception& child,
      const std::string& msg = ""
    );

    template <typename... Args>
    static std::string createMsg(Args&&... args) {
      std::stringstream stream;
      createMsg(stream, std::forward<Args>(args)...);
      return stream.str();
    }

    template <typename... Args>
    static std::runtime_error createError(Args&&... args) {
      return std::runtime_error(createMsg(std::forward<Args>(args)...));
    }

  #define BackTrackMsg(child, msg) backTrack(__FILE__, __LINE__, __func__, child, msg)
  #define BackTrack(child) backTrack(__FILE__, __LINE__, __func__, child)

}; // Exception

#define defaultCatch catch (const std::exception& e) { throw Domabot::Exception::BackTrack(e); }

} // Domabot

#endif // Domabot_Exception_h
