/**
 * @file Exception.h
 * @brief Domabot Exception class header file.
 * @details Also contains support macros for back tracing.
*/
#ifndef Domabot_Exception_h
#define Domabot_Exception_h

#include <functional>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Domabot {

/**
 * @brief Exception class inherits from std::runtime_error.
 *
 * @details Allow nesting exception messages by internal childs list. Useful for backtracing.
 */
class Exception : public std::runtime_error {
  protected:
    using Childs =  std::list<std::string>;

    Childs m_childs = {}; ///< exception messages on next (nested) level.

    /**
     * @brief End method for error message generation by argument list.
     * @details Calls at end arguments list (at end recursion).
     *
     * @param[in] stream String stream to concat message.
     * @param[in] last Last message part.
     */
    template <typename Last>
    static void createMsg(std::stringstream &stream, Last&& last) {
      stream << std::forward<Last>(last);
    }

    /**
     * @brief Middle method for error message generation by argument list.
     * @details Recursively calls for middle arguments in list (not first and not last).
     *
     * @param[in] stream String stream to concat message.
     * @param[in] first First queued message part.
     * @param[in] args Others message parts in queue.
     */
    template <typename First, typename... Args>
    static void createMsg(std::stringstream& stream, First&& first, Args&&... args) {
      stream << std::forward<First>(first);
      createMsg(stream, std::forward<Args>(args)...);
    }

  public:
    Exception() = delete;

    /**
     * @brief Create Exception object from string.
     * @param[in] msg Source string.
     */
    Exception(const std::string& msg = "");

    /**
     * @brief Create Exception object from std::exception.
     * @details Typically used in nested exceptions lists for backtracing.
     * @param[in] e Source exception.
     */
    Exception(const std::exception& e);

    Exception(const Exception&) = default;
    Exception(Exception&&) = default;
    Exception& operator=(const Exception&) = default;
    Exception& operator=(Exception&&) = default;

    virtual ~Exception() = default;

    /**
     * @brief Check is there nested exceptions.
     * @return True if no nested exceptions, otherwise if there is - false.
     */
    bool isChildsEmpty() const noexcept;

    /**
     * @brief Add child message exception to nested list.
     * @details Extracts exception message by what().
     * @param[in] child Exception to add.
     */
    void add(const std::exception& child);

    /**
     * @brief Add child message exception to nested list.
     * @param[in] child Message to add.
     */
    void add(const std::string& child);

    /**
     * @brief Self check for nested exceptions levels.
     * @throws If nested levels there, otherwise do nothing.
     */
    void checkSelf();

    /**
     * @brief Compose exception message.
     * @details Message will be composed from current and all nested levels
     * recursively at intended list.
     * @return Composed message.
     */
    std::string toString() const;

    /**
     * @brief Compose exception message and put it into std::runtime_error.
     * @details Message will be composed from current and all nested levels
     * recursively at intended list.
     * @return std::runtime_error with composed message.
     */
    std::runtime_error toRuntimeError() const;

    /**
     * @brief Create back track exception.
     * @details The Back track exception is nested exceptions list. Every
     * exception in list contains file name, line number and function name that
     * throw it.
     * @param[in] file Current file name, always must be __FILE__ macro.
     * @param[in] line Line number in current file, always must be __LINE__ macro.
     * @param[in] func Trowed function name, always must be __func__ macro.
     * @param[in] child Child exception, top exception on previous back track level.
     * @param[in] msg Additional message for current creating exception on this
     * back track level.
     * @return Created exception on top level of list.
     */
    static std::runtime_error backTrack(
      const std::string& file,
      const int& line,
      const std::string& func,
      const std::exception& child,
      const std::string& msg = ""
    );

    /**
     * @brief Compose exception message from any argument list.
     * @details Used recursively calls for all arguments and but its in
     * single stringstream. Useful for create message for backtracing.
     * @param[in] args Arguments list that can be converted to string by << operator.
     * @return Composed string message.
     */
    template <typename... Args>
    static std::string createMsg(Args&&... args) {
      std::stringstream stream;
      createMsg(stream, std::forward<Args>(args)...);
      return stream.str();
    }

    /**
     * @brief Compose std::runtime error with message from any argument list.
     * @details Used createMsg to obtain message and then use it for
     * runtime_error constructor. Typically used for create & throw for bottom
     * level exceptions.
     * @param[in] args Arguments list of any type.
     * @return Create exception as std::runtime_error.
     */
    template <typename... Args>
    static std::runtime_error createError(Args&&... args) {
      return std::runtime_error(createMsg(std::forward<Args>(args)...));
    }

  /**
   * @brief Create next (current) back trace level.
   * @details Create next (current) back trace level by nested previous with
   * custom message.
   * @param child Previous exception, that will be nested for current.
   * Typically obtained from catch argument.
   * @param msg Error message on current level. Typically generated by
   * createMsg method
   * @return Next level exception for throw command.
   */
  #define BackTrackMsg(child, msg) backTrack(__FILE__, __LINE__, __func__, child, msg)

  /**
   * @brief Create next (current) back trace level.
   * @details Create next (current) back trace level by nested previous.
   * @param child Previous exception, that will be nested for current.
   * Typically obtained from catch argument.
   * @return Next level exception for throw command.
   */
  #define BackTrack(child) backTrack(__FILE__, __LINE__, __func__, child)

}; // Exception

/**
 * @brief Defines typical catch block for back tracing.
 * @details Should use for every method/function without custom catch block.
 */
#define defaultCatch catch (const std::exception& e) { throw Domabot::Exception::BackTrack(e); }

} // Domabot

#endif // Domabot_Exception_h
