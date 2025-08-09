/**
 * @file StringTools.h
 * @brief Domabot String Tools class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CLI__STRINGTOOLS_H_
#define DOMABOT_CLI__STRINGTOOLS_H_

#include <domabot_common_lib/Exception.h>

#include <string>
#include <sstream>

namespace Domabot {

/**
 * @brief String tools.
 * @details Full static class for different string processing tool-methods.
 */
class StringTools {
  public:
    /**
     * @brief Convert any string typed number to number type.
     * @tparam T Trivial number type.
     * @param[in] str String with number to conversion.
     * @return Number value defined as template param T.
     */
    template <typename T>
    static T stringToNumber(const std::string& str) try {
      T value;
      std::stringstream ss(str);
      ss >> value;
      if (ss.fail() || !ss.eof()) {
        throw Exception::createError("Fail to converts string \"", str,
          "\" to number");
      }
      return value;
    } defaultCatch
};  // StringTools

}  // namespace Domabot

#endif  // DOMABOT_CLI__STRINGTOOLS_H_
