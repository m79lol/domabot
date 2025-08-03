#ifndef Domabot_StringTools_h
#define Domabot_StringTools_h

#include <domabot_common_lib/Exception.h>

#include <string>
#include <sstream>

namespace Domabot {

class StringTools {
  public:
    template <typename T>
    static T stringToNumber(const std::string& str) try {
      T value;
      std::stringstream ss(str);
      ss >> value;
      if (ss.fail() || !ss.eof()) {
        throw Exception::createError("String to number conversion failed.");
      }
      return value;
    } defaultCatch

};

} // Domabot

#endif // Domabot_StringTools_h