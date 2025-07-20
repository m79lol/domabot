
#include <domabot_controller/Exception.h>

namespace Domabot {

std::runtime_error Exception::backTrack(
  const std::string& file,
  const int& line,
  const std::string& func,
  const std::exception& child, const std::string& msg
) {
  static const std::string pathSignature = "/src/";

  std::string tmp = child.what();

  const std::string search = "\n  ";
  const std::string replace = "\n    ";
  std::string::size_type n = 0;
  while ((n = tmp.find(search, n)) != std::string::npos) {
    tmp.replace( n, search.length(), replace);
    n += replace.length();
  }

  return std::runtime_error(
    file.substr(file.find(pathSignature) + pathSignature.length()) + ":" +
    std::to_string(line) + " in " + func + (msg.empty() ? "" : " - " + msg) +
    "\n  " + tmp
  );
};

} // Domabot
