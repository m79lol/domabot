
#include <domabot_common_lib/Exception.h>

namespace Domabot {

Exception::Exception(const std::string& msg) : runtime_error(msg) {};

Exception::Exception(const std::exception& e) : runtime_error(e.what()) {};

bool Exception::isChildsEmpty() const noexcept {
  return m_childs.empty();
};

void Exception::add(const std::exception& child) {
  m_childs.emplace_back(child.what());
};

void Exception::add(const std::string& child) {
  m_childs.emplace_back(child);
};

void Exception::checkSelf() {
  if (isChildsEmpty()) {
    return;
  }
  throw std::runtime_error(toString());
};

std::string Exception::toString() const {
  std::string msg = what();
  for (const auto& e : m_childs) {
    msg += std::string("\n  ") + e;
  }
  return msg;
};

std::runtime_error Exception::toRuntimeError() const {
  return std::runtime_error(toString());
};

std::runtime_error Exception::backTrack(
  const std::string& file,
  const int& line,
  const std::string& func,
  const std::exception& child, const std::string& msg
) {
  std::string tmp = child.what();

  static const std::string search = "\n  ";
  static const std::string replace = "\n    ";

  for (std::string::size_type n = 0; (n = tmp.find(search, n)) != std::string::npos; ) {
    tmp.replace( n, search.length(), replace);
    n += replace.length();
  }

  static const std::string pathSignature = "/src/";
  return std::runtime_error(
    file.substr(file.find(pathSignature) + pathSignature.length()) + ":" +
    std::to_string(line) + " in " + func + (msg.empty() ? "" : " - " + msg) +
    "\n  " + tmp
  );
};

} // Domabot
