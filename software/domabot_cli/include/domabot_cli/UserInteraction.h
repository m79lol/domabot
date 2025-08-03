#ifndef Domabot_UserInteraction_h
#define Domabot_UserInteraction_h

#include <domabot_common_lib/Exception.h>

#include <iomanip>
#include <iostream>

namespace Domabot {

class UserInteraction {
  public:
    static bool askUser(const std::string& question);
    static std::string askInput(const std::string& sentence);

    template<typename ResultType>
    struct Option {
      const std::string m_shortcut;
      const std::string m_description;
      const ResultType m_result;
      Option(const std::string& shortcut, const std::string& desc, const ResultType& res) noexcept
        : m_shortcut(shortcut), m_description(desc), m_result(res) {};
    };

    template<typename ResultType>
    static ResultType proposeOptions(
      const std::string& preface,
      const std::vector<Option<ResultType>>& options
    ) try {
      if (options.empty()) {
        throw Exception::createError("Options weren't provided!");
      }

      size_t maxShortcutLength = 0;
      for (const auto &option: options) {
        maxShortcutLength = std::max(maxShortcutLength, option.m_shortcut.size());
      }

      while (true) {
        std::cout << preface;
        for (size_t i = 0; i < options.size(); ++i) {
          std::cout << "\n\t[ " << std::setw(maxShortcutLength)
                    << options[i].m_shortcut << " ] - " << options[i].m_description;
        }
        std::cout << "\n  ans: ";
        std::string input{};
        std::cin.clear();
        std::cin >> input;
        for (size_t choice = 0; choice < options.size(); ++choice) {
          if (input == options[choice].m_shortcut) {
            return options[choice].m_result;
          }
        }
      }
    } defaultCatch


}; // UserInteraction


} // Domabot

#endif // Domabot_UserInteraction_h