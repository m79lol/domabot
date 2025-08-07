/**
 * @file UserInteraction.h
 * @brief Domabot User Interaction class header file.
*/
#ifndef Domabot_UserInteraction_h
#define Domabot_UserInteraction_h

#include <domabot_common_lib/Exception.h>

#include <iomanip>
#include <iostream>

namespace Domabot {

/**
 * @brief User Interaction tools.
 *
 * @details Full static class for different method for interaction user by
 * standard console in & out threads.
 */
class UserInteraction {
  public:
    /**
     * @brief Ask user about simple closed question.
     * @details Question has only yes or no answer. Hint about that will be append
     * to question string.
     *
     * @param[in] question Question text
     * @return true if user answer is "yes", otherwise if answer
     * was "no".
     */
    static bool askUser(const std::string& question);

    /**
     * @brief Ask user input some string value.
     * @details Also used as pause function for await any user attention
     * to continue.
     *
     * @param[in] sentence Ask text.
     * @return User intput string.
     */
    static std::string askInput(const std::string& sentence);

    /**
     * @brief The user answer option struct, used to proposeOptions method.
     * @details Describes all helping fields to process user answer option.
     * Required default ResultType enum class of user answer options.
     */
    template<typename ResultType>
    struct Option {
      const std::string m_shortcut; ///< Shortcut option name.
      const std::string m_description; ///< Full option text.
      const ResultType m_result; ///< Enum item linked to this option.
    };

    /**
     * @brief Ask user and propose list of answer options.
     * @details User may choose only one option for answer. Method requires
     * pre define ResultType enum class for options.
     *
     * @param[in] preface Ask text.
     * @param[in] options Array of answer options as vector of Option struct
     * @return User choosen option.
     */
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
