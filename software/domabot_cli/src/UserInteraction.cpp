/**
 * @file UserInteraction.cpp
 * @brief Domabot User Interaction class source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_cli/UserInteraction.h>

#include <iostream>

namespace Domabot {

bool UserInteraction::askUser(const std::string& question) try {
  while (1) {
    std::cout << question << " (y/n): ";
    std::string tmp;
    std::cin >> tmp;
    std::cout << std::endl;

    if (tmp == "y") {
      return true;
    }
    if (tmp == "n") {
      return false;
    }
  }
} defaultCatch

std::string UserInteraction::askInput(const std::string& sentence) try {
  std::cout << sentence;
  std::string tmp;
  std::cin >> tmp;
  return tmp;
} defaultCatch

}  // namespace Domabot
