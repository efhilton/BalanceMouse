
#include <../include/StringUtilities.hpp>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

namespace efhilton {

/**
 * @brief Removes leading and trailing whitespace from a string
 * @param str Input string to trim
 * @return std::string Trimmed string
 *
 * Example:
 *   std::string result = trimString("  hello  "); // Returns "hello"
 *   std::string empty = trimString("   "); // Returns ""
 */
std::string StringUtilities::trimString(const std::string &str) {
  if (str.empty())
    return str;

  size_t first = str.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos)
    return "";

  size_t last = str.find_last_not_of(" \t\n\r\f\v");
  return str.substr(first, (last - first + 1));
}

/**
 * @brief Pads a string with specified characters to reach desired length
 * @param str String to pad
 * @param length Desired total length
 * @param leftPad Character to use for left padding
 * @param rightPad Character to use for right padding
 * @return std::string Padded string
 *
 * Example:
 *   std::string result = padString("hello", 10);           // Returns "  hello
 * " std::string fancy = padString("title", 11, '*', '*'); // Returns "** title
 * **" std::string withEmoji = padString("🎉", 6);           // Returns "  🎉  "
 */
std::string StringUtilities::padString(const std::string &str, size_t length,
                                      char leftPad, char rightPad) {
  std::string trimmed = " " + trimString(str) + " ";

  // Count actual display width by treating emojis as width 2
  size_t displayWidth = 0;
  for (size_t i = 0; i < trimmed.length();) {
    if ((trimmed[i] & 0xF0) == 0xF0) { // 4-byte UTF-8 sequence (emoji)
      displayWidth += 2;
      i += 4;
    } else if ((trimmed[i] & 0xE0) == 0xE0) { // 3-byte UTF-8 sequence
      displayWidth += 2;
      i += 3;
    } else if ((trimmed[i] & 0xC0) == 0xC0) { // 2-byte UTF-8 sequence
      displayWidth += 1;
      i += 2;
    } else { // 1-byte UTF-8 sequence (ASCII)
      displayWidth++;
      i++;
    }
  }

  if (displayWidth >= length)
    return trimmed;

  size_t padTotal = length - displayWidth;
  size_t padLeft = padTotal / 2;
  size_t padRight = padTotal - padLeft;

  return std::string(padLeft, leftPad) + trimmed +
         std::string(padRight, rightPad);
}

/**
 * @brief Capitalizes all characters in a string
 * @param str String to capitalize
 * @return std::string Capitalized string
 *
 * Example:
 *   std::string result = capitalizeString("hello"); // Returns "HELLO"
 *   std::string empty = capitalizeString(""); // Returns ""
 */
std::string StringUtilities::capitalizeString(const std::string &str) {
  if (str.empty())
    return str;
  std::string result = trimString(str);
  std::transform(result.begin(), result.end(), result.begin(), ::toupper);
  return result;
}

/**
 * @brief Formats a KeyAndValue pair with proper padding
 * @param kv KeyAndValue object to format
 * @param keyLength Length for key portion (defaults to PAGE_WIDTH/2)
 * @return std::string Formatted string with key and value padded and separated
 * by " : "
 *
 * Example:
 *   KeyAndValue kv("name", "John");
 *   std::string result = formatKeyAndValue(kv);
 *   // Returns a string like "      name      :       John      "
 *   // Total width will be PAGE_WIDTH characters
 */
std::string StringUtilities::formatKeyAndValue(const KeyAndValue &kv,
                                              size_t keyLength = PAGE_WIDTH /
                                                                 2) {
  std::string paddedKey = padString(kv.getKey(), keyLength);
  std::string paddedValue =
      padString(kv.getValue(), PAGE_WIDTH - keyLength - 3);
  return paddedKey + " : " + paddedValue;
}

/**
 * @brief Prints a formatted banner with title, content, and decorative border
 * @param title Banner title
 * @param content Either a string message or vector of KeyAndValue pairs
 * @param length Total width of banner
 * @param leftPad Left padding character
 * @param rightPad Right padding character
 * @param emoji Emoji to use in footer
 *
 * Example with string:
 *   printBanner("Welcome", "Hello World!");
 *   // +-------- WELCOME --------+
 *   // |     Hello World!        |
 *   // +----------🎺-----------+
 *
 * Example with KeyValue pairs:
 *   std::vector<KeyAndValue> items = {
 *       KeyAndValue("Name", "John"),
 *       KeyAndValue("Age", "30")
 *   };
 *   printBanner("User Info", items);
 *   // +------- USER INFO -------+
 *   // |    Name    :  John      |
 *   // |    Age     :   30       |
 *   // +----------🎺-----------+
 */
void StringUtilities::printBanner(
    const std::string &title,
    const std::variant<std::string, const std::vector<KeyAndValue>> &content,
    const size_t length, const char leftPad, char rightPad,
    const std::string &emoji) {
  // Print header
  const std::string capitalizedTitle = capitalizeString(trimString(title));
  const std::string header =
      padString(capitalizedTitle, length, leftPad, rightPad);
  std::cout << header << std::endl;

  // Print content
  if (std::holds_alternative<std::string>(content)) {
    // Handle string message with word wrapping
    std::string trimmedMsg = trimString(std::get<std::string>(content));
    size_t pos = 0;
    while (pos < trimmedMsg.length()) {
      size_t lineLen = length - 4; // Account for padding chars and spaces
      size_t endPos = pos + lineLen;

      if (endPos < trimmedMsg.length()) {
        endPos = trimmedMsg.rfind(' ', endPos);
        if (endPos == std::string::npos || endPos <= pos)
          endPos = pos + lineLen;
      } else {
        endPos = trimmedMsg.length();
      }

      std::string line = trimmedMsg.substr(pos, endPos - pos);
      std::cout << leftPad << ' ' << padString(line, length - 4) << ' '
                << rightPad << std::endl;
      pos = endPos;
      while (pos < trimmedMsg.length() && trimmedMsg[pos] == ' ')
        pos++;
    }
  } else {
    // Handle vector of KeyAndValue items
    for (const auto &item : std::get<const std::vector<KeyAndValue>>(content)) {
      std::cout << leftPad << ' ' << formatKeyAndValue(item) << ' ' << rightPad
                << std::endl;
    }
  }

  // Print footer
  const std::string footer = padString(emoji, length, leftPad, rightPad);
  std::cout << footer << std::endl;
}

} // namespace efhilton