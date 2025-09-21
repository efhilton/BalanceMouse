#pragma once
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace efhilton {

/**
 * @brief A class to store and manage key-value pairs
 *
 * Example:
 *   KeyAndValue kv("name", "John");
 *   std::cout << kv.toString(); // Prints "name: John"
 */
class KeyAndValue
{
public:
    /**
   * @brief Construct a new Key And Value object
   * @param key The key string
   * @param value The value string
   *
   * Example:
   *   KeyAndValue kv("name", "John");
   */
    KeyAndValue(std::string key, std::string value)
        : key_(std::move(key))
        , value_(std::move(value))
    {}

    /**
   * @brief Get the key string
   * @return std::string The key
   *
   * Example:
   *   KeyAndValue kv("name", "John");
   *   std::string key = kv.getKey(); // Returns "name"
   */
    [[nodiscard]] std::string getKey() const { return key_; }

    /**
   * @brief Get the value string
   * @return std::string The value
   *
   * Example:
   *   KeyAndValue kv("name", "John");
   *   std::string value = kv.getValue(); // Returns "John"
   */
    [[nodiscard]] std::string getValue() const { return value_; }

    /**
   * @brief Set the value string
   * @param value New value to set
   *
   * Example:
   *   KeyAndValue kv("name", "John");
   *   kv.setValue("Jane"); // Changes value to "Jane"
   */
    void setValue(const std::string &value) { value_ = value; }

    /**
   * @brief Convert to string representation
   * @return std::string Formatted as "key: value"
   *
   * Example:
   *   KeyAndValue kv("name", "John");
   *   std::string str = kv.toString(); // Returns "name: John"
   */
    [[nodiscard]] std::string toString() const { return key_ + ": " + value_; }

private:
    std::string key_;
    std::string value_;
};

class StringUtilities
{
public:
    static constexpr size_t PAGE_WIDTH{80};
    static void printBanner(
        const std::string &title,
        const std::variant<std::string, const std::vector<KeyAndValue>> &content,
        size_t length = PAGE_WIDTH,
        char leftPad = '(',
        char rightPad = ')',
        const std::string &emoji = "🎺");
    static std::string trimString(const std::string &str);
    static std::string padString(
        const std::string &str, size_t length, char leftPad = ' ', char rightPad = ' ');
    static std::string capitalizeString(const std::string &str);
    static std::string formatKeyAndValue(const KeyAndValue &kv, size_t keyLength);
};

} // namespace efhilton