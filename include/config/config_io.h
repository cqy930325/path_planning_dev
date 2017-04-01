/** PerceptIn license **/

#ifndef CONFIG_INCLUDE_CONFIG_IO_H_
#define CONFIG_INCLUDE_CONFIG_IO_H_

#include <map>
#include <memory>
#include <string>

namespace PI {
namespace config {

// Forward declaration.
class Config;

/**
 * @brief Read sections of Configs from a JSON file.
 * @param filename Name of the JSON file to read configs from.
 * @return A map from section name to a pointer of Config read from the JSON
 *         file.
 */
std::map<std::string, std::shared_ptr<Config>> ReadConfigs(
    const std::string &filename);

/**
 * @brief Read a specific section of config from a JSON file.
 * @param filename Name of the JSON file to read config from.
 * @param section Name of the section to read.
 * @return Pointer to the Config. Nullptr if the section does not exist in the
 *         JSON file.
 */
std::shared_ptr<Config> ReadConfig(const std::string &filename,
                                   const std::string &section);

/**
 * @brief Write sections of Configs to a JSON file.
 * @param filename Name of the JSON file to write configs to.
 * @param configs A map from section name to a pointer of Config to be written
 *                to the JSON file.
 * @return True if the configs are written to file.
 */
bool WriteConfigs(
    const std::string &filename,
    const std::map<std::string, std::shared_ptr<const Config>> &configs);

/**
 * @brief Append a section of Config to a JSON file.
 * @param filename Name of the JSON file to write config to.
 * @param section Name of the section to write to the file.
 * @param config Pointer to the Config to be written to file.
 * @param overwrite True to allow the function to overwrite if the section
 *                  already exists in the JSON file. False, avoid overwriting.
 * @return True if the Config is written to file.
 */
bool AddConfigToFile(const std::string &filename,
                     const std::string &section,
                     const std::shared_ptr<const Config> config,
                     const bool overwrite);

}  // namespace config
}  // namespace PI

#endif  // CONFIG_INCLUDE_CONFIG_IO_H_
