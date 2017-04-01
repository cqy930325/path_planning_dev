/** PerceptIn license **/

#ifndef MODEL_INCLUDE_CALIB_IO_H_
#define MODEL_INCLUDE_CALIB_IO_H_

#include <memory>
#include <string>

namespace PI {
namespace model {

// Forward declaration.
class CalibrationTree;

/**
 * @brief Read a calibration tree from a JSON file.
 * @param filename Name of the JSON file to read the calibration tree from.
 * @return Pointer to the calibration tree.
 */
std::shared_ptr<CalibrationTree> ReadCalibration(const std::string &filename);

/**
 * @brief Write a calibration tree from a JSON file.
 * @param filename Name of the JSON file to write the calibration tree to.
 * @param calibration_tree Pointer to the calibration tree.
 * @return True if the calibration is written.
 */
bool WriteCalibration(const std::string &filename,
                      std::shared_ptr<const CalibrationTree> calibration_tree);

}  // namespace model
}  // namespace PI

#endif  // MODEL_INCLUDE_CALIB_IO_H_
