/*
 * This file is part of the LUMPI Labeling project:
 *
 *     https://github.com/St3ff3nBusch/LUMPI-Labeling
 *
 * Licensed under the GNU Affero General Public License.
 * You may obtain a copy of the License at
 *
 *     https://www.gnu.org/licenses/agpl-3.0.html.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Steffen Busch
 * Email: steffen.busch@ikg.uni-hannover.de
 * Orcid: 0000-0002-5607-9040
 * Date:  2024-11-29
 */

#ifndef PARSER_H
#define PARSER_H
#include <vector>
#include <string>
#include "model/StructsAndMore.h"
#include "model/geometries/Trajectory.h"
#include <opencv2/opencv.hpp>
#include <QJsonObject>
namespace ikg{
/**
 * \class Parser
 * \brief A class for storing and loading files.
 *
 * This class provides various methods for parsing different foramts, ply pointclouds, trajectoreis/labels, numpy arrays, csv filesa and the LUMPI json meta file.
 */
class Parser {
public:
	/**
	 * @brief Load a point cloud from the IKG benchmark dataset.
	 * @param path The path to the dataset.
	 * @return The loaded point cloud.
	 */
	static pcl::PointCloud<IKGB> load_cloud_ikg_benchmark(const std::string &path);

	/**
	 * @brief Load a point cloud from the IKG mobile mapping dataset.
	 * @param path The path to the dataset.
	 * @return The loaded point cloud.
	 */
	static pcl::PointCloud<IKGB> load_cloud_ikg_mobile_mapping(const std::string &path);

	/**
	 * @brief Read trajectories from a file.
	 * @param fileName The name of the file.
	 * @param cameraFrame Whether to use the camera frame.
	 * @param simple Whether to use a simple format.
	 * @param ply Whether to use the PLY format.
	 * @return A vector of trajectories.
	 */
	static std::vector<Trajectory> read_trajectories(const std::string &fileName, const bool &cameraFrame = true, const bool &simple = false, const bool &ply = false);

	/**
	 * @brief Get a list of files in a directory.
	 * @param directoryName The name of the directory.
	 * @return A vector of file names.
	 */
	static std::vector<std::string> files_at_directory(const std::string &directoryName);

	/**
	 * @brief Read a numpy array from a file.
	 * @param name The name of the file.
	 * @return A pair containing the data and the shape of the array.
	 */
	static std::pair<std::vector<double>, std::vector<int>> read_numpy(const std::string &name);

	/**
	 * @brief Read a numpy array into an OpenCV matrix.
	 * @param path The path to the file.
	 * @return The loaded OpenCV matrix.
	 */
	static cv::Mat read_numpy_cv_mat(const std::string &path);

	/**
	 * @brief Save an OpenCV matrix to a numpy file.
	 * @param name The name of the file.
	 * @param img The OpenCV matrix to save.
	 * @return True if the file was saved successfully, false otherwise.
	 */
	static bool save_numpy(const std::string &name, const cv::Mat img);

	/**
	 * @brief Save data to a numpy file.
	 * @param name The name of the file.
	 * @param data The data to save.
	 * @param size The shape of the data.
	 * @return True if the file was saved successfully, false otherwise.
	 */
	static bool save_numpy(const std::string &name, const std::vector<double> &data, const std::vector<int> &size);

	/**
	 * @brief Convert a session to a sensor.
	 * @param session The session to convert.
	 * @return The converted sensor.
	 */
	static Sensor session_to_sensor(const QJsonObject &session);

	/**
	 * @brief Read background data from a file.
	 * @param th The threshold data.
	 * @param angleNormalizer The angle normalizer data.
	 * @param path The path to the file.
	 */
	static void read_background(std::unordered_map<int, cv::Mat> &th, std::unordered_map<int, double> &angleNormalizer, const std::string &path);

	/**
	 * @brief Read a point cloud from a CSV file and subsample it.
	 * @param name The name of the file.
	 * @param res The resolution for subsampling.
	 * @return The subsampled point cloud.
	 */
	static std::vector<IKGB> read_point_cloud_csv_supsampled(const std::string &name, const double &res);

	/**
	 * @brief Read a CSV file into a vector of vectors of doubles.
	 * @param name The name of the file.
	 * @param delimiter The delimiter used in the file.
	 * @return The loaded data.
	 */
	static std::vector<std::vector<double>> read_csv_file_double(const std::string &name, const std::string &delimiter = ",");

	/**
	 * @brief Read meta data from a lumpi file.
	 * @param path The path to the file.
	 * @return A pair containing the sensor data and the mapping data.
	 */
	static std::pair<std::map<int, Sensor>, std::map<int, std::map<int, int>>> read_meta_lumpi(const std::string &path);

	/**
	 * @brief Write a point cloud to a PLY file.
	 * @param name The name of the file.
	 * @param cloud The point cloud to write.
	 */
	static void write_ikgb_point_cloud_to_ply(const std::string &name, const pcl::PointCloud<IKGB> &cloud);

	/**
	 * @brief Write trajectories to a file.
	 * @param traj The trajectories to write.
	 * @param path The path to the file.
	 * @param simple Whether to use a simple format.
	 * @param append Whether to append to the file.
	 * @param ply Whether to use the PLY format.
	 */
	static void write_trajectories(const std::vector<ikg::Trajectory> &traj, const std::string &path, const bool &simple = true, const bool &append = false, const bool &ply = false);
};
}
#endif /* PARSER_H*/
