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

#ifndef INCLUDE_MODEL_DETECTION_LUMPIPIPELINE_H_
#define INCLUDE_MODEL_DETECTION_LUMPIPIPELINE_H_
#include "opencv2/opencv.hpp"
#include "StructsAndMore.h"
#include "pcl/point_cloud.h"
#include "model/geometries/BoundingBox.h"
#include <pcl/visualization/pcl_visualizer.h>
namespace ikg {

/**
 * \class LUMPIPipeline
 * \brief A class for processing and analyzing point cloud data.
 *
 * This class provides various methods for processing point cloud data, including pose correction,
 * filtering by time, background initialization and removal, video generation, and visualization.
 */
class LUMPIPipeline {
public:
		/**
		 * \brief Correct the pose based on the given point cloud.
		 * @param p Pose to be corrected
		 * @param pc Point cloud data
		 */
		static void correct_pose(Pose &p, const pcl::PointCloud<IKGB> &pc);

		/**
		 * \brief Constructor for LUMPIPipeline.
		 */
		LUMPIPipeline();

		/**
		 * \brief Filter point cloud by time.
		 * @param pc Point cloud data
		 * @param minTime Minimum time
		 * @param maxTime Maximum time
		 * @return Filtered point cloud
		 */
		static pcl::PointCloud<IKGB> filter_pc_by_time(const pcl::PointCloud<IKGB> &pc, double minTime, double maxTime);

		/**
		 * \brief Initialize background data.
		 * @param path Path to background data
		 */
		void init_back_ground(std::string path);

		/**
		 * \brief Generate a video from trajectories.
		 * @param trajectories Vector of trajectories
		 * @param videoPath Path to input video
		 * @param outPath Path to output video
		 * @param s Sensor data
		 * @param skip Frames to skip
		 * @param start Start frame
		 * @param end End frame
		 * @return True if successful, false otherwise
		 */
		static bool generate_video(const std::vector<ikg::Trajectory> &trajectories, std::string videoPath, std::string outPath, ikg::Sensor s, int skip = 1, int start = -1, int end = -1);

		/**
		 * \brief Convert LUMPI point cloud to RGB point cloud.
		 * @param pc LUMPI point cloud
		 * @return RGB point cloud
		 */
		static pcl::PointCloud<pcl::PointXYZRGB> lumpi_to_rgb(const pcl::PointCloud<IKGB> &pc);

		/**
		 * \brief Read background data.
		 * @param th Output thresholds
		 * @param angleNormalizer Output angle normalizer
		 * @param path Path to background data
		 */
		static void read_background(std::unordered_map<int, cv::Mat> &th, std::unordered_map<int, double> &angleNormalizer, std::string path);

		/**
		 * \brief Remove background from point cloud.
		 * @param pc Point cloud data
		 * @param th Thresholds
		 * @param degNormalizer Degree normalizer
		 * @return Pair of foreground and background indices
		 */
		static std::pair<std::vector<int>, std::vector<int>> remove_back_ground(const pcl::PointCloud<IKGB> &pc, const std::unordered_map<int, cv::Mat> &th, const std::unordered_map<int, double> &degNormalizer);

		/**
		 * \brief Get color for a class ID.
		 * @param id Class ID
		 * @return Color
		 */
		static cv::Vec3b switch_class_color(int id);

		/**
		 * \brief Get description for a class ID.
		 * @param id Class ID
		 * @param yolo Flag to use YOLO class descriptions
		 * @return Class description
		 */
		static std::string switch_class_description(int id, bool yolo = false);

		/**
		 * \brief Visualize bounding boxes.
		 * @param current_measurement Vector of current measurements
		 * @param viewer PCL visualizer
		 * @param thick Line thickness
		 * @param name Name of the visualization
		 * @param showIds Flag to show IDs
		 */
		static void visualOutputBoundingBox(const std::vector<BoundingBox> &current_measurement, pcl::visualization::PCLVisualizer::Ptr viewer, int thick, std::string name, bool showIds);

		/**
		 * \brief Destructor for LUMPIPipeline.
		 */
		virtual ~LUMPIPipeline();

public:
	/**
	 * \brief Degree normalization map.
	 */
	std::unordered_map<int, double> degNorm;

	/**
	 * \brief Thresholds for background subtraction.
	 */
	std::unordered_map<int, cv::Mat> thresholds;

	/**
	 * \brief Map of point clouds.
	 */
	std::map<int, pcl::PointCloud<IKGB>> clouds;
};

} /* namespace ikg */

#endif /* INCLUDE_MODEL_DETECTION_LUMPIPIPELINE_H_ */
