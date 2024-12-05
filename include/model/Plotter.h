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

#ifndef PLOT_H_
#define PLOT_H_
#include "model/StructsAndMore.h"
#include <opencv2/opencv.hpp>
#include "model/geometries/Trajectory.h"

namespace ikg {
/**
 * \class Plotter
 * \brief A class for plotting heat maps, bounding boxes and trajectories.
 */
class Plotter {
public:
	Plotter();
	virtual ~Plotter();
	/**
	 * @brief Generate a heat map from a double matrix.
	 * @param image The matrix with double values.
	 * @param colorScheme The color scheme for the heat map.
	 * @return The generated heat map.
	 */
	static cv::Mat get_heat_map(cv::Mat image, int colorScheme = cv::COLORMAP_JET);
	/**
	 * @brief Generate a heat map from a double matrix.
	 * @param image The matrix with double values.
	 * @param colorScheme The color scheme for the heat map.
	 * @return The generated heat map with a color legend.
	 */
	static cv::Mat get_heat_map_with_legend(cv::Mat image, int colorScheme = cv::COLORMAP_JET);

	/**
	 * @brief Generate a heat map from a set of points.
	 * @param points The points to generate the heat map from.
	 * @param width The width of the heat map.
	 * @param height The height of the heat map.
	 * @return The generated heat map.
	 */
	cv::Mat generate_heat_map(const std::vector<cv::Point2f> &points, int width, int height);

	/**
	 * @brief Plot a heat map on an image.
	 * @param img The image to plot the heat map on.
	 * @param heatMap The heat map to plot.
	 * @param alpha The transparency of the heat map.
	 */
	void plot_heat_map(cv::Mat &img, const cv::Mat &heatMap, double alpha = 0.5);

	/**
	 * @brief Plot a trajectory on an image.
	 * @param img The image to plot on.
	 * @param traj The trajectory to plot.
	 * @param color The color of the trajectory.
	 * @param thickness The thickness of the trajectory lines.
	 */
	void plot_trajectory(cv::Mat &img, const Trajectory &traj, const cv::Scalar &color, int thickness = 1);

	/**
	 * @brief Plot a bounding box on an image.
	 * @param img The image to plot on.
	 * @param bbox The bounding box to plot.
	 * @param color The color of the bounding box.
	 * @param thickness The thickness of the bounding box lines.
	 */
	void plot_bounding_box(cv::Mat &img, const BoundingBox &bbox, const cv::Scalar &color, int thickness = 1);

	/**
	 * @brief Plot a point cloud on an image.
	 * @param img The image to plot on.
	 * @param cloud The point cloud to plot.
	 * @param color The color of the points.
	 */
	void plot_point_cloud(cv::Mat &img, const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Scalar &color);
};

} /* namespace ikg */

#endif /* PLOT_H_ */
