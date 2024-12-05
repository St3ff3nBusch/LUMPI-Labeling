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

#ifndef INCLUDE_MODEL_GEOMETRIES_POSE_H_
#define INCLUDE_MODEL_GEOMETRIES_POSE_H_
#include "model/geometries/BoundingBox.h"
#include "opencv4/opencv2/opencv.hpp"
namespace ikg {

/**
 * @class Pose
 * @brief Represents a 3D pose with various attributes such as position, orientation, and bounding box.
 *
 * The Pose class encapsulates the properties and behaviors of a 3D pose, including its position in space,
 * orientation, bounding box, and other attributes. It provides methods for calculating distances between poses,
 * interpolating between poses, and other utility functions.
 */
class Pose {
	static const double TWO_PI;
	static const double THREE_PI;
public:
	/**
	 * @brief The trajectory ID.
	 */
	int tId;

	/**
	 * @brief The pose ID.
	 */
	int p_id;

	/**
	 * @brief The class ID of the pose.
	 */
	int classId;

	/**
	 * @brief The bounding box of the pose.
	 */
	BoundingBox bb;

	/**
	 * @brief The x-coordinate of the pose.
	 */
	double x;

	/**
	 * @brief The y-coordinate of the pose.
	 */
	double y;

	/**
	 * @brief The z-coordinate of the pose.
	 */
	double z;

	/**
	 * @brief The width of the pose.
	 */
	double w;

	/**
	 * @brief The length of the pose.
	 */
	double l;

	/**
	 * @brief The height of the pose.
	 */
	double h;

	/**
	 * @brief The time of the pose.
	 */
	double time;

	/**
	 * @brief The score of the pose.
	 */
	double score;

	/**
	 * @brief The visibility of the pose.
	 */
	double visibility;

	/**
	 * @brief The orientation of the pose.
	 */
	double orientation;

	/**
	 * @brief The sine of the orientation.
	 */
	double so;

	/**
	 * @brief The cosine of the orientation.
	 */
	double co;

	/**
	 * @brief The position of the pose.
	 */
	int pos;

	/**
	 * @brief The bounding rectangle of the pose.
	 */
	cv::Rect2d r;

	/**
	 * @brief The shape of the pose.
	 */
	std::vector<double> shape;

	/**
	 * @brief Whether the pose is standing.
	 */
	bool standing;

	/**
	 * @brief Whether the pose is interpolated.
	 */
	bool interpolated;

	/**
	 * @brief Whether the pose is finalized.
	 */
	bool finalized;

	/**
	 * @brief Calculate the 2D distance between two poses.
	 * @param a The first pose.
	 * @param b The second pose.
	 * @return The 2D distance.
	 */
	static double distance_in_2d(const Pose &a, const Pose &b);

	/**
	 * @brief Interpolate between two poses.
	 * @param p1 The first pose.
	 * @param p2 The second pose.
	 * @param s The interpolation factor.
	 * @return The interpolated pose.
	 */
	static Pose interpolate(const Pose &p1, const Pose &p2, double s);

	/**
	 * @brief Copy constructor for Pose.
	 * @param a The Pose to copy from.
	 */
	Pose(const Pose &a);

	/**
	 * @brief Constructor for Pose.
	 * @param x The x-coordinate.
	 * @param y The y-coordinate.
	 * @param z The z-coordinate.
	 * @param time The time.
	 * @param t_id The trajectory ID.
	 * @param p_id The pose ID.
	 */
	Pose(double x = 0, double y = 0, double z = 0, double time = 0, int t_id = 0, int p_id = 0);

	/**
	 * @brief Assignment operator for Pose.
	 * @param a The Pose to assign from.
	 * @return The assigned Pose.
	 */
	Pose& operator=(const Pose &a);

	/**
	 * @brief Overloaded output stream operator for Pose.
	 * @param outStream The output stream.
	 * @param p The Pose object.
	 * @return The output stream.
	 */
	friend std::ostream& operator<<(std::ostream &outStream, const Pose &p);

	/**
	 * @brief Destructor for Pose.
	 */
	virtual ~Pose();

	/**
	 * @brief Interpolate the heading between two angles.
	 * @param a The first angle.
	 * @param b The second angle.
	 * @param s The interpolation factor.
	 * @return The interpolated heading.
	 */
	static double interpolate_heading(double a, double b, double s);

	// ...existing code...
};
} /* namespace ikg */

#endif /* INCLUDE_MODEL_GEOMETRIES_POSE_H_ */
