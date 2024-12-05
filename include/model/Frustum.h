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

#ifndef SRC_MODEL_DETECTION_FRUSTUM_H_
#define SRC_MODEL_DETECTION_FRUSTUM_H_
#include"opencv2/opencv.hpp"

namespace frustum {

/**
 * @brief Represents a plane in 3D space.
 * 
 * The Plane class defines a plane using a normal vector and a point on the plane.
 * It provides functionality to calculate the distance from a point to the plane.
 */
class Plane {
public:
	/**
	 * @brief The normal vector of the plane.
	 */
	cv::Point3d n;

	/**
	 * @brief A point on the plane.
	 */
	cv::Point3d p;

	/**
	 * @brief The distance from the origin.
	 */
	double d;

	/**
	 * @brief Constructor for Plane class.
	 * @param nx The x component of the normal vector.
	 * @param ny The y component of the normal vector.
	 * @param nz The z component of the normal vector.
	 * @param distance The distance from the origin.
	 */
	Plane(double nx=0, double ny=0, double nz=0, double distance=0);

	/**
	 * @brief Destructor for Plane class.
	 */
	virtual ~Plane();

	/**
	 * @brief Get the distance from a point to the plane.
	 * @param p The point.
	 * @return The distance from the point to the plane.
	 */
	template<class Point3d>
	double get_distance(const Point3d &p) const {
		return n.x * p.x + n.y * p.y + n.z * p.z - d;
	}
};

/**
 * @brief Represents a frustum in 3D space.
 * 
 * The Frustum class defines a frustum using a set of planes. It provides functionality
 * to check if points and lines are within the frustum and to filter lines based on their
 * intersection with the frustum.
 */
class Frustum {
public:
	/**
	 * @brief The planes that define the frustum.
	 */
	std::vector<Plane> planes;

	/**
	 * @brief The near plane distance.
	 */
	double near;

	/**
	 * @brief The far plane distance.
	 */
	double far;

	/**
	 * @brief Default constructor for Frustum.
	 */
	Frustum();

	/**
	 * @brief Constructor with camera intrinsic matrix, image rows, and image columns.
	 * @param camera_intrinsic_matrix The intrinsic camera matrix.
	 * @param image_rows The number of image rows.
	 * @param image_cols The number of image columns.
	 */
	Frustum(cv::Mat camera_intrinsic_matrix, int image_rows = 900, int image_cols = 1600);

	/**
	 * @brief Destructor for Frustum.
	 */
	virtual ~Frustum();

	/**
	 * @brief Filters valid lines. A line is valid if at least one point is within the frustum. The other point is set to the plane intersecting point.
	 * @param lines The lines to filter.
	 * @return True if there are valid lines, false otherwise.
	 */
	bool culling(std::vector<std::pair<cv::Point3d, cv::Point3d>> &lines) const;

	/**
	 * @brief Check if a line intersects with a plane.
	 * @param startPoint The start point of the line.
	 * @param endPoint The end point of the line.
	 * @param plane The plane to check intersection with.
	 * @param intersectingPoint The point of intersection.
	 * @return True if the line intersects with the plane, false otherwise.
	 */
	bool plane_line_intersection(const cv::Point3d &startPoint, const cv::Point3d &endPoint, const Plane &plane, cv::Point3d &intersectingPoint) const;

	/**
	 * @brief Check if a point is within the frustum.
	 * @param p The point to check.
	 * @return True if the point is within the frustum, false otherwise.
	 */
	template<class Point3d>
	bool within_frustum(const Point3d &p) const {
		for (const auto &pl : planes) {
			if (pl.get_distance(p) <-0.000001)//threshold <0 to catch precision errors and display points at frustum plane
				return false;
		}
		return true;
	}
};
} /* namespace frustum */

#endif /* SRC_MODEL_DETECTION_FRUSTUM_H_ */
