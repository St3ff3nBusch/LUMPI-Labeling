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

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_
//#include <iostream>
#include <opencv2/opencv.hpp>
#include "model/Frustum.h"
namespace ikg {

/**
 * @brief A class representing a 3D bounding box.
 * 
 * This class provides various functionalities for handling 3D bounding boxes,
 * including construction from points, transformation, intersection calculations,
 * and visualization. It supports operations such as getting the projection matrix,
 * calculating intersection over union (IoU) in both 2D and 3D, and plotting the
 * bounding box on an image.
 */
class BoundingBox {
	/*
	 * @brief the 12 lines of the box with pairs of corners indices
	 */
	static const std::vector<std::pair<int, int>> boxLines;
public:
	/**
	 * @brief The class ID of the bounding box.
	 */
	int classId;

	/**
	 * @brief The color of the bounding box.
	 */
	cv::Vec3b color;

	/**
	 * @brief The width of the bounding box.
	 */
	double width;

	/**
	 * @brief The height of the bounding box.
	 */
	double height;

	/**
	 * @brief The length of the bounding box.
	 */
	double length;

	/**
	 * @brief Half of the length of the bounding box.
	 */
	double l_2;

	/**
	 * @brief Half of the width of the bounding box.
	 */
	double w_2;

	/**
	 * @brief Half of the height of the bounding box.
	 */
	double h_2;

	/**
	 * @brief The timestamp of the bounding box.
	 */
	double timestamp;

	/**
	 * @brief The ID of the bounding box.
	 */
	double id;

	/**
	 * @brief The angle of the bounding box.
	 */
	double angle;

	/**
	 * @brief Cosine of the angle.
	 */
	double ca;

	/**
	 * @brief Sine of the angle.
	 */
	double sa;

	/**
	 * @brief The center of the bounding box.
	 */
	cv::Point3d center;

	/**
	 * @brief The u vector of the bounding box.
	 */
	cv::Point3d u;

	/**
	 * @brief The v vector of the bounding box.
	 */
	cv::Point3d v;

	/**
	 * @brief The w vector of the bounding box.
	 */
	cv::Point3d w;

	/**
	 * @brief The minimum dimensions of the bounding box.
	 */
	cv::Point3d minDim;

	/**
	 * @brief The maximum dimensions of the bounding box.
	 */
	cv::Point3d maxDim;

	/**
	 * @brief The corners of the bounding box.
	 */
	std::vector<cv::Point3d> corners;

	/**
	 * @brief Get the projection matrix of the bounding box.
	 * @return The projection matrix.
	 */
	cv::Mat get_projection_matrix() const;

	/**
	 * @brief Get the lines that define the edges of the bounding box.
	 * @return A vector of pairs of points that define the lines.
	 */
	std::vector<std::pair<cv::Point3d, cv::Point3d>> get_lines() const;

	/**
	 * @brief Construct a bounding box from a set of points.
	 * @param points The points to enclose.
	 * @param timeStep The time step of the bounding box.
	 * @param trajectoryID The trajectory ID.
	 */
	BoundingBox(const std::vector<cv::Point3d> &points, double timeStep, int trajectoryID = -1);

	/**
	 * @brief Get the visible lines of the bounding box.
	 * @param frustum The frustum for visibility checking.
	 * @return A vector of pairs of points that define the visible lines.
	 */
	std::vector<std::pair<cv::Point3d, cv::Point3d>> get_visibil_lines(const frustum::Frustum &frustum) const;

	/**
	 * @brief Get the overlapping volume with another bounding box.
	 * @param b The other bounding box.
	 * @param cameraFlag Whether to use camera coordinates.
	 * @return The overlapping volume.
	 */
	double overlapp(const BoundingBox &b, bool cameraFlag = false) const;

	/**
	 * @brief Get the intersection over union in 3D with another bounding box.
	 * @param b The other bounding box.
	 * @param camera Whether to use camera coordinates.
	 * @return The intersection over union.
	 */
	double intersection_over_union_3d(const BoundingBox &b, bool camera = false) const;

	/**
	 * @brief Get the intersection over union in 2D with another bounding box.
	 * @param b The other bounding box.
	 * @param camera Whether to use camera coordinates.
	 * @return The intersection over union.
	 */
	double intersection_over_union_2d(const BoundingBox &b, bool camera = false) const;

	/**
	 * @brief Invert the heading of the bounding box.
	 */
	void invert_heading();

	/**
	 * @brief Transform the bounding box with a projection matrix.
	 * @param p The projection matrix.
	 * @param cameraFlag Whether to use camera coordinates.
	 * @param yawFlag Whether to set roll and pitch to zero.
	 * @return The transformed bounding box.
	 */
	BoundingBox transform(cv::Mat p, bool cameraFlag = false, bool yawFlag = false);

	/**
	 * @brief Get the birds eye rectangle of the bounding box.
	 * @param cameraFrame Whether to use camera coordinates.
	 * @return The birds eye rectangle.
	 */
	cv::RotatedRect get_birds_eye_rectangle(bool cameraFrame = false) const;

	/**
	 * @brief Construct a bounding box.
	 * @param x The x-coordinate.
	 * @param y The y-coordinate.
	 * @param z The z-coordinate.
	 * @param l The length.
	 * @param w The width.
	 * @param h The height.
	 * @param heading The heading angle.
	 * @param cameraCoordinateFrame Whether to use camera coordinates.
	 */
	BoundingBox(double x, double y, double z, double l, double w, double h, double heading, bool cameraCoordinateFrame = false);

	/**
	 * @brief Get the enclosing bounding box.
	 * @return A pair of points defining the enclosing bounding box.
	 */
	std::pair<cv::Point3d, cv::Point3d> enclousing_bounding_box() const;

	/**
	 * @brief Get the camera rectangle of the bounding box.
	 * @param intrinsic The intrinsic camera matrix.
	 * @param distortion The distortion coefficients.
	 * @param frustum The frustum for visibility checking.
	 * @return The camera rectangle.
	 */
	cv::Rect2d get_camera_rect(const cv::Mat intrinsic, const cv::Mat distortion, const frustum::Frustum &frustum) const;

	/**
	 * @brief Get the top left and bottom right corners of a set of lines.
	 * @param lines2D The lines to check.
	 * @return A pair of points defining the top left and bottom right corners.
	 */
	std::pair<cv::Point2d, cv::Point2d> get_top_left_and_bottum_right(const std::vector<std::pair<cv::Point2d, cv::Point2d>> &lines2D) const;

	/**
	 * @brief Update the bounding box.
	 */
	void update();

	/**
	 * @brief Adjust the bounding box to enclose a set of points.
	 * @param data The points to enclose.
	 * @return The adjusted bounding box.
	 */
	template<class T>
	static BoundingBox adjust(const std::vector<T> &data);

	/**
	 * @brief Interpolate between two bounding boxes.
	 * @param b1 The first bounding box.
	 * @param b2 The second bounding box.
	 * @param scale The interpolation factor.
	 * @param cameraFrame Whether to use camera coordinates.
	 * @return The interpolated bounding box.
	 */
	static BoundingBox interpolate(BoundingBox b1, BoundingBox b2, double scale, bool cameraFrame = false);

	/**
	 * @brief Calculate the distance of a point to the bounding box surface.
	 * @param p The point.
	 * @return The distance to the surface.
	 */
	double dist_to_surface(cv::Point3d p);

	/**
	 * @brief Default constructor for BoundingBox.
	 */
	BoundingBox();

	/**
	 * @brief Get the volume of the bounding box.
	 * @return The volume.
	 */
	double get_volume() const;

	/**
	 * @brief Destructor for BoundingBox.
	 */
	~BoundingBox();

	/**
	 * @brief Overloaded output stream operator for BoundingBox.
	 * @param outStream The output stream.
	 * @param p The BoundingBox object.
	 * @return The output stream.
	 */
	friend std::ostream& operator<<(std::ostream &outStream, const BoundingBox &p);

	/**
	 * @brief Assignment operator for BoundingBox.
	 * @param b The BoundingBox to assign from.
	 * @return The assigned BoundingBox.
	 */
	BoundingBox& operator=(const BoundingBox &b);

	/**
	 * @brief Copy constructor for BoundingBox.
	 * @param b The BoundingBox to copy from.
	 */
	BoundingBox(const BoundingBox &b);

	/**
	 * @brief Update the orientation of the bounding box.
	 * @param roll The roll angle.
	 * @param pitch The pitch angle.
	 * @param yaw The yaw angle.
	 * @param camFrame Whether to use camera coordinates.
	 */
	void update_orientation(double roll, double pitch, double yaw, bool camFrame = false);

	/**
	 * @brief Plot the bounding box on an image.
	 * @param img The image to plot on.
	 * @param color The color of the lines.
	 * @param intr The intrinsic camera matrix.
	 * @param distortion The distortion coefficients.
	 * @param thick The thickness of the lines.
	 * @param frustum The frustum for visibility checking.
	 * @return True if the bounding box was plotted, false otherwise.
	 */
	bool plot(cv::Mat img, cv::Scalar color, cv::Mat intr, cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F), double thick = 1, const frustum::Frustum &frustum = frustum::Frustum()) const;

	/**
	 * @brief Project the lines of the bounding box.
	 * @param lines The lines to project.
	 * @param intr The intrinsic camera matrix.
	 * @param distortion The distortion coefficients.
	 * @return A vector of pairs of points defining the projected lines.
	 */
	std::vector<std::pair<cv::Point2d, cv::Point2d>> project_lines(const std::vector<std::pair<cv::Point3d, cv::Point3d>> &lines, const cv::Mat intr, const cv::Mat distortion) const;

	/**
	 * @brief Check if a point is within the bounding box.
	 * @param p The point to check.
	 * @return True if the point is within the bounding box, false otherwise.
	 */
	template<class T>
	bool within(const T &p) const{
		if (p.x < minDim.x || p.y < minDim.y || p.z < minDim.z)
			return false;
		if (p.x > maxDim.x || p.y > maxDim.y || p.z > maxDim.z)
			return false;
		cv::Point3d d(p.x - center.x, p.y - center.y, p.z - center.z);
		if (fabs(u.dot(d)) > l_2)
			return false;
		if (fabs(v.dot(d)) > w_2)
			return false;
		if (fabs(w.dot(d)) > h_2)
			return false;
		return true;

	}
};

} /* namespace ikg */

#endif /* BOUNDINGBOX_H_ */
