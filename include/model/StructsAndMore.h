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

#ifndef STRUCTSANDMORE_H_
#define STRUCTSANDMORE_H_
#include "model/geometries/Trajectory.h"
#include "model/geometries/BoundingBox.h"
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "Frustum.h"
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
#include <chrono>
using namespace std::chrono;
static auto startTime = std::chrono::steady_clock::now();
static auto endTime = std::chrono::steady_clock::now();
static std::string printTime() {
	endTime = std::chrono::steady_clock::now();
	auto elapsed_seconds = endTime - startTime;
	std::stringstream ss;
	ss << "elapsed time " << std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count() / 1000000.0 << "s\n";
	startTime = std::chrono::steady_clock::now();
	return ss.str();
}


//#define TICK(...) do{}while(0);
//#define TOCK(...) do{}while(0);
#define TICK(...)startTime=std::chrono::steady_clock::now();
#define TOCK(...)std::cout<<"In:"<<__FILENAME__<<"at:"<<__LINE__<<":"<<__VA_ARGS__<<" time:"<<printTime()<<std::endl;
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define TIME_MEASURE
#ifdef TIME_MEASURE
#define TIME_MS(...)	cout<<"Time:"<<chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - __VA_ARGS__).count()<<endl;
#else
#define TIME_MS(...) do{}while(0);
//#define DEB2(...) do{}while(0)
#endif
#define VER(...) do{}while(0);

#define CHECKRUNTIME
#ifdef CHECKRUNTIME
#define RUN(...) __VA_ARGS__
//#define DEB2(...) __VA_OPT__(,) std::cout<< __VA_ARGS__<<std::endl
#else
#define RUN(...) do{}while(0);
#endif

const double Pi2 = 2 * M_PI;
const double Pi3 = M_PI * 3;
const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;
const double sec2msec=pow(10,6);
const double msec2sec=1./pow(10,6);

/**
 * @brief Struct representing a point with various attributes.
 */
struct IKGB {
    double x, y, z; ///< Coordinates of the point.
    std::uint32_t adjustedtime; ///< Adjusted time.
    std::uint8_t id; ///< ID of the point.
    PCL_ADD_INTENSITY_8U; ///< Intensity attribute.
    PCL_ADD_RGB; ///< RGB color attributes.
    PCL_ADD_NORMAL4D; ///< Normal vector attributes.
    std::uint8_t ray; ///< Ray attribute.
    std::uint16_t azimuth; ///< Azimuth angle.
    std::uint16_t distance; ///< Distance attribute.
    std::uint32_t index; ///< Index attribute.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< Ensure aligned memory allocation.
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
		IKGB,                    // Registration point type macro
		(float,x,x) (float,y,y) (float,z,z) (std::uint32_t,adjustedtime,adjustedtime)(std::uint8_t, id,id) (std::uint8_t, intensity,intensity)(std::uint8_t,b,b) (std::uint8_t,r,r) (std::uint8_t,g,g)(float, normal_x,normal_x) (float, normal_y,normal_y) (float, normal_z,normal_z)
		(	std::uint8_t,ray,ray) (	unsigned short ,azimuth,azimuth)(	unsigned short ,distance,distance)(	unsigned int ,index,index)
		);

namespace ikg {

/**
 * @brief Class representing a sensor with various attributes.
 */
class Sensor {
public:
    int sessionId; ///< @brief Session ID of the sensor.
    int expId; ///< @brief Experiment ID associated with the sensor.
    int deviceId; ///< @brief Device ID of the sensor.
    std::string type; ///< @brief Type of the sensor (e.g., camera, lidar).
    cv::Mat extrinsic; ///< @brief Extrinsic matrix for the sensor.
    cv::Mat intrinsic; ///< @brief Intrinsic matrix for the sensor.
    cv::Mat distortion; ///< @brief Distortion coefficients for the sensor.
    cv::Mat tvec; ///< @brief Translation vector of the sensor.
    cv::Mat rvec; ///< @brief Rotation vector of the sensor.
    frustum::Frustum frustum; ///< @brief Frustum object representing the sensor's field of view.
    double fps; ///< @brief Frames per second rate of the sensor.
    double startTime; ///< @brief Start time of the sensor's operation.
    double horizontalResolution; ///< @brief Horizontal resolution of the sensor.
    std::vector<double> angles; ///< @brief Vector of angles related to the sensor.

    /**
     * @brief Default constructor.
     */
    Sensor();

    /**
     * @brief Copy constructor.
     * @param s Sensor object to copy from.
     */
    Sensor(const Sensor &s);

    /**
     * @brief Assignment operator.
     * @param s Sensor object to assign from.
     * @return Reference to the assigned Sensor object.
     */
    Sensor& operator=(const Sensor &s);
};


/**
 * @brief Generates a color legend image.
 * @param minValue Minimum value for the legend.
 * @param maxValue Maximum value for the legend.
 * @param height Height of the legend image.
 * @return Color legend image.
 */
cv::Mat color_legend(double minValue, double maxValue, int height = 1000);

/**
 * @brief Finds the minimum and maximum 3D points.
 * @tparam T Type of the points.
 * @param points Vector of points.
 * @return Pair of minimum and maximum 3D points.
 */
template<class T>
std::pair<cv::Point3d, cv::Point3d> find_min_max_3D(const std::vector<T> &points) {
	double minX = DBL_MAX, maxX =
			-DBL_MAX, minY = DBL_MAX, maxY = -DBL_MAX, maxZ =
					-DBL_MAX;
	double minZ =DBL_MAX;
	for (const auto &p : points) {
		if (p.x > maxX)
			maxX = p.x;
		if (p.y > maxY)
			maxY = p.y;
		if (p.z > maxZ)
			maxZ = p.z;
		if (p.x < minX)
			minX = p.x;
		if (p.y < minY)
			minY = p.y;
		if (p.z < minZ)
			minZ = p.z;
	}
	return {cv::Point3d(minX,minY,minZ),cv::Point3d(maxX,maxY,maxZ)};
}

/**
 * @brief Finds the minimum and maximum 2D points.
 * @tparam T Type of the points.
 * @param points Vector of points.
 * @return Pair of minimum and maximum 2D points.
 */
template<class T>
std::pair<cv::Point2d, cv::Point2d> find_min_max_2D(const std::vector<T> &points) {
	double minX = std::numeric_limits<double>::max(), maxX =
			std::numeric_limits<double>::lowest(), minY = std::numeric_limits<
			double>::max(), maxY = std::numeric_limits<double>::lowest();
	for (const auto &p : points) {
		if (p.x > maxX)
			maxX = p.x;
		if (p.y > maxY)
			maxY = p.y;
		if (p.x < minX)
			minX = p.x;
		if (p.y < minY)
			minY = p.y;
	}
	return {cv::Point2d(minX,minY),cv::Point2d(maxX,maxY)};
}

/**
 * @brief Calculates the rotation matrix.
 * @param alpha Rotation angle around the x-axis.
 * @param beta Rotation angle around the y-axis.
 * @param gamma Rotation angle around the z-axis.
 * @param R Output rotation matrix.
 * @return True if the calculation is successful.
 */
bool calculate_rotation_matrix(double alpha, double beta, double gamma, cv::Mat &R);

/**
 * @brief Converts a 32-bit float to a 16-bit float.
 * @param out Output 16-bit float.
 * @param in Input 32-bit float.
 */
void float16(uint16_t* __restrict out, const float in);

/**
 * @brief Converts a 16-bit float to a 32-bit float.
 * @param out Output 32-bit float.
 * @param in Input 16-bit float.
 */
void float32(float* __restrict out, const uint16_t in);

/**
 * @brief Splits a string by given delimiters.
 * @param s Input string.
 * @param chars Delimiters.
 * @return Vector of split strings.
 */
const std::vector<std::string> split(std::string& s, const std::string chars);
/**
 * @brief Compare the filename as a numerical value, even when it includes a double notation, such as 1.0.txt.
 */
struct compareDouble {
	/**
	 * @brief Operator will split the string at ".".
	 */
	bool operator()(const std::string &a, const std::string &b) {
		std::string del = ".";
		int pos = 0;
		std::string a1 = a;
		std::string b1 = b;
		std::vector<std::string> tmp1 = split(a1, del);
		std::vector<std::string> tmp2 = split(b1, del);
		if (tmp1.size() > 3 && tmp2.size() > 3) {
			return stod(tmp1[0] + tmp1[1]) < stod(tmp2[0] + tmp2[1]);
		} else {
			return tmp1[0] < tmp2[0];
		}
	}

};
} // namespace ikg

#endif /* STRUCTSANDMORE_H_ */
