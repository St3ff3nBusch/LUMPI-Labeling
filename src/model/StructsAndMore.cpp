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

#ifndef STRUCTSANDMORE_C_
#define STRUCTSANDMORE_C_
#include "model/StructsAndMore.h"
#include <dirent.h>
#include<filesystem>
using namespace std;
namespace ikg {

cv::Mat get_colors() {
	cv::Mat color(100, 1, CV_8U, cv::Scalar(0));
	for (int i = 0; i < 100; i++) {
		color.at<char>(i, 0) = 255 - floor(255 / 100.0 * i);
	}
	cv::applyColorMap(color, color, cv::COLORMAP_JET);
	return color.clone();
}

void add_label(cv::Mat img, const double &maxValue, const double &minValue, const double &scale) {
	double per = (maxValue - minValue) / 100.0;
	for (int i = 0; i < 100; i++) {
		std::stringstream ss;
		switch (i) {
		case 0:
			ss << std::scientific << std::setprecision(2) << maxValue;
			break;
		case 95:
			ss << std::scientific << std::setprecision(2) << minValue;
			break;
		default:
			if (i % 20 == 0) {
				ss << std::scientific << std::setprecision(2) << per * (100 - i) + minValue;
			}
			break;
		}
		cv::putText(img, ss.str(), cv::Point2d(0, i * 10 * scale + max(20.0, 20 * scale)), 1, max(1.0, 2 * scale), cv::Scalar(0, 0, 0), max(1.0, 2 * scale));
	}
}

cv::Mat color_legend(double minValue, double maxValue, int height) {
	double scale = height / 1000.0;
	cv::Mat img(1000 * scale, 250 * scale, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat color = get_colors();
	for (int i = 0; i < 100; i++) {
		cv::line(img, cv::Point2d(50 * scale, i * 10 * scale), cv::Point2d(150 * scale, i * 10 * scale), color.at<cv::Vec3b>(i, 0), (max(1.0, 10 * scale)));
	}
	add_label(img, maxValue, minValue, scale);
	return img.clone();
}

void float32(float *__restrict out, const uint16_t in) {
	uint32_t mantissa;
	uint32_t sign;
	uint32_t exponent;
	mantissa = in & 0x7fff;                       // Non-sign bits
	sign = in & 0x8000;                       // Sign bit
	exponent = in & 0x7c00;                       // Exponent
	mantissa <<= 13;                       // Align mantissa on MSB
	sign <<= 16;                       // Shift sign bit into position
	mantissa += 0x38000000;                       // Adjust bias
	mantissa = (exponent == 0 ? 0 : mantissa);                       // Denormals-as-zero
	mantissa |= sign;                       // Re-insert sign bit
	*((uint32_t*) out) = mantissa;
}
void float16(uint16_t *__restrict out, const float in) {
	uint32_t inu = *((uint32_t*) &in);
	uint32_t mantissa;
	uint32_t sign;
	uint32_t exponent;
	mantissa = inu & 0x7fffffff;                 // Non-sign bits
	sign = inu & 0x80000000;                 // Sign bit
	exponent = inu & 0x7f800000;                 // Exponent
	mantissa >>= 13;                 // Align mantissa on MSB
	sign >>= 16;                 // Shift sign bit into position
	mantissa -= 0x1c000;                 // Adjust bias
	mantissa = (exponent < 0x38800000) ? 0 : mantissa;                 // Flush-to-zero
	mantissa = (exponent > 0x8e000000) ? 0x7bff : mantissa;                 // Clamp-to-max
	mantissa = (exponent == 0 ? 0 : mantissa);                 // Denormals-as-zero
	mantissa |= sign;                 // Re-insert sign bit
	*((uint16_t*) out) = mantissa;
}

std::vector<std::string> readFilesFromDirectory(string path) {
	vector<std::string> fnames;
	if (std::filesystem::is_directory(path.c_str())) { // is p a directory?
		DIR *dir;
		dir = opendir(path.c_str());
		struct dirent *ent;
		while ((ent = readdir(dir)) != NULL) {
			if ((string) ent->d_name == "." || (string) ent->d_name == "..") {
				continue;
			}
			string ts = ent->d_name;
			fnames.push_back(ent->d_name);
		}
		closedir(dir);
		compareDouble c;
		sort(fnames.begin(), fnames.end(), c);
	}
	return fnames;
}

bool calculate_rotation_matrix(double alpha, double beta, double gamma, cv::Mat &R) {
	bool ret = true;

	R = cv::Mat::zeros(3, 3, CV_64FC1);
	cv::Mat rotationX = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat rotationY = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat rotationZ = cv::Mat::eye(3, 3, CV_64FC1);
//	 	static double deg2Rad=M_PI/180.0;
	double ca = cos(alpha), cb = cos(beta), cg = cos(gamma);
	double sa = sin(alpha), sb = sin(beta), sg = sin(gamma);
	// rotation x-axis
	rotationX.at<double>(1, 1) = ca;
	rotationX.at<double>(1, 2) = -sa;
	rotationX.at<double>(2, 1) = sa;
	rotationX.at<double>(2, 2) = ca;

	// rotation y-axis
	rotationY.at<double>(0, 0) = cb;
	rotationY.at<double>(0, 2) = sb;
	rotationY.at<double>(2, 0) = -sb;
	rotationY.at<double>(2, 2) = cb;

	// rotation z-axis
	rotationZ.at<double>(0, 0) = cg;
	rotationZ.at<double>(0, 1) = -sg;
	rotationZ.at<double>(1, 0) = sg;
	rotationZ.at<double>(1, 1) = cg;

	R = rotationZ * rotationY * rotationX;

	return ret;
}

double dist(const Pose &a, const Pose &b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(dx * dx + dy * dy);
}
double dist2(const Pose &a, const Pose &b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return dx * dx + dy * dy;
}
double distHeading(const Pose &a, const Pose &b) {
	double d = a.orientation - b.orientation;
	if (d > M_PI)
		d -= Pi2;
	else if (d < -M_PI)
		d += Pi2;
	return d;
}
double dist(cv::Point2d a, cv::Point2d b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
double dist(std::pair<double, double> a, std::pair<double, double> b) {
	return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

const std::vector<std::string> split(std::string &input, const std::string delimiter) {
	std::string currenToken { "" };
	std::vector<std::string> tokens;
	for (auto n : input) {
		bool shouldSplit = false, isDelimiter = false;
		for (const auto &c : delimiter) {
			if (n == c) {
				isDelimiter = true;
				if (currenToken != "") {
					shouldSplit = true;
				}
			}
		}
		if (shouldSplit) {
			shouldSplit = false;
			tokens.push_back(currenToken);
			currenToken = "";
			continue;
		}
		if (!shouldSplit && !isDelimiter)
			currenToken += n;
	}
	if (currenToken != "")
		tokens.push_back(currenToken);
	return tokens;
}

Sensor::Sensor():horizontalResolution(-1),startTime(-1.),fps(-1),deviceId(-1),expId(-1),sessionId(-1){};
Sensor::Sensor(const Sensor &s){
		frustum=s.frustum;
		horizontalResolution=s.horizontalResolution;
		type=s.type;
		s.extrinsic.copyTo(extrinsic);
		s.intrinsic.copyTo(intrinsic);
		s.distortion.copyTo(distortion);
		s.tvec.copyTo(tvec);
		deviceId=s.deviceId;
		sessionId=s.sessionId;
		expId=s.expId;
		s.rvec.copyTo(rvec);
		angles=s.angles;
		fps=s.fps;
		startTime=s.startTime;
	};
	Sensor& Sensor::operator=(const Sensor &s){
		frustum=s.frustum;
		horizontalResolution=s.horizontalResolution;
			type=s.type;
			deviceId=s.deviceId;
			expId=s.expId;
			sessionId=s.sessionId;
			s.extrinsic.copyTo(extrinsic);
			s.intrinsic.copyTo(intrinsic);
			s.distortion.copyTo(distortion);
			s.tvec.copyTo(tvec);
			s.rvec.copyTo(rvec);
			angles=s.angles;
			fps=s.fps;
			startTime=s.startTime;
			return *this;
		};
} //namespace ikg
#endif /* STRUCTSANDMORE_C_ */
