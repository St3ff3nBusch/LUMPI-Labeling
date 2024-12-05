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

#include <model/geometries/BoundingBox.h>
#include <model/geometries/Pose.h>

#include <map>
#include "StructsAndMore.h"
#include<unordered_map>
#include<unordered_set>
using namespace std;
using namespace ikg;
using namespace cv;
const std::vector<std::pair<int, int>> BoundingBox::boxLines = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 0, 3 }, { 5, 6 }, { 6, 7 }, { 4, 7 }, { 0, 5 }, { 2, 7 }, { 6, 1 }, { 5, 4 }, { 4, 3 } };

BoundingBox::BoundingBox(const BoundingBox &b) :
			classId(b.classId), color(b.color), width(b.width), height(b.height), length(b.length), l_2(b.l_2), w_2(b.w_2), h_2(b.h_2), timestamp(b.timestamp), id(b.id), angle(b.angle), ca(b.ca), sa(
					b.sa), center(b.center), u(b.u), v(b.v), w(b.w), minDim(b.minDim), maxDim(b.maxDim), corners(b.corners) {
	}
cv::Mat BoundingBox::get_projection_matrix() const {
	return cv::Mat_<double>(4, 4) << (ca, -sa, 0, center.x, sa, ca, 0, center.y, 0, 0, 1, center.z, 0, 0, 0, 1);
}
double BoundingBox::get_volume() const {
	return width * height * length;
}
std::vector<std::pair<cv::Point3d, cv::Point3d>> BoundingBox::get_lines() const {
	std::vector<std::pair<cv::Point3d, cv::Point3d>> lines;
	for (const auto &l : boxLines) {
		lines.push_back( { corners[l.first], corners[l.second] });
	}
	return lines;
}
bool BoundingBox::plot(Mat img, Scalar color,const Mat intr,const cv::Mat distortion, double thick, const frustum::Frustum &frustum) const {
	if (intr.empty())
		return false;
	std::vector<std::pair<cv::Point3d, cv::Point3d>> lines;
	if (!frustum.planes.empty()){
		lines=get_visibil_lines(frustum);
	}else {
		lines= get_lines();
	}
	if(lines.empty()){
		return false;
	}
	auto projectedLines = project_lines(lines, intr, distortion);
	for (int i = 0; i < projectedLines.size() ; i ++)
		cv::line(img, projectedLines[i].first, projectedLines[i].second, color, thick);
	return true;
}
std::vector<std::pair<cv::Point3d, cv::Point3d>> BoundingBox::get_visibil_lines(const frustum::Frustum &frustum) const {
	auto lines = get_lines();
	if (frustum.culling(lines))
		return lines;
	else
		return std::vector<std::pair<cv::Point3d, cv::Point3d>>();
}
std::vector<std::pair<cv::Point2d, cv::Point2d>> BoundingBox::project_lines(const std::vector<std::pair<cv::Point3d, cv::Point3d>>& lines,const cv::Mat intr,const cv::Mat distortion) const {
	vector<cv::Point3f> obj;
	std::vector<std::pair<cv::Point2d, cv::Point2d>> projectedLines;
	obj.reserve(lines.size() * 2);
	for (const auto &l : lines) {
		obj.push_back(cv::Point3f(l.first.x, l.first.y, l.first.z));
		obj.push_back(cv::Point3f(l.second.x, l.second.y, l.second.z));
	}
	vector<cv::Point2f> projectedPoints;
	cv::projectPoints(obj, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), intr, distortion, projectedPoints);
	for (int i = 0; i < projectedPoints.size() - 1; i += 2) {
		projectedLines.push_back( { cv::Point2d(projectedPoints[i].x, projectedPoints[i].y), cv::Point2d(projectedPoints[i+1].x, projectedPoints[i+1].y) });
	}
	return projectedLines;
}
std::pair<cv::Point2d, cv::Point2d> BoundingBox::get_top_left_and_bottum_right(const std::vector<std::pair<cv::Point2d, cv::Point2d>> &lines2D)const {
	cv::Point2d tl(DBL_MAX, DBL_MAX), br(0, 0);
	for (const auto &p : lines2D) {
		tl.x = min(tl.x, (double) p.first.x);
		tl.y = min(tl.y, (double) p.first.y);
		br.x = max(br.x, (double) p.first.x);
		br.y = max(br.y, (double) p.first.y);
		tl.x = min(tl.x, (double) p.second.x);
		tl.y = min(tl.y, (double) p.second.y);
		br.x = max(br.x, (double) p.second.x);
		br.y = max(br.y, (double) p.second.y);
	}
	return {tl,br};
}
cv::Rect2d BoundingBox::get_camera_rect(const cv::Mat intrinsic, const cv::Mat distortion,const frustum::Frustum &frustum) const {
	if (intrinsic.empty()||intrinsic.cols != 3 || intrinsic.rows != 3)
		return cv::Rect2d(0, 0, 0, 0);
	std::vector<std::pair<cv::Point3d, cv::Point3d>> lines;
	if (!frustum.planes.empty()) {
		lines = get_visibil_lines(frustum);
	} else {
		lines = get_lines();
	}
	if (lines.empty())
		return cv::Rect2d(0, 0, 0, 0);
	auto projectedLines = project_lines(lines, intrinsic, distortion);
	auto tlBr = get_top_left_and_bottum_right(projectedLines);
	return cv::Rect2d(tlBr.first,tlBr.second);
}
BoundingBox BoundingBox::interpolate(BoundingBox b1, BoundingBox b2, double s2, bool cameraFrame) {
	if (s2 < 0.0 || s2 > 1.0) {
			throw std::out_of_range("s2 must be within the range [0, 1]");
	 	}
	double s1 = 1 - s2;
	cv::Point3d center = b1.center * s1 + b2.center * s2;
	double heading = Pose::interpolate_heading(b1.angle, b2.angle, s2);
	double w = b1.width * s1 + b2.width * s2;
	double l = b1.length * s1 + b2.length * s2;
	double h = b1.height * s1 + b2.height * s2;
	BoundingBox b3(center.x, center.y, center.z, l, w, h, heading, cameraFrame);
	b3.timestamp = b1.timestamp * s1 + b2.timestamp * s2;
	b3.id = b1.id * s1 + b2.id * s2;
	return b3;
}
std::ostream& operator<<(std::ostream &outStream, const BoundingBox &p) {
	outStream << p.center.x << "," << p.center.y << "," << p.center.z << "," << p.length << "," << p.width << "," << p.height << "," << p.angle;
	return outStream;
}

BoundingBox& BoundingBox::operator=(const BoundingBox &b) {
	classId = b.classId;
	angle = b.angle;
	width = b.width;
	height = b.height;
	length = b.length;
	l_2 = length / 2;
	w_2 = width / 2;
	h_2 = height / 2;
	id = b.id;
	timestamp = b.timestamp;
	corners = b.corners;
	center = b.center;
	u = b.u;
	v = b.v;
	w = b.w;
	ca = b.ca;
	sa = b.sa;
	minDim = b.minDim;
	maxDim = b.maxDim;
	color = b.color;
	return *this;
}

void BoundingBox::update_orientation(double roll, double pitch, double yaw, bool camFrame) {
	if (!camFrame) {
		cv::Mat R;
		angle = fmod(yaw + M_PI * 3, M_PI * 2) - M_PI;
		calculate_rotation_matrix(roll, pitch, angle, R);
		cv::Mat U = (cv::Mat_<double>(3, 1) << 1, 0, 0);
		cv::Mat V = (cv::Mat_<double>(3, 1) << 0, 1, 0);
		cv::Mat W = (cv::Mat_<double>(3, 1) << 0, 0, 1);
		U = R * U;
		V = R * V;
		W = R * W;
		u = cv::Point3d(U.at<double>(0, 0), U.at<double>(1, 0), U.at<double>(2, 0));
		v = cv::Point3d(V.at<double>(0, 0), V.at<double>(1, 0), V.at<double>(2, 0));
		w = cv::Point3d(W.at<double>(0, 0), W.at<double>(1, 0), W.at<double>(2, 0));
	}
	update();
}

void BoundingBox::update() {
	ca = cos(angle);
	sa = sin(angle);
	corners.clear();
	l_2 = length / 2.;
	w_2 = width / 2.;
	h_2 = height / 2.;
	auto u_2 = u * 0.5 * length;
	auto v_2 = v * 0.5 * width;
	auto w_2 = w * height * 0.5;
	corners.push_back(center - u_2 - v_2 - w_2);
	corners.push_back(center - u_2 + v_2 - w_2);
	corners.push_back(center + u_2 + v_2 - w_2);
	corners.push_back(center + u_2 - v_2 - w_2);
	corners.push_back(center + u_2 - v_2 + w_2);
	corners.push_back(center - u_2 - v_2 + w_2);
	corners.push_back(center - u_2 + v_2 + w_2);
	corners.push_back(center + u_2 + v_2 + w_2);
	auto p = enclousing_bounding_box();
	minDim = p.first;
	maxDim = p.second;
}
cv::RotatedRect BoundingBox::get_birds_eye_rectangle(bool cameraFrame) const {
	if (corners.size()<3)
		return cv::RotatedRect();
	if (cameraFrame) {
		return cv::RotatedRect(cv::Point2f(corners[0].z, corners[0].x), cv::Point2f(corners[1].z, corners[1].x), cv::Point2f(corners[2].z, corners[2].x));

	}
	cv::RotatedRect r;
	try {
		r = cv::RotatedRect(cv::Point2f(corners[0].x, corners[0].y), cv::Point2f(corners[1].x, corners[1].y), cv::Point2f(corners[2].x, corners[2].y));
	} catch (...) {
		cerr << "center" << center << " lwh:" << length << "," << width << "," << height << endl;
		cerr << "volume" << get_volume() << endl;
		cerr << "vectors:" << u << v << w << endl;
		cerr << "gets birds eye rect" << corners[0] << corners[1] << corners[2] << endl;
		for (auto p : corners) {
			cerr << p << endl;
		}

	}
	return r;
}
std::pair<cv::Point3d, cv::Point3d> BoundingBox::enclousing_bounding_box() const {
	return ikg::find_min_max_3D(corners);
}
BoundingBox::BoundingBox(double x, double y, double z, double length, double width, double h, double heading, bool cameraCoordinateFrame) {
	angle = heading;

	ca = cos(angle);
	sa = sin(angle);
	this->length = length;
	this->width = width;
	height = h;
	if (!cameraCoordinateFrame) {
		u = cv::Point3d(ca, sa, 0);
		v = cv::Point3d(-sa, ca, 0);
		w = cv::Point3d(0, 0, 1);
	} else {
		u = cv::Point3d(ca, 0, -sa);
		w = cv::Point3d(0, -1, 0);
		v = cv::Point3d(sa, 0, ca);
	}
	id = 0;
	center.x = x;
	center.y = y;
	center.z = z;
	update();
	timestamp = 0;

}
double BoundingBox::intersection_over_union_3d(const BoundingBox &b, bool camera) const {
	double volume1 = get_volume();
	double volume2 = b.get_volume();
	if (volume1 == 0 || volume2 == 0) {
		return 0;
	}
	double a = overlapp(b, camera);
	return a / (volume1 + volume2 - a);
}
double BoundingBox::intersection_over_union_2d(const BoundingBox &b, bool camera) const {
	cv::RotatedRect r1 = b.get_birds_eye_rectangle(camera);
	cv::RotatedRect r2 = get_birds_eye_rectangle(camera);
	vector<cv::Point2f> corners;
	double con = 0;
	int c = cv::rotatedRectangleIntersection(r1, r2, corners);
	if (c == cv::INTERSECT_NONE || corners.size() == 0)
		return 0;
	for (int k = 0; k < corners.size(); k++) {
		const auto &p = corners[k];
		if (k > 0) {
			const auto &p2 = corners[k - 1];
			con += p.x * p2.y - p.y * p2.x;
		}
		if (k == 0) {
			const auto &p2 = corners.back();
			con += p.x * p2.y - p.y * p2.x;
		}
	}
	con = fabs(con * 0.5);
	double a1 = r1.size.height * r1.size.width;
	double a2 = r2.size.height * r2.size.width;
	return con / (a1 + a2 - con);
}
void BoundingBox::invert_heading(){
	update_orientation(0, 0, fmod(angle+Pi2,Pi2)-M_PI, false);
}
double BoundingBox::overlapp(const BoundingBox &b, bool camera) const {
	cv::RotatedRect r1 = b.get_birds_eye_rectangle(camera);
	cv::RotatedRect r2 = get_birds_eye_rectangle(camera);
	double h = -1;
	double z1 = center.z, z2 = b.center.z;
	if (camera) {
		z1 = center.y;
		z2 = b.center.y;
	}
	if (b.height * 0.5 + height * 0.5 < fabs(z2 - z1))
		return 0;
	double top = min(z2 + b.h_2, z1 + h_2);
	double bottom = max(z2 - b.h_2, z1 - h_2);
	h = top - bottom;
	if (h < 0)
		return 0;
	if (b.maxDim.x < minDim.x || b.maxDim.y < minDim.y || b.maxDim.z < minDim.z)
		return 0;
	if (b.minDim.x > maxDim.x || b.minDim.y > maxDim.y || b.minDim.z > maxDim.z)
		return 0;
	vector<cv::Point2f> corners;
	double con = 0;
	int c = cv::rotatedRectangleIntersection(r1, r2, corners);
	if (corners.size() == 0)
		return 0;
	for (int k = 0; k < corners.size(); k++) {
		const auto &p = corners[k];
		if (k > 0) {
			const auto &p2 = corners[k - 1];
			con += p.x * p2.y - p.y * p2.x;
		}
		if (k == 0) {
			const auto &p2 = corners.back();
			con += p.x * p2.y - p.y * p2.x;
		}
	}
	con = fabs(con * 0.5);
	return con * h;
}

BoundingBox BoundingBox::transform(cv::Mat p, bool cam, bool yaw) {
	BoundingBox b = *this;
	if(p.empty()||p.rows!=4||p.cols!=4)
		return b;
	cv::Mat C = (cv::Mat_<double>(4, 1) << center.x, center.y, center.z, 1);
	cv::Mat U = (cv::Mat_<double>(3, 1) << u.x, u.y, u.z);
	cv::Mat V = (cv::Mat_<double>(3, 1) << v.x, v.y, v.z);
	cv::Mat W = (cv::Mat_<double>(3, 1) << w.x, w.y, w.z);
	C = p * C;
	U = p(cv::Rect(0, 0, 3, 3)) * U;
	V = p(cv::Rect(0, 0, 3, 3)) * V;
	W = p(cv::Rect(0, 0, 3, 3)) * W;
	b.angle = atan2(U.at<double>(1, 0), U.at<double>(0, 0));
	if (cam)
		b.angle = atan2(-U.at<double>(2, 0), U.at<double>(0, 0));

	b.ca = cos(b.angle);
	b.sa = sin(b.angle);
	if (yaw && cam) {
		U = (cv::Mat_<double>(3, 1) << b.ca, 0, -b.sa);
		W = (cv::Mat_<double>(3, 1) << 0, -1, 0);
		V = (cv::Mat_<double>(3, 1) << b.sa, 0, b.ca);
	} else if (yaw) {
		U = (cv::Mat_<double>(3, 1) << b.ca, b.sa, 0);
		V = (cv::Mat_<double>(3, 1) << -b.sa, b.ca, 0);
		W = (cv::Mat_<double>(3, 1) << 0, 0, 1);
	}
	b.center = cv::Point3d(C.at<double>(0, 0), C.at<double>(1, 0), C.at<double>(2, 0));
	b.u = cv::Point3d(U.at<double>(0, 0), U.at<double>(1, 0), U.at<double>(2, 0));
	b.v = cv::Point3d(V.at<double>(0, 0), V.at<double>(1, 0), V.at<double>(2, 0));
	b.w = cv::Point3d(W.at<double>(0, 0), W.at<double>(1, 0), W.at<double>(2, 0));
	b.update();
	return b;
}

BoundingBox::BoundingBox() {
	length = 1;
	width = 1;
	height = 1;
	angle = 0;
	timestamp = 0;
	id = 0;
	u = cv::Point3d(1, 0, 0);
	v = cv::Point3d(0, 1, 0);
	w = cv::Point3d(0, 0, 1);
	update();

}

BoundingBox::~BoundingBox() {
}
template<class T>
BoundingBox BoundingBox::adjust(const std::vector<T> &data) {
	if (data.empty()) {
		throw std::invalid_argument("Data vector is empty");
	}
	BoundingBox bb;
	bb.id = 0;
	bb.timestamp = 0;
	cv::Mat src(data.size(), 3, CV_64F, cv::Scalar(0));
	cv::Point3d c(0, 0, 0);
	std::unordered_map<int, std::unordered_set<int>> grid;
	double res_ = 1. / 0.01;
	std::vector<cv::Point3d> locP, filtered;
	locP.reserve(data.size());
	filtered.reserve(data.size());
	for (const auto &p : data) {
		int x = p.x * res_;
		int y = p.y * res_;
		if (grid.count(x) > 0 && grid[x].count(y) > 0)
			continue;
		locP.push_back(p);
		grid[x].insert(y);
		c.x += p.x;
		c.y += p.y;
		c.z += p.z;
	}

	c /= (double) locP.size();
	for (auto &p : locP)
		p -= c;
	cv::Mat S = cv::Mat::zeros(2, 2, CV_64F);
	for (int i = 0; i < locP.size(); i++) {
		S.at<double>(0, 0) += locP[i].x * locP[i].x;
		S.at<double>(0, 1) += locP[i].y * locP[i].x;
		S.at<double>(1, 1) += locP[i].y * locP[i].y;
	}
	S.at<double>(1, 0) = S.at<double>(0, 1);
	double T0 = S.at<double>(0, 0);
	double D = S.at<double>(0, 0) * S.at<double>(1, 1) - S.at<double>(1, 0) * S.at<double>(0, 1);
	double T2 = T0 * T0;
	double L1 = T0 / 2 + sqrt(T2 / 4 - D);
	cv::Point2d l1(L1 - S.at<double>(1, 1), S.at<double>(0, 1));
	bb.angle = atan2(l1.y, l1.x);
	double cs = cos(bb.angle);
	double ss = sin(bb.angle);
	double minZ = DBL_MAX;
	double minW = DBL_MAX, maxW = -DBL_MAX;
	double minL = DBL_MAX, maxL = -DBL_MAX;
	double minH = DBL_MAX, maxH = -DBL_MAX;
	bb.u = cv::Point3d(cs, ss, 0); //p.asmPoints[20 - 1] - p.asmPoints[92 - 1];
	bb.v = cv::Point3d(-ss, cs, 0);	//p.asmPoints[7 - 1] - p.asmPoints[61 - 1];
	bb.w = cv::Point3d(0, 0, 1);		//u.cross(v);
	for (int i = 0; i < locP.size(); i++) {
		maxL = std::max(maxL, (bb.u.dot(locP[i])));
		maxW = std::max(maxW, (bb.v.dot(locP[i])));
		maxH = std::max(maxH, (bb.w.dot(locP[i])));
		minL = std::min(minL, (bb.u.dot(locP[i])));
		minW = std::min(minW, (bb.v.dot(locP[i])));
		minH = std::min(minH, (bb.w.dot(locP[i])));
		minZ = std::min(minZ, locP[i].z);
	}
	bb.height = maxH - minH;
	bb.length = maxL - minL;
	bb.width = maxW - minW;
	auto u_2 = bb.u * 0.5 * bb.length;
	auto v_2 = bb.v * 0.5 * bb.width;
	auto w_2 = bb.w * 0.5 * bb.height;
	bb.center.x = c.x;
	bb.center.y = c.y;
	bb.center.z = c.z;
	bb.center += bb.u * (maxL + minL) * 0.5;
	bb.center += bb.v * (maxW + minW) * 0.5;
	bb.center += bb.w * (maxH + minH) * 0.5;
	bb.update();
	return bb;
}
double BoundingBox::dist_to_surface(cv::Point3d p) {
	cv::Point3d delta = cv::Point3d(p.x, p.y, p.z) - center;
	double d = 0;
	double du = (u.dot(delta));
	double dv = (v.dot(delta));
	double dw = (w.dot(delta));
	double bdl = fabs(du) - l_2;
	double bdv = fabs(dv) - w_2;
	double bdw = fabs(dw) - h_2;
	bool withinLength = false, withinWidth = false, withinHeigth = false;
	withinLength = bdl < 0;
	withinHeigth = bdw < 0;
	withinWidth = bdv < 0;
	if (withinLength && withinHeigth && withinWidth) {
		return d = std::min(bdw * bdw, std::min(bdv * bdv, bdl * bdl));
	}
	if (!withinLength)
		d += bdl * bdl;
	if (!withinWidth)
		d += bdv * bdv;
	if (!withinHeigth)
		d += bdw * bdw;
	return d;
}
