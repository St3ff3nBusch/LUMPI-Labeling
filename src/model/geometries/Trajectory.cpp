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

#include <model/geometries/Trajectory.h>
#include <model/StructsAndMore.h>
#include <iostream>
#include <map>
#include <set>
using namespace std;
using namespace ikg;

std::ostream& operator<<(std::ostream &outStream, const Trajectory &t) {
	outStream.precision(10);
	outStream << "<trajectory>" << t.id << "," << t.classId << std::endl << "<pose_data>" << std::endl;
	for (int i = 0; i < t.poseData.size(); i++) {
		outStream << t.poseData[i] << std::endl;
	}
	outStream << "</pose_data>" << std::endl << "</trajectory>";
	return outStream;
}
Trajectory::Trajectory() {
	id = 0;
	classId = 0;

}
Trajectory::~Trajectory() {

}

double Trajectory::calc_curvature(const Pose &A, const Pose &B, const Pose &C) {
	cv::Point2d v1(A.x - B.x, A.y - B.y);
	cv::Point2d v2(B.x - C.x, B.y - C.y);
	cv::Point2d v3(C.x - A.x, C.y - A.y);
	//K = 2*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)) / sqrt( ...
	//((x2-x1)^2+(y2-y1)^2)*((x3-x2)^2+(y3-y2)^2)*((x1-x3)^2+(y1-y3)^2) );
	double l1 = v1.x*v1.x+v1.y*v1.y;
	double l2 =v2.x*v2.x+v2.y*v2.y;
	double l3 = v3.x*v3.x+v3.y*v3.y;
	if (l1 * l2 * l3 == 0)
		return 0;
	double a = A.x * (v2.y) + B.x * (v3.y) + C.x * (v1.y);
	return 2 * a / (l1 * l2 * l3);
}
void Trajectory::insert_pose(const Pose &p) {
	int pose = 0;
	sort(poseData.begin(), poseData.end(), [](const Pose &p1, const Pose &p2) {
		return p1.time < p2.time;
	});
	for (const auto &p2 : poseData) {
		if (p.time > p2.time) {
			break;
		}
		pose++;
	}
	poseData.insert(poseData.begin() + pose, p);
}

void Trajectory::interpolate_between_key_frames(double keyframe, double frequence) {
	if (poseData.empty())
		return;
	map<double, Pose> keyPose;
	for (auto &p : poseData) {
		if ((int) floor(p.time *frequence) % (int) keyframe == 0) {
			keyPose[floor(p.time * frequence) / frequence] = p;
		}
	}
	if (keyPose.empty())
		return;
	cv::Point3d dvS;
	double oS;
	double lS;
	double startTime = poseData.front().time;
	if (keyPose.find(startTime) == keyPose.end()) {
		auto it = keyPose.begin();
		auto it2 = next(it, 1);
		double dt = it2->second.time - it->second.time;
		dvS = (it->second.bb.center - it2->second.bb.center) / dt;
		oS = it->second.bb.angle;
		lS = it2->first - it->first;
	}
	cv::Point3d dvE;
	double oE;
	double lE;
	double endTime = poseData.back().time;
	if (keyPose.find(endTime) == keyPose.end()) {
		auto it = keyPose.rbegin();
		auto it2 = next(it, 1);
		double dt = it2->second.time - it->second.time;
		dvE = (it->second.bb.center - it2->second.bb.center) / dt;
		oE = it->second.bb.angle;
		lE = it2->first - it->first;
	}
	for (auto &p : poseData) {
		if ((int) floor(p.time * frequence) % (int) keyframe == 0)
			continue;
		else if (p.time <= keyPose.begin()->first) {
			double s = (keyPose.begin()->first - p.time) / lS;
			p.bb = keyPose.begin()->second.bb;
			p.bb.center += dvS * s;
			p.bb.update_orientation(oS, 0, 0, false);

		} else if (p.time >= keyPose.rbegin()->first) {
			double s = (p.time - keyPose.rbegin()->first) / lS;
			p.bb = keyPose.rbegin()->second.bb;
			p.bb.center += dvE * s;
			p.bb.update_orientation(oS, 0, 0, false);

		} else {
			auto it = keyPose.lower_bound(p.time);
			auto it2 = prev(it);
			if (it == keyPose.begin())
				continue;
			double l = it->second.time - it2->second.time;
			double l2 = p.time - it2->second.time;
			p.bb = Pose::interpolate(it2->second, it->second, l2 / l).bb;
		}
		auto tmp = p.bb.enclousing_bounding_box();
		p.r = cv::Rect2d(cv::Point2d(tmp.first.x, tmp.first.y), cv::Point2d(tmp.second.x, tmp.second.y));
		p.w = p.r.width;
		p.h = p.r.height;
		p.x = p.r.x;
		p.y = p.r.y;
	}
}
bool Trajectory::is_finialized() const {
	for (auto &p : poseData) {
		if (p.finalized)
			return true;
	}
	return false;
}
bool Trajectory::merge_time(const Trajectory &t, double distTh) {
	int sIndex = -1, eIndex = -1;
	map<int, Pose> newPoses;
	for (auto p : poseData)
		newPoses.insert( { p.time * 10, p });
	for (auto p : t.poseData)
		newPoses.insert( { p.time * 10, p });
	poseData.clear();
	for (auto p : newPoses)
		poseData.push_back(p.second);
	return true;
}
bool Trajectory::get_pose_at(double time, Pose &p, int index) const {
	if (poseData.empty())
		return false;
	if (time < poseData.front().time || time > poseData.back().time)
		return false;
	if (index < 0) {
		auto it = std::lower_bound(poseData.begin(), poseData.end(), time, [](const Pose &a, double value) {
			return a.time < value;
		});
		if (it == poseData.end())
			return false;
		if (it->time == time) {
			p = *it;
			return true;
		}
		if (it == poseData.begin())
			return false;
		const Pose &p1 = *prev(it);
		const Pose &p2 = *it;
		double s = (time - p1.time) / (p2.time - p1.time);
		p = Pose::interpolate(p1, p2, s);
		return true;
	} else {
		if (index > poseData.size() - 1 || index < 0)
			return false;
		const auto &p1 = poseData[index];
		if (p1.time == time) {
			p = p1;
			return true;
		}
		if (index == 0)
			return false;
		if (time > p1.time && index < poseData.size() - 1) {
			const auto &p2 = poseData[index - 1];
			double s = (time - p1.time) / (p2.time - p1.time);
			p = Pose::interpolate(p1, p2, s);
			return true;
		}
		return false;
	}
}
void Trajectory::remove_duplicate(double frequence) {
	for (auto it = next(poseData.begin(), 1); it != poseData.end();) {
		if (floor(prev(it)->time * 10) == floor(it->time * 10)) {
			it = poseData.erase(it);
		} else {
			it++;
		}
	}
}
void Trajectory::update_interpolated_pose_bounding_boxes() {
	if (poseData.empty())
		return;
	Pose *lastFixPose = &*poseData.begin();
	Pose *nextFix = &*poseData.begin();
	for (int i = 1; i < poseData.size() - 1; i++) {
		if (poseData[i].interpolated) {
			if (poseData[i].time > nextFix->time) {
				for (int j = i + 1; j < poseData.size(); j++) {
					if (!poseData[j].interpolated) {
						nextFix = &poseData[j];
						break;
					}
				}
			}
			poseData[i].bb = BoundingBox::interpolate(lastFixPose->bb, nextFix->bb, (poseData[i].time - lastFixPose->time) / (nextFix->time - lastFixPose->time), false);
		} else if (poseData[i].standing) {
			poseData[i].bb = lastFixPose->bb;
		} else {
			lastFixPose = &poseData[i];
		}
	}
}
std::vector<int> Trajectory::valid(double dist, double curv) {
	std::vector<int> invalid;
	for (int i = 1; i < poseData.size() - 1; i++) {
		double c = calc_curvature(poseData[i - 1], poseData[i], poseData[i + 1]);
		double d = Pose::distance_in_2d(poseData[i - 1], poseData[i]);
		if (fabs(c) > curv || d > dist) {
			invalid.push_back(i);
		}
	}
	double d = Pose::distance_in_2d(poseData[poseData.size() - 1], poseData[poseData.size() - 2]);
	if (d > dist)
		invalid.push_back(poseData.size() - 1);
	return invalid;

}

