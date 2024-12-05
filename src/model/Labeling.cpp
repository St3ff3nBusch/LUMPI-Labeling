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

#include "Labeling.h"
#include "Plotter.h"
#include <pcl/visualization/common/common.h>
#include "Frustum.h"
#include "LUMPIPipeline.h"
#include <pcl/surface/convex_hull.h>
//#define VERBOSE_LABELING(...) std::cout<<"In:"<<__FILENAME__<<"at:"<<__LINE__<<":"<<__VA_ARGS__<<std::endl;
#define VERBOSE_LABELING(...) while(false){};
using namespace std;
namespace ikg {
std::vector<IKGB> Labeling::split_point_segment(std::vector<IKGB> &points, double th) {
	VERBOSE_LABELING("split point segment")
	sort(points.begin(), points.end(), [](const IKGB &p1, const IKGB &p2) {
		return p1.adjustedtime < p2.adjustedtime;
	});
	vector<double> dt;
	vector<int> split;
	for (int i = 1; i < points.size(); i++)
		dt.push_back((double) points[i].adjustedtime - points[i - 1].adjustedtime);
	vector<int> splits;
	for (int i = 0; i < dt.size(); i++) {
		if (dt[i] > th) {
			split.push_back(i);
		}
	}
	if (split.empty())
		return points;
	std::vector<IKGB> filterdPoints;
	int maxSplit = split.front();
	int s = 0, e = split.front();
	if (split.size() < 2) {
		if (split.front() > points.size() / 2) {
			filterdPoints = std::vector<IKGB>(points.begin() + s, points.begin() + e);
		} else {
			filterdPoints = std::vector<IKGB>(points.begin() + e, points.end());
		}
	}
	for (int i = 1; i < split.size(); i++) {
		int size = split[i] - split[i - 1];
		if (size > maxSplit) {
			s = e + 1;
			e = split[i];
			maxSplit = size;
		}
	}
	filterdPoints = std::vector<IKGB>(points.begin() + s, points.begin() + e);
	return filterdPoints;
}

cv::Mat Labeling::generate_camera_global_view(double time, int camId) {
	VERBOSE_LABELING("generate cam view")
	cv::Mat img(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
	if (cams.count(camId) < 1 || caps.count(camId) < 1)
		return img.clone();
	int frame = floor(time * pointCloudFrequenz);
	Sensor cam = cams.at(camId);
	int cFrame = time * cam.fps;
	auto cap = caps.at(camId);
	cap.set(cv::CAP_PROP_POS_FRAMES, cFrame);
	cap.read(img);
	if (img.empty()) {
		img = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
		return img.clone();
	}
	camViewObjCenter.clear();
	if (posePerTime.count(frame) > 0) {
		for (const auto &p : posePerTime.at(frame)) {
			auto t = id2traj.at(p.second->tId);
			Pose p2 = *p.second;
			t->get_pose_at(time, p2);
			const auto bb0 = p2.bb;
			auto bb = p2.bb.transform(cam.extrinsic);
			cv::Vec3b color(0, 255, 0);
			if (t->id == activObjId)
				color = cv::Vec3b(0, 0, 255);
			bb.plot(img, color, cam.intrinsic, cam.distortion, 1, cam.frustum);
			auto r = bb.get_camera_rect(cam.intrinsic, cam.distortion, cam.frustum);
			camViewObjCenter.insert( { p.second->tId, r });
			cv::putText(img, to_string(t->id).c_str(), r.tl(), 1, 1., color, 1);
			if (models.count(p.first) > 0) {
				vector<cv::Point3f> obj;
				for (const auto &mp : models.at(p.first)) {
					cv::Point3f p(bb0.ca * mp.x - bb0.sa * mp.y + bb0.center.x, bb0.sa * mp.x + bb0.ca * mp.y + bb0.center.y, mp.z + bb0.center.z);
					cv::Mat P = cam.extrinsic * (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);
					if (cam.frustum.within_frustum<cv::Point3f>(cv::Point3f(P.at<double>(0, 0), P.at<double>(1, 0), P.at<double>(2, 0))))
						obj.push_back(p);
				}
				if (obj.empty())
					continue;
				vector<cv::Point2f> imgP;
				cv::projectPoints(obj, cam.rvec, cam.tvec, cam.intrinsic, cam.distortion, imgP);
				vector<cv::Point> contour;
				for (auto ip : imgP) {
					contour.push_back(cv::Point(ip.x, ip.y));
				}
				vector<cv::Point> hull;
				if (contour.empty())
					continue;
				cv::convexHull(contour, hull);
				if (hull.size() > 0) {
					cv::Vec3b color = LUMPIPipeline::switch_class_color(id2traj[p.second->tId]->classId);
					cv::fillConvexPoly(img, hull, color);
				}
			}
		}
	}
	return img.clone();
}
void Labeling::reset_pc_views() {
	VERBOSE_LABELING("reset views")
	globPcView.reset(new pcl::visualization::PCLVisualizer("global", false));
	globPcView->setBackgroundColor(255, 255, 255);
	modelPcView.reset(new pcl::visualization::PCLVisualizer("model", false));
	modelPcView->setBackgroundColor(255, 255, 255);
	for (int i = 0; i < slidingWindow * 2 + 1; i++) {
		pcViews[i].reset(new pcl::visualization::PCLVisualizer("Viewer3d" + to_string(i), false));
		pcViews[i]->setBackgroundColor(255, 255, 255);
	}
}

void Labeling::init(std::string dataPath, std::string trajectoryPath, int expId) {
	VERBOSE_LABELING("init")
	clouds.clear();
	camView.clear();
	topView.clear();
	sideView.clear();
	this->dataPath = dataPath;
	this->measurementId = expId;
	load_back_ground();
	cameraPath = dataPath + "/" + measurementSubString + to_string(expId) + "/cam/";
	metaPath = dataPath + "/meta.json";
	pointCloudPath = dataPath + "/" + measurementSubString + to_string(expId) + "/lidar/";
	this->trajectoryPath = trajectoryPath;
	actualIndex = actualStepSize * 3;
	load_camera_parameters_and_videos();
	reset_pc_views();
	VERBOSE_LABELING("loading trajectories")
	load_trajectories(this->trajectoryPath);
	if (!pcTrajectories.empty())
		activObjId = pcTrajectories.front().id;
	load_point_cloud_sliding_window();
	if (!cams.empty()) {
		for (auto &c : clouds) {
			selectedCam[c.first] = cams.begin()->first;
		}

		globalCameraId = cams.begin()->first;
	}
	update_pc_index_list();
}
void Labeling::add_obj(cv::Point3d p) {
	int id = 0;
	if (!id2traj.empty()) {
		id = id2traj.rbegin()->first + 1;
	}
	Trajectory t;
	t.id = id;
	Pose po;
	po.bb = BoundingBox(p.x, p.y, p.z, 1, 1, 1, 0, false);
	po.tId = id;
	po.time = actualIndex /pointCloudFrequenz;
	t.poseData.push_back(po);
	pcTrajectories.push_back(t);
	update_obj_ptr();
}
double Labeling::get_time_of_largest_point_segment(std::vector<double> &times, double th) {
	sort(times.begin(), times.end());
	vector<vector<double>> segments(1);
	segments.back().push_back(times.front());
	for (int i = 1; i < times.size(); i++) {
		if (times[i] - times[i - 1] > th) {
			segments.push_back(vector<double>());
		}
		segments.back().push_back(times[i]);
	}
	sort(segments.begin(), segments.end(), [](auto const &a, auto const &b) {
		return a.size() > b.size();
	});
	return accumulate(segments.front().begin(), segments.front().end(), 0.) / (double) segments.front().size() * msec2sec;
}
void Labeling::grid_to_point_cloud(const unordered_map<int, unordered_map<int, unordered_map<int, int> > > &grid, const double &resolution, vector<IKGB> &points) {
	for (const auto &x : grid) {
		for (const auto &y : x.second) {
			for (const auto &z : y.second) {
				IKGB p;
				p.x = x.first * resolution;
				p.y = y.first * resolution;
				p.z = z.first * resolution;
				p.index = z.second;
				points.push_back(p);
			}
		}
	}
}

void Labeling::point_vector_to_xyz_cloud(const vector<IKGB> &points, pcl::PointCloud<pcl::PointXYZ> &cloudXYZ) {
	for (const auto &p : points) {
		pcl::PointXYZ p2;
		p2.x = p.x;
		p2.y = p.y;
		p2.z = p.z;
		cloudXYZ.push_back(p2);
	}
}

vector<int> Labeling::get_convex_hull_indices(const vector<IKGB> &points) {
	pcl::PointCloud<pcl::PointXYZ> tmpCloud;
	point_vector_to_xyz_cloud(points, tmpCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(tmpCloud.makeShared());
	chull.reconstruct(*cloud_hull);
	pcl::PointIndices indices;
	chull.getHullPointIndices(indices);
	vector<int> id = indices.indices;
	return id;
}

void Labeling::aggregate_points(const int &tId, const int &sIndex, const int &eIndex, const double &resolution, const int &stepSize) {
	VERBOSE_LABELING("GENERATE"<<tId)
	if (id2traj.count(tId) < 1)
		return;
	auto t = id2traj.at(tId);
	unordered_map<int, unordered_map<int, unordered_map<int, int>>> grid;
	int startFrame = floor(t->poseData.front().time * pointCloudFrequenz);
	int count = 0;
	for (auto &p : t->poseData) {
		int index = floor(p.time * pointCloudFrequenz);
		if (index < sIndex || index > eIndex)
			continue;
		if ((index - startFrame) % stepSize > 0)
			continue;
		stringstream ss;
		ss << pointCloudPath << "/" << std::setw(6) << std::setfill('0') << index << ".ply";
		auto pc = Parser::load_cloud_ikg_benchmark(ss.str());
		std::pair<std::vector<int>, std::vector<int>> fg = LUMPIPipeline::remove_back_ground(pc, background, backgroundAngleNormalizer);
		vector<double> times;
		for (const int &i : fg.first) {
			if (p.bb.within<IKGB>(pc.points[i])) {
				double x = pc.points[i].x - p.bb.center.x;
				double y = pc.points[i].y - p.bb.center.y;
				double z = pc.points[i].z - p.bb.center.z;
				double x2 = x * p.bb.ca + y * p.bb.sa;
				double y2 = x * -p.bb.sa + y * p.bb.ca;
				grid[floor(x2 / resolution)][floor(y2 / resolution)][floor(z / resolution)]++;
				count++;
				times.push_back(pc.points[i].adjustedtime);
			}
		}
		p.time=get_time_of_largest_point_segment(times, 0.01 * sec2msec);
		p.visibility = times.size();
	}
	vector<IKGB> points;
	grid_to_point_cloud(grid, resolution, points);
	vector<int> id = get_convex_hull_indices(points);
	cv::Mat tmp(id.size(), 1, CV_64F, cv::Scalar(0));
	for (int i = 0; i < id.size(); i++) {
		tmp.at<double>(i, 0) = points[id[i]].index;
	}
	auto heat = ikg::Plotter::get_heat_map(tmp);
	models[tId].clear();
	auto &modelPoints = models.at(tId);
	modelPoints.reserve(id.size());
	for (int i = 0; i < id.size(); i++) {
		cv::Vec3b color = heat.at<cv::Vec3b>(i, 0);
		int index = id[i];
		points[index].b = color[0];
		points[index].g = color[1];
		points[index].r = color[2];
		modelPoints.push_back(points[index]);
	}
	generate_model_view();
}
void Labeling::creat_time_heat_map(const std::vector<IKGB> &pointCloud, cv::Mat &heatMap) {
	VERBOSE_LABELING("create time heat map")
	if (pointCloud.empty())
		return;
	cv::Mat times(pointCloud.size(), 1, CV_64F, cv::Scalar(0));
	for (int i = 0; i < pointCloud.size(); i++) {
		times.at<double>(i, 0) = pointCloud[i].adjustedtime;
	}
	heatMap = ikg::Plotter::get_heat_map(times);
}
bool Labeling::get_active_bb_at_index(int i, Pose *&p) {
	if (posePerObj.count(activObjId) > 0 && posePerObj.at(activObjId).count(i) > 0) {
		p = posePerObj.at(activObjId).at(i);
		VERBOSE_LABELING("label pointer"<<p->bb.angle)
		return true;
	}
	return false;

}
std::pair<int, double> Labeling::get_obj_at(cv::Point2d pi, double startTime, double endTime, bool activeObjOnly) const{
	VERBOSE_LABELING(pi<<":"<<startTime<<" end:"<<endTime<<"actiOnly")
	double minDist = DBL_MAX;
	int id = -1;
	double t;
	if (activeObjOnly) {
		if (posePerObj.count(activObjId) < 1)
			return {-1,0};
		pi -= cv::Point2d(singleTrajectoryWidth / 2, singleTrajectoryHeight / 2);
		pi *= singleTrajectoryResolution;
		pi += singleTrajectoryStart;
		const auto &m = posePerObj.at(activObjId);
		for (const auto &p : m) {
			if (p.first % actualStepSize > 0)
				continue;
			cv::Point2d d = cv::Point2d(p.second->bb.center.x, p.second->bb.center.y) - pi;
			double dist = cv::norm(d);
			if (dist < minDist) {
				minDist = dist;
				id = p.second->tId;
				t = p.second->time;
			}
		}
	} else {
		pi -= globalTrajectoryCenter;
		pi *= globalTrajectoryResolution;
		if (endTime <= startTime)
			endTime = startTime + 1./pointCloudFrequenz;
		auto itS = posePerTime.lower_bound(startTime * pointCloudFrequenz);
		auto itE = posePerTime.lower_bound(endTime * pointCloudFrequenz);
		while (itS != itE) {
			for (const auto &p : itS->second) {
				cv::Point2d d = cv::Point2d(p.second->bb.center.x, p.second->bb.center.y) - pi;
				double dist = cv::norm(d);
				if (dist < minDist) {
					minDist = dist;
					id = p.second->tId;
					t = p.second->time;
				}
			}
			itS++;
		}
	}
	return {id,t};
}
std::pair<int, double> Labeling::get_obj_at(cv::Point3d c, int index)const{
	std::pair<int, double> closetObject;
	double minDist = DBL_MAX;
	int id = -1;
	if (posePerTime.count(index) > 0) {
		for (const auto &p : posePerTime.at(index)) {
			cv::Point3d d = p.second->bb.center - cv::Point3d(c.x, c.y, c.z);
			double dist = cv::norm(d);
			if (dist < minDist) {
				minDist = dist;
				id = p.first;
			}
		}
	}
	return{id,minDist};
}
void Labeling::update_obj_ptr() {
	posePerObj.clear();
	posePerTime.clear();
	id2traj.clear();
	for (auto &t : pcTrajectories) {
		for (auto &p : t.poseData) {
			int ind = floor(p.time * pointCloudFrequenz);
			p.tId = t.id;
			posePerTime[ind][t.id] = &p;
			posePerObj[t.id][ind] = &p;
		}
		id2traj[t.id] = &t;
	}
}
bool Labeling::erase_pose(int objId, double startTime, double endTime) {
	VERBOSE_LABELING("erase"<<objId<<"at"<<startTime<<"end"<<endTime)
	if (endTime < startTime)
		endTime = startTime;
	if (id2traj.count(objId) < 1)
		return false;
	auto t = id2traj.at(objId);
	for (auto it = t->poseData.rbegin(); it < t->poseData.rend(); it++) {
		if (it->finalized)
			continue;
		if (it->time >= startTime && it->time <= endTime)
			t->poseData.erase((it + 1).base());
	}
	if (t->poseData.empty())
		erase_obj(t->id);
	update_obj_ptr();
	return true;
}
bool Labeling::erase_obj(int objId) {
	VERBOSE_LABELING("ERASE"<<objId)
	auto it = pcTrajectories.begin();
	while (it != pcTrajectories.end()) {
		if (it->id == objId) {
			for (const auto &p : it->poseData) {
				if (p.finalized)
					return false;
			}
			pcTrajectories.erase(it);
			break;
		}
		it++;
	}
	update_obj_ptr();
	return true;
}
std::vector<int> Labeling::get_intersecting_obj(int objId, double time) {
	VERBOSE_LABELING("get intersecting")
	std::vector<int> objs;
	VERBOSE_LABELING("get intersecting"<<objId<<"at"<<time)
	if (posePerObj.count(objId) < 1 || posePerObj.at(objId).count(time * pointCloudFrequenz) < 1)
		return objs;
	auto o = posePerTime.at(time * pointCloudFrequenz).at(objId);
	for (const auto &t : posePerTime.at(time * pointCloudFrequenz)) {
		if (t.first != objId && o->bb.overlapp(t.second->bb) > 0) {
			objs.push_back(t.second->tId);
		}
	}
	return objs;
}

void Labeling::extrapolate(int objId, double toTime, bool standing) {
	VERBOSE_LABELING("extrapolate"<<toTime)
	if (toTime < 0)
		return;
	Trajectory *t = id2traj.at(objId);
	for (auto &p : t->poseData) {
		p.time = floor(p.time * pointCloudFrequenz) / pointCloudFrequenz;
	}
	double speed = 0;
	Pose startP = t->poseData.front();
	int index = 0;
	while (!t->poseData.empty() && (int) (startP.time * 10) % actualStepSize > 0) {
		startP = *t->poseData.erase(t->poseData.begin());
	}
	Pose endP = t->poseData.back();
	index = t->poseData.size() - 1;
	while (!t->poseData.empty() && (int) (endP.time * 10) % actualStepSize > 0) {
		t->poseData.pop_back();
		endP = t->poseData.back();
	}
	VERBOSE_LABELING("STARTTIME"<<startP.time<<"END"<<endP.time)
	if (standing) {
		while (toTime < (int) (t->poseData.front().time * pointCloudFrequenz) / pointCloudFrequenz) {

			auto &p = startP;
			p.time -= 1./pointCloudFrequenz;
			t->poseData.insert(t->poseData.begin(), p);
			t->poseData.back().interpolated = false;
			t->poseData.back().standing = true;
		}
		while (toTime > (int) (t->poseData.back().time * pointCloudFrequenz) / pointCloudFrequenz) {
			auto &p = endP;
			p.time += 1./pointCloudFrequenz;
			t->poseData.push_back(p);
			t->poseData.back().interpolated = false;
			t->poseData.back().standing = true;
		}
	} else {
		if (toTime < startP.time) {
			for (int i = 1; i < t->poseData.size(); i++) {
				double dt = (t->poseData[i].time - startP.time);
				if (dt * pointCloudFrequenz > actualStepSize) {
					speed = cv::norm(startP.bb.center - t->poseData[i].bb.center) / dt;
					break;
				}
			}

		}
		if (toTime > endP.time) {
			for (int i = t->poseData.size() - 2; i > -1; i--) {
				double dt = (endP.time - t->poseData[i].time);
				if (dt * pointCloudFrequenz > actualStepSize) {
					speed = cv::norm(endP.bb.center - t->poseData[i].bb.center) / dt;
					break;
				}
			}

		}
		VERBOSE_LABELING("extrapolate speed"<<speed)
		while (toTime < (int) (t->poseData.front().time * pointCloudFrequenz) / pointCloudFrequenz) {
			auto &p = startP;
			p.time = floor(p.time * pointCloudFrequenz - 1) / pointCloudFrequenz;
			VERBOSE_LABELING("extra:"<<p.bb.center<<"v"<<p.bb.ca<<","<< p.bb.sa);
			p.bb.center -= cv::Point3d(p.bb.ca, p.bb.sa, 0) /pointCloudFrequenz * speed;
			p.bb.update();
			t->poseData.insert(t->poseData.begin(), p);
			t->poseData.front().interpolated = true;
		}
		bool endFlag = false;
		while (toTime > (int) (t->poseData.back().time * pointCloudFrequenz) / pointCloudFrequenz) {
			endFlag = true;
			auto &p = endP;
			p.tId = t->id;
			p.time = floor(p.time * pointCloudFrequenz + 1) / pointCloudFrequenz;
			p.bb.center += cv::Point3d(p.bb.ca, p.bb.sa, 0) /pointCloudFrequenz * speed;
			p.bb.update();
			t->poseData.push_back(p);
			t->poseData.back().interpolated = true;
		}
		if (endFlag) {
			t->poseData.back().interpolated = false;
		} else {
			t->poseData.front().interpolated = false;
		}
	}
	update_obj_ptr();
}
void Labeling::interpolate(int objId, double start, double end) {
	VERBOSE_LABELING("interpolate objId"<<objId<<"start"<<start<<"end"<<end)
	if (posePerObj.count(objId) < 1)
		return;
	auto &t = posePerObj.at(objId);
	for (auto it = t.begin(); it != t.end(); it++) {
		if (it->second->finalized)
			continue;
		if (it->second->time < start || it->second->time > end)
			continue;
		if (it->second->interpolated && it != posePerObj.at(objId).begin() && it != posePerObj.at(objId).end()) {
			auto it1 = prev(it);
			while (!it1->second->finalized && (it1->second->interpolated || it1->first % actualStepSize > 0)) {
				if (it1 == posePerObj.at(objId).begin())
					break;
				it1--;
			}
			auto it2 = next(it, 1);
			while (it2 != posePerObj.at(objId).end() && !it2->second->finalized && (it2->second->interpolated || it2->first % actualStepSize > 0)) {
				it2++;

			}
			if (it2 == posePerObj.at(objId).end()) {
				it2--;
				break;
			}
			double s = (it->first - it1->first) / (double) (it2->first - it1->first);
			VERBOSE_LABELING("time"<<it->first<<"s"<<s)
			it->second->bb = Pose::interpolate(*it1->second, *it2->second, s).bb;
		} else if (it->second->standing) {
			auto it2 = t.lower_bound(floor(it->second->time * pointCloudFrequenz) - actualStepSize);
			if (it2 == t.end()) {
				it->second->bb = t.begin()->second->bb;
			} else {
				it->second->bb = it2->second->bb;
			}

		}
	}
	auto &t2 = *id2traj.at(objId);
	for (int i = 1; i < t2.poseData.size(); i++) {
		if (cv::norm(t2.poseData[i - 1].bb.center - t2.poseData[i].bb.center) < 0.001 && fabs(t2.poseData[i - 1].bb.angle - t2.poseData[i].bb.angle) < 0.001) {
			t2.poseData[i].standing = true;
		}
	}
}
pcl::PointCloud<IKGB>* Labeling::get_cloud(int i, bool recolor) {
	VERBOSE_LABELING("get cloud"<<i);
	if (clouds.count(i) > 0)
		if (recolor && flagColorObj) {
			VERBOSE_LABELING("RECOLOR"<<i)
			for (auto &p2 : clouds.at(i).points) {
				p2.r = 0;
				p2.g = 0;
				p2.b = 0;
			}
			if (posePerTime.count(i) > 0) {
				for (auto p : posePerTime.at(i)) {
					cv::Vec3b pcColor = LUMPIPipeline::switch_class_color(id2traj[p.second->tId]->classId);
					for (auto &p2 : clouds.at(i).points) {
						if (p.second->bb.within<IKGB>(p2)) {
							p2.r = (std::uint8_t) pcColor[2];
							p2.g = (std::uint8_t) pcColor[1];
							p2.b = (std::uint8_t) pcColor[0];
						}
					}
				}
			}
			return &clouds.at(i);
		} else {
			return &clouds.at(i);
		}

	stringstream ss;
	ss << pointCloudPath << "/" << std::setw(6) << std::setfill('0') << i << ".ply";
	if (i == tmpCloud.first)
		return &tmpCloud.second;
	tmpCloud.first = i;
	tmpCloud.second = Parser::load_cloud_ikg_benchmark(ss.str());
	/* replace the loading method with the following code to load a mobile mapping cloud from the institute for Cartography and Geoinformatics:
	 tmpCloud.second= Parser::load_cloud_ikg_mobile_mapping("../mobile_mapping/003.ply");
	 cv::Point3d center(0,0,0);
	 for(auto&p:tmpCloud.second){
	 center.x+=p.x;
	 center.y+=p.y;
	 center.z+=p.z;
	 }
	 center/=(double)tmpCloud.second.size();
	 for(auto &p:tmpCloud.second){
	 p.x-=center.x;
	 p.y-=center.y;
	 p.z-=center.z;
	 }
	 */
	if (removeBackground && background.size() > 0) {

		std::pair<std::vector<int>, std::vector<int>> fg = LUMPIPipeline::remove_back_ground(tmpCloud.second, background, backgroundAngleNormalizer);
		pcl::PointCloud<IKGB> tmp2;
		VERBOSE_LABELING(fg.first.size()<<"bg:"<<fg.second.size());
		if (!fg.first.empty()) {
			for (auto id : fg.first)
				tmp2.points.push_back(tmpCloud.second.points[id]);
			tmpCloud.second = tmp2;
		}
	}
	if (flagColorObj) {
		if (posePerTime.count(i) > 0) {
			for (auto p : posePerTime.at(i)) {
				cv::Vec3b pcColor = LUMPIPipeline::switch_class_color(id2traj[p.second->tId]->classId);
				for (auto &p2 : tmpCloud.second.points) {
					if (p.second->bb.within<IKGB>(p2)) {
						p2.r = (std::uint8_t) pcColor[2];
						p2.g = (std::uint8_t) pcColor[1];
						p2.b = (std::uint8_t) pcColor[0];
					}
				}
			}
		}
	}
	return &tmpCloud.second;
}
std::vector<IKGB> Labeling::get_points_at_obj(int index, int objId) {
	std::vector<IKGB> points;
	VERBOSE_LABELING("get points")
	if (posePerTime.count(index) < 1 || posePerTime.at(index).count(objId) < 1)
		return points;
	auto po = posePerTime.at(index).at(objId);
	for (const auto &p : get_cloud(index)->points) {
		if (po->bb.within<IKGB>(p))
			points.push_back(p);
	}
	return points;
}
cv::Mat Labeling::generate_single_trajectory_view(int objId) {
	VERBOSE_LABELING("generate single trajectory view")
	cv::Mat img(singleTrajectoryHeight, singleTrajectoryWidth, CV_8UC3, cv::Scalar(0, 0, 0));
	VERBOSE_LABELING(img.size())
	if (posePerObj.count(objId) < 1 || posePerObj.at(objId).empty())
		return img.clone();
	if (posePerObj.count(objId) < 1 || posePerObj.at(objId).empty())
		return img.clone();
	BoundingBox sBB = posePerObj.at(objId).begin()->second->bb;
	if (posePerObj.at(objId).count(actualIndex) > 0)
		sBB = posePerObj.at(objId).at(actualIndex)->bb;
	singleTrajectoryStart = cv::Point2d(sBB.center.x, sBB.center.y);
	std::vector<cv::Point2d> imgPoints, headingPts;
	for (const auto &p : posePerObj.at(objId)) {
		if (p.first % actualStepSize > 0)
			continue;
		BoundingBox bb = p.second->bb;
		cv::Point2d ip1(bb.center.x - singleTrajectoryStart.x, bb.center.y - singleTrajectoryStart.y);
		ip1 /= singleTrajectoryResolution;
		ip1.y += singleTrajectoryHeight * 0.5;
		ip1.x += singleTrajectoryWidth * 0.5;
		imgPoints.push_back(ip1);
		headingPts.push_back(ip1 + cv::Point2d(bb.ca, bb.sa) / singleTrajectoryResolution /singleTrajectoryResolution);
		cv::circle(img, ip1, 2, cv::Scalar(0, 255, 0));
		cv::putText(img, to_string((int) floor(p.second->time * pointCloudFrequenz)), ip1 + cv::Point2d(0, -20), 1, 1, cv::Scalar(0, 255, 0));
	}
	for (int i = 0; i < imgPoints.size(); i++) {
		cv::arrowedLine(img, imgPoints[i], headingPts[i], cv::Scalar(125, 125, 125), 1);
	}
	return img.clone();
}
void Labeling::finalized(int objId) {
	if (id2traj.count(objId) < 1)
		return;
	auto t = id2traj.at(objId);
	for (auto &p : t->poseData) {
		p.finalized = true;
	}
}
void Labeling::definalized(int objId) {
	if (id2traj.count(objId) < 1)
		return;
	auto t = id2traj.at(objId);
	for (auto &p : t->poseData) {
		p.finalized = false;
	}
}

void Labeling::generate_model_view() {
	modelPcView->removeAllPointClouds();

	VERBOSE_LABELING("UPDATE MODEL VIEW"<<activObjId<<"count"<<models.count(activObjId))
	if (models.count(activObjId) < 1)
		return;
	VER(models.at(activObjId).size())
	pcl::PointCloud<pcl::PointXYZRGBA> tmp;
	for (const auto &p : models.at(activObjId)) {
		VER(p.x<<";"<<p.y<<","<<p.z<<":"<<p.r<<","<<p.b<<","<<p.g)
		pcl::PointXYZRGBA p2;
		p2.x = p.x;
		p2.y = p.y;
		p2.z = p.z;
		p2.r = p.r;
		p2.b = p.b;
		p2.g = p.g;
		tmp.points.push_back(p2);
	}
	modelPcView->addPointCloud(tmp.makeShared(), "model");
	modelPcView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize * 2, "model");
}

void Labeling::update_pc_bb(int index) {
	int cIndex = -1;
	VERBOSE_LABELING("update bb"<<index)
	if (index > -1)
		cIndex = (index - actualIndex) / actualStepSize + slidingWindow;
	else
		index = actualIndex;
	VERBOSE_LABELING("update actual"<<index)
	if (posePerTime.count(index) > 0) {
		vector<BoundingBox> bbs;
		for (auto p : posePerTime.at(index)) {
			p.second->bb.color = cv::Vec3b(0, 255, 0);
			if (p.second->tId == activObjId)
				p.second->bb.color = cv::Vec3b(0, 0, 255);
			else if (p.second->finalized)
				p.second->bb.color = cv::Vec3b(255, 0, 0);
			bbs.push_back(p.second->bb);
			bbs.back().id = p.second->tId;
		}
		if (pcViews.count(cIndex) > 0) {
			LUMPIPipeline::visualOutputBoundingBox(bbs, pcViews.at(cIndex), 1, "clear", false);
			LUMPIPipeline::visualOutputBoundingBox(bbs, pcViews.at(cIndex), 1, "bb", false);

		} else {
			LUMPIPipeline::visualOutputBoundingBox(bbs, globPcView, 1, "clear", false);
			LUMPIPipeline::visualOutputBoundingBox(bbs, globPcView, 1, "bb", false);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGBA> tmpC;
	point_cloud_to_XYZRGBA(*get_cloud(index), tmpC);
	if (globPcView->contains("cloud"))
		globPcView->removePointCloud("cloud");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(tmpC.makeShared());
	globPcView->addPointCloud(tmpC.makeShared(), rgb, "cloud");		//
	globPcView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");

}

cv::Mat Labeling::generate_camera_view(int index, int objId, int camId) {
	VERBOSE_LABELING("generate view" << index << ":" << objId << ":" << camId);
	vector<IKGB> points = get_points_at_obj(index, objId);
	double time = index /pointCloudFrequenz;
	cv::Mat img(200, 200, CV_8UC3, cv::Scalar(255, 255, 255));
	if (posePerObj.count(objId) > 0 && posePerObj.at(objId).count(index) > 0) {
		double pTime = 0;
		const auto &p = posePerObj.at(objId).at(index);
		int cIndex = (index - actualIndex) / actualStepSize + slidingWindow;
		for (const auto &p : points) {
			pTime += p.adjustedtime;
		}
		pTime /= points.size();
		pTime *= msec2sec;
		if (points.size() > 1)
			time = pTime;
	}
	if (cams.count(camId) < 1) {
		camView[index] = img.clone();
		return img.clone();
	}
	int frame = time * cams.at(camId).fps;
	auto cap = caps.at(camId);
	if (frame < 0 || frame > cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)
		return img.clone();
	cap.set(cv::CAP_PROP_POS_FRAMES, frame);
	if (!cap.read(img)) {
		cv::Mat tmp(200, 200, CV_8UC3, cv::Scalar(255, 255, 255));
		return tmp.clone();
	}
	if (posePerObj.count(objId) < 1)
		return img.clone();
	map<int, Pose*> track = posePerObj.at(objId);
	auto it = track.lower_bound(index);
	if (it == track.end()) {
		camView[index] = img.clone();
		return img.clone();
	}
	BoundingBox tmpBB;
	if (it->first == index) {
		tmpBB = it->second->bb.transform(cams[camId].extrinsic);
	} else if (it != track.begin()) {
		auto it2 = prev(it);
		double s = (time - it2->second->time) / (it->second->time - it2->second->time);
		auto newP = Pose::interpolate(*it2->second, *it->second, s);
		tmpBB = newP.bb.transform(cams[camId].extrinsic);
	} else {
		camView[index] = img.clone();
		return img.clone();

	}
	tmpBB.plot(img, cv::Scalar(0, 255, 0), cams[camId].intrinsic, cams.at(camId).distortion, 4, cams[camId].frustum);
	cv::Mat P = (cv::Mat_<double>(3, 1) << tmpBB.center.x, tmpBB.center.y, tmpBB.center.z);
	P = cams.at(camId).intrinsic * P;
	P /= P.at<double>(2, 0);
	cv::Point2d tl(P.at<double>(0, 0) - 100 / zoomCam, P.at<double>(1, 0) - 100 / zoomCam);
	cv::Point2d br(P.at<double>(0, 0) + 100 / zoomCam, P.at<double>(1, 0) + 100 / zoomCam);
	cv::Rect2d tmpR(tl, br);
	vector<cv::Point3f> objPoints;
	for (auto p : points)
		objPoints.push_back(cv::Point3f(p.x, p.y, p.z));		//
	if (!objPoints.empty()) {
		vector<cv::Point2f> imgPoints;
		cv::projectPoints(objPoints, cams.at(camId).rvec, cams.at(camId).tvec, cams.at(camId).intrinsic, cams.at(camId).distortion, imgPoints);
		for (auto &p : imgPoints) {
			cv::circle(img, p, 1, cv::Scalar(0, 255, 0), -1);
		}
	}
	if (tmpR.width > img.cols || tmpR.height > img.rows) {
		camView[index] = img.clone();
		return img.clone();
	}
	if (tl.x < 0) {
		tl.x = 0;
		br.x = 200 / zoomCam;
	} else if (br.x > img.cols - 1) {
		tl.x = img.cols - 200 / zoomCam - 1;
		br.x = img.cols - 1;
	}
	if (tl.y < 0) {
		tl.y = 0;
		br.y = 200 / zoomCam;
	} else if (br.y > img.rows - 1) {
		tl.y = img.rows - 200 / zoomCam - 1;
		br.y = img.rows - 1;
	}
	camView[index] = img(cv::Rect(tl, br)).clone();
	return img(cv::Rect(tl, br)).clone();
}
cv::Mat Labeling::generate_top_view(int index, int objId) {
	VERBOSE_LABELING("gernerate top view")
	cv::Mat tv(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));

	if (posePerTime.count(index) < 1 || posePerTime.at(index).count(objId) < 1) {
		return tv.clone();
	}
	auto &po = posePerTime.at(index).at(objId);
	if (po->finalized)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 0, 0));
	else if (po->standing)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(0, 0, 0));
	else if (po->interpolated)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 125, 125));
	BoundingBox &bb = po->bb;
	cv::Point2d off(columViewSize * 0.5, rowViewSize * 0.5);
	BoundingBox filter(bb);
	filter.length = columViewSize * resolution / zoomTop;
	filter.width = rowViewSize * resolution / zoomTop;
	filter.height += 2;
	filter.update();
	double ch = cos(-bb.angle);
	double sh = sin(-bb.angle);
	vector<IKGB> within = get_points_at_obj(index, objId);
	for (const auto &p : get_cloud(index)->points) {
		cv::Point3d p2(p.x, p.y, p.z);
		if (!filter.within(p2))
			continue;
		p2 = p2 - bb.center;
		cv::Point2d p3(p2.x * ch - sh * p2.y, p2.x * sh + p2.y * ch);
		p3 /= resolution / zoomTop;
		p3 += off;
		cv::circle(tv, p3, pointSize, cv::Scalar(0, 255, 0), -1);
	}
	if (!within.empty()) {
		cv::Mat times(within.size(), 1, CV_64F, cv::Scalar(0));
		for (int i = 0; i < within.size(); i++) {
			times.at<double>(i, 0) = within[i].adjustedtime;
		}
		auto heat = ikg::Plotter::get_heat_map(times);
		for (int i = 0; i < within.size(); i++) {
			cv::Point3d p2(within[i].x, within[i].y, within[i].z);
			p2 = p2 - bb.center;
			cv::Point2d p3(p2.x * ch - sh * p2.y, p2.x * sh + p2.y * ch);
			p3 /= resolution / zoomTop;
			p3 += off;
			cv::circle(tv, p3, pointSize, heat.at<cv::Vec3b>(i, 0), -1);
		}
	}

	if (posePerTime.count(index) > 0) {
		VERBOSE_LABELING("vis poses")
		for (auto p : posePerTime.at(index)) {
			Pose tmp = *p.second;
			tmp.bb.center -= bb.center;
			cv::Point3d c = tmp.bb.center;
			tmp.bb.center.x = ch * c.x - sh * c.y;
			tmp.bb.center.y = sh * c.x + ch * c.y;
			tmp.bb.update_orientation(0, 0, fmod(p.second->bb.angle - bb.angle + Pi3, Pi2) - M_PI);
			tmp.bb.update();
			auto r = tmp.bb.get_birds_eye_rectangle(false);
			cv::Point2f vertices[4];
			r.points(vertices);
			for (int i = 0; i < 4; i++)
				cv::line(tv, vertices[i] / resolution * zoomTop + cv::Point2f(off.x, off.y), vertices[(i + 1) % 4] / resolution * zoomTop + cv::Point2f(off.x, off.y), cv::Scalar(125, 125, 125), 1);
		}
	}
	cv::Point2d tl(-bb.l_2 / resolution * zoomTop + off.x, -bb.w_2 / resolution * zoomTop + off.y);
	cv::Point2d br(bb.l_2 / resolution * zoomTop + off.x, bb.w_2 / resolution * zoomTop + off.y);
	cv::rectangle(tv, cv::Rect2d(tl, br), cv::Scalar(0, 0, 255), 1);
	topView[index] = tv.clone();
	return tv.clone();
}
cv::Mat Labeling::generate_back_view(int index, int objId) {
	VERBOSE_LABELING("generate back view")
	cv::Mat tv(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));
	if (posePerTime.count(index) < 1 || posePerTime.at(index).count(objId) < 1) {
		backView[index] = tv.clone();
		return tv.clone();
	}
	auto po = posePerTime.at(index).at(objId);
	if (po->finalized)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 0, 0));
	else if (po->standing)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(0, 0, 0));
	else if (po->interpolated)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 125, 125));
	cv::Point2d off(columViewSize / 2, rowViewSize / 2);
	BoundingBox &bb = po->bb;
	BoundingBox filter(bb);
	filter.length += 2;
	filter.width = columViewSize * resolution / zoomBack;
	filter.height = rowViewSize * resolution / zoomBack;
	filter.update();
	double ch = cos(-bb.angle);
	double sh = sin(-bb.angle);
	vector<IKGB> within = get_points_at_obj(index, activObjId);
	for (const auto &p : get_cloud(index)->points) {
		cv::Point3d p2(p.x, p.y, p.z);
		if (!filter.within(p2))
			continue;
		p2 = p2 - bb.center;
		cv::Point2d p3(p2.x * sh + p2.y * ch, -p2.z);
		p3 /= resolution / zoomBack;
		p3 += off;
		cv::circle(tv, p3, pointSize, cv::Scalar(0, 255, 0), -1);
	}
	if (!within.empty()) {
		cv::Mat times(within.size(), 1, CV_64F, cv::Scalar(0));
		for (int i = 0; i < within.size(); i++) {
			times.at<double>(i, 0) = within[i].adjustedtime;
		}
		auto heat = ikg::Plotter::get_heat_map(times);
		for (int i = 0; i < within.size(); i++) {
			cv::Point3d p2(within[i].x, within[i].y, within[i].z);
			p2 = p2 - bb.center;
			cv::Point2d p3(p2.x * sh + p2.y * ch, -p2.z);
			p3 /= resolution / zoomBack;
			p3 += off;
			cv::circle(tv, p3, pointSize, heat.at<cv::Vec3b>(i, 0), -1);
		}
	}
	VERBOSE_LABELING("rectangle")
	cv::Point2d tl(-bb.w_2 / resolution * zoomBack + off.x, -bb.h_2 / resolution * zoomBack + off.y);
	cv::Point2d br(bb.w_2 / resolution * zoomBack + off.x, bb.h_2 / resolution * zoomBack + off.y);
	cv::rectangle(tv, cv::Rect2d(tl, br), cv::Scalar(0, 0, 255), 1);
	backView[index] = tv.clone();
	return tv.clone();
}

void Labeling::set_back_ground_color_local_view(Pose *po, cv::Mat &tv) {
	if (po->finalized)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 0, 0));
	else if (po->standing)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(0, 0, 0));
	else if (po->interpolated)
		tv = cv::Mat(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(125, 125, 125));
}

cv::Mat Labeling::generate_side_view(int index, int objId) {
	VERBOSE_LABELING("gernerate side view")
	cv::Mat tv(rowViewSize, columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));
	if (posePerTime.count(index) < 1 || posePerTime.at(index).count(objId) < 1) {
		sideView[index] = tv.clone();
		return tv.clone();
	}
	Pose *po = posePerTime.at(index).at(objId);
	set_back_ground_color_local_view(po, tv);
	cv::Point2d off(columViewSize * 0.5, rowViewSize * 0.5);
	BoundingBox &bb = po->bb;
	BoundingBox filter(bb);
	filter.length = columViewSize * resolution / zoomSide;
	filter.width += 2;
	filter.height = rowViewSize * resolution / zoomSide;
	filter.update();
	double ch = cos(-bb.angle);
	double sh = sin(-bb.angle);
	vector<IKGB> within = get_points_at_obj(index, objId);
	for (const auto &p : get_cloud(index)->points) {
		if (!filter.within(p))
			continue;
		cv::Point3d p2(p.x, p.y, p.z);
		p2 = p2 - bb.center;
		cv::Point2d p3(p2.x * ch - sh * p2.y, -p2.z);
		p3 /= resolution / zoomSide;
		p3 += off;
		cv::circle(tv, p3, pointSize, cv::Scalar(0, 255, 0), -1);
	}
	if (!within.empty()) {
		cv::Mat heat;
		creat_time_heat_map(within, heat);
		for (int i = 0; i < within.size(); i++) {
			cv::Point3d p2(within[i].x, within[i].y, within[i].z);
			p2 = p2 - bb.center;
			cv::Point2d p3(p2.x * ch - sh * p2.y, -p2.z);
			p3 /= resolution / zoomSide;
			p3 += off;
			cv::circle(tv, p3, pointSize, heat.at<cv::Vec3b>(i, 0), -1);
		}
	}
	cv::Point2d tl(-bb.l_2 / resolution * zoomSide + off.x, -bb.height * 0.5 / resolution * zoomSide + off.y);
	cv::Point2d br(bb.l_2 / resolution * zoomSide + off.x, bb.height * 0.5 / resolution * zoomSide + off.y);
	cv::rectangle(tv, cv::Rect2d(tl, br), cv::Scalar(0, 0, 255), 1);
	sideView[index] = tv.clone();
	return tv.clone();
}
bool Labeling::merge_track(int id1, int id2) {
	VERBOSE_LABELING("merge tracks")
	auto &t1 = *id2traj.at(id1);
	auto &t2 = *id2traj.at(id2);
	for (const auto &p : t1.poseData) {
		if (p.finalized)
			return false;
	}
	for (const auto &p : t2.poseData) {
		if (p.finalized)
			return false;
	}
	auto size = t1.poseData.back().bb;
	for (auto &p : t2.poseData) {
		p.bb.width = size.width;
		p.bb.length = size.length;
		p.bb.height = size.height;
		p.bb.update();
	}
	t1.merge_time(t2, 1);
	erase_obj(id2);
	update_obj_ptr();
	return true;
}
bool Labeling::split_track(int id1, double w, SplitOption option) {
	VERBOSE_LABELING("split Track"<<id1<<":"<<w)
	if (id2traj.count(id1) < 1)
		return false;
	auto t = id2traj.at(id1);
	if (option == SplitOption::SPLITOBJECT) {
		for (const auto &p : t->poseData) {
			if (p.finalized)
				return false;
		}
	}
	Trajectory t2;
	t2.id = id2traj.rbegin()->first + 1;
	for (auto &p : t->poseData) {
		cv::Point3d v(-p.bb.sa, p.bb.ca, 0);
		switch (option) {
		case SplitOption::LEFT:
			v = cv::Point3d(-p.bb.sa, p.bb.ca, 0);
			break;
		case SplitOption::SPLITOBJECT:
			v = cv::Point3d(-p.bb.sa, p.bb.ca, 0);
			break;
		case SplitOption::RIGHT:
			v = cv::Point3d(p.bb.sa, -p.bb.ca, 0);
			break;
		case SplitOption::FRONT:
			v = cv::Point3d(p.bb.ca, p.bb.sa, 0);
			break;
		case SplitOption::BEHIND:
			v = cv::Point3d(-p.bb.ca, -p.bb.sa, 0);
			break;
		default:
			break;
		}
		Pose p2 = p;
		p2.tId = t2.id;
		p2.finalized = false;
		if (option == SplitOption::SPLITOBJECT) {
			p2.bb.center -= v * p.bb.width * w;
			p2.bb.width *= w;
			p.bb.width *= (1 - w);
			p.bb.center += v * p.bb.width * (1 - w);
		} else {
			p2.bb.center += v * w;
		}

		p2.bb.id = t2.id;
		if (option == SplitOption::SPLITOBJECT) {
			p.bb.update();
		}
		p2.bb.update();
		t2.poseData.push_back(p2);
	}
	pcTrajectories.push_back(t2);
	update_obj_ptr();
	return true;
}
bool Labeling::split_track_at_time(double time) {
	VERBOSE_LABELING("split Track at time"<<time<<":id"<<activObjId)
	if (id2traj.count(activObjId) < 1)
		return false;
	auto &t = id2traj.at(activObjId);
	for (const auto &p : t->poseData) {
		if (p.finalized)
			return false;
	}
	if (time < t->poseData.front().time || time > t->poseData.back().time)
		return false;
	Trajectory t2;
	t2.id = id2traj.rbegin()->first + 1;
	for (auto it = t->poseData.begin(); it != t->poseData.end();) {
		if (it->time > time) {
			t2.poseData.push_back(*it);
			t2.poseData.back().tId = t2.id;
			t2.poseData.back().bb.id = t2.id;
			it = t->poseData.erase(it);
		} else {
			it++;
		}

	}
	pcTrajectories.push_back(t2);
	update_obj_ptr();
	return true;
}
cv::Mat Labeling::generate_global_point_cloud_view() {
	VERBOSE_LABELING("UPDATE GLOBAL")
	cv::Point2d off = globalTrajectoryCenter;
	globPcView->removeAllPointClouds();
	globPcView->removeAllShapes();
	cv::Mat img(globalTrajectoryHeight, globalTrajectoryWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	VERBOSE_LABELING("UPDATE GLOBAL"<<img.size())
	pcl::PointCloud<pcl::PointXYZRGBA> tmpC;
	set_point_cloud(actualIndex, -1);
	vector<BoundingBox> bbs;
	unordered_map<int, cv::Vec3b> colors;
	VERBOSE_LABELING("GET bounding boxes")
	if (posePerTime.count(actualIndex) > 0) {
		for (auto p : posePerTime.at(actualIndex)) {
			colors[p.second->tId] = cv::Vec3b(0, 255, 0);
			if (p.second->tId == activObjId)
				colors[p.second->tId] = cv::Vec3b(0, 0, 255);
			bbs.push_back(p.second->bb);
			bbs.back().id = p.second->tId;

		}
	}
	map<int, vector<cv::Point2d>> poses2D;
	auto it = posePerTime.lower_bound(actualIndex - slidingWindow * actualStepSize);
	while (it != posePerTime.end() && it->first < actualIndex + (slidingWindow + 1) * actualStepSize) {
		for (const auto p : it->second)
			poses2D[p.first].push_back(cv::Point2d(p.second->bb.center.x, p.second->bb.center.y));
		it++;
	}
	for (const auto &o : poses2D) {
		cv::Vec3b c(125, 125, 125);
		if (o.first == activObjId)
			continue;

		for (int i = 1; i < o.second.size(); i++) {
			cv::line(img, o.second[i - 1] / globalTrajectoryResolution + off, o.second[i] / globalTrajectoryResolution + off, c, 1);
			if (i == o.second.size() - 1)
				cv::putText(img, to_string(o.first), o.second[i] / globalTrajectoryResolution + off, 1, 1, cv::Scalar(0, 255, 0), 1);
		}
	}
	if (poses2D.count(activObjId) > 0) {
		auto &o = poses2D.at(activObjId);
		for (int i = 1; i < o.size(); i++) {
			cv::line(img, o[i - 1] / globalTrajectoryResolution + off, o[i] / globalTrajectoryResolution + off, cv::Scalar(0, 0, 255), 2);

			if (i == o.size() - 1)
				cv::putText(img, to_string(activObjId), o[i] / globalTrajectoryResolution + off, 1, 1, cv::Scalar(0, 0, 255), 1);
		}
	}
	VERBOSE_LABELING("show glob boxes")
	update_pc_bb(-1);
	return img.clone();

}

void Labeling::point_cloud_to_XYZRGBA(const pcl::PointCloud<IKGB> &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &tmpC) {
	for (auto &p : cloud.points) {
		pcl::PointXYZRGBA p2;
		p2.x = p.x;
		p2.y = p.y;
		p2.z = p.z;
		p2.r = p.r;
		p2.g = p.g;
		p2.b = p.b;
		tmpC.points.push_back(p2);
	}
}

void Labeling::set_point_cloud(int index, int view) {
	VERBOSE_LABELING("SET cloud"<<index)
	pcl::PointCloud<pcl::PointXYZRGBA> tmpC;
	point_cloud_to_XYZRGBA(*get_cloud(index), tmpC);
	pcl::visualization::PCLVisualizer::Ptr v;
	if (view < 0) {
		v = globPcView;
	} else if (pcViews.count(view) > 0) {
		v = pcViews.at(view);
	} else
		return;
	VERBOSE_LABELING("set pointcloud"<<view)
	if (v->contains("cloud"))
		v->removePointCloud("cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(tmpC.makeShared());
	v->addPointCloud(tmpC.makeShared(), rgb, "cloud");		//
	v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");
}
void Labeling::load_point_cloud_sliding_window() {
	VERBOSE_LABELING("load clouds")
	int s = actualIndex - slidingWindow * actualStepSize;
	int e = actualIndex + slidingWindow * actualStepSize;
	vector<int> del;
	for (auto it = clouds.begin(); it != clouds.end(); it++) {
		if (it->first % actualStepSize != 0 || it->first < s || it->first > e)
			del.push_back(it->first);
	}
	for (int id : del) {
		selectedCam.erase(id);
		clouds.erase(id);
		topView.erase(id);
		backView.erase(id);
		sideView.erase(id);
		camView.erase(id);
		selectedCam.erase(id);
	}
	if (clouds.size() > 10) {
		clouds.clear();
		topView.clear();
		backView.clear();
		sideView.clear();
		camView.clear();
		selectedCam.clear();
	}
	inter.clear();
	for (int j = 0; j < slidingWindow * 2 + 1; j++) {
		int index = actualIndex - slidingWindow * actualStepSize + j * actualStepSize;
		if (clouds.count(index) < 1) {
			clouds[index] = *get_cloud(index);

			if (selectedCam.count(index - actualStepSize) > 0) {
				selectedCam[index] = selectedCam.at(index - actualStepSize);
			} else if (selectedCam.count(index + actualStepSize) > 0) {
				selectedCam[index] = selectedCam.at(index + actualStepSize);
			} else {
				if (!cams.empty()) {
					selectedCam[index] = cams.begin()->first;
				}
			}
			generate_top_view(index, activObjId);
			generate_side_view(index, activObjId);
			generate_back_view(index, activObjId);
		}
		set_point_cloud(index, j);
	}
}

void Labeling::load_camera_parameters_and_videos() {
	VERBOSE_LABELING("load camera parameters and videos")
	cams.clear();
	caps.clear();
	std::pair<map<int, Sensor>, std::map<int, map<int, int>>> meta;
	try {
		meta = Parser::read_meta_lumpi(metaPath);
	} catch (...) {
		cerr << "cant find meta file at " << metaPath << endl;
	}
	for (auto p : meta.first) {
		auto &s = p.second;
		if (s.expId != measurementId || s.type != "camera")
			continue;
		cv::Mat ext = cv::Mat::eye(4, 4, CV_64F);
		cv::Mat R;
		cv::Rodrigues(s.rvec, R);
		R.copyTo(ext(cv::Rect(0, 0, 3, 3)));
		cv::Mat t = s.tvec.clone();
		t.copyTo(ext(cv::Rect(3, 0, 1, 3)));
		ext.copyTo(s.extrinsic);
		cams[s.deviceId] = s;
		caps[s.deviceId] = cv::VideoCapture(cameraPath + "/" + to_string(s.deviceId) + "/video.mp4");
		cv::Mat tmp;
		VERBOSE_LABELING("load camera"<<s.deviceId)
		if (caps[s.deviceId].read(tmp)) {
			VERBOSE_LABELING("FRUSTUM"<<"tmp"<<tmp.size()<<","<<cams[s.deviceId].intrinsic)
			cams[s.deviceId].frustum = frustum::Frustum(cams[s.deviceId].intrinsic, tmp.rows, tmp.cols);
		}
		VERBOSE_LABELING("load cam"<<s.deviceId)
	}

}
void Labeling::load_trajectories(std::string path) {
	VERBOSE_LABELING("load trajectories")
	pcTrajectories.clear();
	pcTrajectories = Parser::read_trajectories(path, false, true);
	for (auto &t : pcTrajectories) {
		for (auto &p : t.poseData) {
			p.time = floor(p.time * pointCloudFrequenz) / pointCloudFrequenz;
		}
		sort(t.poseData.begin(), t.poseData.end(), [](const Pose &p1, const Pose &p2) {
			return p1.time < p2.time;
		});
	}
	string tmpPath = dataPath + "/Measurement" + to_string(measurementId) + "/model/";
	models.clear();
	posePerObj.clear();
	posePerTime.clear();
	id2traj.clear();
	for (auto &t : pcTrajectories) {
		string modelPath = tmpPath + "/" + to_string(t.id) + ".csv";
		for (auto &p : t.poseData) {
			if (t.classId < 0 || t.classId > classNames.size() - 1)
				t.classId = classNames.size() - 1;
			if (p.score > 3)
				p.finalized = true;
			p.tId = t.id;
			p.classId = t.classId;
			posePerTime[(int) (p.time * pointCloudFrequenz)][t.id] = &p;
			posePerObj[t.id][(int) (p.time * pointCloudFrequenz)] = &p;
		}
		for (int i = 1; i < t.poseData.size(); i++) {
			if (cv::norm(t.poseData[i - 1].bb.center - t.poseData[i].bb.center) < 0.001 && fabs(t.poseData[i - 1].bb.angle - t.poseData[i].bb.angle) < 0.001) {
				t.poseData[i].standing = true;
			}
		}
		id2traj[t.id] = &t;
	}
	VERBOSE_LABELING("loaded"<<pcTrajectories.size()<<" trajectories")

}
void Labeling::load_back_ground() {
	VERBOSE_LABELING("load back ground")
	LUMPIPipeline::read_background(background, backgroundAngleNormalizer, dataPath + "/" + measurementSubString + to_string(measurementId) + "/background/");
}

Labeling::Labeling() {
	VER("LABELING")
	pointCloudFrequenz = 10;
	measurementSubString = "Measurement";
	addingFlag = false;
	removeBackground = false;
	flagColorObj = false;
	activObjId = 0;
	measurementId = 0;
	zoomCam = 1;
	zoomTop = 1;
	zoomBack = 1;
	zoomSide = 1;
	pointSize = 1;
	columViewSize = 300;
	flagShowModel = true;
	rowViewSize = 180;
	globalTrajectoryWidth = 1000;
	globalTrajectoryHeight = 500;
	globalTrajectoryCenter = cv::Point2d(globalTrajectoryWidth / 2, globalTrajectoryHeight / 2);
	classNames = { "person", "car", "bicycle", "motorcycle", "bus", "truck", "van", "unknown" };
	singleTrajectoryHeight = 1080;
	singleTrajectoryWidth = 1920;
	globalCameraId = -1;
	globalTrajectoryResolution = 0.2;
	singleTrajectoryResolution = 0.1;
	resolution = 1. / 50;
	stepSizes = { 1, 5, 10, 25, 50, 100, 200, 300, 500 };
	actualStepSize = 1;
	actualIndex = actualStepSize * 3;
	dataPath = "";
	cameraPath = dataPath + "/" + measurementSubString + to_string(measurementId) + "/cam/";
	metaPath = dataPath + "/newMeta.json";
	pointCloudPath = dataPath + "/" + measurementSubString + to_string(measurementId) + "/lidar/";
	trajectoryPath = "";
	slidingWindow = 2;
	for (int i = 0; i < slidingWindow * 2 + 1; i++) {
		pcViews[i].reset(new pcl::visualization::PCLVisualizer("Viewer3d" + to_string(i), false));
		pcViews[i]->setBackgroundColor(255, 255, 255);
	}
	globPcView.reset(new pcl::visualization::PCLVisualizer("global", false));
	globPcView->setBackgroundColor(255, 255, 255);
	modelPcView.reset(new pcl::visualization::PCLVisualizer("model", false));
	modelPcView->setBackgroundColor(255, 255, 255);
	VER("CREATE LABELING")
}
void Labeling::interpolate_single_between_key_frames(int objId, double time) {
	VERBOSE_LABELING("inter polate single"<<objId<<"at"<<time)
	int frame = floor(time * pointCloudFrequenz / actualStepSize) * actualStepSize;
	int start = frame - actualStepSize;
	int end = frame + actualStepSize;
	auto sP = *posePerObj.at(activObjId).lower_bound(start)->second;
	auto eP = *posePerObj.at(activObjId).lower_bound(end)->second;
	VERBOSE_LABELING("start"<<start<<"end"<<end)
	if (id2traj.count(activObjId) < 1)
		return;
	auto &t = id2traj.at(activObjId);
	bool flagAddPos = false;
	for (int i = start + 1; i < end; i++) {
		if (posePerObj.at(t->id).count(i) < 1) {
			Pose p = sP;
			p.interpolated = true;
			p.finalized = false;
			p.time = i /pointCloudFrequenz;
			VERBOSE_LABELING("ADD"<<p.time)
			t->insert_pose(p);
			flagAddPos = true;
		}
	}
	if (flagAddPos)
		update_obj_ptr();
	auto &t2 = id2traj.at(activObjId);
	VERBOSE_LABELING("pose data"<<t->poseData.size())
	for (auto &p : t2->poseData) {
		VERBOSE_LABELING("check"<<p.time<<"sp"<<sP.time<<"ep"<<eP.time)
		if (!p.finalized && p.time > sP.time && p.time < eP.time) {
			VERBOSE_LABELING("interpolate"<<p.time);
			p.bb = Pose::interpolate(sP, eP, (p.time - sP.time) / (eP.time - sP.time)).bb;
			p.bb.length = sP.bb.length;
			p.bb.width = sP.bb.width;
			p.bb.height = sP.bb.height;
			p.interpolated = true;
			p.standing = false;
		}
	}
	update_obj_ptr();
}
void Labeling::set_standing_pose_by_key_frames(int objId, double time) {
	VERBOSE_LABELING("set standing")
	int frame = floor(time * pointCloudFrequenz / actualStepSize) * actualStepSize;
	int start = frame - actualStepSize;
	auto startPose = posePerObj.at(activObjId).lower_bound(start)->second;
	auto standingPose = posePerObj.at(activObjId).lower_bound(frame)->second;
	if (id2traj.count(activObjId) < 1)
		return;
	auto t = id2traj.at(activObjId);
	for (auto &p : t->poseData) {
		if (p.finalized)
			continue;
		if (p.time > standingPose->time)
			break;
		if (p.time > startPose->time) {
			p.bb = startPose->bb;
			p.standing = true;
			p.interpolated = false;
		}
	}
}
void Labeling::interpolate_between_key_frames() {
	VERBOSE_LABELING("interpolate between key frames")
	auto &t = *id2traj.at(activObjId);
	auto &m = posePerObj.at(t.id);
	for (auto &p : t.poseData) {
		if (p.finalized)
			continue;
		if (((int) (p.time * pointCloudFrequenz)) % actualStepSize > 0) {
			int f1 = floor(p.time * pointCloudFrequenz / actualStepSize) * actualStepSize;
			int f2 = f1 + actualStepSize;
			auto p1 = m.find(f1);
			auto p2 = m.find(f2);
			if (p1 == m.end() || p2 == m.end())
				continue;
			double s = (p.time - p1->second->time) / (p2->second->time - p1->second->time);
			VERBOSE_LABELING("s"<<s<<"time"<<p.time<<":"<<p1->second->time<<";"<<p2->second->time)
			VERBOSE_LABELING("heading:"<<p.bb.angle<<"->"<<p1->second->bb.angle<<";"<<p2->second->bb.angle);
			p.bb = Pose::interpolate(*p1->second, *p2->second, s).bb;
			VERBOSE_LABELING("="<<p.bb.angle)
			p.bb.id = t.id;
			p.interpolated = true;
		}
	}
}
void Labeling::update_pc_index_list() {
	VERBOSE_LABELING("load pc indizies")
	pointCloudIndices.clear();
	auto tmpNames = Parser::files_at_directory(pointCloudPath);
	sort(tmpNames.begin(), tmpNames.end());
	for (auto &s : tmpNames) {
		auto tmp = ikg::split(s, ".");
		if (tmp.size() < 2)
			continue;
		if (tmp.back() != "ply")
			continue;
		int index = stoi(tmp[0]);
		if (index % actualStepSize == 0)
			pointCloudIndices.insert(index);
	}
}
Labeling::~Labeling() {
}

} /* namespace ikg */
