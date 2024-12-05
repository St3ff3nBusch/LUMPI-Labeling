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

#include <LUMPIPipeline.h>
#include "filesystem"
#include "model/Frustum.h"
using namespace std;
#include "opencv2/opencv.hpp"
#include <Parser.h>
#include "model/Plotter.h"
#include <pcl/visualization/cloud_viewer.h>
namespace ikg {


void LUMPIPipeline::correct_pose(Pose &p, const pcl::PointCloud<IKGB> &pc) {

	vector<cv::Point3d> candidates;

	double minX = p.bb.center.x - fabs(p.bb.u.x * 2) - fabs(p.bb.v.x) * 1;
	double maxX = p.bb.center.x + fabs(p.bb.u.x * 2) + fabs(p.bb.v.x) * 1;
	double minY = p.bb.center.y - fabs(p.bb.u.y * 2) - fabs(p.bb.v.y) * 1;
	double maxY = p.bb.center.y + fabs(p.bb.u.y) * 2 + fabs(p.bb.v.y) * 1;
	double minO = p.bb.angle - 0.17;
	double maxO = p.bb.angle + 0.17;
	double offx = p.bb.l_2 * fabs(p.bb.u.x) + p.bb.w_2 * fabs(p.bb.v.x);
	double offy = p.bb.l_2 * fabs(p.bb.u.y) + p.bb.w_2 * fabs(p.bb.v.y);
	for (auto po : pc.points) {
		if (po.x > maxX + offx)
			continue;
		if (po.x < minX - offx)
			continue;
		if (po.y > maxY + offy)
			continue;
		if (po.y < minY - offy)
			continue;
		candidates.push_back(cv::Point3d(po.x, po.y, po.z));
	}

	double minDist = 0;
	double maxInlier = 0;
	double newX = p.bb.center.x, newY = p.bb.center.y, newA = p.bb.angle;
	for (int j = 0; j < candidates.size(); j++) {
		auto &po = candidates[j];
		if (p.bb.within<cv::Point3d>(po)) {
			maxInlier++;
			minDist += p.bb.dist_to_surface(po);
		}
	}
	for (double x = minX; x < maxX; x += 0.02) {
		for (double y = minY; y < maxY; y += 0.02) {
			for (double a = minO; a < maxO; a += 0.01) {
				int inlier = 0;
				BoundingBox b(x, y, p.bb.center.z, p.bb.length, p.bb.width, p.bb.height, a);
				double dist = 0;
				vector<int> indexInlier;
				for (int j = 0; j < candidates.size(); j++) {
					auto &po = candidates[j];
					if (b.within<cv::Point3d>(po)) {
						inlier++;
					}
				}
				if (inlier < maxInlier)
					continue;
				else if (inlier >= maxInlier) {
					for (int id : indexInlier)
						dist += b.dist_to_surface(candidates[id]);
					if (inlier > maxInlier || dist < minDist) {
						maxInlier = inlier;
						minDist = dist;
						newX = x;
						newY = y;
						newA = a;
					}
				}
			}
		}

	}
	VER("minimum" <<minDist)
	p.bb.center.x = newX;
	p.bb.center.y = newY;
	p.bb.angle = newA;
	p.bb.update();
}
LUMPIPipeline::LUMPIPipeline() {
}
LUMPIPipeline::~LUMPIPipeline() {
}


pcl::PointCloud<IKGB> LUMPIPipeline::filter_pc_by_time(const pcl::PointCloud<IKGB> &pc, double minTime, double maxTime) {
	pcl::PointCloud<IKGB> filtered;
	maxTime *= sec2msec;
	minTime *= sec2msec;
	for (const auto &p : pc) {
		if (p.adjustedtime < minTime || p.adjustedtime > maxTime) {
			continue;
		}
		filtered.push_back(p);
	}
	return filtered;
}
void LUMPIPipeline::init_back_ground(std::string path) {
	read_background(thresholds, degNorm, path);
}
bool LUMPIPipeline::generate_video(const std::vector<ikg::Trajectory> &trajectories, std::string videoPath, std::string outPath, ikg::Sensor s, int skip, int start, int end) {

	bool ok;
	map<int, cv::Scalar> colors;
	cv::VideoWriter videoWriter;
	cv::VideoCapture cap(videoPath);
	cv::Mat img;
	auto ext = s.extrinsic.inv();
	bool init = false;
	int lastFrame = cap.get(cv::CAP_PROP_FRAME_COUNT) - 1;
	int firstFrame = 0;
	if (start > 0 && start < end)
		firstFrame = start / 10. * s.fps;
	if (end > 0 && end / 10 * s.fps < lastFrame && end > start)
		lastFrame = end / 10 * s.fps;
	for (int i = firstFrame; i < lastFrame; i += skip) {
		VER("FRAME"<<i<<"/"<<lastFrame)
		cap.set(cv::CAP_PROP_POS_FRAMES, i);
		cap.read(img);
		if (!init && !outPath.empty()) {
			init = true;
			videoWriter = cv::VideoWriter(outPath, cv::VideoWriter::fourcc('X', '2', '6', '4'), 8, cv::Size(img.cols, img.rows)); //cv::VideoWriter::fourcc('M', 'J', 'P', 'G')
		}
		for (auto &t : trajectories) {
			Pose tmp;
			cv::Scalar c(255, 255, 255);
			stringstream ssId;
			ssId << " id:" << (int) t.id;
			if (t.get_pose_at(i / s.fps, tmp)) {
				if (tmp.finalized == 4)
					c = cv::Scalar(0, 255, 0);
				auto tmp2 = tmp.bb.transform(ext);
				VER(tmp2.center)
				if (tmp2.plot(img, c, s.intrinsic,s.distortion, 1,s.frustum )) {
					auto r = tmp2.get_camera_rect(s.intrinsic, s.distortion,s.frustum);
					cv::rectangle(img, r, c, 1);
					string className = LUMPIPipeline::switch_class_description(tmp.classId);
					cv::putText(img, className + ssId.str(), cv::Point2d(r.tl().x, r.tl().y - 10), 1, 1, cv::Scalar(125, 0, 0), 1);
				}
			}
		}
		cv::putText(img, to_string(i), cv::Point2d(10, 50), 1, 3, cv::Scalar(255, 255, 255), 1);
		if (!outPath.empty())
			videoWriter.write(img);
	}
	if (!outPath.empty())
		videoWriter.release();
	return true;
}
pcl::PointCloud<pcl::PointXYZRGB> LUMPIPipeline::lumpi_to_rgb(const pcl::PointCloud<IKGB> &pc) {
	pcl::PointCloud<pcl::PointXYZRGB> out;
	for (const auto &p : pc) {
		pcl::PointXYZRGB p2;
		p2.x = p.x;
		p2.y = p.y;
		p2.z = p.z;
		p2.r = p.r;
		p2.g = p.g;
		p2.b = p.b;
		out.points.push_back(p2);
	}
	return out;
}
void LUMPIPipeline::read_background(std::unordered_map<int, cv::Mat> &th, std::unordered_map<int, double> &angleNormalizer, std::string path) {
	auto bgFiles = Parser::files_at_directory(path);
	for (auto f : bgFiles) {
		auto tmp = ikg::split(f, ".");
		if (tmp.back() == "npy") {
			th[stoi(tmp.front())] = Parser::read_numpy_cv_mat(path + "/" + f) - 200 * 0.2;
		}
	}

	ifstream bgMeta(path + "/meta_background.txt");
	string line;
	while (getline(bgMeta, line)) {
		auto tmp = ikg::split(line, ",");
		if (tmp[0] == "azimuth normalizer")
			angleNormalizer[stoi(tmp[1])] = stod(tmp[2]);
	}

}

std::pair<std::vector<int>, std::vector<int>> LUMPIPipeline::remove_back_ground(const pcl::PointCloud<IKGB> &pc, const std::unordered_map<int, cv::Mat> &th,
		const std::unordered_map<int, double> &degNormalizer) {
	vector<int> bg, fg;
	static bool *mark, init = false;
	static int markSize = 500000; //todo make share pointer
	if (!init) {
		init = true;
		mark = reinterpret_cast<bool*>(malloc(markSize * sizeof(bool)));
	}
	if (pc.size() > markSize) {
		markSize = pc.size() * 1.1;
		mark = reinterpret_cast<bool*>(malloc(markSize * sizeof(bool)));
	}
#pragma omp parallel for num_threads(6)
	for (int i = 0; i < pc.size(); i++) {
		const IKGB &p = pc.points[i];
		if (th.count((int) p.id) < 1) {
			fg.push_back(i);
			continue;
		} //make more efficient
		int id = (int) p.id;
		int az = (int) ((int) p.azimuth * degNormalizer.at((int) p.id));
		cv::Mat tmp = th.at((int) p.id);
		int r = (int) p.ray;
		if (az < 0 || az > tmp.cols - 1 || r < 0 || r > tmp.rows - 1) {
			fg.push_back(i);
			continue;
		} //todo make more efficient
		if ((int) p.distance > th.at(id).at<double>(r, az))
			mark[i] = false;
		else
			mark[i] = true;
	}
	for (int i = 0; i < pc.points.size(); i++) {
		if (mark[i])
			fg.push_back(i);
		else
			bg.push_back(i);
	}
	return {fg,bg};
}
cv::Vec3b LUMPIPipeline::switch_class_color(int id) {
	cv::Vec3b pcColor;
	switch (id) {
	case 0:
		pcColor = cv::Vec3b(0, 255, 0);
		break;
	case 1:
		pcColor = cv::Vec3b(255, 0, 0);
		break;
	case 2:
		pcColor = cv::Vec3b(0, 0, 255);
		break;
	case 3:
		pcColor = cv::Vec3b(0, 255, 255);
		break;
	case 4:
		pcColor = cv::Vec3b(255, 0, 255);
		break;
	case 5:
		pcColor = cv::Vec3b(255, 255, 0);
		break;
	case 6:
		pcColor = cv::Vec3b(255, 255, 255);
		break;
	default:
		pcColor = cv::Vec3b(125, 125, 125);
		break;
	}
	return pcColor;
}
std::string LUMPIPipeline::switch_class_description(int id, bool yolo) {
	static vector<string> yoloClass = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
			"bird", "cat", "dog" };
	static map<int, string> lumpiClass = { { 0, "person" }, { 1, "car" }, { 2, "bicycle" }, { 3, "motorcycle" }, { 4, "bus" }, { 5, "truck" }, { -1, "unknown" }, { -2, "background" } };
	if (yolo) {
		if (id < 0 || id > (int) yoloClass.size() - 1)
			return "unknown";
		return yoloClass[id];
	}

	else {
		return lumpiClass[id];
	}
//			horse
//			sheep
//			cow
//			elephant
//			bear
//			zebra
//			giraffe
//			backpack
//			umbrella
//			handbag
//			tie
//			suitcase
//			frisbee
//			skis
//			snowboard
//			sports ball
//			kite
//			baseball bat
//			baseball glove
//			skateboard
//			surfboard
//			tennis racket
//			bottle
//			wine glass
//			cup
//			fork
//			knife
//			spoon
//			bowl
//			banana
//			apple
//			sandwich
//			orange
//			broccoli
//			carrot
//			hot dog
//			pizza
//			donut
//			cake
//			chair
//			couch
//			potted plant
//			bed
//			dining table
//			toilet
//			tv
//			laptop
//			mouse
//			remote
//			keyboard
//			cell phone
//			microwave
//			oven
//			toaster
//			sink
//			refrigerator
//			book
//			clock
//			vase
//			scissors
//			teddy bear
//			hair drier
//			toothbrush}
}
void LUMPIPipeline::visualOutputBoundingBox(
		const std::vector<BoundingBox> & current_measurement,pcl::visualization::PCLVisualizer::Ptr viewer,int thick ,std::string name,bool showIds) {
if(name=="clear"){
	viewer->removeAllShapes();
	return;
}
if(viewer->contains(name.c_str())){
viewer->removePolygonMesh(name.c_str());
}

pcl::PointCloud<pcl::PointXYZRGBA> pointCloudTest;
vector<pcl::Vertices> vTotal;
for (int i = 0; i < current_measurement.size(); i++) {
	auto &car = current_measurement[i];
	if (car.corners.size() < 8) {
		cerr << "no coners" << endl;
		continue;
	}
	for (int j = 0; j < car.corners.size(); j++) {

		const auto & p = car.corners[j];
		cv::Vec3b c=car.color;
		if(!(j>1&&j<5)&&j!=7)c*=0.3;
		pcl::PointXYZRGBA tmp;
		tmp.x = p.x;
		tmp.y = p.y;
		tmp.z = p.z;
		tmp.b = c[0];
		tmp.g = c[1];
		tmp.r = c[2];
		pointCloudTest.push_back(tmp);
	}
	//bottom
	pcl::Vertices v1;
	v1.vertices.push_back(i * 8 + 0);
	v1.vertices.push_back(i * 8 + 1);
	v1.vertices.push_back(i * 8 + 2);
	v1.vertices.push_back(i * 8 + 3);
	v1.vertices.push_back(i * 8 + 0);
	vTotal.push_back(v1);
	pcl::Vertices v2;
	v2.vertices.push_back(i * 8 + 5);
	v2.vertices.push_back(i * 8 + 6);
	v2.vertices.push_back(i * 8 + 7);
	v2.vertices.push_back(i * 8 + 4);
	vTotal.push_back(v2);
	//front
	pcl::Vertices v3;
	v3.vertices.push_back(i * 8 + 0);
	v3.vertices.push_back(i * 8 + 5);
	v3.vertices.push_back(i * 8 + 6);
	v3.vertices.push_back(i * 8 + 1);
	v3.vertices.push_back(i * 8 + 0);
	vTotal.push_back(v3);
	//left
	pcl::Vertices v4;
	v4.vertices.push_back(i * 8 + 0);
	v4.vertices.push_back(i * 8 + 5);
	v4.vertices.push_back(i * 8 + 4);
	v4.vertices.push_back(i * 8 + 3);
	v4.vertices.push_back(i * 8 + 0);
	vTotal.push_back(v4);
	//right
	pcl::Vertices v5;
	v5.vertices.push_back(i * 8 + 1);
	v5.vertices.push_back(i * 8 + 2);
	v5.vertices.push_back(i * 8 + 7);
	v5.vertices.push_back(i * 8 + 6);
	v5.vertices.push_back(i * 8 + 1);
	vTotal.push_back(v5);
}
viewer->addPolygonMesh<pcl::PointXYZRGBA>(pointCloudTest.makeShared(),
		vTotal, name.c_str());
viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, thick, name.c_str());
viewer->setRepresentationToWireframeForAllActors();
if(showIds){
for (int i = 0; i < min((int)current_measurement.size(),1); i++) {//current_measurement.size()
	auto &car = current_measurement[i];
	if(car.corners.size()<8)continue;
	stringstream ss;
	ss<<setprecision(0)<<std::fixed<<to_string((int)car.id)<<endl;
	string bbId=ss.str();
	pcl::PointXYZRGBA pos;//
	pos.x=car.corners[4].x;
	pos.y=car.corners[4].y;
	pos.z=car.corners[4].z;
}
}
}

} /* namespace ikg */
