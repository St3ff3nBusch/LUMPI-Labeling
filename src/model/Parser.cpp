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

#include <fstream>
#include <iostream>
#include "model/StructsAndMore.h"
#include <dirent.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sstream>
#include <sstream>
#include <filesystem>
#include <Parser.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QVariantHash>
using namespace ikg;
using namespace std;
Sensor Parser::session_to_sensor(const QJsonObject &session) {
	Sensor s;
	s.sessionId = session.value("sessionId").toInt();
	s.deviceId = session.value("deviceId").toInt();
	s.expId = session.value("experimentId").toInt();
	s.fps = session.value("fps").toDouble(0);
	s.horizontalResolution = session.value("horizontal_resolution").toDouble(0);
	auto p0 = session.value("extrinsic").toArray();
	s.extrinsic = cv::Mat::eye(4, 4, CV_64F);
	s.startTime = 0;
	for (int i = 0; i < p0.size(); i++) {
		auto p1 = p0[i].toArray();
		for (int j = 0; j < p1.size(); j++) {
			s.extrinsic.at<double>(i, j) = p1[j].toDouble();
		}
	}
	s.type = session.value("type").toString().toStdString();
	if (session.contains("intrinsic")) {
		auto e0 = session.value("intrinsic").toArray();
		s.intrinsic = cv::Mat::eye(3, 3, CV_64F);
		for (int i = 0; i < e0.size(); i++) {
			auto e1 = e0[i].toArray();
			for (int j = 0; j < e1.size(); j++) {
				s.intrinsic.at<double>(i, j) = e1[j].toDouble();
			}
		}
	}
	if (session.contains("rvec")) {
		s.rvec = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		auto r0 = session.value("rvec").toArray();
		for (int i = 0; i < r0.size(); i++) {
			auto r1 = r0[i].toArray();
			for (int j = 0; j < r1.size(); j++) {
				s.rvec.at<double>(j, i) = r1[j].toDouble();
			}
		}
	}
	if (session.contains("tvec")) {
		s.tvec = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		auto t0 = session.value("tvec").toArray();
		for (int i = 0; i < t0.size(); i++) {
			auto t1 = t0[i].toArray();
			for (int j = 0; j < t1.size(); j++) {
				s.tvec.at<double>(j, i) = t1[j].toDouble();
			}
		}
	}
	if (session.contains("distortion")) {
		auto d0 = session.value("distortion").toArray();
		s.distortion = cv::Mat::zeros(5, 1, CV_64F);
		for (int i = 0; i < d0.size(); i++) {
			auto d1 = d0[i].toArray();
			for (int j = 0; j < d1.size(); j++) {
				s.distortion.at<double>(j, i) = d1[j].toDouble();
			}
		}
	}
	if (session.contains("angles")) {
		auto a0 = session.value("angles").toArray();
		for (int i = 0; i < a0.size(); i++) {
			s.angles.push_back(a0[i].toDouble());
		}
	}
	return s;
}

std::pair<map<int, Sensor>, std::map<int, map<int, int>>> Parser::read_meta_lumpi(
		const std::string &path) {
	std::map<int, map<int, int>> mapSess;
	std::map<int, Sensor> sensors;
	QString val;
	QFile file(path.c_str());
	file.open(QIODevice::ReadOnly | QIODevice::Text);
	val = file.readAll();
	file.close();
	QJsonDocument jsonResponse = QJsonDocument::fromJson(val.toUtf8());
	if (jsonResponse.isEmpty())
	if (jsonResponse.isEmpty()) {
		cerr << "empty json" << endl;
		return {sensors,mapSess};
	}
	QJsonObject jsonObj = jsonResponse.object();
	set<int> expIds, devIds;
	for (auto k : jsonObj.value("session").toObject().toVariantHash().keys()) {
		string sessId = (k.toStdString());
		auto tmp = jsonObj.value("session").toObject().value(sessId.c_str()).toObject();
		auto s = session_to_sensor(tmp);
		sensors.insert( { s.sessionId, s });
		mapSess[s.expId][s.deviceId] = s.sessionId;
	}
	return {sensors,mapSess};
}
void Parser::write_ikgb_point_cloud_to_ply(const std::string &name, const pcl::PointCloud<IKGB> &cloud) {
	ofstream file;
	file.open(name.c_str(), ios::binary);
	stringstream header;
	header << "ply" << endl;
	header << "format binary_little_endian 1.0" << endl;
	header << "element vertex " << cloud.points.size() << endl;
	header << "property float x" << endl;
	header << "property float y" << endl;
	header << "property float z" << endl;
	header << "property uint time" << endl;
	header << "property uchar id" << endl;
	header << "property uchar intensity" << endl;
	header << "property uchar ray" << endl;
	header << "property ushort azimuth" << endl;
	header << "property ushort distance" << endl;
	header << "end_header" << endl;
	int headerSize = sizeof(unsigned char) * header.str().size();
	file.write((char*) header.str().c_str(), headerSize);
	for (int i = 0; i < cloud.points.size(); i++) {
		const IKGB &p = cloud.points[i];
		float x = p.x;
		float y = p.y;
		float z = p.z;
		std::uint8_t id = p.id;
		file.write((char*) &x, sizeof(float));
		file.write((char*) &y, sizeof(float));
		file.write((char*) &z, sizeof(float));
		file.write((char*) &p.adjustedtime, sizeof(std::uint32_t));
		file.write((char*) &p.id, sizeof(std::uint8_t));
		file.write((char*) &p.intensity, sizeof(std::uint8_t));
		file.write((char*) &p.ray, sizeof(std::uint8_t));
		file.write((char*) &p.azimuth, sizeof(std::uint16_t));
		file.write((char*) &p.distance, sizeof(std::uint16_t));
	}
	file.close();
}

void Parser::write_trajectories(const std::vector<Trajectory> &traj, const std::string & path,const bool &simple,
		const bool &append, const bool &ply) {
	ofstream file;
	std::multimap<double, Pose> det;
	int count = 0;
	for (const auto &t : traj) {
		for (auto p : t.poseData) {
			if (p.r.width == 0 || p.r.height == 0) {
				p.r = cv::Rect2d(p.x, p.y, p.w, p.h);
			}
			p.tId = t.id;
			det.insert( { p.time, p });
		}
	}
	if (ply) {
		if (append)
			file.open(path.c_str(), ios::binary | ios::app);
		else {
			file.open(path.c_str(), ios::binary);
			stringstream header;
			header << "ply" << endl;
			header << "format binary_little_endian 1.0" << endl;
			header << "element vertex " << det.size() << endl;
			header << "property double time" << endl;
			header << "property int id" << endl;
			header << "property float u" << endl;
			header << "property float v" << endl;
			header << "property float w" << endl;
			header << "property float h" << endl;
			header << "property float score" << endl;
			header << "property int class" << endl;
			header << "property float visibility" << endl;
			header << "property float x" << endl;
			header << "property float y" << endl;
			header << "property float z" << endl;
			header << "property float length" << endl;
			header << "property float width" << endl;
			header << "property float height" << endl;
			header << "property float heading" << endl;
			if (!simple) {
				header << "element face " << det.size() << endl;

				header << "property list int float shape" << endl;
			}
			header << "end_header" << endl;
			int headerSize = sizeof(unsigned char) * header.str().size();
			file.write((char*) header.str().c_str(), headerSize);
		}
		for (auto &d : det) {
			float tmp[5];
			tmp[0] = (float) d.second.r.x;
			tmp[1] = (float) d.second.r.y;
			tmp[2] = (float) d.second.r.width;
			tmp[3] = (float) d.second.r.height;
			tmp[4] = (float) d.second.score;
			file.write((char*) &d.second.time, sizeof(double));
			file.write((char*) (&d.second.tId), sizeof(int));
			file.write((char*) &tmp, sizeof(tmp));
			file.write((char*) (&d.second.classId), sizeof(int));
			float tmp2[8];
			tmp2[0] = (float) d.second.visibility;
			tmp2[1] = (float) d.second.bb.center.x;
			tmp2[2] = (float) d.second.bb.center.y;
			tmp2[3] = (float) d.second.bb.center.z;
			tmp2[4] = (float) d.second.bb.length;
			tmp2[5] = (float) d.second.bb.width;
			tmp2[6] = (float) d.second.bb.height;
			tmp2[7] = (float) d.second.bb.angle;
			file.write((char*) &tmp2, sizeof(tmp2));
		}
		if (!simple) {
			for (auto &d : det) {
				int size = d.second.shape.size();
				file.write((char*) &size, sizeof(int));
				float tmp3[size];
				for (int j = 0; j < d.second.shape.size(); j++) {
					tmp3[j] = (float) d.second.shape[j];
				}
				file.write((char*) &tmp3, sizeof(tmp3));
			}
		}
		file.close();
	} else {
		if (!append) {
			file.open(path.c_str());
			file
					<< "time,object id, 2d rectangle: top left x,top left y, width,height,score,class_id,visibility,3D box: center x,y,z,length, width ,height,heading,[optional arbitarry many double: shape parameter or point index or embeddings]"
					<< endl;
		} else
			file.open(path.c_str(), ios::app);
		for (const auto &d : det) {
			//		if(d.second.class_id!=1)continue;
			file << std::fixed << setprecision(3) << d.second.time << "," << (int) d.second.tId
					<< "," << d.second.r.tl().x << "," << d.second.r.tl().y << ","
					<< d.second.r.width << "," << d.second.r.height << "," << d.second.score << ","
					<< (int) d.second.classId << "," << d.second.visibility << ",";
			file << d.second.bb.center.x << "," << d.second.bb.center.y << ","
					<< d.second.bb.center.z << "," << d.second.bb.length << "," << d.second.bb.width
					<< "," << d.second.bb.height << "," << d.second.bb.angle;
			if (!simple) {
				for (auto &s : d.second.shape)
					file << "," << s;
			}
			file << endl;
		}
		file.close();
	}
}
pcl::PointCloud<IKGB> Parser::load_cloud_ikg_benchmark(const string &path) {
	pcl::PointCloud<IKGB> pc;
	if (!std::filesystem::exists(path.c_str()))
		return pc;
	int fd;
	/* Information about the file. */
	struct stat s;
	int status;
	size_t size;
	/* The file name to open. */
	const char *file_name = path.c_str();
	/* The memory-mapped thing itself. */
	void *mapped;
	char *data;
	/* Open the file for reading. */
	fd = open(path.c_str(), O_RDONLY);
//  check (fd < 0, "open %s failed: %s", file_name, strerror (errno));
	/* Get the size of the file. */
	status = fstat(fd, &s);
	if (status < 0)
		return pc;
//  check (status < 0, "stat %s failed: %s", file_name, strerror (errno));
	size = s.st_size;
	int num_vertex = 0;
	/* Memory-map the file. */
	mapped = mmap(0, size, PROT_READ, MAP_PRIVATE, fd, 0);
	data = reinterpret_cast<char*>(mapped);
	std::string header(reinterpret_cast<const char*>(data), 500);
	std::vector<string> tmp2 = ikg::split(header, "\n");
	for (auto &l : tmp2) {
		std::vector<string> tmp = ikg::split(l, ", \t");
		for (int i = 0; i < tmp.size(); i++) {
			if (tmp[i].compare("element") == 0 && tmp[i + 1].compare("vertex") == 0) {

				num_vertex = stoi(tmp[i + 2]);
			}
			if (tmp[i].compare("end_header") == 0) {
				goto reading;
			}
		}
	}
	reading:
	int sizeVertex = (3 * sizeof(float) + sizeof(unsigned int) + 3 * sizeof(char)
			+ 2 * sizeof(unsigned short));
	int numBytes = num_vertex * sizeVertex;
	pc.resize(num_vertex);
	pc.points.resize(num_vertex);
	static std::map<std::uint8_t, cv::Scalar> colors;
	int offset = size - numBytes;
#pragma omp parallel for num_threads(6)
	for (int i = 0; i < num_vertex; i++) {
		int index = offset + i * sizeVertex;
		pc.points[i].x = *reinterpret_cast<float*>(&data[index]);
		pc.points[i].y = *reinterpret_cast<float*>(&data[index + sizeof(float)]);
		pc.points[i].z = *reinterpret_cast<float*>(&data[index + 2 * sizeof(float)]);
		pc.points[i].adjustedtime = *reinterpret_cast<std::uint32_t*>(&data[index
				+ 3 * sizeof(float)]);
		pc.points[i].id = *reinterpret_cast<std::uint8_t*>(&data[index + 3 * sizeof(float)
				+ sizeof(std::uint32_t)]);
		pc.points[i].intensity = *reinterpret_cast<std::uint8_t*>(&data[index + 3 * sizeof(float)
				+ sizeof(std::uint32_t) + sizeof(std::uint8_t)]);
		pc.points[i].ray = *reinterpret_cast<std::uint8_t*>(&data[index + 3 * sizeof(float)
				+ sizeof(std::uint32_t) + 2 * sizeof(std::uint8_t)]);
		pc.points[i].azimuth = *reinterpret_cast<std::uint16_t*>(&data[index + 3 * sizeof(float)
				+ sizeof(std::uint32_t) + 3 * sizeof(std::uint8_t)]);
		pc.points[i].distance = *reinterpret_cast<std::uint16_t*>(&data[index + 3 * sizeof(float)
				+ sizeof(std::uint32_t) + 3 * sizeof(std::uint8_t) + sizeof(std::uint16_t)]);
		pc.points[i].index = i;
	}
	munmap(mapped,size);
	close(fd);
	return pc;
}
pcl::PointCloud<IKGB> Parser::load_cloud_ikg_mobile_mapping(const std::string &path) {
	pcl::PointCloud<IKGB> pc;
//read header
	ifstream file(path);
	string line;
	int num_vertex = 0;
	vector<int> index = { -1, -1, -1, -1, -1, -1, -1 };	//proberties x ,y,z,i,nx,ny,nz,(r,g,b)
	int indexCount = 0;
	while (getline(file, line)) {
		//                           cout<<line<<endl;
		std::vector<string> tmp = ikg::split(line, ", \t");
		for (int i = 0; i < tmp.size(); i++) {
			if (tmp[i].compare("element") == 0 && tmp[i + 1].compare("vertex") == 0) {
				num_vertex = stoi(tmp[i + 2]);
			}
			if (tmp[i].compare("end_header") == 0) {
				goto readPoints;
			}
		}
	}
	readPoints:
	int sizeVertex = 3 * sizeof(double)+1*sizeof(float)+1*sizeof(short)+sizeof(int);//
	int numBytes = num_vertex * sizeVertex;
	char *data = new char[numBytes]; //to seperate in smaller arrrays
	file.read(data, numBytes);
	pc.resize(num_vertex);
	pc.points.resize(num_vertex);
#pragma omp parallel for num_threads(6)
	for (int i = 0; i < num_vertex; i++) {
		int index = i * sizeVertex;
		pc.points[i].x = *reinterpret_cast<double*>(&data[index]);
		pc.points[i].y = *reinterpret_cast<double*>(&data[index + sizeof(double)]);
		pc.points[i].z = *reinterpret_cast<float*>(&data[index + 2 * sizeof(double)]);
	}
	delete data;
	return pc;
}
std::vector<Trajectory> Parser::read_trajectories(const std::string &fileName,const bool &cameraFrame,const bool &simple,
		const bool & ply) {
	std::vector<Trajectory> trajectories;
	Trajectory t;
	ifstream file(fileName.c_str());			//
	string line;
	unordered_map<int, Trajectory> trajectoryMap;
		auto csv = read_csv_file_double(fileName);
		for (auto &data : csv) {
			if (data.size() < 9)
				continue;
			Pose p;
			p.time = data[0];
			p.tId = data[1];
			p.r = cv::Rect2d(data[2], data[3], data[4], data[5]);
			p.w = data[4];
			p.h = data[5];
			p.x = data[2];
			p.y = data[3];
			p.score = data[6];
			p.classId = data[7];
			p.visibility = data[8];
			if (data.size() > 15) {
				p.bb = BoundingBox(data[9], data[10], data[11], data[12], data[13], data[14],
						data[15], cameraFrame);
				p.bb.classId = p.classId;
			}
			p.bb.id = p.tId;
			if (!simple && ((int) data.size() - 16) > 0) {
				cv::Mat tmpM(data.size() - 16, 1, CV_32F);
				int index = 0;
				for (int i = 16; i < data.size(); i++) {
					p.shape.push_back(data[i]);
				}
			}
			trajectoryMap[p.tId].poseData.push_back(p);
		}
	for (auto &t : trajectoryMap) {
		if(t.second.poseData.size()<1)continue;
		t.second.classId = t.second.poseData.front().classId;
		trajectories.push_back(t.second);
		trajectories.back().id = t.first;
		for (int j = 0; j < trajectories.back().poseData.size(); j++) {
			trajectories.back().poseData[j].pos = j;
		}
	}
	sort(trajectories.begin(),trajectories.end(),[](Trajectory &a ,Trajectory&b){return a.id<b.id;});
	return trajectories;
}
std::pair<std::vector<double>, std::vector<int> > Parser::read_numpy(const std::string & name) {
	std::vector<double> data;
	std::vector<int> dim;
	ifstream file(name);
	for (int i = 0; i < 8; i++) {
		char c;
		file.read(reinterpret_cast<char*>(&c), sizeof(char));
	}
	unsigned short length;
	file.read(reinterpret_cast<char*>(&length), sizeof(unsigned short));
	char header[length];
	file.read(reinterpret_cast<char*>(&header), sizeof(char) * length);
	string s = "";
	for (int i = 0; i < length; i++) {
		s += header[i];
	}
	vector<string> tmp = ikg::split(s, ": ,()''><");
	bool flagSize = false;
	bool order = false;
	enum dtyps {
		inv = -1, f2 = 1, f4 = 2, f8 = 3
	};
	int dtype = -1;
	for (int i = 0; i < tmp.size(); i++) {
		if (tmp[i] == "}")
			break;
		if (flagSize) {
			dim.push_back(stoi(tmp[i]));
		}
		if (tmp[i] == "shape") {
			flagSize = true;
		}
		if (tmp[i] == "fortran_order" && tmp[i + 1] == "True") {
			order = true;
		}
		if (tmp[i] == "descr") {
			if (tmp[i + 1] == "f2") {
				dtype = f2;
			}
			if (tmp[i + 1] == "f4") {
				dtype = f4;
			}
			if (tmp[i + 1] == "f8") {
				dtype = f8;
			}
		}
	}
	auto fpos = file.tellg();
	file.seekg(0, std::ios::end);
	int ld = file.tellg() - fpos;
	file.seekg(fpos);
	int e = 1;
	if (order)
		std::reverse(dim.begin(), dim.end());
	for (int i : dim) {
		e *= i;
	}
	if (dtype == f2) {
		unsigned short dat[ld / sizeof(unsigned short)];
		file.read(reinterpret_cast<char*>(&dat), sizeof(char) * ld);
		for (auto i = 0; i < ld / sizeof(unsigned short); i++) {
			float f;
			ikg::float32(&f, dat[i]);
			data.push_back(*reinterpret_cast<float*>(&f));
		}
	} else if (dtype == f4) {
		float *dat = new float[ld / sizeof(float)]; //[ld / sizeof(float) / 1000];
		data = vector<double>(ld / sizeof(float));
		file.read(reinterpret_cast<char*>(&dat[0]), sizeof(char) * ld);
		for (int i = 0; i < ld / sizeof(float); i++) {
			data[i] = dat[i];
		}
	} else if (dtype == f8) {
		double dat[ld / sizeof(double)];
		file.read(reinterpret_cast<char*>(&dat), sizeof(char) * ld);
		for (auto i = 0; i < ld / sizeof(double); i++) {
			double tmp = *reinterpret_cast<double*>(&dat[i]);
			data.push_back(tmp);
		}
	}
	file.close();
	return {data,dim};
}
cv::Mat Parser::read_numpy_cv_mat(const string & path) {
	cv::Mat matrix;
	auto p = read_numpy(path);
	if (p.second.size() < 2)
		return matrix.clone();
	matrix = cv::Mat::zeros(p.second[0], p.second[1], CV_64F);
	memcpy(matrix.data, p.first.data(), p.first.size() * sizeof(double));
	return matrix.clone();
}
bool Parser::save_numpy(const std::string &name, const cv::Mat img) {
	vector<double> data;
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			if (img.type() == 6) {
				data.push_back(img.at<double>(i, j));
			} else if (img.type() == 5) {
				data.push_back(img.at<float>(i, j));
			} else if (img.type() == 4) {
				data.push_back(img.at<int>(i, j));
			} else if (img.type() == 3) {
				data.push_back(img.at<short>(i, j));
			} else if (img.type() == 2) {
				data.push_back(img.at<unsigned short>(i, j));
			}
		}
	}
	save_numpy(name, data, (vector<int> ) { img.rows, img.cols });
	return true;
}
bool Parser::save_numpy(const std::string &name, const std::vector<double> &data, const std::vector<int> &size) {
	ofstream file(name, std::ofstream::binary);
	char version = 1;
	char v = 0;
	string magic = "\x93NUMPY";
	file << magic;
	file << version;
	file << v;
	string header = "{'descr': '<f2', 'fortran_order': False, 'shape': (";
	for (int i = 0; i < size.size() - 1; i++)
		header += to_string(size[i]) + ",";
	header += to_string(size.back());
	header += "), }";
	cout << header << endl;
	unsigned short hl = header.size();
	for (int i = 0; i < (64 - (hl + magic.size()) % 64) - 1; i++) {
		header += "\x20";
	}
	header += "\n";
	hl = header.size();
	file.write(reinterpret_cast<char*>(&hl), sizeof(hl));
	file << header;
	for (int i = 0; i < data.size(); i++) {
		u_int16_t f = 0;
		ikg::float16(&f, (float) data[i]);
//		float f2;
//		ikg::float32(&f2, f);
		file.write(reinterpret_cast<char*>(&f), 2);
	}
	file.close();
	return false;
}

std::vector<IKGB> Parser::read_point_cloud_csv_supsampled(const std::string &name, const double &res) {
	std::vector<IKGB> points;
	auto data = read_csv_file_double(name);
	for (auto d : data) {
		if (d.size() < 4)
			continue;
		IKGB p;
		p.x = d[0];
		p.y = d[1];
		p.z = d[2];
		p.distance = d[3];
		points.push_back(p);
	}
	unordered_map<int, unordered_map<int, unordered_map<int, int>>> grid;
	double res_ = 1. / res;
	for (auto &p : points) {
		int x = floor(p.x * res_);
		int y = floor(p.y * res_);
		int z = floor(p.z * res_);
		grid[x][y][z] += p.distance;
	}
	points.clear();
	for (auto &itX : grid) {
		for (auto &itY : itX.second) {
			for (auto &itZ : itY.second) {
				IKGB p;
				p.x = itX.first * res;
				p.y = itY.first * res;
				p.z = itZ.first * res;
				p.distance = itZ.second;
				points.push_back(p);
			}
		}
	}
	return points;
}

std::vector<std::vector<double>> Parser::read_csv_file_double(const string &name, const string & delimiter) {
	if (!std::filesystem::exists(name.c_str()))
		return std::vector<std::vector<double>>();
	ifstream file(name, file.binary);
	file.seekg(0, file.end);
	std::string text(file.tellg(), 0);
	file.seekg(0);
	file.read(text.data(), text.size());
	auto lines = ikg::split(text, "\n");
	std::vector<std::vector<double>> data(lines.size());
	for (int i = 0; i < lines.size(); i++) {
		auto tmp = ikg::split(lines[i], delimiter);
		vector<double> d;
		for (auto &s : tmp) {
			try {
				data[i].push_back(stod(s));
			} catch (...) {
				if(i>0)
				cerr << "parsing error \"" << s << "\" is no double" << endl;
			}
		}
	}
	return data;
}

std::vector<string> Parser::files_at_directory(const string & directoryName) {
	vector<std::string> fnames;
	if(std::filesystem::is_directory(std::filesystem::status(directoryName.c_str()))){
		DIR *dir;
		dir = opendir(directoryName.c_str());
		struct dirent *ent;
		int count = 0;
		while ((ent = readdir(dir)) != NULL) {
			count++;
		}
		int numberOfFiles = count;
		count = 0;
		dir = opendir(directoryName.c_str());
		while ((ent = readdir(dir)) != NULL) {
			if ((string) ent->d_name == "." || (string) ent->d_name == "..") {
				continue;
			}
			fnames.push_back(ent->d_name);
		}
	}
	return fnames;
}
