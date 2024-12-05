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

#include "model/geometries/Pose.h"
using namespace std;
namespace ikg {
const double Pose::TWO_PI=M_PI*2;
const double Pose::THREE_PI=M_PI*3;

double Pose::distance_in_2d(const Pose &a, const Pose &b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(dx * dx + dy * dy);
}
std::ostream& operator<<(std::ostream &outStream, const Pose &p) {
	outStream << setprecision(20) << "<pose>" << p.time << ",";
	outStream<< p.x << "," << p.y << "," << p.z << ",";
	outStream<<p.l<<","<<p.w<<","<<p.h<< "," << p.orientation << ",";
	for (const auto& e : p.shape)
		outStream << e << ",";
	outStream<<"</pose>";
	return outStream;
}
Pose& Pose::operator=(const Pose &a) {
	bb = a.bb;
	classId = a.classId;
	p_id = a.p_id;
	tId = a.tId;
	pos = a.pos;
	x = a.x;
	y = a.y;
	z = a.z;
	w = a.w;
	l = a.l;
	h = a.h;
	time = a.time;
	interpolated = a.interpolated;
	finalized = a.finalized;
	orientation = a.orientation;
	standing = a.standing;
	shape = a.shape;
	r = a.r;
	score = a.score;
	visibility = a.visibility;
	co = a.co;
	so = a.so;
	return *this;
}

double Pose::interpolate_heading(double a, double b,   double s2) {
	double deltaAngle=fmod(b-a+ THREE_PI, TWO_PI) - M_PI;
	return fmod(a+s2*deltaAngle+ THREE_PI, TWO_PI) - M_PI;
}

Pose Pose::interpolate(const Pose &p1, const Pose &p2, double s2) {
	Pose resultPose;
	double s1 = 1 - s2;
	resultPose.x = p1.x * s1 + p2.x * s2;
	resultPose.y = p1.y * s1 + p2.y * s2;
	resultPose.z = p1.z * s1 + p2.z * s2;
	resultPose.time = p1.time * s1 + p2.time * s2;
	resultPose.tId = p1.tId;
	resultPose.score = p1.score * s1 + p2.score * s2;
	resultPose.classId = p1.classId;
	resultPose.visibility = p1.visibility * s1 + p2.visibility * s2;
	resultPose.interpolated = true;
	resultPose.standing = p1.standing && p2.standing;
	resultPose.finalized = false;
	cv::Point2d tl = p1.r.tl() * s1 + p2.r.tl() * s2;
	resultPose.r = cv::Rect2d(tl.x, tl.y, p1.r.width * s1 + p2.r.width * s2, p1.r.height * s1 + p2.r.height * s2);
	resultPose.orientation=interpolate_heading(p1.orientation, p2.orientation, s2);
	resultPose.bb = BoundingBox::interpolate(p1.bb, p2.bb, s2, false);
	return resultPose;
}

Pose::Pose(double x, double y, double z, double time, int t_id, int p_id) :
		x(x), y(y), z(z), standing(false), interpolated(false), tId(t_id), orientation(0), p_id(p_id), w(0), l(0), h(0), time(time) {
	score = 0;
	classId = 0;
	visibility = 0;
	pos = 0;
	standing = false;
	interpolated = false;
	finalized = false;
	co = 1;
	so = 0;
}

Pose::Pose(const Pose &a) {
	*this = a;
}
Pose::~Pose() =default;

} /* namespace ikg */
