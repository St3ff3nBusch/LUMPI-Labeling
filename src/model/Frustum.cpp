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

#include "Frustum.h"
using namespace std;
namespace frustum {
Plane::~Plane() {
}
Plane::Plane(double a, double b, double c, double d) :
		d(d) {
	n = cv::Point3d(a, b, c);
	n /= cv::norm(n);
	p = cv::Point3d(0, 0, 0);
	if (d > 0 && c > 0)
		p = cv::Point3d(0, 0, d / c);
	else if (d > 0 && b > 0)
		p = cv::Point3d(0, d / b, 0);
	else if (d > 0 && a > 0)
		p = cv::Point3d(d / a, 0, 0);
}
Frustum::Frustum() {
	near = 0;
	far = 0;
}
Frustum::Frustum(cv::Mat intrinsic, int rows, int cols) {
	far = 300;
	near=1;
	double nw = cols/(2 * intrinsic.at<double>(0, 0))*near;
	double nh = rows/ (2 * intrinsic.at<double>(1, 1))* near;
	planes.push_back(Plane(0, 0, 1, near));
	planes.push_back(Plane(0, 0, -1, -far));
	planes.push_back(Plane(1, 0,nw, 0));
	planes.push_back(Plane(-1, 0, nw, 0));
	planes.push_back(Plane(0, 1, nh, 0));
	planes.push_back(Plane(0, -1, nh, 0));
}

bool Frustum::plane_line_intersection(const cv::Point3d &startPoint, const cv::Point3d &endPoint, const Plane &plane, cv::Point3d &intersectingPoint) const {
	cv::Point3d v = endPoint - startPoint;
	double p = v.dot(plane.n);
	if (p == 0)
		return false;
	double d = (plane.d-plane.n.dot(startPoint)) / p;
	if (d < 0 || d > 1)
		return false;
	intersectingPoint = startPoint + v * d;
	return true;
}
bool Frustum::culling(std::vector<std::pair<cv::Point3d, cv::Point3d>> &lines) const {
	vector<bool> validLines(lines.size(), false);
	for (int i = 0; i < lines.size(); i++) {
		auto &l = lines[i];
		bool v1 = within_frustum<cv::Point3d>(l.first);
		bool v2 = within_frustum<cv::Point3d>(l.second);
		if (v1 && v2)
			validLines[i] = true;
		else if (!v1 && !v2)
			validLines[i] = false;
		else if (v1) {
			for (const auto & p : planes) {
				plane_line_intersection(l.first, l.second, p, l.second);
			}
			validLines[i] = within_frustum<cv::Point3d>(l.second);
		} else {
			for (const auto & p : planes) {
				if(plane_line_intersection(l.first, l.second, p, l.first)){
				}else{
				}
			}
			validLines[i] = within_frustum<cv::Point3d>(l.first);
		}
	}
	auto it=lines.begin();
	for(int i=0;i<validLines.size();i++){
		if(!validLines[i]){
			it=lines.erase(it);
		}
		else ++it;
	}
	return !lines.empty();
}
Frustum::~Frustum() {
}

} /* namespace ikg */
