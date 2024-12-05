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

#include "Plotter.h"
using namespace std;
using namespace cv;
using namespace ikg;
cv::Mat Plotter::get_heat_map(cv::Mat image, int colorScheme) {
	cv::Mat tmpMat1, tmpMat2, tmpMat3;
	image.copyTo(tmpMat1);
	double minV2, maxV2;
	cv::minMaxLoc(tmpMat1, &minV2, &maxV2);
	tmpMat1 -= minV2;
	tmpMat1.convertTo(tmpMat2, CV_8U, 255.0 / (maxV2 - minV2));
	cv::applyColorMap(tmpMat2, tmpMat3, cv::COLORMAP_JET);
	return tmpMat3.clone();
}
cv::Mat Plotter::get_heat_map_with_legend(cv::Mat image, int colorScheme) {
	cv::Mat tmpMat1, tmpMat2, tmpMat3;
	image.copyTo(tmpMat1);
	double minV2, maxV2;
	cv::minMaxLoc(tmpMat1, &minV2, &maxV2);
	tmpMat1 -= minV2;
	tmpMat1.convertTo(tmpMat2, CV_8U, 255.0 / (maxV2 - minV2));
	cv::applyColorMap(tmpMat2, tmpMat3, cv::COLORMAP_JET);
	cv::hconcat(tmpMat3, ikg::color_legend(minV2, maxV2, tmpMat3.rows), tmpMat3);
	return tmpMat3.clone();
}


