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

#ifndef INCLUDE_MODEL_GEOMETRIES_TRAJECTORY_H_
#define INCLUDE_MODEL_GEOMETRIES_TRAJECTORY_H_
#include "opencv4/opencv2/opencv.hpp"
#include "include/model/geometries/Pose.h"
namespace ikg {

/**
 * @class Trajectory
 * @brief Represents a trajectory consisting of multiple poses.
 *
 * The Trajectory class provides functionality to manage and manipulate a series of poses
 * that form a trajectory. It includes methods for calculating curvature, interpolating
 * between key frames, merging trajectories, and validating the trajectory based on
 * distance and curvature thresholds.
 */
class Trajectory {
public:
	/**
	 * @brief The ID of the trajectory.
	 */
	int id;

	/**
	 * @brief The class ID of the trajectory.
	 */
	int classId;

	/**
	 * @brief The pose data of the trajectory.
	 */
	std::vector<Pose> poseData;

	/**
	 * @brief Overloaded output stream operator for Trajectory.
	 * @param outStream The output stream.
	 * @param t The Trajectory object.
	 * @return The output stream.
	 */
	friend std::ostream& operator<<(std::ostream &outStream, const Trajectory &t);

	/**
	 * @brief Default constructor for Trajectory.
	 */
	explicit Trajectory();

	/**
	 * @brief Destructor for Trajectory.
	 */
	virtual ~Trajectory();

	/**
	 * @brief Calculate the curvature given three poses.
	 * @param A The first pose.
	 * @param B The second pose.
	 * @param C The third pose.
	 * @return The curvature.
	 */
	static double calc_curvature(const Pose &A, const Pose &B, const Pose &C);

	/**
	 * @brief Get the pose at a specific time.
	 * @param time The time to get the pose at.
	 * @param p The pose to populate.
	 * @param index The index to start searching from.
	 * @return True if pose found, false otherwise.
	 */
	bool get_pose_at(double time, Pose &p, int index = -1) const;

	/**
	 * @brief Insert a pose into the trajectory.
	 * @param p The pose to insert.
	 */
	void insert_pose(const Pose &p);

	/**
	 * @brief Interpolate between key frames.
	 * @param keyframe The keyframe interval.
	 * @param frequence The frequency of interpolation.
	 */
	void interpolate_between_key_frames(double keyframe, double frequence);

	/**
	 * @brief Check if the trajectory is finalized.
	 * @return True if finalized, false otherwise.
	 */
	bool is_finialized() const;

	/**
	 * @brief Update the bounding boxes of interpolated poses.
	 */
	void update_interpolated_pose_bounding_boxes();

	/**
	 * @brief Merge another trajectory into this one based on time.
	 * @param t The other trajectory.
	 * @param distTh The distance threshold.
	 * @return True if merged successfully, false otherwise.
	 */
	bool merge_time(const Trajectory &t, double distTh = DBL_MAX);

	/**
	 * @brief Remove duplicate poses based on frequency.
	 * @param frequence The frequency to check for duplicates.
	 */
	void remove_duplicate(double frequence);

	/**
	 * @brief Validate the trajectory based on distance and curvature.
	 * @param dist The distance threshold.
	 * @param curv The curvature threshold.
	 * @return A vector of invalid pose indices.
	 */
	std::vector<int> valid(double dist, double curv);

	inline Trajectory& operator=(const Trajectory &t) {
		if (this == &t) {
			return *this;
		}
		poseData = t.poseData;
		id = t.id;
		classId=t.classId;
		return *this;
	}
	inline Trajectory(const Trajectory &t) {
		*this=t;
	}
	inline Pose& operator[](std::size_t idx) {
		return poseData[idx];
	}
	inline const Pose& operator[](std::size_t idx) const {
		return poseData[idx];
	}
};

} /* namespace ikg */

#endif /* INCLUDE_MODEL_GEOMETRIES_TRAJECTORY_H_ */
