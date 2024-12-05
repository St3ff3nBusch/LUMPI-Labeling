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

#ifndef SRC_MODEL_DETECTION_LABELING_H_
#define SRC_MODEL_DETECTION_LABELING_H_
#include "opencv2/opencv.hpp"
#include "StructsAndMore.h"
#include "model/geometries/BoundingBox.h"
//#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include "vtkRenderWindow.h"
//#include <QtGui>
#include <QObject>
//#include <QtWidgets>
//#include <QtCore>

#include "Parser.h"
namespace ikg {
/**
 * @enum SplitOption
 * @brief The split options used to distinguish between different splitting modes.
 */
enum SplitOption {
	LEFT, RIGHT, FRONT, BEHIND, SPLITOBJECT
};
/**
 * @class Labeling
 * @brief The Labeling class provides functionalities for labeling and processing point cloud data.
 */
class Labeling: public QObject {

	Q_OBJECT
	signals:
	/**
	 * @brief Signal to update the point cloud view.
	 */
	void update_point_cloud_view();
public slots:
	/**
	 * @brief Merge track id2 into track id1 by adding the poses of id2 which are later than the last pose or earlier than the first pose of id1.
	 * @param[in] id1 ID of the first track.
	 * @param[in] id2 ID of the second track, which will be deleted if the merging is successful.
	 * @return True if the merge was successful, false otherwise.
	 */
	bool merge_track(int id1,int id2);
	/**
	 * @brief Splits a track into two simultaneously existing objects.
	 * @param[in] id1 ID of the track to split.
	 * @param[in] w Width factor for splitting.
	 * @param[in] option Split option to determine the splitting mode.
	 * @return True if the split was successful, false otherwise.
	 */
	bool split_track(int id1, double w, SplitOption option);
	/**
	 * @brief Split a track at the given time into two successively occurring tracks.
	 * @param[in] time Time at which to split the track.
	 * @return True if the split was successful, false otherwise.
	 */
	bool split_track_at_time(double time);
	/**
	 * @brief Generate a back view of an object by transforming 3D points to the yz-plane of the object coordinate system.
	 * Points that occlude the object are removed by a bounding box filter.
	 * All points are colored according to their timestamp.
	 * @param[in] index Point cloud index.
	 * @param[in] objId ID of the object.
	 * @return The generated back view image.
	 */
	cv::Mat generate_back_view(int index, int objId);
	/**
	 * @brief Generate a camera view of an object by transforming a 3D bounding box into a camera image and cropping the image around this box.
	 * 3D points within the bounding box are also drawn into the camera image as green circles.
	 * @param[in] index Point cloud index.
	 * @param[in] objId ID of the object.
	 * @param[in] camId ID of the camera.
	 * @return The generated camera view image.
	 */
	cv::Mat generate_camera_view(int index, int objId, int camId);
	/**
	 * @brief Generate a side view of an object by transforming 3D points to the xz-plane of the object coordinate system.
	 * Points that occlude the object are removed by a bounding box filter.
	 * All points are colored according to their timestamp.
	 * @param[in] index Point cloud index.
	 * @param[in] objId ID of the object.
	 * @return The generated side view image.
	 */
	cv::Mat generate_side_view(int index, int objId);
	/**
	 * @brief Generate a bird's eye view of a trajectory, drawing positions as circles and headings with arrows.
	 * @param[in] objId ID of the object.
	 * @return The generated single trajectory view image.
	 */
	cv::Mat generate_single_trajectory_view(int objId);
	/**
	 * @brief Generate a top view of an object by transforming 3D points to the xy-plane of the object coordinate system.
	 * Points that occlude the object are removed by a bounding box filter.
	 * All points are colored according to their timestamp, and bounding boxes from other objects are also drawn.
	 * @param[in] index Point cloud index.
	 * @param[in] objId ID of the object.
	 * @return The generated top view image.
	 */
	cv::Mat generate_top_view(int index, int objId);
	/**
	 * @brief Interpolate the object's pose between the startTime and endTime.
	 * @param[in] objId ID of the object.
	 * @param[in] startTime Start time for interpolation (default is -DBL_MAX).
	 * @param[in] endTime End time for interpolation (default is DBL_MAX).
	 */
	void interpolate(int objId, double startTime = -DBL_MAX, double endTime = DBL_MAX);
	/**
	 * @brief Extrapolate a track for the object with the given objectId up to time toTime.
	 * The speed is calculated from the last two or first two poses, or the speed is set to zero if the standingFlag is set.
	 * @param[in] objId ID of the object.
	 * @param[in] toTime Time up to which to extrapolate.
	 * @param[in] standingFlag Flag to indicate if the object is standing (default is false).
	 */
	void extrapolate(int objId, double toTime, bool standingFlag = false);
public:
	/** @brief Stores all class name options. */
	std::vector<std::string> classNames;
	/** @brief Substring used for measurement identification. */
	std::string measurementSubString;
	/** @brief Stores the convex hull of aggregated point clouds. */
	std::unordered_map<int, std::vector<IKGB>> models;
	/** @brief Defines the size of GUI elements (columns). */
	int columViewSize;
	/** @brief Defines the size of GUI elements (rows). */
	int rowViewSize;
	/** @brief Defines the measurement ID for data path generation and metadata loading. */
	int measurementId;
	/** @brief Stores the active object ID. */
	int activObjId;
	/** @brief Stores the actual point cloud index. */
	int actualIndex;
	/** @brief Stores the global camera ID. */
	int globalCameraId;
	/** @brief Path to the point cloud data. */
	std::string pointCloudPath;
	/** @brief Path to the camera data. */
	std::string cameraPath;
	/** @brief Path to the metadata. */
	std::string metaPath;
	/** @brief Stores camera parameters and video paths. */
	std::map<int, Sensor> cams;
	/** @brief Stores the selected camera ID for each operational row. */
	std::map<int, int> selectedCam;
	/** @brief Stores the visible point clouds. */
	std::map<int, pcl::PointCloud<IKGB>> clouds;
	/** @brief Stores a temporal cloud with its index for efficient access. */
	std::pair<int, pcl::PointCloud<IKGB>> tmpCloud;
	/** @brief Access the videos. */
	std::unordered_map<int, cv::VideoCapture> caps;
	/** @brief Stores the single camera views per operation row. */
	std::unordered_map<int, std::map<double, cv::Mat>> camViews;
	/** @brief Stores single point cloud views per operational row. */
	std::map<int, pcl::visualization::PCLVisualizer::Ptr> pcViews;
	/** @brief Renders the global point cloud at the actual index. */
	pcl::visualization::PCLVisualizer::Ptr globPcView;
	/** @brief Renders the point of the convex hull for the aggregated points of a track. */
	pcl::visualization::PCLVisualizer::Ptr modelPcView;
	/** @brief Path to the data. */
	std::string dataPath;
	/** @brief Path to the trajectory data. */
	std::string trajectoryPath;
	/** @brief Stores the 2D views per operation row (top view). */
	std::map<int, cv::Mat> topView;
	/** @brief Stores the 2D views per operation row (side view). */
	std::map<int, cv::Mat> sideView;
	/** @brief Stores the 2D views per operation row (back view). */
	std::map<int, cv::Mat> backView;
	/** @brief Stores the 2D views per operation row (camera view). */
	std::map<int, cv::Mat> camView;
	/** @brief Maps poses to point cloud indices. */
	std::map<int, std::unordered_map<int, Pose*>> posePerTime;
	/** @brief Maps poses to object IDs. */
	std::unordered_map<int, std::map<int, Pose*>> posePerObj;
	/** @brief Maps object IDs to trajectories. */
	std::map<int, Trajectory*> id2traj;
	/** @brief Stores interpolation flags. */
	std::unordered_map<int, bool> inter;
	/** @brief Stores all step size options. */
	std::set<int> stepSizes;
	/** @brief Stores the 2D center of the trajectory view for choosing trajectories. */
	cv::Point2d globalTrajectoryCenter;
	/** @brief Stores the chosen step size. */
	int actualStepSize;
	/** @brief The track database all maps refer to these tracks. */
	std::vector<Trajectory> pcTrajectories;
	/** @brief Stores all possible point cloud indices. */
	std::set<int> pointCloudIndices;
	/** @brief Stores colors for different objects. */
	std::unordered_map<int, cv::Vec3b> colors;
	/** @brief Defines how many operation columns should be generated (2*slidingWindow+1). */
	int slidingWindow;
	/** @brief Shifts the single trajectory view to the trajectory start. */
	cv::Point2d singleTrajectoryStart;
	/** @brief Stores zoom information for different views (top view). */
	double zoomTop;
	/** @brief Stores zoom information for different views (camera view). */
	double zoomCam;
	/** @brief Stores zoom information for different views (side view). */
	double zoomSide;
	/** @brief Stores zoom information for different views (back view). */
	double zoomBack;
	/** @brief Stores resolution information for different views. */
	double resolution;
	/**@brief Stores resolution information for the single trajectory view. */
	double singleTrajectoryResolution;
	/** @brief Stores resolution information for the global trajectory view. */
	double globalTrajectoryResolution;
	/** @brief Defines the size of the single trajectory view (width). */
	int singleTrajectoryWidth;
	/** @brief Defines the size of the single trajectory view (height). */
	int singleTrajectoryHeight;
	/** @brief Defines the size of the global trajectory view (width). */
	int globalTrajectoryWidth;
	/** @brief Defines the size of the global trajectory view (height). */
	int globalTrajectoryHeight;
	/** @brief Stores point size information. */
	double pointSize;
	/** @brief Stores the point cloud frequency. */
	double pointCloudFrequenz;
	/** @brief Flag to indicate if objects are being added. */
	bool addingFlag;
	/** @brief Flag to indicate if the background should be removed. */
	bool removeBackground;
	/** @brief Flag to indicate if objects should be colored. */
	bool flagColorObj;
	/** @brief Flag to indicate if the model should be shown. */
	bool flagShowModel;
	/** @brief Stores background information. */
	std::unordered_map<int, cv::Mat> background;
	/** @brief Stores background meta information. */
	std::unordered_map<int, double> backgroundAngleNormalizer;
	/** @brief Stores the center of objects in the camera view. */
	std::unordered_map<int, cv::Rect2d> camViewObjCenter;
	/**
	 * @brief Constructor for the Labeling class.
	 */
	Labeling();
	/**
	 * @brief Destructor for the Labeling class.
	 */
	virtual ~Labeling();
	/**
	 * @brief Add a new 1m x 1m x 1m bounding box at the specified position.
	 * @param[in] position Position to add the bounding box.
	 */
	void add_obj(cv::Point3d position);
	/**
	 * @brief Aggregate all points within the object bounding box for a single track starting at startIndex up to endIndex by incrementing point cloud index with stepSize.
	 * @param[in] tId ID of the track.
	 * @param[in] sIndex Start index.
	 * @param[in] eIndex End index.
	 * @param[in] resolution Resolution for aggregation.
	 * @param[in] stepSize Step size for aggregation.
	 */
	void aggregate_points(const int &tId, const int &sIndex, const int &eIndex, const double &resolution, const int &stepSize);
	/**
	 * @brief Unset the finalize flag at each pose for the specified object.
	 * @param[in] objId ID of the object.
	 */
	void definalized(int objId);
	/**
	 * @brief Erase the object with the specified ID.
	 * @param[in] objId ID of the object.
	 * @return True if the object was successfully erased, false otherwise.
	 */
	bool erase_obj(int objId);
	/**
	 * @brief Erase poses of an object between startTime and endTime. If startTime is smaller than endTime, only the pose at startTime is deleted.
	 * @param[in] objId ID of the object.
	 * @param[in] startTime Start time to delete poses from.
	 * @param[in] endTime End time to delete poses up to (default is -1).
	 * @return True if the poses were successfully erased, false otherwise.
	 */
	bool erase_pose(int objId, double startTime, double endTime = -1);
	/**
	 * @brief Set the finalize flag at each pose for the specified object.
	 * @param[in] objId ID of the object.
	 */
	void finalized(int objId);
	/**
	 * @brief Render objects and their masks at a point cloud index to the corresponding camera frame.
	 * Masks are generated for each object if its model exists by calculating its convex hull.
	 * @param[in] time Time for generating the view.
	 * @param[in] camId ID of the camera.
	 * @return The generated camera global view image.
	 */
	cv::Mat generate_camera_global_view(double time, int camId);
	/**
	 * @brief Return the pose of the active object at the specified point cloud index.
	 * @param[in] pointCloudIndex Point cloud index.
	 * @param[out] pose Pointer to the pose of the active object.
	 * @return True if the pose was successfully retrieved, false otherwise.
	 */
	bool get_active_bb_at_index(int pointCloudIndex, Pose *&pose);
	/**
	 * @brief Get a point cloud at the specified index. If not yet loaded, load the point cloud from disk.
	 * @param[in] index Point cloud index.
	 * @param[in] recolor Flag to indicate if the point cloud should be recolored (default is false).
	 * @return Pointer to the point cloud.
	 */
	pcl::PointCloud<IKGB>* get_cloud(int index, bool recolor = false);
	/**
	 * @brief Get all IDs of objects whose bounding boxes intersect with the specified object ID at a given time.
	 * @param[in] objId ID of the object.
	 * @param[in] time Time for checking intersections.
	 * @return Vector of intersecting object IDs.
	 */
	std::vector<int> get_intersecting_obj(int objId, double time);
	/**
	 * @brief Select the nearest object pose to a given 2D point.
	 * The selection can be constrained by time and/or the active object ID.
	 * @param[in] p 2D point.
	 * @param[in] startTime Start time for selection.
	 * @param[in] endTime End time for selection.
	 * @param[in] activeObjOnly Flag to indicate if only the active object should be considered.
	 * @return Pair of object ID and time.
	 */
	std::pair<int, double> get_obj_at(cv::Point2d p, double startTime, double endTime, bool activeObjOnly)const;
	/**
	 * @brief Select the object from the objects at the specified point cloud index that is closest to the given 3D point.
	 * @param[in] p 3D point.
	 * @param[in] index Point cloud index.
	 * @return Pair of object ID and distance.
	 */
	std::pair<int, double> get_obj_at(cv::Point3d p, int index)const;
	/**
	 * @brief Get all points within an object's bounding box. Returns an empty vector if the object doesn't exist at the specified point cloud index.
	 * @param[in] index Point cloud index.
	 * @param[in] objId ID of the object.
	 * @return Vector of points within the object's bounding box.
	 */
	std::vector<IKGB> get_points_at_obj(int index, int objId);
	/**
	 * @brief Initialize the labeling session by loading the sensor calibration data from the meta file, setting the data path, and loading objects from a file.
	 * @param[in] dataPath Path to the data.
	 * @param[in] trajectoryPath Path to the trajectory data.
	 * @param[in] expId Experiment ID.
	 */
	void init(std::string dataPath, std::string trajectoryPath, int expId);
	/**
	 * @brief Interpolate all poses for the actual object between key frames (defined by step size).
	 */
	void interpolate_between_key_frames();
	/**
	 * @brief Interpolates between key frames (defined by actual step size) around a time stamp.
	 * @param[in] objId ID of the object.
	 * @param[in] time Time in seconds to interpolate based on the previous and next key frame.
	 */
	void interpolate_single_between_key_frames(int objId, double time);
	/**
	 * @brief Load the background into member variables.
	 */
	void load_back_ground();
	/**
	 * @brief Load the camera parameters and the video streams.
	 */
	void load_camera_parameters_and_videos();
	/**
	 * @brief Load point clouds from i-slidingWindow*actualStepSize to i+slidingWindow*actualStepSize.
	 * @param[in] i Point cloud index.
	 */
	void load_point_cloud_sliding_window();
	/**
	 * @brief Load object tracks.
	 * @param[in] path Path to the trajectory data.
	 */
	void load_trajectories(std::string path);
	/**
	 * @brief Reset all point cloud views by removing all point clouds and setting the background to white color.
	 */
	void reset_pc_views();
	/**
	 * @brief Set the point cloud in the map at view to the point cloud with the specified index.
	 * @param[in] index Point cloud index.
	 * @param[in] view Position in the map rendered in the corresponding view.
	 */
	void set_point_cloud(int index, int view);
	/**
	 * @brief Set all poses between the previous and next key frame to the pose of the previous key frame.
	 * @param[in] objId ID of the object.
	 * @param[in] time Time in seconds.
	 */
	void set_standing_pose_by_key_frames(int objId, double time);
	/**
	 * @brief Gives the biggest subset of points with homogeneous timestamps with a time difference smaller than the threshold.
	 * @param[in] points Vector of points.
	 * @param[in] threshold Time difference threshold.
	 * @return Vector of points in the largest homogeneous subset.
	 */
	std::vector<IKGB> split_point_segment(std::vector<IKGB> &points, double threshold);
	/**
	 * @brief Update the global point cloud view: visualizing the actual point cloud and all bounding boxes at this point cloud index.
	 * @return The generated global point cloud view image.
	 */
	cv::Mat generate_global_point_cloud_view();
	/**
	 * @brief Render the active object to the model view, if the aggregated point cloud grid exists.
	 */
	void generate_model_view();
	/**
	 * @brief Update all pose pointers.
	 */
	void update_obj_ptr();
	/**
	 * @brief Render all bounding boxes at the specified point cloud index.
	 * @param[in] index Point cloud index.
	 */
	void update_pc_bb(int index);
	/**
	 * @brief Update point cloud indices to match the file list.
	 */
	void update_pc_index_list();
	/**
	 * @brief Set the background color of bird's eye, side, and back view, depending on the pose state.
	 * Default color is white, finalized color is blue, interpolated color is gray, and standing color is black.
	 * @param[in] po Pointer to the pose.
	 * @param[out] tv Image to set the background color.
	 */
	void set_back_ground_color_local_view(Pose *po, cv::Mat &tv);
	/**
	 * @brief Create a heat map for the times of the given points.
	 * @param[in] pointCloud Vector of points.
	 * @param[out] heat_map Generated heat map.
	 */
	void creat_time_heat_map(const std::vector<IKGB> &pointCloud, cv::Mat &heat_map);
	/**
	 * @brief Return the mean timestamp of the largest point segment.
	 * Segment by sorting the times and splitting segments with a time difference above the threshold.
	 * @param[in] time Vector of times.
	 * @param[in] th Time difference threshold.
	 * @return Mean timestamp of the largest point segment.
	 */
	double get_time_of_largest_point_segment(std::vector<double> &time, double th);
	/**
	 * @brief Transform a 3D grid [x,y,z,count] to a point cloud.
	 * @param[in] grid 3D grid.
	 * @param[in] resolution Resolution for transformation.
	 * @param[out] points Generated point cloud.
	 */
	void grid_to_point_cloud(const std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, int> > > &grid, const double &resolution, std::vector<IKGB> &points);
	/**
	 * @brief Transform a point vector to a xyz cloud.
	 * @param[in] points Vector of points.
	 * @param[out] cloudXYZ Generated xyz cloud.
	 */
	void point_vector_to_xyz_cloud(const std::vector<IKGB> &points, pcl::PointCloud<pcl::PointXYZ> &cloudXYZ);
	/**
	 * @brief Transform an IKGB point cloud to a pcl::XYZRGBA cloud.
	 * @param[in] points IKGB point cloud.
	 * @param[out] tmpC Generated XYZRGBA cloud.
	 */
	void point_cloud_to_XYZRGBA(const pcl::PointCloud<IKGB> &points, pcl::PointCloud<pcl::PointXYZRGBA> &tmpC);
	/**
	 * @brief Get the indices of all points at the convex hull for the given IKGB point vector.
	 * @param[in] points Vector of IKGB points.
	 * @return Vector of indices of points at the convex hull.
	 */
	std::vector<int> get_convex_hull_indices(const std::vector<IKGB> &points);
};

} /* namespace ikg */

#endif /* SRC_MODEL_DETECTION_LABELING_H_ */
