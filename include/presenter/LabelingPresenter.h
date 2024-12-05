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

#ifndef SRC_PRESENTER_LABELINGPRESENTER_H_
#define SRC_PRESENTER_LABELINGPRESENTER_H_

#include <qobject.h>
#include "model/Labeling.h"
#include "view/LabelingView.h"
#include "vtkRenderWindow.h"
#include "PresenterUtils.hpp"

namespace ikg {

/**
 * @class LabelingPresenter
 * @brief Manages the interaction between the Labeling model and the Labeling view.
 */
class LabelingPresenter: public QWidget {
Q_OBJECT //
public slots:
	/**
	 * @brief Adds a new object at the actual point cloud index by picking a point.
	 */
	void add_obj();

	/**
	 * @brief Changes the current camera perspective per sliding window frame.
	 * @param index The index of the camera.
	 * @param camId The ID of the camera.
	 */
	void cam_id_change(int index, int camId);

	/**
	 * @brief Changes the current camera perspective for the global view.
	 * @param index The index of the camera.
	 */
	void cam_global_id_change(const QString &index);

	/**
	 * @brief Triggers background subtraction.
	 * @param check The state of the checkbox.
	 */
	void check_background(int check);

	/**
	 * @brief Triggers point cloud coloring by class ID.
	 * @param check The state of the checkbox.
	 */
	void check_color_by_class_id(int check);

	/**
	 * @brief Changes the class ID of the active object.
	 * @param id The new class ID.
	 */
	void class_id_change(int id);

	/**
	 * @brief Shows the context menu for single frame operations.
	 * @param pos The position where the context menu should appear.
	 */
	void context_menu_view_table(QPoint pos);

	/**
	 * @brief Shifts the bounding box x-, y-coordinates and heading to fit better for all points within the bounding box.
	 */
	void correct_next();

	/**
	 * @brief Sets all positions of the actual object as non-finalized.
	 */
	void definalized_object();

	/**
	 * @brief Deletes all intersecting bounding boxes at the current time index via context menu, skipping finalized ones.
	 */
	void delete_intersecting_bounding_boxes();

	/**
	 * @brief Deletes an object.
	 */
	void delete_obj();

	/**
	 * @brief Deletes a single pose by context menu.
	 */
	void delete_pose();

	/**
	 * @brief Duplicates a track with different shift options.
	 */
	void duplicate();

	/**
	 * @brief Extrapolates a track to the chosen time step via context menu.
	 */
	void extrapolate();

	/**
	 * @brief Extrapolates a track with a static pose to the chosen time step via context menu.
	 */
	void extrapolate_static();

	/**
	 * @brief Updates the combo box for global camera IDs.
	 */
	void fill_global_camera_index();

	/**
	 * @brief Updates the combo box for point cloud indices.
	 */
	void fill_pc_indices();

	/**
	 * @brief Finalizes a single pose by context menu.
	 */
	void finalize_pose();

	/**
	 * @brief Finalizes the complete track for the active object.
	 */
	void finalized_object();

	/**
	 * @brief Generates a convex hull of the object by aggregating points within the track bounding boxes.
	 */
	void generate_model();

	/**
	 * @brief Saves a video for the current camera by choosing start time, end time, and frame rate.
	 */
	void generate_video();

	/**
	 * @brief Adapts the heading spin box, controls circular value, and fixes finalized headings.
	 * @param c The column index.
	 */
	void heading_value_change(int c);

	/**
	 * @brief Interpolates a single pose via context menu.
	 */
	void interpolate();

	/**
	 * @brief Interpolates all positions between key frames for all non-finalized objects.
	 */
	void interpolate_between_key_frames();

	/**
	 * @brief Inverts the current heading.
	 */
	void invert_heading();

	/**
	 * @brief Inverts the headings of the complete trajectory.
	 */
	void invert_all_headings();

	/**
	 * @brief Loads data paths, meta data, and trajectories.
	 */
	void load();

	/**
	 * @brief Marks all positions between a start and end index as standing and sets its pose to the starting pose.
	 */
	void mark_intervall_as_standing();

	/**
	 * @brief Marks a single pose as a standing pose via context menu.
	 */
	void mark_pose_as_standing();

	/**
	 * @brief Merges intersecting objects at the current point cloud index into the active track, keeping active track positions.
	 */
	void merge_intersection();

	/**
	 * @brief Merges tracks by choosing track IDs.
	 */
	void merge_trajectories();

	/**
	 * @brief Shows the next point cloud, incrementing the point cloud with the actual step size.
	 */
	void next_point_cloud();

	/**
	 * @brief Changes the active object.
	 * @param index The index of the new active object.
	 */
	void obj_id_change(const QString &index);

	/**
	 * @brief Updates the view to the point cloud index.
	 * @param index The new point cloud index.
	 */
	void point_cloud_index_change(const QString &index);

	/**
	 * @brief Changes the point size.
	 * @param size The new point size.
	 */
	void point_size_change(double size);

	/**
	 * @brief Shows the previous point cloud, decrementing the point cloud with the actual step size.
	 */
	void previouse_point_cloud();

	/**
	 * @brief Renders the point cloud and bounding box for the actual point cloud index.
	 */
	void render_globView();

	/**
	 * @brief Saves trajectories.
	 */
	void save();

	/**
	 * @brief Splits a track at a time into two successive tracks.
	 */
	void split_at_time();

	/**
	 * @brief Changes the point cloud index step size.
	 * @param index The new step size index.
	 */
	void step_size_change(int index);

	/**
	 * @brief Updates the back view.
	 * @param index The index of the view to update.
	 * @param refresh Whether to refresh the view.
	 */
	void update_back_view(int index, bool refresh = false);

	/**
	 * @brief Updates the bounding box dimensions by value change.
	 */
	void update_bounding_box_dimensions();

	/**
	 * @brief Updates the camera view of the operation cell.
	 * @param index The index of the view to update.
	 * @param refresh Whether to refresh the view.
	 */
	void update_cam_view(int index, bool refresh = false);

	/**
	 * @brief Updates the global view.
	 */
	void update_global();

	/**
	 * @brief Updates the heading for the operation cell.
	 * @param i The index of the operation cell.
	 * @param heading The new heading value.
	 */
	void update_heading(int i, double heading);

	/**
	 * @brief Renders points at the convex hull of aggregated points for the active object.
	 */
	void update_model_view();

	/**
	 * @brief Updates the combo box for object IDs.
	 * @param box The combo box to update.
	 */
	void update_object_ids(QComboBox *box);

	/**
	 * @brief Updates the side view.
	 * @param index The index of the view to update.
	 * @param refresh Whether to refresh the view.
	 */
	void update_side_view(int index, bool refresh = false);

	/**
	 * @brief Updates the single trajectory view.
	 * @param objId The ID of the object to update.
	 */
	void update_single_trajectory_view(int objId);

	/**
	 * @brief Updates the top view.
	 * @param index The index of the view to update.
	 * @param refresh Whether to refresh the view.
	 */
	void update_top_view(int index, bool refresh = false);

	/**
	 * @brief Callback function for point picking.
	 * @param event The point picking event.
	 * @param args Additional arguments.
	 */
	static void point_picking_callback(const pcl::visualization::PointPickingEvent &event, void *args);

public:
	/**
	 * @brief The Labeling model.
	 */
	Labeling model;

	/**
	 * @brief The Labeling view.
	 */
	LabelingView *view;

	/**
	 * @brief Marks the operation cell with the mouse cursor.
	 */
	int activeViewRow, activeViewCol;

	/**
	 * @brief Constructor for LabelingPresenter.
	 * @param view The Labeling view.
	 */
	LabelingPresenter(LabelingView* view);

	/**
	 * @brief Destructor for LabelingPresenter.
	 */
	virtual ~LabelingPresenter();

	/**
	 * @brief Event filter to enable zooming, shifting, and rotation.
	 * @param obj The object to filter events for.
	 * @param event The event to filter.
	 * @return True if the event was handled, false otherwise.
	 */
	bool eventFilter(QObject *obj, QEvent *event);

	/**
	 * @brief Connects all view elements via this presenter function to the model.
	 * @param view The Labeling view.
	 */
	void connect_view_elements(LabelingView *view);

	/**
	 * @brief Initializes all views at loading and GUI resizing.
	 */
	void fill_view();

	/**
	 * @brief Fills the operation table with the sliding window views for the label action.
	 */
	void fill_operation_table();

	/**
	 * @brief Handles camera view events.
	 * @param event The event to handle.
	 * @return True if the event was handled, false otherwise.
	 */
	bool handel_camera_view_event(QEvent *event);

	/**
	 * @brief Handles operation cell events.
	 * @param i The row index of the cell.
	 * @param l The column index of the cell.
	 * @param event The event to handle.
	 * @return True if the event was handled, false otherwise.
	 */
	bool handel_operation_cell_event(int i, int l, QEvent *event);

	/**
	 * @brief Handles global trajectory view events.
	 * @param event The event to handle.
	 * @return True if the event was handled, false otherwise.
	 */
	bool handel_global_trajectory_event(QEvent *event);

	/**
	 * @brief Handles single trajectory view events.
	 * @param event The event to handle.
	 * @return True if the event was handled, false otherwise.
	 */
	bool handel_single_trajectory_event(QEvent *event);

	/**
	 * @brief Updates the global camera view.
	 */
	void update_glob_cam_view();

	/**
	 * @brief Updates a single operation column.
	 * @param i The index of the column to update.
	 * @param refresh Whether to refresh the column.
	 * @return The updated index.
	 */
	int update_view_column(int i, bool refresh = false);

	/**
	 * @brief Updates all operation views.
	 * @param refresh Whether to refresh the views.
	 */
	void update_operation_views(bool refresh = false);

	/**
	 * @brief Sets the zoom factor for the operation view.
	 * @param row The row index of the view.
	 * @param s The zoom factor.
	 * @param index The index of the view.
	 */
	void update_zoom_factor_operation_cell(int row, double s, int index);

	/**
	 * @brief Updates the GUI at object change.
	 */
	void set_box_dimension();

	/**
	 * @brief Sets the class combo box at object change.
	 */
	void set_class_combo_box();

	/**
	 * @brief Sets the model image sizes for view generation.
	 */
	void set_model_view_dimensions();

	/**
	 * @brief Updates the object ID combo box at object change.
	 */
	void set_object_id_combo_box();

	/**
	 * @brief Sets the point cloud index to the first object occurrence at object change.
	 */
	void set_point_cloud_index_to_first_obj_occurence();

	/**
	 * @brief Shifts the pose.
	 * @param t The translation vector.
	 * @param pose The pose to shift.
	 * @param column The column index.
	 */
	void shift_pose(cv::Point2d t, Pose *pose, int column);

	/**
	 * @brief Recolors the sliding window point clouds.
	 */
	void recolor_sliding_window_point_clouds();
};

} /* namespace ikg */

#endif /* SRC_PRESENTER_LABELINGPRESENTER_H_ */
