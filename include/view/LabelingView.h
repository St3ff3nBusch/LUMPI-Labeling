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

#ifndef SRC_VIEW_LABELINGVIEW_H_
#define SRC_VIEW_LABELINGVIEW_H_
#include <QtWidgets>
#include <QObject>
#if VTK_MAJOR_VERSION >= 9
#include <QVTKOpenGLNativeWidget.h>
#else
#include <QVTKWidget.h>
#endif

namespace ikg {

/**
 * @class LabelingView
 * @brief The LabelingView class provides the GUI for labeling operations.
 * 
 * This class contains the GUI elements and their functionalities for labeling operations.
 * It includes tabs for different views, tables for operations, and various widgets for user interaction.
 */
class LabelingView: public QWidget {
public:
	/**
	 * @brief Constructor for LabelingView.
	 * 
	 * Initializes the GUI elements and sets up the layout.
	 */
	LabelingView();

	/**
	 * @brief Destructor for LabelingView.
	 */
	virtual ~LabelingView();

	/**
	 * @brief Fills the operation table with widgets.
	 * 
	 * This method sets up the operation table with various widgets for different labeling operations.
	 */
	void fill_operation_table();

	/**
	 * @brief Generates the buttons used in the GUI.
	 * 
	 * This method creates and initializes the buttons used for various operations in the GUI.
	 */
	void generate_buttons();

	/**
	 * @brief Generates the spin boxes used in the GUI.
	 * 
	 * This method creates and initializes the spin boxes used for setting dimensions and point sizes.
	 */
	void generate_spin_boxes();

	/**
	 * @brief Sets tool tips for the GUI elements.
	 * 
	 * This method sets descriptive tool tips for various GUI elements to assist the user.
	 */
	void set_tool_tips();

	/**
	 * @brief Tabs of labeling GUI.
	 */
	QWidget *controll, *view, *globView, *camView;

	/**
	 * @brief Labels for displaying trajectories and camera views.
	 */
	QLabel *trajectoryView, *camViewLabel;
	QLabel *singleTrajectoryView, *globalTrajectoryView;

	/**
	 * @brief Tables for operational views and operations.
	 */
	QTableWidget *tableViews, *tableOperations;

	/**
	 * @brief Tab widget for different views.
	 */
	QTabWidget *tabs;

	/**
	 * @brief Combo boxes for selecting point cloud index, object by ID, step size, object class, and camera perspective.
	 */
	QComboBox *comboObjectIds, *comboPcIndex, *comboGlobalCamId, *comboStepSize, *comboObjectClass;

	/**
	 * @brief Buttons for various operations.
	 */
	QPushButton *buttonNextIndex, *buttonPrevIndex, *buttonDelete, *buttonAdd, *buttonMerge, *buttonSplit, *buttonDuplicate, *buttonSave, *buttonLoad, *buttonInterpolate;
	QPushButton *buttonFinalizeObj, *buttonDefinalizeObj, *buttonRenderGlob, *buttonGenerateModel;
	QPushButton *buttonSetStanding, *buttonGenerateVideo;

	/**
	 * @brief Check boxes for activating background subtraction and point cloud coloring by class ID.
	 */
	QCheckBox *checkRemoveBackground, *checkColorObj;

	/**
	 * @brief Spin boxes for changing bounding box dimensions and point size.
	 */
	QDoubleSpinBox *spinLength, *spinWidth, *spinHeight, *spinPointSize;

	/**
	 * @brief Point cloud renderers.
	 */
#if VTK_MAJOR_VERSION >= 9
	QVTKOpenGLNativeWidget *globalQvtk, *modelQvtk;
#else
	QVTKWidget *globalQvtk, *modelQvtk;
#endif
};

} /* namespace ikg */

#endif /* SRC_VIEW_LABELINGVIEW_H_ */
