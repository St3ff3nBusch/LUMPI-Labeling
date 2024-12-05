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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Steffen Busch
 * Email: steffen.busch@ikg.uni-hannover.de
 * Orcid: 0000-0002-5607-9040
 * Date:  2024-11-29
 */

#include "view/LabelingView.h"

namespace ikg {

void LabelingView::fill_operation_table() {
	tableOperations = new QTableWidget;
	tableOperations->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	QStringList hLabels;
	hLabels << "Actions"<<"Actions";
	QStringList vLabels;
	vLabels << "index|\nstep" << "objId\n/camId" << "prev/\nnext" << "length|\nclass" << "width|\npcSize" << "height" << "remBG|\ncolor" << "" << "" << "" << "" << "" << "" << "";
	tableOperations->setRowCount(14);
	tableOperations->setColumnCount(2);
	tableOperations->setHorizontalHeaderLabels(hLabels);
	tableOperations->setVerticalHeaderLabels(vLabels);
	for(int i=0;i<tableOperations->rowCount();i++){
	tableOperations->setRowHeight(i, 50);
	}
	tableOperations->setCellWidget(0, 0, comboPcIndex);
	tableOperations->setCellWidget(0, 1, comboStepSize);
	tableOperations->setCellWidget(1, 0, comboObjectIds);
	tableOperations->setCellWidget(1, 1, comboGlobalCamId);
	tableOperations->setCellWidget(2, 0, buttonPrevIndex);
	tableOperations->setCellWidget(2, 1, buttonNextIndex);
	tableOperations->setCellWidget(3, 0, spinLength);
	tableOperations->setCellWidget(3, 1, comboObjectClass);
	tableOperations->setCellWidget(4, 0, spinWidth);
	tableOperations->setCellWidget(4, 1, spinPointSize);
	tableOperations->setCellWidget(5, 0, spinHeight);
	tableOperations->setCellWidget(6, 0, checkRemoveBackground);
	tableOperations->setCellWidget(6, 1, checkColorObj);
	tableOperations->setCellWidget(7, 0, buttonLoad);
	tableOperations->setCellWidget(7, 1, buttonSave);
	tableOperations->setCellWidget(8, 0, buttonAdd);
	tableOperations->setCellWidget(8, 1, buttonDelete);
	tableOperations->setCellWidget(9, 0, buttonMerge);
	tableOperations->setCellWidget(9, 1, buttonSplit);
	tableOperations->setCellWidget(10, 0, buttonDuplicate);
	tableOperations->setCellWidget(10, 1, buttonInterpolate);
	tableOperations->setCellWidget(11, 0, buttonFinalizeObj);
	tableOperations->setCellWidget(11, 1, buttonDefinalizeObj);
	tableOperations->setCellWidget(12, 0, buttonSetStanding);
	tableOperations->setCellWidget(12, 1, buttonGenerateModel);
	tableOperations->setCellWidget(13, 0, buttonRenderGlob);
	tableOperations->setCellWidget(13, 1, buttonGenerateVideo);
}

void LabelingView::generate_buttons() {
	buttonNextIndex = new QPushButton(">>");
	buttonPrevIndex = new QPushButton("<<");
	buttonDelete = new QPushButton("delete obj");
	buttonAdd = new QPushButton("add obj");
	buttonInterpolate = new QPushButton("interpolate");
	buttonGenerateVideo = new QPushButton("gen video");
	buttonMerge = new QPushButton("merge");
	buttonSplit = new QPushButton("split");
	buttonDuplicate = new QPushButton("duplicate");
	buttonSave = new QPushButton("save");
	buttonLoad = new QPushButton("load");
	buttonFinalizeObj = new QPushButton("finalize");
	buttonDefinalizeObj = new QPushButton("definalize");
	buttonRenderGlob = new QPushButton("render");
	buttonSetStanding = new QPushButton("set standing");
	buttonGenerateModel = new QPushButton("gen model");

}

void LabelingView::generate_spin_boxes() {
	spinLength = new QDoubleSpinBox;
	spinWidth = new QDoubleSpinBox;
	spinHeight = new QDoubleSpinBox;
	spinPointSize = new QDoubleSpinBox();
	spinPointSize->setSingleStep(0.1);
	spinPointSize->setMinimum(0.1);
	spinLength->setSingleStep(0.01);
	spinLength->setRange(0.01, 500);
	spinWidth->setRange(0.01, 500);
	spinHeight->setRange(0.01, 500);
	spinWidth->setSingleStep(0.01);
	spinHeight->setSingleStep(0.01);
}

LabelingView::LabelingView() {
	trajectoryView = new QLabel("");
	singleTrajectoryView = new QLabel;
	tabs = new QTabWidget;
	view = new QWidget;
	controll = new QWidget;
	globView = new QWidget;
	camView = new QWidget;
	QGridLayout *camLayout = new QGridLayout;
#if VTK_MAJOR_VERSION >= 9
	modelQvtk = new QVTKOpenGLNativeWidget(this);
	globalQvtk = new QVTKOpenGLNativeWidget(this);
#else
	modelQvtk = new QVTKWidget(this);
	globalQvtk = new QVTKWidget(this);
#endif
	comboObjectIds = new QComboBox();
	comboPcIndex = new QComboBox();
	comboStepSize = new QComboBox();
	comboGlobalCamId=new QComboBox();
	comboObjectClass = new QComboBox;
	tableViews = new QTableWidget;
	tableViews->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	QGridLayout *layout = new QGridLayout;
	tabs->addTab(globView, "Global");
	tabs->addTab(tableViews, "Operation");
	tabs->addTab(singleTrajectoryView, "Trajectory");
	tabs->addTab(camView, "Camera");
	tabs->addTab(modelQvtk, "Model");
	tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	generate_spin_boxes();
	generate_buttons();
	checkColorObj = new QCheckBox();
	checkRemoveBackground = new QCheckBox();
	fill_operation_table();
	tabs->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	layout->addWidget(tabs, 0, 0, 2, 6);
	layout->addWidget(tableOperations, 0, 6, 2, 1);
	QGridLayout *layoutGlob = new QGridLayout;
	globalQvtk->setMinimumHeight(300);
	globalTrajectoryView = new QLabel;
	layoutGlob->addWidget(globalQvtk, 0, 0, 1, 2);
	layoutGlob->addWidget(globalTrajectoryView, 2, 0, 1, 1);
	globView->setLayout(layoutGlob);
	camViewLabel = new QLabel;
	camLayout->addWidget(camViewLabel, 0, 0, 1, 1);
	camView->setLayout(camLayout);
	this->setLayout(layout);
	this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void LabelingView::set_tool_tips() {
	for(int i=0;i<tableViews->columnCount();i++){
		tableViews->cellWidget(0, i)->setToolTip("Point cloud view");
		tableViews->cellWidget(1, i)->setToolTip("Choose camera id");
		tableViews->cellWidget(2, i)->setToolTip("Change heading");
		tableViews->cellWidget(3, i)->setToolTip("Camera view, use mouse wheel to zoom");
		tableViews->cellWidget(4, i)->setToolTip("Birdseye view, use mouse wheel to zoom, left click to translate and ctrl+left click to rotate");
		tableViews->cellWidget(5, i)->setToolTip("Side view, use mouse wheel to zoom and left click to translate");
		tableViews->cellWidget(6, i)->setToolTip("Back view, use mouse wheel to zoom and left click to translate");

	}
	globalQvtk->setToolTip("Shift+ left click for obj selection");
	globalTrajectoryView->setToolTip("Left click obj/time choose\n mouse weel fot zoom \n right click to move view");
	singleTrajectoryView->setToolTip("Left click to chose point cloud index\n mouse wheel to zoom");
	camViewLabel->setToolTip("Left click to chose an object");
	buttonLoad->setToolTip("Load a label file");
	buttonSave->setToolTip("Save labels to a file");
	buttonAdd->setToolTip("Create a new object, by picking a point from the global point cloud viewer");
	buttonDelete->setToolTip("Delete the trajectory of an object");
	buttonMerge->setToolTip("Select two object to  merge by id, reference object track will be extends about the poses of the other track");
	buttonSplit->setToolTip("Split a track int to two seperate one at a specific time ");
	buttonDuplicate->setToolTip("Duplicate an track, the dublicated one will be shifted in different direction to follow the origin track");
	buttonInterpolate->setToolTip("Interpolate the track of the active object with the choosen step size");
	buttonFinalizeObj->setToolTip("Finalize the active track, the poses can't move,deleted, interpolated, split or merged accidentily");
	buttonDefinalizeObj->setToolTip("Definalize the active track, all poses could be changed again");
	buttonSetStanding->setToolTip("Define a standing intervall for the track, all poses of this intervall are set to the intervall start pose");
	buttonGenerateModel->setToolTip("Aggregate all points of a track, for an intervall and step size to generate 3D convex hull and display the object mask");
	buttonRenderGlob->setToolTip("Render global point cloud vire to a file");
	buttonGenerateVideo->setToolTip("Save a video from the active camera perspective for an given intervall and frame rate");
	comboPcIndex->setToolTip("Chose a point cloud index");
	comboStepSize->setToolTip("Chose a step size");
	comboObjectIds->setToolTip("Chose an obejct by id, point cloud will be set to first occurence if it is not yet occured");
	comboObjectClass->setToolTip("Chose the class of the actual object");
	buttonNextIndex->setToolTip("Increment the point cloud index with choosen step size");
	buttonPrevIndex->setToolTip("Decrement the point cloud index with choosen step size");
	spinHeight->setToolTip("Set actual Obejct bounding box height");
	spinWidth->setToolTip("Set actual Obejct bounding box width");
	spinLength->setToolTip("Set actual Obejct bounding box length");
	spinPointSize->setToolTip("Set the point size for visualization");
	checkRemoveBackground->setToolTip("Activate background subtraction");
	checkColorObj->setToolTip("Activate point cloud coloring");
}
LabelingView::~LabelingView() {
	// TODO Auto-generated destructor stub
}

} /* namespace ikg */
