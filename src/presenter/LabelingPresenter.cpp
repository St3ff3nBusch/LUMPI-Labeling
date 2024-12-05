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

#include "presenter/LabelingPresenter.h"
#include "model/LUMPIPipeline.h"
#include <filesystem>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include<QFormLayout>
#include <QtWidgets>
#include<QtGui>
#include <string>
#include <QLineEdit>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QtCore>
//static std::ofstream flogFile("log_LUMPI_LabelingTool"+".txt");
//#define VERBOSE_PRESENTER(...) std::cout<<"In:"<<__FILENAME__<<"at:"<<__LINE__<<":"<<__VA_ARGS__<<std::endl;
#define VERBOSE_PRESENTER(...) while(false){};
using namespace std;
namespace ikg {
void LabelingPresenter::generate_model() {
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	QDialog dialog(view);
	QFormLayout form(&dialog);
	form.addRow(new QLabel("Options:"));
	QList<QObject*> fields;
	QDoubleSpinBox *gridSize = new QDoubleSpinBox(&dialog);
	gridSize->setValue(0.1);
	gridSize->setRange(0.03, 1);
	int end = floor(model.id2traj.at(model.activObjId)->poseData.back().time * model.pointCloudFrequenz);
	int start = floor(model.id2traj.at(model.activObjId)->poseData.front().time * model.pointCloudFrequenz);
	QSpinBox *startIndex = new QSpinBox(&dialog);
	startIndex->setRange(start, end - 1);
	startIndex->setValue(start);
	QSpinBox *endIndex = new QSpinBox(&dialog);
	endIndex->setRange(start + 1, end);
	endIndex->setValue(end);
	QSpinBox *stepSize = new QSpinBox(&dialog);
	stepSize->setValue(5);
	stepSize->setRange(1, 1000);
	form.addRow(QString("grid resolution"), gridSize);
	form.addRow(QString("start index"), startIndex);
	form.addRow(QString("end index"), endIndex);
	form.addRow(QString("step Size"), stepSize);
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
	if (dialog.exec() == QDialog::Accepted) {
		model.aggregate_points(model.activObjId, startIndex->value(), endIndex->value(), gridSize->value(), stepSize->value());
	}
}
void LabelingPresenter::generate_video() {

	string pathOut;
	ikg::save_file_dialog(pathOut, view, "Save video");
	bool ok = false;
	static int id = 7;
	id = QInputDialog::getInt(view, "Camera Number", QString("Camera:"), id, 0, INT_MAX, 1, &ok);
	if (!ok)
		return;
	static int speed = 4;
	speed = QInputDialog::getInt(view, "Speed", QString("Speed:"), speed, 0, INT_MAX, 1, &ok);
	if (!ok)
		return;
	static int start = -1;
	start = QInputDialog::getInt(view, "Start (-1->first frame)", QString("Start:"), start, 0, INT_MAX, 1, &ok);
	if (!ok)
		return;
	static int end = -1;
	end = QInputDialog::getInt(view, "End (-1->last frame)", QString("End:"), end, 0, INT_MAX, 1, &ok);
	if (!ok)
		return;
	if (model.cams.count(id) < 1) {
		pop_up_msg("Camera with id " + to_string(id) + " is not found");
		return;
	}
	Sensor c = model.cams.at(id);
	cv::Mat tmp = c.extrinsic.inv();
	tmp.copyTo(c.extrinsic);
	LUMPIPipeline::generate_video(model.pcTrajectories, model.cameraPath + "/" + to_string(c.deviceId) + "/video.mp4", pathOut, c, speed, start, end);
	ikg::pop_up_msg("video generated");

}
void LabelingPresenter::update_model_view() {
	VERBOSE_PRESENTER("update_model_view")
	model.generate_model_view();
}
void LabelingPresenter::render_globView() {

	QString fileName2 = QFileDialog::getSaveFileName(this, tr("Save bar diagramm"), "data/untitled.png", tr("Images (*.png)"));
	if (fileName2 == QString::null)
		return;
	string fname = fileName2.toUtf8().constData();
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(model.globPcView->getRenderWindow());
	windowToImageFilter->Update();
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetWriteToMemory(1);
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
	auto rawPngBuffer = writer->GetResult();
	auto rawPointer = rawPngBuffer->GetPointer(0);
	auto total_size = rawPngBuffer->GetDataSize() * rawPngBuffer->GetDataTypeSize();
	std::vector<unsigned char> buffer(rawPointer, rawPointer + total_size);
	cv::Mat mat = cv::imdecode(buffer, 1);
	cv::imwrite(fname, mat);
}

void LabelingPresenter::check_background(int check) {
	model.removeBackground = check == Qt::CheckState::Checked;
	model.clouds.clear();
	if (model.background.empty())
		model.load_back_ground();
	model.load_point_cloud_sliding_window();
	update_operation_views(true);
	update_global();
}
void LabelingPresenter::check_color_by_class_id(int check) {
	model.flagColorObj = check == Qt::CheckState::Checked;
	model.clouds.clear();
	model.load_point_cloud_sliding_window();
	update_operation_views(true);
	update_global();
}
void LabelingPresenter::split_at_time() {
	bool ok;
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	double startTime = model.id2traj.at(model.activObjId)->poseData.front().time;
	double endTime = model.id2traj.at(model.activObjId)->poseData.back().time;
	double t = QInputDialog::getDouble(view, "get time (s) for split", QString("Time:"), min(endTime, max(startTime, model.actualIndex / model.pointCloudFrequenz)), startTime, endTime, 1, &ok);
	if (!ok)
		return;
	for (auto p : model.id2traj.at(model.activObjId)->poseData) {
		if (p.finalized) {
			pop_up_msg("cant split finialized object");
			return;
		}
	}
	model.split_track_at_time(t);
	update_object_ids(view->comboObjectIds);
	update_operation_views(true);
	update_global();
	update_single_trajectory_view(model.activObjId);
}
void LabelingPresenter::duplicate() {
	QDialog dialog(view);
	QFormLayout form(&dialog);
	form.addRow(new QLabel("Options:"));
	QList<QObject*> fields;
	QComboBox *comboOptions = new QComboBox(&dialog);
	comboOptions->addItem("left");
	comboOptions->addItem("right");
	comboOptions->addItem("front");
	comboOptions->addItem("behind");
	comboOptions->addItem("splitObj");
	form.addRow(QString("right"), comboOptions);
	fields << dynamic_cast<QObject*>(comboOptions);
	QDoubleSpinBox *w = new QDoubleSpinBox(&dialog);
	w->setMinimum(0.1);
	w->setSingleStep(0.01);
	form.addRow(QString("Distance"), w);
	fields << dynamic_cast<QObject*>(w);
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
	if (dialog.exec() == QDialog::Accepted) {
		int option = dynamic_cast<QComboBox*>(fields[0])->currentIndex();
		double dist = dynamic_cast<QDoubleSpinBox*>(fields[1])->value();
		if (!model.split_track(model.activObjId, dist, (SplitOption) option)) {
			ikg::pop_up_msg("Cant split obj" + to_string(model.activObjId));
			return;
		}
		obj_id_change(QString(to_string(model.activObjId).c_str()));
		update_operation_views(true);
		update_global();
		update_single_trajectory_view(model.activObjId);
		update_object_ids(view->comboObjectIds);
	}
}
void LabelingPresenter::mark_intervall_as_standing() {
	if (model.posePerObj.count(model.activObjId) < 1)
		return;
	QDialog dialog(view);
	QFormLayout form(&dialog);
	auto &activeTrack = model.posePerObj.at(model.activObjId);
	QSpinBox *spinFrom = new QSpinBox();
	spinFrom->setMinimum(activeTrack.begin()->first);
	spinFrom->setMaximum(activeTrack.rbegin()->first);
	QSpinBox *spinTo = new QSpinBox();
	spinTo->setMinimum(activeTrack.begin()->first);
	spinTo->setMaximum(activeTrack.rbegin()->first);
	int end = floor(model.id2traj.at(model.activObjId)->poseData.back().time * model.pointCloudFrequenz);
	int start = floor(model.id2traj.at(model.activObjId)->poseData.front().time * model.pointCloudFrequenz);
	spinFrom->setRange(start, end - 1);
	spinFrom->setValue(start);
	spinTo->setRange(start + 1, end);
	spinTo->setValue(end);
	form.addRow(new QLabel("Standing poses"));
	form.addRow(QString("From:"), spinFrom);
	form.addRow(QString("to:"), spinTo);
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
	form.addRow(&buttonBox);
	if (dialog.exec() == QDialog::Accepted) {
		int from = spinFrom->value();
		int to = spinTo->value();
		if (to < from) {
			ikg::pop_up_msg("frame from " + to_string(from) + " is smaller as frame to " + to_string(to));
			return;
		}
		auto it = activeTrack.find(from);
		if (it->second->finalized) {
			ikg::pop_up_msg("object" + to_string(model.activObjId) + "is finialized");
			return;
		}
		if (it == activeTrack.end()) {
			ikg::pop_up_msg("error cant find frame" + to_string(to));
			return;
		}
		auto p = *it->second;
		while (it->first < to + 1) {
			(it->second)->bb = p.bb;
			it->second->standing = true;
			it->second->interpolated = false;
			it++;
		}
	}
	update_operation_views(true);
}
void LabelingPresenter::merge_trajectories() {
	QDialog dialog(view);
	QFormLayout form(&dialog);
	form.addRow(new QLabel("Reference ID"));
	QList<QComboBox*> fields;
	QComboBox *comboOptions = new QComboBox(&dialog);
	update_object_ids(comboOptions);
	form.addRow(QString("Reference ID"), comboOptions);
	fields << comboOptions;
	QComboBox *mer = new QComboBox(&dialog);
	update_object_ids(mer);
	form.addRow(QString("Merging ID"), mer);
	fields << mer;
	QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
	form.addRow(&buttonBox);
	QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
	QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
	if (dialog.exec() == QDialog::Accepted) {
		int id1 = dynamic_cast<QComboBox*>(fields[0])->currentText().toInt();
		int id2 = dynamic_cast<QComboBox*>(fields[1])->currentText().toInt();
		if (id1 == id2) {
			ikg::pop_up_msg("Cant merge same ID");
			return;
		}
		if (id2 == model.activObjId)
			obj_id_change(QString(to_string(id1).c_str()));
		if (!model.merge_track(id1, id2))
			ikg::pop_up_msg("cant merge finalized model");
		update_object_ids(view->comboObjectIds);
	}

	update_operation_views(true);
	update_global();
	update_single_trajectory_view(model.activObjId);
}
void LabelingPresenter::next_point_cloud() {
	model.actualIndex = model.actualIndex + model.actualStepSize;
	point_cloud_index_change(QString(to_string(model.actualIndex).c_str()));
}

void LabelingPresenter::previouse_point_cloud() {
	model.actualIndex = max(0, model.actualIndex - model.actualStepSize);
	point_cloud_index_change(QString(to_string(model.actualIndex).c_str()));
}
void LabelingPresenter::update_bounding_box_dimensions() {
	if (model.posePerObj.count(model.activObjId) < 1)
		return;
	for (auto &p : model.posePerObj.at(model.activObjId)) {
		p.second->bb.length = view->spinLength->value();
		p.second->bb.width = view->spinWidth->value();
		p.second->bb.height = view->spinHeight->value();
		p.second->bb.update();
	}
	model.get_cloud(model.actualIndex, model.flagColorObj);
	update_operation_views(true);
	update_global();
}

void LabelingPresenter::update_heading(int i, double h) {
	Pose *p;
	if (!model.get_active_bb_at_index(i, p) || p->finalized)
		return;
	p->bb.update_orientation(0, 0, h, false);
	int cIndex = (i - model.actualIndex) / model.actualStepSize + model.slidingWindow;
	QDoubleSpinBox *spinH = dynamic_cast<QDoubleSpinBox*>(view->tableViews->cellWidget(2, cIndex));
	QSignalBlocker block(spinH);
	spinH->setValue(h);
}

void LabelingPresenter::shift_pose(cv::Point2d t, Pose *pose, int row) {
	if (pose->finalized)
		return;
	switch (row) {
	case 4:
		t *= model.resolution / model.zoomTop;
		pose->bb.center.x += pose->bb.ca * t.x - pose->bb.sa * t.y;
		pose->bb.center.y += pose->bb.sa * t.x + pose->bb.ca * t.y;
		break;
	case 5:
		t *= model.resolution / model.zoomSide;
		pose->bb.center.x += pose->bb.ca * t.x;
		pose->bb.center.y += pose->bb.sa * t.x;
		pose->bb.center.z += -t.y;
		break;
	case 6:
		t *= model.resolution / model.zoomSide;
		pose->bb.center.x += -pose->bb.sa * t.x;
		pose->bb.center.y += pose->bb.ca * t.x;
		pose->bb.center.z += -t.y;
		break;
	default:
		break;
	}
	pose->interpolated = false;
	pose->standing = false;
	pose->bb.update();
}

void LabelingPresenter::update_zoom_factor_operation_cell(int row, double s, int index) {
	switch (row) {
	case 3: {
		model.zoomCam = max(0.1, model.zoomCam * s);
		update_cam_view(index, true);
		break;
	}
	case 4: {
		model.zoomTop = max(0.1, model.zoomTop * s);
		update_top_view(index, true);
		break;
	}
	case 5: {
		model.zoomSide = max(0.1, model.zoomSide * s);
		update_side_view(index, true);
		break;
	}
	case 6: {
		model.zoomBack = max(0.1, model.zoomBack * s);
		update_back_view(index, true);
		break;
	}
	default:
		break;
	}
}
bool LabelingPresenter::handel_global_trajectory_event(QEvent *event) {
	auto wheel = static_cast<QWheelEvent*>(event);
	auto click = static_cast<QMouseEvent*>(event);
	static cv::Point2d lastPoint;
	static bool rightButtonPressed = false;
	if (event->type() == QEvent::Wheel) {
		double s = wheel->delta() > 0 ? 1.05 : 0.95;
		model.globalTrajectoryResolution = max(0.0000001, model.globalTrajectoryResolution * s);
		update_global();
	} else if (event->type() == QEvent::MouseButtonPress && click->buttons() == Qt::LeftButton) {
		cv::Point2d c(click->pos().x(), click->pos().y());
		double start = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz;
		double end = start + 2 * model.slidingWindow * model.actualStepSize / model.pointCloudFrequenz;
		auto p = model.get_obj_at(c, start, end, false);
		if (p.first > -1) {
			obj_id_change(QString(to_string(p.first).c_str()));
			int index = (int) (p.second * model.pointCloudFrequenz / model.actualStepSize) * model.actualStepSize;
			point_cloud_index_change(QString(to_string(index).c_str()));
		}
	} else if (event->type() == QEvent::MouseButtonPress && click->buttons() == Qt::RightButton) {
		rightButtonPressed = true;
		lastPoint = cv::Point2d(click->pos().x(), click->pos().y());
	} else if (event->type() == QEvent::MouseMove && rightButtonPressed) {
		QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
		cv::Point2d pos = cv::Point2d(click->pos().x(), click->pos().y());
		cv::Point2d t = pos - lastPoint;
		lastPoint = pos;
		model.globalTrajectoryCenter += t * model.globalTrajectoryResolution;
		update_global();
	}
	if (event->type() == QEvent::MouseButtonRelease) {
		rightButtonPressed = false;
	}
	return true;
}
bool LabelingPresenter::handel_single_trajectory_event(QEvent *event) {
	static cv::Point2d lastPoint;
	static bool rightButtonPressed = false;
	if (event->type() == QEvent::Wheel) {
		auto wheel = static_cast<QWheelEvent*>(event);
		double s = wheel->delta() > 0 ? 1.05 : -0.95;
		model.singleTrajectoryResolution = max(0.005, model.singleTrajectoryResolution * s);
		update_single_trajectory_view(model.activObjId);
	} else if (event->type() == QEvent::MouseButtonPress) { //&&view->globalTrajectoryView->isVisible()
		auto w = static_cast<QMouseEvent*>(event);
		cv::Point2d position(w->pos().x(), w->pos().y());
		VERBOSE_PRESENTER("GET PRESS LEFT"<<position)
		auto tmp3 = view->singleTrajectoryView->pixmap()->rect().topLeft();
		auto tmp = view->singleTrajectoryView->mapFromParent(w->pos());
		auto tmp2 = view->singleTrajectoryView->pos();
		cv::Point2d position2(tmp.x(), tmp.y());
		cv::Point2d position3(tmp2.x(), tmp2.y());
		cv::Point2d position4(tmp3.x(), tmp3.y());
		VERBOSE_PRESENTER("local"<<position2<<":"<<position3<<":"<<position4);
		VERBOSE_PRESENTER(position-position3<<";"<<position-position4)
		int l, t, r, b;
		view->singleTrajectoryView->getContentsMargins(&l, &t, &r, &b);
		VERBOSE_PRESENTER("MARGIN"<<l<<";"<<t<<","<<r<<","<<b)
		VERBOSE_PRESENTER("OBJ ID"<<model.activObjId<<"count"<<model.posePerObj.count(model.activObjId))
		if (model.posePerObj.count(model.activObjId) < 1)
			return false;
		auto p2 = model.get_obj_at(position, model.posePerObj.at(model.activObjId).begin()->first / model.pointCloudFrequenz,
				model.posePerObj.at(model.activObjId).rbegin()->first / model.pointCloudFrequenz, true);
		VERBOSE_PRESENTER(
				"TIME"<<model.posePerObj.at(model.activObjId).begin()->first / model.pointCloudFrequenz << "," << model.posePerObj.at(model.activObjId).rbegin()->first /model.pointCloudFrequenz)
		if (p2.first > -1) {
			int index = (int) (floor(p2.second * model.pointCloudFrequenz) / model.actualStepSize) * model.actualStepSize;
			point_cloud_index_change(QString(to_string((int) (index)).c_str()));
		}
		update_single_trajectory_view(model.activObjId);
	}
	return true;
}

bool LabelingPresenter::handel_camera_view_event(QEvent *event) {
	if (event->type() == QEvent::MouseButtonPress) {
		auto w = static_cast<QMouseEvent*>(event);
		cv::Point2i pos(w->pos().x(), w->pos().y());
		vector<int> ids;
		for (auto p : model.camViewObjCenter) {
			if (p.second.contains(pos)) {
				ids.push_back(p.first);
			}
		}
		if (ids.size() == 1) {
			obj_id_change(QString(to_string(ids.front()).c_str()));
		} else {
			QDialog dialog(view);
			QFormLayout form(&dialog);
			form.addRow(new QLabel("Chose object:"));
			QList<QObject*> fields;
			QComboBox *comboOptions = new QComboBox(&dialog);
			for (int i : ids) {
				comboOptions->addItem(to_string(i).c_str());
			}
			form.addRow(QString("object"), comboOptions);
			fields << dynamic_cast<QObject*>(comboOptions);
			QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
			form.addRow(&buttonBox);
			QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
			QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
			if (dialog.exec() == QDialog::Accepted) {
				auto combo = dynamic_cast<QComboBox*>(fields[0]);
				int oId = combo->itemText(combo->currentIndex()).toInt();
				obj_id_change(QString(to_string(oId).c_str()));
			}
		}
	}
	return false;
}
bool LabelingPresenter::handel_operation_cell_event(int row, int column, QEvent *event) {
	static cv::Point2d lastPosition;
	bool ctrlPressed = QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ControlModifier);
	static bool leftButtonDown = false;
	int index = model.actualIndex + (column - model.slidingWindow) * model.actualStepSize;
	if (event->type() == QEvent::Wheel) {
		auto w = static_cast<QWheelEvent*>(event);
		double s = w->delta() > 0.0 ? 1.05 : 0.96;
		update_zoom_factor_operation_cell(row, s, index);
		update_operation_views(true);
	} else if (event->type() == QEvent::MouseButtonPress) {
		auto w = static_cast<QMouseEvent*>(event);
		lastPosition = cv::Point2d(w->pos().x(), w->pos().y());
		leftButtonDown = true;
	} else if (event->type() == QEvent::MouseMove && leftButtonDown) {
		if (model.posePerObj.count(model.activObjId) < 1 || model.posePerObj.at(model.activObjId).count(index) < 1) { //no object at this poitn cloud index
			return false;
		}
		auto w = static_cast<QMouseEvent*>(event);
		cv::Point2d position(w->pos().x(), w->pos().y());
		Pose *pose = model.posePerObj.at(model.activObjId).at(index);
		cv::Point2d t = position - lastPosition;
		if (!ctrlPressed) { //translate the bounding box
			shift_pose(t, pose, row);
		} else if (ctrlPressed && row == 4) { //rotate the bounding box
			update_heading(index, atan2(position.y - 100, position.x - 100));

		}
		pose->interpolated = false;
		model.id2traj.at(model.activObjId)->update_interpolated_pose_bounding_boxes();
		for (int i = -model.slidingWindow * 2; i < 2 * model.slidingWindow + 1; i++) {
			int tmpIndex = model.actualIndex + i * model.actualStepSize;
			if (model.posePerObj.at(model.activObjId).count(tmpIndex) > 0 && (pose->interpolate || i == 0))
				model.get_cloud(tmpIndex, true);
		}
		lastPosition = position;
		update_operation_views(true);
		update_global();
		update_single_trajectory_view(model.activObjId);
	} else if (event->type() == QEvent::MouseButtonRelease) {
		leftButtonDown = false;
	}
	return false;
}
bool LabelingPresenter::eventFilter(QObject *obj, QEvent *event) {

	if (event->type() == QEvent::Resize && obj == view->tabs) {
		VERBOSE_PRESENTER("HANDLE RESIZE")
		fill_view();
	}
	for (int i = 0; i < view->tableViews->columnCount(); i++) {
		for (int j = 2; j < view->tableViews->rowCount(); j++) {
			if (view->tableViews->cellWidget(j, i) == obj && view->tableViews->cellWidget(j, i)->isVisible()) {
				handel_operation_cell_event(j, i, event);
			}
		}
	}
	if (obj == view->globalTrajectoryView && view->globalTrajectoryView->isVisible()) {
		handel_global_trajectory_event(event);
	} else if (obj == view->singleTrajectoryView && view->singleTrajectoryView->isVisible()) {
		handel_single_trajectory_event(event);
	} else if (obj == view->camViewLabel && view->camViewLabel->isVisible()) {
		handel_camera_view_event(event);
	}
	return QWidget::eventFilter(obj, event);
}

void LabelingPresenter::connect_view_elements(LabelingView *view) {
	QObject::connect(view->comboObjectClass, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LabelingPresenter::class_id_change);
	QObject::connect(view->comboObjectIds, QOverload<const QString&>::of(&QComboBox::currentIndexChanged), this, &LabelingPresenter::obj_id_change);
	QObject::connect(view->comboGlobalCamId, QOverload<const QString&>::of(&QComboBox::currentIndexChanged), this, &LabelingPresenter::cam_global_id_change);
	QObject::connect(view->comboPcIndex, QOverload<const QString&>::of(&QComboBox::currentIndexChanged), this, &LabelingPresenter::point_cloud_index_change);
	QObject::connect(view->comboStepSize, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LabelingPresenter::step_size_change);
	QObject::connect(view->spinLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this]() {
		this->update_bounding_box_dimensions();
	}
	);
	QObject::connect(view->spinWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this]() {
		this->update_bounding_box_dimensions();
	}
	);
	QObject::connect(view->spinHeight, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this]() {
		this->update_bounding_box_dimensions();
	}
	);
	QObject::connect(view->checkRemoveBackground, &QCheckBox::stateChanged, this, &LabelingPresenter::check_background);
	QObject::connect(view->checkColorObj, &QCheckBox::stateChanged, this, &LabelingPresenter::check_color_by_class_id);
	QObject::connect(view->spinPointSize, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LabelingPresenter::point_size_change);
	QObject::connect(view->buttonNextIndex, &QPushButton::clicked, this, &LabelingPresenter::next_point_cloud);
	QObject::connect(view->buttonPrevIndex, &QPushButton::clicked, this, &LabelingPresenter::previouse_point_cloud);
	QObject::connect(view->buttonDelete, &QPushButton::clicked, this, &LabelingPresenter::delete_obj);
	QObject::connect(view->buttonAdd, &QPushButton::clicked, this, &LabelingPresenter::add_obj);
	QObject::connect(view->buttonSave, &QPushButton::clicked, this, &LabelingPresenter::save);
	QObject::connect(view->buttonLoad, &QPushButton::clicked, this, &LabelingPresenter::load);
	QObject::connect(view->buttonDuplicate, &QPushButton::clicked, this, &LabelingPresenter::duplicate);
	QObject::connect(view->buttonSplit, &QPushButton::clicked, this, &LabelingPresenter::split_at_time);
	QObject::connect(view->buttonMerge, &QPushButton::clicked, this, &LabelingPresenter::merge_trajectories);
	QObject::connect(view->buttonInterpolate, &QPushButton::clicked, this, &LabelingPresenter::interpolate_between_key_frames);
	QObject::connect(view->buttonFinalizeObj, &QPushButton::clicked, this, &LabelingPresenter::finalized_object);
	QObject::connect(view->buttonDefinalizeObj, &QPushButton::clicked, this, &LabelingPresenter::definalized_object);
	QObject::connect(view->buttonRenderGlob, &QPushButton::clicked, this, &LabelingPresenter::render_globView);
	QObject::connect(view->buttonGenerateModel, &QPushButton::clicked, this, &LabelingPresenter::generate_model);
	QObject::connect(view->buttonSetStanding, &QPushButton::clicked, this, &LabelingPresenter::mark_intervall_as_standing);
	QObject::connect(view->buttonGenerateVideo, &QPushButton::clicked, this, &LabelingPresenter::generate_video);
}

LabelingPresenter::LabelingPresenter(LabelingView *view) :
		view(view) {
	activeViewRow = 0;
	activeViewCol = 0;
	fill_view();
	view->checkRemoveBackground->setChecked(model.removeBackground);
	view->checkColorObj->setChecked(model.flagColorObj);
	connect_view_elements(view);
	view->tableViews->setContextMenuPolicy(Qt::CustomContextMenu);
	QObject::connect(view->tableViews, &QTableWidget::customContextMenuRequested, this, &LabelingPresenter::context_menu_view_table);
	view->singleTrajectoryView->installEventFilter(this);
	view->globalTrajectoryView->installEventFilter(this);
	view->camViewLabel->installEventFilter(this);
	view->tabs->installEventFilter(this);
}
void LabelingPresenter::fill_global_camera_index() {
	QSignalBlocker block(view->comboGlobalCamId);
	view->comboGlobalCamId->clear();
	for (auto c : model.cams) {
		view->comboGlobalCamId->addItem(QString(to_string(c.first).c_str()));
	}
	view->comboPcIndex->setCurrentIndex(0);
}
void LabelingPresenter::fill_pc_indices() {
	view->comboPcIndex->blockSignals(true);
	view->comboPcIndex->clear();
	if (model.pointCloudIndices.empty())
		return;
	for (auto i : model.pointCloudIndices) {
		view->comboPcIndex->addItem(to_string(i).c_str());
	}
	auto tmp = model.pointCloudIndices.find(model.actualIndex);
	if (tmp == model.pointCloudIndices.end())
		tmp = model.pointCloudIndices.begin();
	view->comboPcIndex->setCurrentIndex(distance(model.pointCloudIndices.begin(), tmp));
	view->comboPcIndex->blockSignals(false);
}
void LabelingPresenter::point_cloud_index_change(const QString &index) {
	VERBOSE_PRESENTER("pc index"<<index.toStdString().c_str())
	int pcIndex = index.toInt();
	pcIndex = floor((pcIndex / (double) model.actualStepSize) * model.actualStepSize);
	model.actualIndex = pcIndex;
	VERBOSE_PRESENTER("pc index"<<model.actualIndex)
	if (!model.pointCloudIndices.empty()) {
		auto tmp = model.pointCloudIndices.find(model.actualIndex);
		if (tmp == model.pointCloudIndices.end())
			tmp = model.pointCloudIndices.begin();
		model.actualIndex = *tmp;
		view->comboPcIndex->blockSignals(true);
		view->comboPcIndex->setCurrentIndex(distance(model.pointCloudIndices.begin(), tmp));
		view->comboPcIndex->blockSignals(false);
	}
	model.load_point_cloud_sliding_window();
	double start = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz;
	double end = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz;
	model.interpolate(model.activObjId, start, end);
	update_operation_views(false);
	update_global();
	update_single_trajectory_view(model.activObjId);
}
void LabelingPresenter::update_object_ids(QComboBox *box) {
	VERBOSE_PRESENTER("update obj ids")
	box->blockSignals(true);

	box->clear();
	if (model.id2traj.empty())
		return;
	for (const auto &t : model.id2traj) {
		box->addItem(to_string(t.first).c_str());
	}
	box->setCurrentIndex(distance(model.id2traj.begin(), model.id2traj.find(model.activObjId)));
	box->blockSignals(false);
}
void LabelingPresenter::step_size_change(int index) {
	model.clouds.clear();
	model.topView.clear();
	model.backView.clear();
	model.sideView.clear();
	model.camView.clear();
	model.actualStepSize = *next(model.stepSizes.begin(), index);
	model.update_pc_index_list();
	model.actualIndex = floor(model.actualIndex / model.actualStepSize) * model.actualStepSize;
	fill_pc_indices();
	point_cloud_index_change(QString(to_string(model.actualIndex).c_str()));
	update_operation_views();
	update_global();
}

void LabelingPresenter::recolor_sliding_window_point_clouds() {
	for (int i = -model.slidingWindow * 2; i < 2 * model.slidingWindow + 1; i++) {
		int tmpIndex = model.actualIndex + i * model.actualStepSize;
		if (model.posePerObj.at(model.activObjId).count(tmpIndex) > 0)
			model.get_cloud(tmpIndex, true);
	}
}

void LabelingPresenter::class_id_change(int id) {
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	model.id2traj.at(model.activObjId)->classId = id;
	recolor_sliding_window_point_clouds();
	update_operation_views(true);
	update_global();
}

void LabelingPresenter::set_box_dimension() {
	VERBOSE_PRESENTER("set box dimesion")
	auto &bb = model.posePerObj.at(model.activObjId).begin()->second->bb;
	QSignalBlocker blockLength(view->spinLength);
	QSignalBlocker blockWidth(view->spinWidth);
	QSignalBlocker blockHeight(view->spinHeight);
	view->spinLength->setValue(bb.length);
	view->spinWidth->setValue(bb.width);
	view->spinHeight->setValue(bb.height);
	bool finalized = model.id2traj.at(model.activObjId)->is_finialized();
	view->spinHeight->setDisabled(finalized);
	view->spinLength->setDisabled(finalized);
	view->spinWidth->setDisabled(finalized);
}

void LabelingPresenter::set_point_cloud_index_to_first_obj_occurence() {
	VERBOSE_PRESENTER("set point cloud index to forst occurence")
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	if (!model.pointCloudIndices.empty()) {
		view->comboPcIndex->blockSignals(true);
		auto tmp = model.pointCloudIndices.find(model.actualIndex);
		if (tmp == model.pointCloudIndices.end())
			tmp = model.pointCloudIndices.begin();
		if (model.posePerObj.at(model.activObjId).count(model.actualIndex) < 1) {
			int firstIndex = floor(model.posePerObj.at(model.activObjId).begin()->first / model.actualStepSize) * model.actualStepSize;
			point_cloud_index_change(QString(to_string(firstIndex).c_str()));
		}
		view->comboPcIndex->blockSignals(false);
	}
}

void LabelingPresenter::set_class_combo_box() {
	VERBOSE_PRESENTER(" set class index")
	QSignalBlocker(view->comboObjectClass);
	if (model.id2traj.count(model.activObjId) > 0) {
		auto &obj = model.id2traj.at(model.activObjId);
		int id = obj->classId;
		if (id > view->comboObjectClass->count()) {
			id = view->comboObjectClass->count() - 1;
		}
		view->comboObjectClass->setCurrentIndex(id);
		view->comboObjectClass->setDisabled(obj->is_finialized());
	}
}

void LabelingPresenter::set_object_id_combo_box() {
	QSignalBlocker(view->comboObjectIds);
	view->comboObjectIds->setCurrentIndex(distance(model.id2traj.begin(), model.id2traj.find(model.activObjId)));
}

void LabelingPresenter::cam_global_id_change(const QString &index) {
	model.globalCameraId = index.toInt();
	update_global();
}
void LabelingPresenter::obj_id_change(const QString &index) {
	VERBOSE_PRESENTER("index")
	model.activObjId = index.toInt();
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	set_point_cloud_index_to_first_obj_occurence();
	set_object_id_combo_box();
	set_class_combo_box();
	if (model.posePerTime.count(model.actualIndex) < 1 || model.posePerTime.at(model.actualIndex).count(model.activObjId) < 1) {
		int pcIndex = model.posePerObj.at(model.activObjId).begin()->first + model.slidingWindow * model.actualStepSize;
		pcIndex = floor(pcIndex / model.actualStepSize) * model.actualStepSize;
		point_cloud_index_change(QString(to_string(pcIndex).c_str()));
	}
	set_box_dimension();
	double startI = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz;
	double endI = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz;
	model.interpolate(model.activObjId, startI, endI);
	update_operation_views(true);
	update_global();
	update_single_trajectory_view(model.activObjId);
}
void LabelingPresenter::cam_id_change(int index, int camId) {
	VERBOSE_PRESENTER("cam id change" << index << "id:" << camId);
	if (model.selectedCam.count(index) > 0)
		model.selectedCam.at(index) = camId;
	update_cam_view(index, true);
}
void LabelingPresenter::update_single_trajectory_view(int objId) {
	VERBOSE_PRESENTER("update single trajectory view")
	cv::Mat tv = model.generate_single_trajectory_view(objId);
	QImage x((const uchar*) tv.data, tv.cols, tv.rows, tv.step, QImage::Format_RGB888);
	x = x.rgbSwapped();
	view->singleTrajectoryView->setFixedSize(tv.cols, tv.rows);
	view->singleTrajectoryView->setScaledContents(true);
	view->singleTrajectoryView->setPixmap(QPixmap::fromImage(x));
}

void LabelingPresenter::update_global() {
	VERBOSE_PRESENTER("update global")
	update_glob_cam_view();
	update_model_view();
	cv::Mat tv = model.generate_global_point_cloud_view();
	QImage x((const uchar*) tv.data, tv.cols, tv.rows, tv.step, QImage::Format_RGB888);
	x = x.rgbSwapped();
	view->globalQvtk->update();
	view->globalTrajectoryView->resize(x.size());
	view->globalTrajectoryView->setPixmap(QPixmap::fromImage(x));
	view->globalTrajectoryView->setFixedSize(tv.cols, tv.rows);
	view->globalTrajectoryView->setScaledContents(true);
}
void LabelingPresenter::update_top_view(int index, bool refresh) {
	int cIndex = (index - model.actualIndex) / model.actualStepSize + model.slidingWindow;
	VERBOSE_PRESENTER("update top view" << index << "cIndex" << cIndex);
	if (cIndex < 0 || cIndex > view->tableViews->columnCount() - 1)
		return;
	cv::Mat tv(model.rowViewSize, model.columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));
	if (refresh || model.topView.count(index) < 1) {
		tv = model.generate_top_view(index, model.activObjId);
	} else if (model.topView.count(index) > 0) {
		tv = model.topView.at(index).clone();
	}
	QLabel *tmp4 = dynamic_cast<QLabel*>(view->tableViews->cellWidget(4, cIndex));
	QImage x((const uchar*) tv.data, tv.cols, tv.rows, tv.step, QImage::Format_RGB888);
	x = x.rgbSwapped();
	tmp4->resize(x.size());
	tmp4->setPixmap(QPixmap::fromImage(x));
}
void LabelingPresenter::update_side_view(int index, bool refresh) {
	VERBOSE_PRESENTER("update side view")
	int cIndex = (index - model.actualIndex) / model.actualStepSize + model.slidingWindow;
	if (cIndex < 0 || cIndex > view->tableViews->columnCount() - 1)
		return;
	cv::Mat tv(model.rowViewSize, model.columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));
	if (refresh || model.sideView.count(index) < 1) {
		tv = model.generate_side_view(index, model.activObjId);
	} else if (model.sideView.count(index) > 0) {
		tv = model.sideView.at(index).clone();
	}
	QLabel *tmp4 = dynamic_cast<QLabel*>(view->tableViews->cellWidget(5, cIndex));
	QImage x((const uchar*) tv.data, tv.cols, tv.rows, tv.step, QImage::Format_RGB888);
	x = x.rgbSwapped();
	tmp4->resize(x.size());
	tmp4->setPixmap(QPixmap::fromImage(x));
}
void LabelingPresenter::update_back_view(int index, bool refresh) {
	VERBOSE_PRESENTER("update back view")
	int cIndex = (index - model.actualIndex) / model.actualStepSize + model.slidingWindow;
	if (cIndex < 0 || cIndex > view->tableViews->columnCount() - 1)
		return;
	cv::Mat tv(model.rowViewSize, model.columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));

	if (refresh || model.backView.count(index) < 1) {
		tv = model.generate_back_view(index, model.activObjId);
	} else if (model.backView.count(index) > 0) {
		tv = model.backView.at(index).clone();
	}
	QLabel *tmp4 = dynamic_cast<QLabel*>(view->tableViews->cellWidget(6, cIndex));
	QImage x((const uchar*) tv.data, tv.cols, tv.rows, tv.step, QImage::Format_RGB888);
	x = x.rgbSwapped();
	tmp4->resize(x.size());
	tmp4->setPixmap(QPixmap::fromImage(x));
}

void LabelingPresenter::update_glob_cam_view() {
	VERBOSE_PRESENTER("update global cam")
	if (model.selectedCam.empty())
		return;
	double time = model.actualIndex / model.pointCloudFrequenz;
	if (model.posePerTime.count(model.actualIndex) > 0 && model.posePerTime.at(model.actualIndex).count(model.activObjId) > 0) {
		time = model.posePerTime.at(model.actualIndex).at(model.activObjId)->time;
	}
	cv::Mat img = model.generate_camera_global_view(time, model.globalCameraId);
	QImage x2((const uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
	x2 = x2.rgbSwapped();
	view->camViewLabel->setFixedSize(img.cols, img.rows);
	view->camViewLabel->setScaledContents(true);
	view->camViewLabel->setPixmap(QPixmap::fromImage(x2));
}
void LabelingPresenter::update_cam_view(int index, bool refresh) {
	VERBOSE_PRESENTER("update cam view");
	int cIndex = (index - model.actualIndex) / model.actualStepSize + model.slidingWindow;
	if (cIndex < 0 || cIndex > view->tableViews->columnCount() - 1)
		return;
	cv::Mat img(model.rowViewSize, model.columViewSize, CV_8UC3, cv::Scalar(255, 255, 255));
	if (refresh || model.camView.count(index) < 1) {
		if (model.selectedCam.count(index) < 1)
			model.selectedCam[index] = model.cams.begin()->first;
		img = model.generate_camera_view(index, model.activObjId, model.selectedCam.at(index)).clone();
	} else if (model.camView.count(index) > 0 && model.posePerObj.count(model.activObjId) > 0 && model.posePerObj.at(model.activObjId).count(index) > 0)
		img = model.camView.at(index).clone();
	cv::resize(img, img, cv::Size(model.columViewSize, model.rowViewSize));
	QLabel *tmp3 = dynamic_cast<QLabel*>(view->tableViews->cellWidget(3, cIndex));
	QImage x2((const uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
	x2 = x2.rgbSwapped();
	tmp3->resize(x2.size());
	tmp3->setPixmap(QPixmap::fromImage(x2));
}

void LabelingPresenter::delete_obj() {
	VERBOSE_PRESENTER("del obj")
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	int delIndex = model.activObjId;
	bool tmp = ikg::pop_up_confirm("Delete object" + to_string(delIndex) + "?");
	if (!tmp)
		return;
	if (!model.erase_obj(delIndex)) {
		ikg::pop_up_msg("Cant remove finalized obj");
		return;
	}
	int newId = -1;
	auto it = model.id2traj.lower_bound(model.activObjId);
	if(it!=model.id2traj.end()){
		newId=it->first;
	}
	model.activObjId = newId;
	update_object_ids(view->comboObjectIds);
	obj_id_change(QString(to_string(newId).c_str()));
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::add_obj() {
	model.addingFlag = true;
	ikg::pop_up_msg("shift left click at global point cloud to insert obj");
}

void LabelingPresenter::context_menu_view_table(QPoint pos) {
	QModelIndex index = view->tableViews->indexAt(pos);
	QMenu *menu = new QMenu(view);
	QAction *actInter = new QAction("interpolate", view);
	QAction *actStanding = new QAction("standing", view);
	QAction *actExtra = new QAction("extrapolate", view);
	QAction *actExtraStand = new QAction("extra standing", view);
	QAction *actDelIntersect = new QAction("del intersect", view);
	QAction *actMergelIntersect = new QAction("merge intersect", view);
	QAction *actDelPose = new QAction("delete", view);
	QAction *actInvertHeading = new QAction("invert heading", view);
	QAction *actInvertAllHeading = new QAction("invert all headings", view);
	QAction *actFinalPose = new QAction("finalize", view);
	QAction *actcorrectNext = new QAction("correctNext", view);
	actStanding->setData(index);
	actInter->setData(index);
	actExtra->setData(index);
	actDelIntersect->setData(index);
	actMergelIntersect->setData(index);
	actDelPose->setData(index);
	actInvertHeading->setData(index);
	actFinalPose->setData(index);
	actcorrectNext->setData(index);
	actExtraStand->setData(index);
	QObject::connect(actcorrectNext, &QAction::triggered, this, &LabelingPresenter::correct_next);
	QObject::connect(actInter, &QAction::triggered, this, &LabelingPresenter::interpolate);
	QObject::connect(actStanding, &QAction::triggered, this, &LabelingPresenter::mark_pose_as_standing);
	QObject::connect(actExtra, &QAction::triggered, this, &LabelingPresenter::extrapolate);
	QObject::connect(actExtraStand, &QAction::triggered, this, &LabelingPresenter::extrapolate_static);
	QObject::connect(actDelIntersect, &QAction::triggered, this, &LabelingPresenter::delete_intersecting_bounding_boxes);
	QObject::connect(actMergelIntersect, &QAction::triggered, this, &LabelingPresenter::merge_intersection);
	QObject::connect(actDelPose, &QAction::triggered, this, &LabelingPresenter::delete_pose);
	QObject::connect(actInvertHeading, &QAction::triggered, this, &LabelingPresenter::invert_heading);
	QObject::connect(actInvertAllHeading, &QAction::triggered, this, &LabelingPresenter::invert_all_headings);
	QObject::connect(actFinalPose, &QAction::triggered, this, &LabelingPresenter::finalize_pose);
	menu->addAction(actInter);
	menu->addAction(actExtra);
	menu->addAction(actExtraStand);
	menu->addAction(actStanding);
	menu->addAction(actDelIntersect);
	menu->addAction(actMergelIntersect);
	menu->addAction(actDelPose);
	menu->addAction(actInvertHeading);
	menu->addAction(actInvertAllHeading);
	menu->addAction(actFinalPose);
	menu->addAction(actcorrectNext);
	menu->popup(view->tableViews->viewport()->mapToGlobal(pos));
}
void LabelingPresenter::interpolate_between_key_frames() {
	VERBOSE_PRESENTER("interpolate between key frames")
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	model.interpolate_between_key_frames();
}
void LabelingPresenter::heading_value_change(int c) {
	QDoubleSpinBox *spinH2 = dynamic_cast<QDoubleSpinBox*>(view->tableViews->cellWidget(2, c));
	double v = fmod(spinH2->value() + Pi3, Pi2) - M_PI;
	QSignalBlocker q(spinH2);
	spinH2->setValue(v);
	Pose *p;
	int index = model.actualIndex - model.slidingWindow * model.actualStepSize + c * model.actualStepSize;
	if (!model.get_active_bb_at_index(index, p))
		return;
	if (p->finalized) {
		spinH2->setValue(p->bb.angle);
		return;
	}
	p->bb.update_orientation(0, 0, v, false);
	p->interpolated = false;
	model.id2traj.at(model.activObjId)->update_interpolated_pose_bounding_boxes();
	update_operation_views(true);
}

void LabelingPresenter::fill_operation_table() {
	view->tableViews->clear();
	view->tableViews->setColumnCount(model.slidingWindow * 2 + 1);
	view->tableViews->setRowCount(7);
	view->tableViews->setRowHeight(0, model.rowViewSize);
	view->tableViews->setRowHeight(3, model.rowViewSize);
	view->tableViews->setRowHeight(4, model.rowViewSize);
	view->tableViews->setRowHeight(5, model.rowViewSize);
	view->tableViews->setRowHeight(6, model.rowViewSize);
	for (int i = 0; i < model.slidingWindow * 2 + 1; i++) {
		view->tableViews->setColumnWidth(i, model.columViewSize);
		QVTKWidget *tmp = new QVTKWidget;
		QLabel *l1 = new QLabel;
		QLabel *l2 = new QLabel;
		QLabel *l3 = new QLabel;
		QLabel *l4 = new QLabel;
		QDoubleSpinBox *spin = new QDoubleSpinBox();
		view->tableViews->setCellWidget(0, i, tmp);
		view->tableViews->setCellWidget(1, i, new QComboBox);
		view->tableViews->setCellWidget(2, i, spin);
		spin->setMaximum(M_PI + 0.01);
		spin->setMinimum(-M_PI - 0.01);
		spin->setDecimals(2);
		spin->setSingleStep(0.01);
		QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this, i] {
			heading_value_change(i);
		}
		);
		view->tableViews->setCellWidget(3, i, l1);
		view->tableViews->setCellWidget(4, i, l2);
		view->tableViews->setCellWidget(5, i, l3);
		view->tableViews->setCellWidget(6, i, l4);
		view->tableViews->cellWidget(3, i)->installEventFilter(this);
		view->tableViews->cellWidget(4, i)->installEventFilter(this);
		view->tableViews->cellWidget(5, i)->installEventFilter(this);
		view->tableViews->cellWidget(6, i)->installEventFilter(this);
		VERBOSE_PRESENTER("PC VIEWS")
		tmp->SetRenderWindow(model.pcViews[i]->getRenderWindow());
		tmp->update();
	}
}

void LabelingPresenter::set_model_view_dimensions() {
	model.singleTrajectoryWidth = view->width() / 6. * 5;
	model.singleTrajectoryHeight = view->height();
	model.globalTrajectoryWidth = view->width() / 6. * 5;
	model.globalTrajectoryHeight = view->height() / 3;
	model.rowViewSize = view->height() / 6.;
	model.columViewSize = view->width() / 6.;
}

void LabelingPresenter::fill_view() {
	VERBOSE_PRESENTER("fill view")
	view->blockSignals(true);
	QSignalBlocker pcBlock(view->comboPcIndex);
	QSignalBlocker odBlock(view->comboObjectIds);
	QSignalBlocker stepBlock(view->comboStepSize);
	QSignalBlocker classBlock(view->comboObjectClass);
	QSignalBlocker b(view);
	view->comboObjectClass->clear();
	for (auto s : model.classNames) {
		view->comboObjectClass->addItem(s.c_str());
	}
	model.reset_pc_views();
	view->spinPointSize->setValue(model.pointSize);
	int w = max(80, view->width() / 7 / 3) * 0.6;
	int h = max(50, view->height() / (view->tableOperations->rowCount() + 1)) * 0.6;
	for (int i = 0; i < view->tableOperations->rowCount(); i++) {
		view->tableOperations->setRowHeight(i, h);
	}
	int h2 = h * 0.6;
	string styleString = "QCheckBox::indicator { width:'" + to_string(h2) + "px'; height:'" + to_string(h2) + "px'}"; //
	view->checkColorObj->setStyleSheet(styleString.c_str());
	view->checkRemoveBackground->setStyleSheet(styleString.c_str());
	view->tableOperations->setColumnWidth(0, max(80, view->width() / 7 / 3));
	view->tableOperations->setColumnWidth(1, max(80, view->width() / 7 / 3));
	set_model_view_dimensions();
	fill_pc_indices();
	fill_global_camera_index();
	update_object_ids(view->comboObjectIds);
	void *d = reinterpret_cast<void*>(this);
	model.globPcView->registerPointPickingCallback(&point_picking_callback, d);
	view->globalQvtk->SetRenderWindow(model.globPcView->getRenderWindow());
	model.globPcView->setupInteractor(view->globalQvtk->GetInteractor(), view->globalQvtk->GetRenderWindow());
	view->modelQvtk->SetRenderWindow(model.modelPcView->getRenderWindow());
	model.modelPcView->setupInteractor(view->modelQvtk->GetInteractor(), view->modelQvtk->GetRenderWindow());
	view->comboStepSize->clear();
	if (!model.stepSizes.empty()) {
		for (int s : model.stepSizes)
			view->comboStepSize->addItem(to_string(s).c_str());
		view->comboStepSize->setCurrentIndex(distance(model.stepSizes.begin(), model.stepSizes.find(model.actualStepSize)));
	}
	fill_operation_table();
	if (!model.pointCloudPath.empty())
		point_cloud_index_change(QString(to_string(model.actualIndex).c_str()));
	update_global();
	update_single_trajectory_view(model.activObjId);
	view->set_tool_tips();
	view->tableViews->setStyleSheet("QTableView:item:selected:focus {background-color: #3399FF;}");
	view->blockSignals(false);
}

void LabelingPresenter::mark_pose_as_standing() {
	VERBOSE_PRESENTER("mark pose as standing")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (model.posePerObj.count(model.activObjId) < 1 || model.posePerObj.at(model.activObjId).count(index) < 1)
		return;
	model.set_standing_pose_by_key_frames(model.activObjId, index / model.pointCloudFrequenz);
	double start = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz + 1. / model.pointCloudFrequenz;
	double end = (model.actualIndex - model.slidingWindow * model.actualStepSize) / model.pointCloudFrequenz - 1. / model.pointCloudFrequenz;
	model.interpolate(model.activObjId, start, end);
	update_view_column(id.column(), true);
}
void LabelingPresenter::delete_pose() {
	VERBOSE_PRESENTER("delete pose")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	double startTime = (index - model.actualStepSize) / model.pointCloudFrequenz + 1. / model.pointCloudFrequenz;
	double endTime = (index + model.actualStepSize) / model.pointCloudFrequenz - 1. / model.pointCloudFrequenz;
	model.erase_pose(model.activObjId, startTime, endTime);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::invert_heading() {
	VERBOSE_PRESENTER("invert heading")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (model.posePerObj.count(model.activObjId) < 1 || model.posePerObj.at(model.activObjId).count(index) < 1)
		return;
	if(model.posePerObj.at(model.activObjId).at(index)->finalized){
		ikg::pop_up_msg("can't interpolate finalized heading");
		return;
	}
	model.posePerObj.at(model.activObjId).at(index)->bb.invert_heading();
	model.interpolate(model.activObjId);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}

void LabelingPresenter::invert_all_headings() {
	VERBOSE_PRESENTER("invert all heading")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	if (model.id2traj.count(model.activObjId) < 1) {
		return;
	}
	if(model.id2traj.at(model.activObjId)->is_finialized()){
		ikg::pop_up_msg("can't interpolate all headings, because at least one pose is finalized");
	}
	for (auto &o : model.posePerObj.at(model.activObjId)) {
		o.second->bb.invert_heading();
	}
	model.interpolate(model.activObjId);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}

void LabelingPresenter::finalize_pose() {
	VERBOSE_PRESENTER("finalize pose")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (model.posePerObj.count(model.activObjId) < 1 || model.posePerObj.at(model.activObjId).count(index) < 1)
		return;
	model.posePerObj.at(model.activObjId).at(index)->finalized = !model.posePerObj.at(model.activObjId).at(index)->finalized;
	update_view_column(id.column(), true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::finalized_object() {
	VERBOSE_PRESENTER("finalized_object")
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	model.finalized(model.activObjId);
	view->comboObjectClass->setDisabled(true);
	view->spinHeight->setDisabled(true);
	view->spinWidth->setDisabled(true);
	view->spinLength->setDisabled(true);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::definalized_object() {
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	model.definalized(model.activObjId);
	view->comboObjectClass->setDisabled(false);
	view->spinHeight->setDisabled(false);
	view->spinWidth->setDisabled(false);
	view->spinLength->setDisabled(false);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::delete_intersecting_bounding_boxes() {
	VERBOSE_PRESENTER("del intersection")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	double t = index / model.pointCloudFrequenz;
	auto ids = model.get_intersecting_obj(model.activObjId, t);
	for (auto id : ids) {
		model.erase_pose(id, t);
	}
	update_view_column(id.column(), true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::correct_next() {
	VERBOSE_PRESENTER("correct next")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	const auto &pc = *model.get_cloud(index);
	if (model.id2traj.count(model.activObjId) < 1 || model.posePerObj.at(model.activObjId).count(index) < 1) {
		return;
	}
	LUMPIPipeline::correct_pose(*model.posePerObj.at(model.activObjId).at(index), pc);
	update_view_column(id.column(), true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::merge_intersection() {
	VERBOSE_PRESENTER("merge intersection")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	double t = index / model.pointCloudFrequenz;
	auto ids = model.get_intersecting_obj(model.activObjId, t);
	if (model.id2traj.count(model.activObjId) < 1) {
		ikg::pop_up_msg("object " + to_string(model.activObjId) + " not found");
		return;
	}
	auto tra = model.id2traj.at(model.activObjId);
	for (auto id : ids) {
		bool skip = false;
		auto tr = model.id2traj.at(id);
		if (id == model.activObjId)
			continue;
		if (tr->is_finialized()) {
			ikg::pop_up_msg("Cant merge object with finalized poses" + to_string(tr->id));
			continue;
		}
		tra->merge_time(*tr, DBL_MAX);
		model.erase_obj(id);
	}
	model.update_obj_ptr();
	update_object_ids(view->comboObjectIds);
	update_operation_views(true);
	update_global();
	update_single_trajectory_view(model.activObjId);
}

void LabelingPresenter::extrapolate() {
	VERBOSE_PRESENTER("extrapolate")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	if (model.posePerObj.count(model.activObjId) < 1) {
		ikg::pop_up_msg("obj not found");
		return;
	}
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (model.posePerObj.at(model.activObjId).count(index) > 0) {
		ikg::pop_up_msg("Cant extrapolate within a track");
		return;
	}
	model.extrapolate(model.activObjId, index / model.pointCloudFrequenz);
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::extrapolate_static() {
	VERBOSE_PRESENTER("extrapolate static")
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	if (model.posePerObj.count(model.activObjId) < 1) {
		ikg::pop_up_msg("obj not found");
		return;
	}
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (model.posePerObj.at(model.activObjId).count(index) > 0) {
		ikg::pop_up_msg("Cant extrapolate within a track");
		return;
	}
	model.extrapolate(model.activObjId, index / model.pointCloudFrequenz, true);
	update_view_column(id.column(), true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::interpolate() {
	VERBOSE_PRESENTER("interpolate")
	if (model.id2traj.count(model.activObjId) < 1)
		return;
	QAction *act = qobject_cast<QAction*>(sender());
	auto id = act->data().value<QModelIndex>();
	auto &t = model.posePerObj.at(model.activObjId);
	int index = model.actualIndex + (id.column() - model.slidingWindow) * model.actualStepSize;
	if (t.begin()->first > index - model.actualStepSize) {
		ikg::pop_up_msg("Cant interpolate start");
		return;
	}
	if (t.rbegin()->first < index + model.actualStepSize) {
		ikg::pop_up_msg("Cant interpolate end");
		return;
	}
	model.interpolate_single_between_key_frames(model.activObjId, index / model.pointCloudFrequenz);
	model.id2traj.at(model.activObjId)->update_interpolated_pose_bounding_boxes();
	update_operation_views(true);
	update_global();
	update_glob_cam_view();
}
void LabelingPresenter::point_size_change(double size) {
	VERBOSE_PRESENTER("Change point size")
	model.pointSize = size;
	if (model.clouds.empty())
		return;
	for (auto &v : model.pcViews) {
		v.second->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, model.pointSize, "cloud");
	}
	model.globPcView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, model.pointSize, "cloud");
	update_operation_views(true);
	update_global();
}
void LabelingPresenter::point_picking_callback(const pcl::visualization::PointPickingEvent &event, void *args) {
	pcl::PointXYZ c;
	double point_size = 3.0;
	event.getPoint(c.x, c.y, c.z);
	VERBOSE_PRESENTER("Clicked Point: ")
	LabelingPresenter *lp = reinterpret_cast<LabelingPresenter*>(args);
	Labeling &l = lp->model;
	if (!l.addingFlag) {
		int id=l.get_obj_at(cv::Point3d(c.x,c.y,c.z), l.actualIndex).first;
		if (id > -1)
			lp->obj_id_change(QString(to_string(id).c_str()));
	} else {
		l.add_obj(cv::Point3d(c.x, c.y, c.z));
		lp->update_object_ids(lp->view->comboObjectIds);
		lp->obj_id_change(QString(to_string(l.pcTrajectories.back().id).c_str()));
		lp->update_operation_views(true);
		lp->update_global();
		l.addingFlag = false;

	}
}
void LabelingPresenter::load() {
	VERBOSE_PRESENTER("load")
	string dataPath, trajectoryPath;
	ikg::open_file_dialog(dataPath, false, view, "load LUMPI data");
	if (!filesystem::exists(dataPath + "/meta.json")) {
		ikg::pop_up_msg("no meta.json exists at data path:" + dataPath);
		return;
	}
	ikg::open_file_dialog(trajectoryPath, true, view, "load trajectoreis data");
	bool ok = false;
	static int id = 7;
	id = QInputDialog::getInt(view, "Measurement Number", QString("Measurement:"), id, 0, INT_MAX, 1, &ok);
	if (!ok)
		return;
	model.init(dataPath, trajectoryPath, id);
	fill_view();
	update_operation_views(true);
	update_single_trajectory_view(model.activObjId);
	update_global();
	obj_id_change(QString(to_string(model.activObjId).c_str()));
}
void LabelingPresenter::save() {
	VERBOSE_PRESENTER("save")
	string path;
	for (auto &t : model.pcTrajectories) {
		for (auto &p : t.poseData) {
			p.classId = t.classId;
			auto pts = p.bb.enclousing_bounding_box();
			p.r = cv::Rect2d(cv::Point2d(pts.first.x, pts.first.y), cv::Point2d(pts.second.x, pts.second.y));
			p.w = p.r.width;
			p.h = p.r.height;
			p.x = p.r.x;
			p.y = p.r.y;
			if (p.finalized)
				p.score = 4;
			else
				p.score = 0;
		}
	}
	ikg::save_file_dialog(path, view, "save Trajectories");
	if (!path.empty()) {
		Parser::write_trajectories(model.pcTrajectories, path, true);
		ikg::pop_up_msg("saved: " + path);
	}
	VERBOSE_PRESENTER(path)
}
int LabelingPresenter::update_view_column(int i, bool refresh) {
	VERBOSE_PRESENTER("update column"<<i<<"refresh"<<refresh)
	int index = model.actualIndex + (-model.slidingWindow + i) * model.actualStepSize;
	QDoubleSpinBox *spinH = dynamic_cast<QDoubleSpinBox*>(view->tableViews->cellWidget(2, i));
	Pose *p;
	spinH->blockSignals(true);
	if (model.get_active_bb_at_index(index, p)) {
		spinH->setValue(p->bb.angle);
	} else {
		spinH->setValue(0.);
	}
	spinH->blockSignals(false);
	QVTKWidget *tmp = dynamic_cast<QVTKWidget*>(view->tableViews->cellWidget(0, i));
	if (i > model.pcViews.size())
		return -1;
	auto v = next(model.pcViews.begin(), i);
	if (v == model.pcViews.end())
		return 0;
	if (model.flagColorObj) {
		model.set_point_cloud(index, i);
	}
	tmp->update();
	QComboBox *select = dynamic_cast<QComboBox*>(view->tableViews->cellWidget(1, i));
	QSignalBlocker block(select);
//	select->disconnect();
	select->clear();
	for (auto &c : model.cams) {
		string camID = to_string(c.first);
		select->addItem(QString(camID.c_str()));
	}
	if (model.selectedCam.count(index) > 0) {
		int tmp2 = std::distance(model.cams.begin(), model.cams.find(model.selectedCam.at(index)));
		select->setCurrentIndex(tmp2);
	}
//	QObject::connect(select, QOverload<int>::of(&QComboBox::currentIndexChanged), [this, select, index]() {
//		cam_id_change(index, select->currentText().toInt());
//	});
	model.update_pc_bb(index);
	update_cam_view(index, refresh);
	update_top_view(index, refresh);
	update_side_view(index, refresh);
	update_back_view(index, refresh);
	return index;
}
void LabelingPresenter::update_operation_views(bool refresh) {
	VERBOSE_PRESENTER("update views")
	QStringList hLabels;
	for (int i = 0; i < view->tableViews->columnCount(); i++) {
		hLabels << to_string(update_view_column(i, refresh)).c_str();
	}
	VERBOSE_PRESENTER("HEADER")
	VERBOSE_PRESENTER(hLabels.size());
	view->tableViews->setHorizontalHeaderLabels(hLabels);
}
LabelingPresenter::~LabelingPresenter() {
}

} /* namespace ikg */
