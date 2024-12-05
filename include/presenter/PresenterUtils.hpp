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

#ifndef INCLUDE_PRESENTER_PRESENTERUTILS_HPP_
#define INCLUDE_PRESENTER_PRESENTERUTILS_HPP_
#include <iostream>
#include <QWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <QDialog>
#include <QLineEdit>
namespace ikg{

/**
 * @brief Opens a file dialog to select a file or directory.
 * 
 * @param q Reference to a string where the selected path will be stored.
 * @param file Boolean indicating whether to select a file (true) or a directory (false).
 * @param window Pointer to the parent widget.
 * @param caption Optional caption for the file dialog.
 */
static void open_file_dialog(std::string &q, bool file, QWidget * window, std::string caption="") {
    QFileDialog _f_dlg(window, QPushButton::tr(caption.c_str()) );
    _f_dlg.setDirectory(QString(q.c_str()));
    if (file) {
    	_f_dlg.setFileMode(QFileDialog::ExistingFile);
    }
    else {
    	_f_dlg.setFileMode(QFileDialog::Directory);
    	//_f_dlg.setOptions(QFileDialog::ShowDirsOnly);
    }
    if (_f_dlg.exec()) {
    	QStringList fnames = _f_dlg.selectedFiles();
        q=fnames.front().toUtf8().constData();
    }
}

/**
 * @brief Opens a file dialog to select a file or directory.
 * 
 * @param q Pointer to a QLineEdit where the selected path will be set.
 * @param file Boolean indicating whether to select a file (true) or a directory (false).
 * @param window Pointer to the parent widget.
 * @param caption Optional caption for the file dialog.
 */
static void open_file_dialog(QLineEdit * q, bool file, QWidget * window, std::string caption="") {

    QString file_path;
    QFileDialog _f_dlg(window,QPushButton::tr(caption.c_str()));
    if (file) {
    	_f_dlg.setFileMode(QFileDialog::ExistingFile);
    }
    else {
    	_f_dlg.setFileMode(QFileDialog::Directory);
    	_f_dlg.setOptions(QFileDialog::ShowDirsOnly);
    }
    if (_f_dlg.exec()) {
    	QStringList fnames = _f_dlg.selectedFiles();
        q->setText(fnames.front());
    }
}

/**
 * @brief Opens a file dialog to save a file.
 * 
 * @param q Pointer to a QLineEdit where the selected path will be set.
 * @param window Pointer to the parent widget.
 * @param caption Optional caption for the file dialog.
 */
static void save_file_dialog(QLineEdit * q, QWidget * window, std::string caption="") {

    QString file_path;
    QFileDialog _f_dlg(window,QPushButton::tr(caption.c_str()));
    _f_dlg.setAcceptMode(QFileDialog::AcceptSave);
    if (_f_dlg.exec()) {
    	QStringList fnames = _f_dlg.selectedFiles();
    	q->setText(fnames.front());
    }
}

/**
 * @brief Opens a file dialog to save a file.
 * 
 * @param q Reference to a string where the selected path will be stored.
 * @param window Pointer to the parent widget.
 * @param caption Optional caption for the file dialog.
 */
static void save_file_dialog(std::string& q, QWidget * window, std::string caption="") {

    QString file_path;
    QFileDialog _f_dlg(window,QPushButton::tr(caption.c_str()));
    _f_dlg.setAcceptMode(QFileDialog::AcceptSave);
    if (_f_dlg.exec()) {
    	QStringList fnames = _f_dlg.selectedFiles();
    	q=fnames.front().toStdString();
    }
}

/**
 * @brief Displays a confirmation pop-up with a message.
 * 
 * @param msg The message to display in the pop-up.
 * @return true if the user clicked OK, false otherwise.
 */
static bool pop_up_confirm(const std::string &msg) {
	QMessageBox msgBox;
	msgBox.setText(msg.c_str());
	msgBox.setInformativeText(msg.c_str());
	msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
	msgBox.setDefaultButton(QMessageBox::Save);
	int ret = msgBox.exec();
	  if (ret==QMessageBox::Ok)return true;
	return false;
}

/**
 * @brief Displays a message pop-up.
 * 
 * @param msg The message to display in the pop-up.
 */
static void pop_up_msg(const std::string& msg) {
	QMessageBox msgBox;
	msgBox.setText(QString(msg.c_str()));
	msgBox.setFocus();
	msgBox.exec();

}
}
#endif /* INCLUDE_PRESENTER_PRESENTERUTILS_HPP_ */
