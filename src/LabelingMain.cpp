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
#include <QMainWindow>
#include <cstdio>
using namespace std;
using namespace ikg;
int main(int argc, char **argv) {
	freopen("output.txt", "w", stdout);
	freopen("error.txt", "w", stderr);
	QApplication app(argc, argv);
	qRegisterMetaType<QVector<int>>("QVector<int>");
	QFile File("data/blackOrange.qss");
	File.open(QFile::ReadOnly);
	QString StyleSheet = QLatin1String(File.readAll());
	app.setStyleSheet(StyleSheet);
	QMainWindow *w = new QMainWindow;
	LabelingView *view = new LabelingView;
	LabelingPresenter *p = new LabelingPresenter(view);
	QRect rec = QApplication::desktop()->screenGeometry();
	int height = rec.height();
	int width = rec.width();
	w->setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, QSize(0.9 * width, 0.9 * height), QApplication::desktop()->availableGeometry()));
	w->setMinimumSize(width / 3, height / 3);
	w->setMaximumSize(width, height);
	w->setCentralWidget(view);
	w->setVisible(true);
	w->setWindowTitle("LUMPI Labeling Tool");
	int ergebins = app.exec();
	return 0;
}

