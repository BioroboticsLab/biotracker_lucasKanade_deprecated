#include "LucasKanade.h"

#include <QApplication>
#include <QIntValidator>
#include <QPushButton>
#include <QPainter>
#include <QColorDialog>

#include <biotracker/TrackingAlgorithm.h>
#include <biotracker/Registry.h>

using namespace BioTracker::Core;

extern "C" {
    void registerTracker() {
        BioTracker::Core::Registry::getInstance().registerTrackerType<LucasKanadeTracker>("Lucas-Kanade");
    }
}

LucasKanadeTracker::LucasKanadeTracker(Settings &settings):
    TrackingAlgorithm(settings),
    m_itemSize(1),
    m_subPixWinSize(10, 10),
    m_winSize(31, 31),
    m_termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03),
    m_shouldTrack(true),
    m_pauseOnInvalidPoint(false),
    m_winSizeSlider(new QSlider(getToolsWidget())),
    m_winSizeValue(new QLabel(QString::number(m_winSize.height), getToolsWidget())),
    m_historySlider(new QSlider(getToolsWidget())),
    m_historyValue(new QLabel("0", getToolsWidget())),
    m_invalidOffset(-99999, -99999),
    m_validColor(QColor::fromRgb(0, 0, 255)),
    m_invalidColor(QColor::fromRgb(255, 0, 0)){

    m_grabbedKeys.insert(Qt::Key_D);

    // initialize gui
    auto ui = getToolsWidget();
    auto layout = new QGridLayout();

    // Checkbox for tracking enable/disable
    auto *chkboxShouldTrack = new QCheckBox("Tracking enabled", ui);
    chkboxShouldTrack->setChecked(true);
    QObject::connect(chkboxShouldTrack, &QCheckBox::stateChanged,
        this, &LucasKanadeTracker::checkboxChanged_shouldTrack);
    layout->addWidget(chkboxShouldTrack, 0, 0, 1, 3);

    // Checkbox for pausing on invalid points
    auto *chkboxInvalidPoints = new QCheckBox("Pause on invalid Point", ui);
    chkboxInvalidPoints->setChecked(false);
    QObject::connect(chkboxInvalidPoints, &QCheckBox::stateChanged,
        this, &LucasKanadeTracker::checkboxChanged_invalidPoint);
    layout->addWidget(chkboxInvalidPoints, 1, 0, 1, 3);


    // history
    auto *lbl_history = new QLabel("history", ui);
    m_historySlider->setMinimum(0);
    m_historySlider->setMaximum(15);
    m_historySlider->setOrientation(Qt::Orientation::Horizontal);
    layout->addWidget(lbl_history, 2, 0, 1, 1);
    layout->addWidget(m_historyValue, 2, 2, 1, 1);
    layout->addWidget(m_historySlider, 3, 0, 1, 3);
    QObject::connect(m_historySlider, &QSlider::valueChanged,
        this, &LucasKanadeTracker::sliderChanged_history);

    // winsize
    auto *lbl_winSize = new QLabel("window size:", ui);
    m_winSizeSlider->setMinimum(10);
    m_winSizeSlider->setMaximum(m_winSize.height);
    m_winSizeSlider->setOrientation(Qt::Orientation::Horizontal);
    m_winSizeSlider->setValue(m_winSize.height);
    QObject::connect(m_winSizeSlider, &QSlider::valueChanged,
        this, &LucasKanadeTracker::sliderChanged_winSize);
    layout->addWidget(lbl_winSize, 4, 0, 1, 1);
    layout->addWidget(m_winSizeValue, 5, 2, 1, 1);
    layout->addWidget(m_winSizeSlider, 5, 0, 1, 2);

    // colors
    auto lbl_color = new QLabel("Change color:", ui);
    layout->addWidget(lbl_color, 6, 0, 1, 1);

    auto validColorBtn = new QPushButton("Valid color", ui);
    QObject::connect(validColorBtn, &QPushButton::clicked,
        this, &LucasKanadeTracker::clicked_validColor);
    layout->addWidget(validColorBtn, 6, 1, 1, 1);

    auto invalidColorBtn = new QPushButton("Invalid color", ui);
    QObject::connect(invalidColorBtn, &QPushButton::clicked,
        this, &LucasKanadeTracker::clicked_invalidColor);
    layout->addWidget(invalidColorBtn, 7, 1, 1, 1);

    // print
    auto printBtn = new QPushButton("Export", ui);
    QObject::connect(printBtn, &QPushButton::clicked,
        this, &LucasKanadeTracker::clicked_print);
    layout->addWidget(printBtn, 8, 0, 1, 1);

    // ===

    ui->setLayout(layout);
}

void LucasKanadeTracker::track(ulong frame, const cv::Mat &imgOriginal) {

    // Landscape vs	portrait
    // [xxxx]		[xx]
    // [xxxx]		[xx]
    //       		[xx]
    //
    const bool isLandscape = imgOriginal.rows > imgOriginal.cols;

    // make sure that the circles are in "good" size, regardless of the video resolution (tiny vs gigantic)
    const int perc_size = 45;
    m_itemSize = !isLandscape ? imgOriginal.rows / perc_size : imgOriginal.cols / perc_size;

    // make the winSize adaptable
    const int currentMaxWinSize = m_winSizeSlider->maximum();
    const int newMaxWinSize = isLandscape ? imgOriginal.cols / 10 : imgOriginal.rows / 10;
    if (currentMaxWinSize != newMaxWinSize && newMaxWinSize > m_winSizeSlider->minimum()) {
        m_winSizeSlider->setMaximum(newMaxWinSize);
    }

    //bool isStepForward = m_currentFrame == (frame - 1);
    bool isStepForward = true; // TODO figure what we want: now the tracker will always track!

    m_currentFrame = frame; // TODO must this be protected from other threads?
    cv::cvtColor(imgOriginal, m_gray, cv::COLOR_BGR2GRAY);

    if ((frame == 0 || isStepForward) && m_shouldTrack) {

        std::vector<InterestPointStatus> filter;
        std::vector<cv::Point2f> currentPoints = getCurrentPoints(frame - 1, filter);

        if (m_prevGray.empty()) {
            m_gray.copyTo(m_prevGray);
        }

        if (!currentPoints.empty()) {
            std::vector<float> err;

            // calculate pyramids:
            const size_t maxLevel = 10;
            std::vector<cv::Mat> prevPyr;
            cv::buildOpticalFlowPyramid(m_prevGray, prevPyr, m_winSize, maxLevel);

            std::vector<cv::Mat> pyr;
            cv::buildOpticalFlowPyramid(m_gray, pyr, m_winSize, maxLevel);

            std::vector<uchar> status;
            std::vector<cv::Point2f> newPoints;
            cv::calcOpticalFlowPyrLK(
                prevPyr, /* prev */
                pyr, /* next */
                currentPoints,	/* prevPts */
                newPoints, /* nextPts */
                status,	/* status */
                err	/* err */
                ,m_winSize,	/* winSize */
                maxLevel, /* maxLevel */
                m_termcrit,	/* criteria */
                0, /* flags */
                0.001 /* minEigThreshold */
            );

            updateCurrentPoints(frame, newPoints, status, filter);
            updateHistorySlider(static_cast<int>(frame));
            updateHistoryText();
        }

        cv::swap(m_prevGray, m_gray);
    }
}

void LucasKanadeTracker::paint(ulong, ProxyMat &, const TrackingAlgorithm::View &) {

}

void LucasKanadeTracker::paintOverlay(ulong, QPainter *painter, const View &) {

    std::vector<InterestPointStatus> filter;
    std::vector<cv::Point2f> newPoints = getCurrentPoints(m_currentFrame, filter);

    // fill the history
    std::vector<std::vector<cv::Point2f>> history;
    std::vector<InterestPointStatus> dummyfilter;
    for (size_t t = 1; t < m_currentHistory; t++) {
        int histTime = m_currentHistory - t;
        if (histTime < 0) {
            break;
        }
        dummyfilter.clear();
        std::vector<cv::Point2f> histPoints = getCurrentPoints(histTime, dummyfilter);
        history.push_back(histPoints);
    }


    size_t i;
    for (i = 0; i < newPoints.size(); i++) {
        if (filter[i] == InterestPointStatus::Non_Existing) {
            // the i`th point became invalid
            continue;
        }

        QColor color = m_validColor;
        auto point = newPoints[i];
        if (filter[i] == InterestPointStatus::Invalid) {
            point -= m_invalidOffset;
            color = m_invalidColor;
        }

        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);

        QFont font = painter->font();
        font.setPixelSize(m_itemSize);
        painter->setFont(font);

        QPen p(color);
        if (i == static_cast<size_t>(m_currentActivePoint)) {
            p.setStyle(Qt::PenStyle::DotLine);
        }

        p.setWidth(m_itemSize / 3 > 0 ? m_itemSize / 3 : 1);

        int itemSizeHalf = m_itemSize / 2;

        painter->setPen(p);
        painter->drawEllipse(x - itemSizeHalf, y - itemSizeHalf, m_itemSize, m_itemSize);
        auto idTxt = QString::number(i);
        painter->drawText(x, y - itemSizeHalf, idTxt);
        painter->drawRect(x, y, 1, 1);

        // paint History
        for (size_t t = 1; t < m_currentHistory; t++) {
            int histTime = m_currentHistory - t;
            if (histTime < 0) {
                break;
            }
            auto histPoint = history[histTime][i];
            int x = static_cast<int>(histPoint.x);
            int y = static_cast<int>(histPoint.y);
            if (x > 0 && y > 0) { // otherwise the point is invalid
                painter->drawRect(x, y, 2, 2);
            }
        }

    }


}

void LucasKanadeTracker::keyPressEvent(QKeyEvent *ev) {
    if (ev->key() == 68) { // => Key: 'd'
        deleteCurrentActivePoint();
    }
}

void LucasKanadeTracker::mouseReleaseEvent(QMouseEvent *e)
{
    switch(e->modifiers()) {
    case Qt::ShiftModifier: {
        this->activateExistingPoint(e->pos());
        break;
    }
    case Qt::ControlModifier: {
        this->tryCreateNewPoint(e->pos());
        break;
    }
    default : {
        this->moveCurrentActivePointTo(e->pos());
        break;
    }
    }
}

// =========== P R I V A T E = F U N C S ============



void LucasKanadeTracker::tryCreateNewPoint(QPoint pos) {
    if (m_gray.empty()) {
        // this usually happens when users try to add interest points upon
        // starting the video as then there is no track being called yet..
        Q_EMIT notifyGUI("First tracking could not be recorded as some data must be initialized. Please try again.");
        Q_EMIT forceTracking();
    } else {
        std::vector<InterestPointStatus> filter;
        std::vector<cv::Point2f> newPoints = getCurrentPoints(m_currentFrame, filter);
        cv::Point2f point = toCv(pos);
        for (auto otherPos : newPoints) {
            if (cv::norm(point - otherPos) <= 5) {
                // the new point is too close to an existing other point.. abort
                Q_EMIT notifyGUI("too close to an existing point..");
                return;
            }
        }

        std::vector<cv::Point2f> tmp;
        tmp.push_back(point);
        cv::cornerSubPix(m_gray, tmp, m_winSize, cv::Size(-1, -1), m_termcrit);

        const auto newPos = tmp[0];
        auto p = std::make_shared<InterestPoint>(); // TODO: this allocation is not 'pretty' as it is unnecessary
        p->setPosition(cv::Point2f(newPos.x, newPos.y));
        p->setValidity(true);

        const size_t id = m_trackedObjects.size(); // position in list + id are correlated
        TrackedObject o(id);
        o.add(m_currentFrame, p);
        m_trackedObjects.push_back(o);

        m_currentActivePoint = id;

        if (m_firstTrackedFrame > static_cast<int>(m_currentFrame)) { // for the history calculation
            m_firstTrackedFrame = m_currentFrame;
        }

        Q_EMIT update();
    }
}

void LucasKanadeTracker::activateExistingPoint(QPoint pos) {
    if (m_trackedObjects.size() > 0) {
        cv::Point2f point = toCv(pos);
        std::vector<InterestPointStatus> filter;
        std::vector<cv::Point2f> newPoints = getCurrentPoints(m_currentFrame, filter);
        size_t currentClosestId = 0;
        double currentMinDist = cv::norm(point - newPoints[0]);
        for (size_t i = 1; i < newPoints.size(); i++) {
            const double dist = cv::norm(point - newPoints[i]);
            if (currentMinDist > dist) {
                currentMinDist = dist;
                currentClosestId = i;
            }
        }
        m_currentActivePoint = currentClosestId;
        Q_EMIT update();
    } else {
        Q_EMIT notifyGUI("There are no points to select");
        m_currentActivePoint = -1;
    }
}

void LucasKanadeTracker::moveCurrentActivePointTo(QPoint pos) {
    if (!m_trackedObjects.empty() && m_currentActivePoint != -1) {
        if (m_currentActivePoint > static_cast<int>(m_trackedObjects.size())) {
            Q_EMIT notifyGUI("Selected point is not in range!");
        } else {
            auto p = std::make_shared<InterestPoint>();
            p->setValidity(true);
            p->setPosition(toCv(pos));
            m_trackedObjects[m_currentActivePoint].add(m_currentFrame, p);
            Q_EMIT update();
        }

    }
}

void LucasKanadeTracker::deleteCurrentActivePoint() {
    if (m_currentActivePoint >= 0) {
        auto o = m_trackedObjects[m_currentActivePoint];
        if (o.hasValuesAtFrame(m_currentFrame)) {
            auto traj = o.get<InterestPoint>(m_currentFrame);
            traj->setValidity(false);
            Q_EMIT update();
        }
    }
}

void LucasKanadeTracker::autoFindInitPoints() {
    if (!m_gray.empty()) {
        //cv::goodFeaturesToTrack(m_gray, m_newPoints, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
        //cv::cornerSubPix(m_gray, m_newPoints, m_subPixWinSize, cv::Size(-1, -1), m_termcrit);
    }
}

std::vector<cv::Point2f> LucasKanadeTracker::getCurrentPoints(
        ulong frameNbr, std::vector<InterestPointStatus> &filter) {
    // TODO: make this implementation more efficient.. please..
    // TODO: find a nicer solution for the filter-issue
    // we want the filter to be empty!
    assert(filter.size() == 0);

    filter.reserve(m_trackedObjects.size());

    std::vector<cv::Point2f> positions;
    positions.reserve(m_trackedObjects.size());

    cv::Point2f dummy(-1, -1);
    for (size_t i = 0; i < m_trackedObjects.size(); i++) {
        auto o = m_trackedObjects[i];
        if (o.hasValuesAtFrame(frameNbr)) {
            auto traj = o.get<InterestPoint>(frameNbr);
            if (!traj->isValid()) {
                filter.push_back(InterestPointStatus::Invalid);

                // TODO: find a nicer solution...
                // we add an offset to the invalid position so that the Lucas-Kanade
                // function does not yield a new position
                positions.push_back(traj->getPosition() + m_invalidOffset);
            } else {
                filter.push_back(InterestPointStatus::Valid);
                positions.push_back(traj->getPosition());
            }
        } else {
            filter.push_back(InterestPointStatus::Non_Existing);
            positions.push_back(dummy);
        }
    }

    // all this trouble with the filter must be done as we directly correlate
    // the index of the vector with the id of the containing object - which has
    // some rather ugly implications for the code.. thus we filter out those
    // points that are not valid

    return positions;
}

void LucasKanadeTracker::updateCurrentPoints(
        ulong frameNbr,
        std::vector<cv::Point2f> positions,
        std::vector<uchar> status,
        std::vector<InterestPointStatus> filter) {
    // TODO: make this implementation more efficient.. please..

    // this must yield true, otherwise we lose the direct index<->id relation
    assert(positions.size() == m_trackedObjects.size());

    // status and positions are directly correlated as each index represents the id
    // of a specific tracked trajectory.
    assert(positions.size() == status.size());

    // filter is directly correlated to the trajectory data as each index represents
    // the id of the trajectory
    assert(filter.size() == positions.size());

    bool somePointsAreInvalid = false;
    for (size_t i = 0; i < positions.size(); i++) {
        if (filter[i] == InterestPointStatus::Valid) {
            auto p = std::make_shared<InterestPoint>(); // TODO: this allocation is not 'pretty' as it is unnecessary
            if (status[i]) {
                p->setValidity(true);
            } else {
                p->setValidity(false);
                somePointsAreInvalid = true;
            }

            p->setPosition(positions[i]);
            m_trackedObjects[i].add(frameNbr, p);
        }
    }

    if (somePointsAreInvalid) {
        Q_EMIT notifyGUI("Some points are invalid");
        if (m_pauseOnInvalidPoint) {
            Q_EMIT pausePlayback(true);
        }
    }
}

cv::Point2f LucasKanadeTracker::toCv(QPoint pos) {
    cv::Point2f point(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
    return point;
}

void LucasKanadeTracker::updateHistorySlider(int frameNbr) {
    if (m_lastTrackedFrame < frameNbr || m_lastTrackedFrame == -1) {
        m_lastTrackedFrame = frameNbr;
    }

    const int currentMaxHist = m_historySlider->maximum();
    const int calculatedMaxHist = maximumHistory();
    if (calculatedMaxHist > currentMaxHist) {
        m_historySlider->setMaximum(calculatedMaxHist);
    }
}

int LucasKanadeTracker::maximumHistory() {
    return m_lastTrackedFrame - m_firstTrackedFrame;
}

void LucasKanadeTracker::updateHistoryText() {
    const int maxSize = maximumHistory();
    m_historyValue->setText(QString::number(m_currentHistory).
        append("/").
        append(QString::number(maxSize)));

}

// ============== GUI HANDLING ==================

void LucasKanadeTracker::checkboxChanged_shouldTrack(int state) {
    m_shouldTrack = state == Qt::Checked;

}

void LucasKanadeTracker::checkboxChanged_invalidPoint(int state) {
    m_pauseOnInvalidPoint = state == Qt::Checked;
}

void LucasKanadeTracker::clicked_validColor() {
    auto *colorDiagNormal = new QColorDialog();
    colorDiagNormal->setCurrentColor(m_validColor);
    QObject::connect(colorDiagNormal, &QColorDialog::colorSelected,
        this, &LucasKanadeTracker::colorSelected_valid);
    colorDiagNormal->open();
}

void LucasKanadeTracker::clicked_invalidColor() {
    auto *colorDiagInvalid = new QColorDialog();
    colorDiagInvalid->setCurrentColor(m_invalidColor);
    QObject::connect(colorDiagInvalid, &QColorDialog::colorSelected,
        this, &LucasKanadeTracker::colorSelected_invalid);
    colorDiagInvalid->open();

}

void LucasKanadeTracker::clicked_print() {
    // TODO: this is a hack (fast and ugly) -> make this nice with
    // Biotracker-ish ways of handling data
    size_t maxTs = 0;
    for (auto o : m_trackedObjects) {
        maxTs = o.maximumFrameNumber() > maxTs ? o.maximumFrameNumber() : maxTs;
    }

    QString output;
    for (size_t frame = 0; frame < maxTs + 1; frame++) {
        for (size_t i = 0; i < m_trackedObjects.size(); i++) {
            auto o = m_trackedObjects[i];
            if (o.hasValuesAtFrame(frame)) {
                auto traj = o.get<InterestPoint>(frame);
                if (traj->isValid()) {
                    output.append(QString::number(frame));
                    output.append(";");
                    output.append(QString::number(i));
                    output.append(";");
                    output.append(QString::number(traj->getPosition().x));
                    output.append(";");
                    output.append(QString::number(traj->getPosition().y));
                    output.append("\n");
                }
            }
        }
    }

    auto fileName = QFileDialog::getExistingDirectory();
    fileName.append(QDir::separator()).append("output.csv");
    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    file.write(output.toLocal8Bit().data(), output.size());
    file.close();

    QString notification("Saved trajectories to file: ");
    notification.append(fileName);
    Q_EMIT notifyGUI(notification.toStdString());

}

void LucasKanadeTracker::colorSelected_invalid(const QColor &color) {
    m_invalidColor = color;
}

void LucasKanadeTracker::colorSelected_valid(const QColor &color) {
    m_validColor = color;
}

void LucasKanadeTracker::sliderChanged_winSize(int value) {
    m_winSize.height = value;
    m_winSize.width = value;
    m_subPixWinSize.height = value;
    m_subPixWinSize.width = value;
    m_winSizeValue->setText(QString::number(value));
}

void LucasKanadeTracker::sliderChanged_history(int value) {
    m_currentHistory = value;
    updateHistoryText();
}
