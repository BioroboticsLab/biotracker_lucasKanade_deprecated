#include "LucasKanade.h"

#include <QApplication>
#include <QIntValidator>
#include <QPushButton>
#include <QPainter>
#include <QColorDialog>

#include <biotracker/TrackingAlgorithm.h>
#include <biotracker/Registry.h>

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
    m_invalidOffset(-99999, -99999),
    m_validColor(QColor::fromRgb(0, 0, 255)),
    m_invalidColor(QColor::fromRgb(255, 0, 0)){

    // initialize gui
    auto ui = getToolsWidget();
    auto layout = new QVBoxLayout();

    // Checkbox for tracking enable/disable
    auto *chkboxShouldTrack = new QCheckBox("Tracking enabled", ui);
    chkboxShouldTrack->setChecked(true);
    QObject::connect(chkboxShouldTrack, &QCheckBox::stateChanged,
        this, &LucasKanadeTracker::checkboxChanged_shouldTrack);
    layout->addWidget(chkboxShouldTrack);

    // Checkbox for pausing on invalid points
    auto *chkboxInvalidPoints = new QCheckBox("Pause on invalid Point", ui);
    chkboxInvalidPoints->setChecked(false);
    QObject::connect(chkboxInvalidPoints, &QCheckBox::stateChanged,
        this, &LucasKanadeTracker::checkboxChanged_invalidPoint);
    layout->addWidget(chkboxInvalidPoints);

    // colors
    auto validColorBtn = new QPushButton("Valid color", ui);
    QObject::connect(validColorBtn, &QPushButton::clicked,
        this, &LucasKanadeTracker::clicked_validColor);
    layout->addWidget(validColorBtn);

    auto invalidColorBtn = new QPushButton("Invalid color", ui);
    QObject::connect(invalidColorBtn, &QPushButton::clicked,
        this, &LucasKanadeTracker::clicked_invalidColor);
    layout->addWidget(invalidColorBtn);

    // ===

    ui->setLayout(layout);
}

void LucasKanadeTracker::track(ulong frame, const cv::Mat &imgOriginal) {
    const int perc_size = 45;
    m_itemSize = imgOriginal.cols > imgOriginal.rows ? imgOriginal.rows / perc_size : imgOriginal.cols / perc_size;

    bool isStepForward = m_currentFrame == (frame - 1);
    m_currentFrame = frame; // TODO must this be protected from other threads?
    if ((frame == 0 || isStepForward) && m_shouldTrack) {
        cv::cvtColor(imgOriginal, m_gray, cv::COLOR_BGR2GRAY);

        std::vector<InterestPointStatus> filter;
        std::vector<cv::Point2f> currentPoints = getCurrentPoints(frame - 1, filter);


        if (!currentPoints.empty()) {
        std::vector<float> err;
        if (m_prevGray.empty()) {
            m_gray.copyTo(m_prevGray);
        }

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
            0, /* maxLevel */
            m_termcrit,	/* criteria */
            0, /* flags */
            0.001 /* minEigThreshold */
        );

        updateCurrentPoints(frame, newPoints, status, filter);
        }

        cv::swap(m_prevGray, m_gray);
    }
}

void LucasKanadeTracker::paint(ulong, ProxyMat &, const TrackingAlgorithm::View &) {

}

void LucasKanadeTracker::paintOverlay(ulong, QPainter *painter, const View &) {

    std::vector<InterestPointStatus> filter;
    std::vector<cv::Point2f> newPoints = getCurrentPoints(m_currentFrame, filter);

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
        p.setWidth(m_itemSize / 10 > 0 ? m_itemSize / 5 : 1);

        int itemSizeHalf = m_itemSize / 2;

        painter->setPen(p);
        painter->drawEllipse(x - itemSizeHalf, y - itemSizeHalf, m_itemSize, m_itemSize);
        auto idTxt = QString::number(i);
        painter->drawText(x, y - itemSizeHalf, idTxt);
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

void LucasKanadeTracker::colorSelected_invalid(const QColor &color) {
    m_invalidColor = color;
}

void LucasKanadeTracker::colorSelected_valid(const QColor &color) {
    m_validColor = color;
}
