#include "LucasKanade.h"

#include <QApplication>
#include <QIntValidator>
#include <QPushButton>
#include <QPainter>

#include <biotracker/TrackingAlgorithm.h>
#include <biotracker/Registry.h>

extern "C" {
    void registerTracker() {
        BioTracker::Core::Registry::getInstance().registerTrackerType<LucasKanadeTracker>("LucasKanadeTracker");
    }
}

LucasKanadeTracker::LucasKanadeTracker(Settings &settings):
    TrackingAlgorithm(settings),
    m_subPixWinSize(10, 10),
    m_winSize(31, 31),
    //m_termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20,0.03) {
    m_termcrit(cv::TermCriteria::COUNT, 20, 0.03) {
}

void LucasKanadeTracker::track(ulong frame, const cv::Mat &imgOriginal) {
    cv::cvtColor(imgOriginal, m_gray, cv::COLOR_BGR2GRAY);

    if (!m_currentPoints.empty()) {

        cv::Mat err;
        if (m_prevGray.empty()) {
            m_gray.copyTo(m_prevGray);
        }
        cv::calcOpticalFlowPyrLK(m_prevGray, m_gray, m_currentPoints, m_newPoints, m_status, err, m_winSize,
            3, m_termcrit, 0, 0.001);
    }
    std::swap(m_newPoints, m_currentPoints);
    std::swap(m_prevGray, m_gray);
}

void LucasKanadeTracker::paint(ProxyMat &, const TrackingAlgorithm::View &) {

}

void LucasKanadeTracker::paintOverlay(QPainter *painter, const View &view) {
    size_t i;
    for (i = 0; i < m_newPoints.size(); i++) {
        if (!m_status[i]) {
            // the i`th point became invalid
            continue;
        }

        auto point = m_newPoints[i];
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);
        painter->setPen(QColor::fromRgb(255, 0, 0));
        painter->drawEllipse(x, y, 15, 15);
    }


}

void LucasKanadeTracker::mouseReleaseEvent(QMouseEvent *e)
{
    switch(e->modifiers()) {
        case Qt::NoModifier: {
        this->moveCurrentActivePointTo(e->pos());
        break;
    }
    case Qt::ShiftModifier: {
        this->activateExistingPoint(e->pos());
        break;
    }
    case Qt::ControlModifier: {
        this->createNewPoint(e->pos());
        break;
    }
    }

}

void LucasKanadeTracker::createNewPoint(QPoint pos) {
    if (m_gray.empty()) {
        // this usually happens when users try to add interest points upon
        // starting the video as then there is no track being called yet..
        Q_EMIT forceTracking();
    } else {
        cv::Point2f point(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
        for (auto otherPos : m_newPoints) {
        if (cv::norm(point - otherPos) <= 5) {
            // the new point is too close to an existing other point.. abort
            Q_EMIT notifyGUI("too close to an existing point..");
            return;
        }
        }
        std::vector<cv::Point2f> tmp;
        tmp.push_back(point);
        cv::cornerSubPix(m_gray, tmp, m_winSize, cv::Size(-1, -1), m_termcrit);
        m_newPoints.push_back(tmp[0]);
    }
}

void LucasKanadeTracker::activateExistingPoint(QPoint pos)
{

}

void LucasKanadeTracker::moveCurrentActivePointTo(QPoint pos)
{

}

void LucasKanadeTracker::autoFindInitPoints() {
    if (!m_gray.empty()) {
        cv::goodFeaturesToTrack(m_gray, m_newPoints, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
        cv::cornerSubPix(m_gray, m_newPoints, m_subPixWinSize, cv::Size(-1, -1), m_termcrit);
    }
}

void LucasKanadeTracker::tryAddNewPoint(cv::Point2f pos)
{
}
