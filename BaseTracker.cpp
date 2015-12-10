#include "BaseTracker.h"

#include <QApplication>
#include <QIntValidator>
#include <QPushButton>
#include <QPainter>

#include <biotracker/TrackingAlgorithm.h>
#include <biotracker/Registry.h>

extern "C" {
    void registerTracker() {
        BioTracker::Core::Registry::getInstance().registerTrackerType<BaseTracker>("BaseTracker");
    }
}

BaseTracker::BaseTracker(Settings &settings)
    : TrackingAlgorithm(settings)
    , m_toolsFrame(std::make_shared<QFrame>()) {
}

void BaseTracker::track(ulong, const cv::Mat &imgOriginal) {
}

void BaseTracker::paint(cv::Mat &image, const TrackingAlgorithm::View &view) {
}

void BaseTracker::paintOverlay(QPainter *painter, const View &view) {
}
