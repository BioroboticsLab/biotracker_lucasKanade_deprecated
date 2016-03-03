#pragma once

#include <QGroupBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <biotracker/TrackingAlgorithm.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctype.h>

#include "InterestPoint.h"

/*
 * Inspiered by:
 * https://github.com/Itseez/opencv/blob/master/samples/cpp/lkdemo.cpp
 */
class LucasKanadeTracker : public TrackingAlgorithm {
    Q_OBJECT
  public:
    LucasKanadeTracker(Settings &settings);

    void track(ulong frameNumber, const cv::Mat &frame) override;
    void paint(ProxyMat &m, View const &view = OriginalView) override;
    void paintOverlay(QPainter *painter, View const &view = OriginalView) override;

  private:
    // --
    const cv::Size m_subPixWinSize;
    const cv::Size m_winSize;
    cv::TermCriteria m_termcrit;
    std::vector<cv::Point2f> m_currentPoints;
    std::vector<cv::Point2f> m_newPoints;
    std::vector<uchar> m_status;
    const int MAX_COUNT = 500;
    cv::Mat m_gray;
    cv::Mat m_prevGray;
    // --

    void mouseReleaseEvent(QMouseEvent *e) override;


    void createNewPoint(QPoint pos);
    void activateExistingPoint(QPoint pos);
    void moveCurrentActivePointTo(QPoint pos);

    /**
     * @brief autoFindInitPoints
     * call this when you want to find let the program find interesting points
     */
    void autoFindInitPoints();

    /**
     * @brief addNewPoint
     * Tries to add a new point, if it is not too close to an already existing one
     * @param pos
     */
    void tryAddNewPoint(cv::Point2f pos);
};
