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
    const int MAX_COUNT = 500;
    cv::Mat m_gray;
    cv::Mat m_prevGray;
    size_t m_currentFrame;

    /**
     * @brief m_invalidOffset
     */
    const cv::Point2f m_invalidOffset;

    /**
     * @brief m_currentActivePoint
     * The currently active point that can be moved by the mouse curor
     */
    int m_currentActivePoint = -1;
    // --

    void mouseReleaseEvent(QMouseEvent *e) override;

    /**
     * @brief createNewPoint
     * Tries to add a new point, if it is not too close to an already
     * existing one
     * @param pos
     */
    void tryCreateNewPoint(QPoint pos);


    void activateExistingPoint(QPoint pos);
    void moveCurrentActivePointTo(QPoint pos);

    /**
     * @brief autoFindInitPoints
     * call this when you want to find let the program find interesting
     * points
     */
    void autoFindInitPoints();

    /**
     * @brief getCurrentPoints
     * gets the locations of all points at the given timeframe
     * @param frameNbr defines the current track-iteratioyn
     * @param filter: marks those indices that point to an invalid
     * 	(for whatever reason) trajectory => OUT-parameter
     *  The index represents the id of the trajectory data
     * @return the list of points with positions
     */
    std::vector<cv::Point2f> getCurrentPoints(
        ulong frameNbr,
        std::vector<InterestPointStatus> &filter);

    /**
     * @brief updateCurrentPoints
     * update the current locations to the serializable trajectory data
     * for the current frame
     * @param frameNbr defines the current track-iteration
     * @param pos list of updated positions, the index equals the
     * the element id
     * @param status defines the list of object status` that happened to each
     * object
     * @param filter: marks those indices that point to an invalid
     * 	(for whatever reason) trajectory
     *  The index represents the id of the trajectory data
     */
    void updateCurrentPoints(
        ulong frameNbr,
        std::vector<cv::Point2f> pos,
        std::vector<uchar> status,
        std::vector<InterestPointStatus> filter);

    cv::Point2f toCv(QPoint p);

};
