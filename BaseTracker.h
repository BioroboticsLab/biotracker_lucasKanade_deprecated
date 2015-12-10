#pragma once

#include <QGroupBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>

#include <biotracker/TrackingAlgorithm.h>

class BaseTracker : public TrackingAlgorithm {
    Q_OBJECT
  public:
    BaseTracker(Settings &settings);

    void track(ulong frameNumber, const cv::Mat &frame) override;
    void paint(cv::Mat &m, View const &view = OriginalView) override;
    void paintOverlay(QPainter *painter) override;

    void postConnect() override;

    std::shared_ptr<QWidget> getToolsWidget() override;

  public Q_SLOTS:
    void changeFilterColor();
    void paintOverlay(QPainter *painter, View const &view = OriginalView) override;

  private:
    std::shared_ptr<QFrame> m_toolsFrame;
};
