#pragma once

#include <opencv2/opencv.hpp>
#include <biotracker/serialization/ObjectModel.h>

class InterestPoint : public ObjectModel {
public:
    InterestPoint();
    virtual ~InterestPoint();
    void setPosition(cv::Point pos);
    cv::Point getPosition();

private:
    cv::Point m_position;
};
