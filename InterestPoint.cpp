#include "InterestPoint.h"

InterestPoint::InterestPoint(): ObjectModel() {

}

InterestPoint::~InterestPoint() {}

void InterestPoint::setPosition(cv::Point pos) {
    m_position = pos;
}

cv::Point InterestPoint::getPosition() {
    return m_position;
}
