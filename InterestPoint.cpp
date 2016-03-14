#include "InterestPoint.h"

InterestPoint::InterestPoint(): ObjectModel() {

}

InterestPoint::~InterestPoint() {}

void InterestPoint::setPosition(cv::Point2f pos) {
    m_position.x = pos.x;
    m_position.y = pos.y;
}

cv::Point2f InterestPoint::getPosition() {
    return m_position;
}

bool InterestPoint::isValid() {
    return m_isValid;
}

void InterestPoint::setValidity(bool v) {
    m_isValid = v;
}
