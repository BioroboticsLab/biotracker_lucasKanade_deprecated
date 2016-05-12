#include "InterestPoint.h"

InterestPoint::InterestPoint(): ObjectModel(), m_userStatus(0) {

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

void InterestPoint::addToUserStatus(const size_t i) {
    if (interestPointMaximumUserStatus <= i) {
        // when we arrive here, we cannot represent any
        // new user state in a bit-wise integer way. Thus,
        // we have to raise an exception..
        throw std::out_of_range("UserStates cannot grow anymore!");
    }

    const size_t bitRep = static_cast<size_t>(1) << i;
    m_userStatus = m_userStatus | bitRep;
}

void InterestPoint::removeFromUserStatus(const size_t i) {
    if (interestPointMaximumUserStatus <= i) {
        throw std::out_of_range("removeFromUserStatus OOR =>" + std::to_string(i));
    }

    const size_t bitRep = ~(1 << i);
    m_userStatus = m_userStatus & bitRep;
}
