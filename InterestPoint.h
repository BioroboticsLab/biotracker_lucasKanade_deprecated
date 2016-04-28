#pragma once

#include <opencv2/opencv.hpp>
#include <biotracker/serialization/ObjectModel.h>

/**
 * @brief The InterestPointStatus enum
 * Show all the stati that the intrest points can yield
 */
enum class InterestPointStatus {
    Valid, // The point is valid and can be tracked
    Invalid,	// the point is not valid (due to the tracking)
                // and should not be tracked!
    Non_Existing	// the point does not exist yet! (because the user
};					// jumped back in time

const size_t interestPointMaximumUserStatus = sizeof(size_t) * 8;

class InterestPoint : public BioTracker::Core::ObjectModel {
public:
    InterestPoint();
    virtual ~InterestPoint();
    void setPosition(cv::Point2f pos);
    cv::Point2f getPosition();
    bool isValid();
    void setValidity(bool v);

    /**
     * @brief addToUserStatus
     * @param i
     */
    void addToUserStatus(const size_t i);

    /**
     * @brief removeFromUserStatus
     * @param i
     */
    void removeFromUserStatus(const size_t i);

    size_t getStatusAsI() {
        return m_userStatus;
    }

private:
    bool m_isValid = true;
    cv::Point2f m_position;
    size_t m_userStatus;
};
