#include "cam_capture/light_position_indicator.hpp"

/**
 * @brief Construct a new BrightnessLevelDetector object
 *
 */
LightPositionIndicator::LightPositionIndicator() : Node("light_position_indicator")
{
    // Subscription declaration and callback binding
    pSubscription = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&LightPositionIndicator::imageCaptureCallback, this, _1));
}

/**
 * @brief Image capture callback funciton.
 *
 * @param image_msg
 */
void LightPositionIndicator::imageCaptureCallback(const sensor_msgs::msg::Image &image_msg)
{
    // Convert to grayscale image
    std::tuple<int, int> coordinates = getCOG(image_msg);

    // Log data
    RCLCPP_INFO(this->get_logger(), "COG of light: x=%d | y=%d", std::get<0>(coordinates), std::get<1>(coordinates));
}

/**
 * @brief Calculate light position from image.
 *
 * @param image_msg captured image.
 * @return double
 */
std::tuple<int, int> LightPositionIndicator::getCOG(const sensor_msgs::msg::Image &image_msg)
{
    // OpenCV image poitner
    cv_bridge::CvImagePtr cv_image;

    // Try to convert to OpenCV image
    try
    {
        cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "ERROR: cv_bridge exception: %s", e.what());
        return std::tuple<int, int>{-1, -1};
    }

    cv::Mat grayscale_image, binary_image;

    // Convert image to grayscale
    cv::cvtColor(cv_image->image, grayscale_image, cv::COLOR_BGR2GRAY);

    // Threshold the grayscaled image from value 180 to MAX_VAL with binary tresholding option
    cv::threshold(grayscale_image, binary_image, 180, 255, cv::THRESH_BINARY);

    // Get moment of binary image
    cv::Moments moments = cv::moments(binary_image, true);

    // Compute center of gravity
    cv::Point2f center_of_gravity;
    center_of_gravity.x = moments.m10 / moments.m00;
    center_of_gravity.y = moments.m01 / moments.m00;

    return std::tuple<int, int>{int(center_of_gravity.x), int(center_of_gravity.y)};
}