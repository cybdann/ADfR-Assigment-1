#include "cam_capture/light_position_indicator.hpp"

/**
 * @brief Construct a new BrightnessLevelDetector object
 *
 */
LightPositionIndicator::LightPositionIndicator() : Node("light_position_indicator")
{
    // Subscription declaration and callback binding
    pSubscription = this->create_subscription<sensor_msgs::msg::Image>("image", 1, std::bind(&LightPositionIndicator::imageCaptureCallback, this, _1));

    // Publisher declaraction and callback binding
    pPublisher = this->create_publisher<std_msgs::msg::String>("light_source_cog", 10);

    // Timer for callback
    pTimer = this->create_wall_timer(500ms, std::bind(&LightPositionIndicator::lightSourceCOGCallback, this));

}

/**
 * @brief Get the COG of the light source
 * 
 * @return std::array<int, 2> 
 */
std::array<int, 2> LightPositionIndicator::getLightSourceCOG()
{
    return this->mLightSourceCOG;
}

/**
 * @brief Set the COG of the light source
 * 
 * @param cog 
 */
void LightPositionIndicator::setLightSourceCOG(std::array<int, 2> cog)
{
    this->mLightSourceCOG = cog;
}

/**
 * @brief Calculate light position from image.
 *
 * @param image_msg captured image.
 * @return double
 */
void LightPositionIndicator::calculateCOG(const sensor_msgs::msg::Image &image_msg)
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
        this->setLightSourceCOG({-1, -1});
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

    this->setLightSourceCOG({int(center_of_gravity.x), int(center_of_gravity.y)});
}

/**
 * @brief Image capture callback funciton.
 *
 * @param image_msg
 */
void LightPositionIndicator::imageCaptureCallback(const sensor_msgs::msg::Image &image_msg)
{
    // Convert to grayscale image
    calculateCOG(image_msg);
}

/**
 * @brief Callback to publish the COG of the light source
 * 
 */
void LightPositionIndicator::lightSourceCOGCallback()
{
    // Get COG
    std::array<int, 2> COG = this->getLightSourceCOG();
    
    // Message to be published
    std_msgs::msg::String msg;

    // There is no light source present
    if(COG[0] < 0 || COG[1] < 0)
    {
        // Log data
        RCLCPP_INFO(this->get_logger(), "No light source present!");

        // Publish data
        pPublisher->publish(msg.set__data("No light source!"));
    }
    else
    {
        // Concat int to string in form of [x,y]
        std::string temp = std::to_string(COG[0]) + "," + std::to_string(COG[1]);
        
        // Log data
        RCLCPP_INFO(this->get_logger(), "COG: x=%d | y=%d", COG[0], COG[1]);

        // Publish value
        pPublisher->publish(msg.set__data(temp));
    }
}


