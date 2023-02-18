#include <functional>
#include <memory>

#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

/**
 * @brief Class containing the LightPositionIndicator.
 *
 */
class LightPositionIndicator : public rclcpp::Node
{
public:
    // Member funciton declaration
    LightPositionIndicator();
    std::tuple<int, int> getCOG(const sensor_msgs::msg::Image &image_msg);

private:
    // Node member pointers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pSubscription;

    // Member function declration
    void imageCaptureCallback(const sensor_msgs::msg::Image &image_msg);
};