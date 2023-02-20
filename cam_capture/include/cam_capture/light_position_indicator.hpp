#include <functional>
#include <memory>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
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
    void calculateCOG(const sensor_msgs::msg::Image &image_msg);

    // Getter
    std::array<int, 2> getLightSourceCOG();

    // Setter
    void setLightSourceCOG(std::array<int, 2> cog);

private:
    // Node member pointers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pSubscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pPublisher;
    rclcpp::TimerBase::SharedPtr pTimer;

    // Member variables
    std::array<int, 2> mLightSourceCOG{-1, -1}; // Initially no light source

    // Member function declration
    void imageCaptureCallback(const sensor_msgs::msg::Image &image_msg);
    void lightSourceCOGCallback();
};