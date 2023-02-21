#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


/**
 * @brief Setpoint class to represent a setpoint of pan(x) and tilt(y)
 */
class Setpoint {
public:
    double x, y;

    Setpoint(double x = 0, double y = 0) : x(x), y(y) {}
};



/**
 * @brief SetpointGenerator class to generate a sequence of setpoints
 */ 
class SetpointGenerator {
public:
    // Constructor to set the number of setpoints to generate
    SetpointGenerator(int numSetpoints) : m_numSetpoints(numSetpoints) {}

    // Generate the sequence of setpoints
    std::vector<Setpoint> generateSetpoints() {
        std::vector<Setpoint> setpoints;

        // Generate setpoints with random x between -0.8 and 1.6, y values between -0.6 and 1.2
        for (int i = 0; i < m_numSetpoints; i++) {
            double x = (double)rand() / RAND_MAX * 1.6 - 0.8;
            double y = (double)rand() / RAND_MAX * 1.2 - 0.6;

            Setpoint setpoint(x, y);
            setpoints.push_back(setpoint);
        }

        return setpoints;
    }

private:
    int m_numSetpoints;
};

/**
 * @brief Minimal publisher example
 */

class Publisher : public rclcpp::Node{
    public: 
        MinimalPublisher() : Node("minimal_publisher"), count_(0){ // Constructor
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // Create a publisher
            timer_ = this->create_wall_timer( 
                500ms, std::bind(&MinimalPublisher::timer_callback, this)); // Create a timer
        }

    private:
        void timer_callback(){
            auto message = std_msgs::msg::String(); // Create a message
            message.data = "Hello, world! " + std::to_string(count_++); // Set the message
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // Log the message
            publisher_->publish(message); // Publish the message
        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[]) {
    // Create SetpointGenerator with 10 setpoints
    SetpointGenerator generator(10);

    // Generate the setpoints
    std::vector<Setpoint> setpoints = generator.generateSetpoints();

    // Output the generated setpoints
    for (int i = 0; i < setpoints.size(); i++) {
        std::cout << "Setpoint " << i+1 << ": (" << setpoints[i].x << ", " << setpoints[i].y << ")" << std::endl;
    }
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();

    return 0;
}