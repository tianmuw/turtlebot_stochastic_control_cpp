#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "irobot_create_msgs/action/navigatetoposition.hpp"
#include "marvelmind_ros2_msgs/msg/hedgepositionaddressed.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <queue>
#include <random>
#include <Eigen/Dense>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <cmath>

class StochasticControlNode : public rclcpp::Node
{
private:
    // Robot namespace
    std::string robotNamespace;

    // State: [x, y] 2d positions.
    Eigen::Vector2f state;
    // Goal position
    Eigen::Vector2f goalState;
    // Static obstacles
    Eigen::MatrixXf obs;
    // Distance
    float d;
    // Safe margin
    float theta;
    // 
    bool achieveGoalHeading = true;

    // Topic name
    std::string marvelTopic;
    std::string acTopic;

    // Qos
    rclcpp::QoS qosProfileSensorData;

    // Topic subscription
    rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePositionAddressed>::SharedPtr subMarvel;

    // Action
    rclcpp::Client<irobot_create_msgs::action::NavigateToPosition>::SharedPtr navClient;
    rclcpp::Subscription<irobot_create_msgs::action::NavigateToPosition::Result>::SharedPtr subResult;
    std::shared_ptr<irobot_create_msgs::action::NavigateToPosition::Goal> navGoal;

    // Timer
    rclcpp::TimerBase::SharedPtr ownTimer;


    // Marvelmind coordinate system
    void marvelCallback(const marvelmind_ros2_msgs::msg::HedgePositionAddressed marvelMsg)
    {
        RCLCPP_INFO(this->get_logger(), "Received the marvel messages.");
        std::cout<< "Current position is " 
                << "x = " << marvelMsg->x_m 
                << ", y = " << marvelMsg->y_m
                <<std::endl;
    }


    // Timer 
    void timerCallback()
    {
        while((state - goalState).norm() > 0.05)
        {
            state = stochasticControl(state, goalState, obs, d, theta);
            navSendGoal(state, achieveGoalHeading);
        }
    }

public:

    StochasticControlNode(const rclcpp::NodeOptions & options) : Node("stochastic_control", options)
    {
        // Get the basic parameters
        robotNamespace = this->get_parameter("robot_namespace");
        state = this->get_parameter("init_state");
        goalState = this->get_parameter("goal_state");
        obs = this->get_parameter("static_obstacles");
        d = this->get_parameter("distance");
        theta = this->get_parameter("safe_margin");

        // Qos
        qosProfileSensorData = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Subcription
        subMarvel = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePositionAddressed>(marvelTopic, 
                                            qosProfileSensorData, std::bind(&StochasticControlNode::marvelCallback, this, _1));

        // Action client
        navClient = this->create_client<irobot_create_msgs::action::NavigateToPosition>(acTopic);
        subResult = this->create_subscription<irobot_create_msgs::action::NavigateToPosition::Result>(
                acTopic + "/result",
                std::bind(&StochasticControlNode::getResult, this, std::placeholders::_1));
        navGoal = std::make_shared<irobot_create_msgs::action::NavigateToPosition::Goal>();

        ownTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StochasticControlNode::timerCallback, this));
    }

    void navSendGoal(Eigen::Vector2f state, bool achieveGoalHeading)
    {
        if (!navClient->wait_for_action_server(std::chrono::seconds(1))) 
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            return;
        }
        navGoal->goal_pose.header.stamp = this->get_clock()->now();
        navGoal->goal_pose.header.frame_id = "marvelmind-073";
        navGoal->goal_pose.pose.position.x = state(0);
        navGoal->goal_pose.pose.position.y = state(1);
        navGoal->achieve_goal_heading = achieveGoalHeading;

        auto sendGoalOptions_ = rclcpp_action::Client<irobot_create_msgs::action::NavigateToPosition>::SendGoalOptions();

        sendGoalOptions_.goal_response_callback =
            [](std::shared_future<rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::NavigateToPosition>::SharedPtr> future) 
        {
            auto goalHandle = future->reulst;
            if(! goalHandle.accepted)
            {
                RCLCPP_INFO(this->get_logger(), "Goal rejected!");
            }
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        };

        sendGoalOptions_.feedback_callback =
            [](rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::NavigateToPosition>::SharedPtr,
            const std::shared_ptr<const irobot_create_msgs::action::NavigateToPosition::Feedback>) 
        {
            RCLCPP_INFO(this->get_logger(), "Got feedback");
        };

        auto goalHandleFuture = navClient->async_send_goal(navGoal, sendGoalOptions_);
    }

    void getResult(const irobot_create_msgs::action::NavigateToPosition::Result::SharedPtr result)
    {
        RCLCPP_INFO(this->get_logger(), "Got result: ");
        geometry_msgs::msg::PoseStamped resultPose = result->pose;
        std::cout << resultPose << std::endl;
    }

    // Stochastic Controller
    Eigen::Vector2f stochasticControl(Eigen::Vector2f state, Eigen::Vector2f goalState, Eigen::MatrixXf obs,
                                        float d, float safeMargin)
    {
        Eigen::Vector2f dx;
        float c;
        float r;
        Eigen::Vector2f newState;
        bool safe = false;
        float stdValue;
        int rowsNum = obs.rows();
        int colsNum = obs.cols();

        dx = goalState - state;
        c = dx.norm();
        r = d / c;
        newState = state + r * dx;
        stdValue = d / std::sqrt(2);


        while(!safe)
        {
            newState = newState + Eigen::Vector2f::Random(2, 1) + stdValue;

            for(int i=0; i<rowsNum; i++)
            {
                Eigen::VectorXf ob = obs.row(i);
                float normValue = (newState - ob).norm();
                if(normValue <= safeMargin)
                {
                    safe = false;
                    break;
                }
                else
                {
                    safe = true;
                }
            }

        }
        return newState;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    //Declare the init state [x, y]
    Eigen::Vector2f initState {{1.5, 2.0}};
    Eigen::Vector2f goalState {{4.5, 5.0}};
    Eigen::MatrixXf obs {
        {1.0, 2.0},
        {3.5, 2.5},
        {2.5, 3.0}
    };
    float d = 1.0;
    float safeMargin = 0.5;

    //auto node = std::make_shared<StochasticControlNode>();
    auto node = std::make_shared<StochasticControlNode>(rclcpp::NodeOptions());
    node->declare_parameter<std::string>("robot_namespace", "yosemite");
    node->declare_parameter<Eigen::Vector2f>("init_state", initState);
    node->declare_parameter<Eigen::Vector2f>("goal_state", goalState);
    node->declare_parameter<Eigen::MatrixXf>("static_obstacles", obs);
    node->declare_parameter<float>("distance", d);
    node->declare_parameter<float>("safe_margin", safeMargin);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}