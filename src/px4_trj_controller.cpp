#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using std::placeholders::_1;

class TrajectoryControl : public rclcpp::Node
{
	public:
		TrajectoryControl() : Node("trajectory_control")
		{
			const rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			const auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
			vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      			"fmu/out/vehicle_status",
				qos,
				std::bind(&TrajectoryControl::vehicle_status_callback, this, _1)
			);

			offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

			auto timer_callback = [this]() -> void {
				RCLCPP_INFO(this->get_logger(), "current state: %i", trh_state_);
				switch (trh_state_)
				{
				case STANDBY:
					if( nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD ){
						trh_state_ = IN_OFFBOARD;
					}
					break;
				case IN_OFFBOARD:
					publish_offboard_control_mode();
					if( arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
						trh_state_ = ARMED;
						trj_start_time_ = this->get_clock()->now();
					}
					break;
				case ARMED:
					publish_offboard_control_mode();
					publish_trajectory_setpoint(rclcpp::Duration(this->get_clock()->now() - trj_start_time_).seconds());
					break;
				
				default:
					break;
				}
			};
			timer_ = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
		}

	private:

		void vehicle_status_callback(const px4_msgs::msg::VehicleStatus & msg)
		{
			this->nav_state_ = (msg.nav_state);
			this->arming_state_ = msg.arming_state;
		}

		void publish_offboard_control_mode()
		{
			px4_msgs::msg::OffboardControlMode msg{};
			msg.position = true;
			msg.velocity = true;
			msg.acceleration = false;
			msg.attitude = false;
			msg.body_rate = false;
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			offboard_control_mode_publisher_->publish(msg);
		}

		void generate_trajectory_setpoint(double t, std::array<double,3UL> &p, float &yaw){
			
			// modify here p and yaw, ENU frame!
			// t starts from 0
			// take into account the takeoff time!
			p[2] = 4;
			p[0] = 3*cos(2*M_PI*0.2*t);
			p[1] = 3*sin(2*M_PI*0.2*t);

			// FLU/ENU 2 ENU
			yaw = M_PI_2 - yaw;
			// to do, yaw wrapping!

			Eigen::Vector3d p_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(Eigen::Vector3d(p.data()));
			p[0] = p_ned[0];
			p[1] = p_ned[1];
			p[2] = p_ned[2];
		}

		void publish_trajectory_setpoint(double t)
		{
			std::array<double,3UL> p = {1.0, 0.0, 4.0};
			float yaw = 0;

			generate_trajectory_setpoint(t, p, yaw);

			px4_msgs::msg::TrajectorySetpoint msg{};
			msg.position[0] = p[0];
			msg.position[1] = p[1];
			msg.position[2] = p[2];
			msg.yaw = yaw; // [-PI:PI]
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher_->publish(msg);
		}

		enum State
		{
			STANDBY, 
			IN_OFFBOARD, 
			ARMED
		} trh_state_;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

		uint8_t nav_state_;
		uint8_t arming_state_;
		rclcpp::Time trj_start_time_;
};

int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;

	std::cout << "Starting trajectory control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryControl>());

	rclcpp::shutdown();
	return 0;
}
