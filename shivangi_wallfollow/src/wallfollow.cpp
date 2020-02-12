#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <stdlib.h>

#define PI 3.14

class WallFollow{
private:
	ros::NodeHandle n;
	ros::Publisher ack_pub;
	ros::Publisher gen_pub;
	ros::Subscriber scan_sub; 
	float kp = 0.04;
	float kd;
	float ki;
	float p_error = 0.0;
	float p_error_last = 0.0;
	float d_error = 0.0;
	float i_error = 0.0;
	float dist_to_maintain = 0.7;
	float lookahead_dist = 0.4;
	float steer_input = 0.0;
	float velocity = 0.0; 

public:
	WallFollow(){
		n = ros::NodeHandle();
		ack_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);
		scan_sub = n.subscribe("scan", 100, &WallFollow::scan_callback, this);
		gen_pub = n.advertise<std_msgs::Bool>("debug", 1);
	}

	~WallFollow(){}

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
		ackermann_msgs::AckermannDriveStamped drive_val;
		std_msgs::Bool dist;
		float inc = scan_msg->angle_increment;

		int left_wall_1 = floor((3*PI)/(2*inc));
		int left_wall_2 = floor((11*PI)/(6*inc));
		float left_range_1 = scan_msg->ranges[left_wall_1];
		float left_range_2 = scan_msg->ranges[left_wall_2]; 
 		float steer_angle_1 = atan((left_range_2*cos(PI/3) - left_range_1)/(left_range_2*sin(PI/3)));
 		float dist_to_leftwall = left_range_1*cos(steer_angle_1);

 		int right_wall_1 = floor((PI)/(2*inc));
		int right_wall_2 = floor((5*PI)/(6*inc));
		float right_range_1 = scan_msg->ranges[right_wall_1];
		
 		if(abs(steer_angle_1) <= (PI/18)){
 			velocity = 1.5;
 			lookahead_dist = 0.0;
 			kp = 0.4;
 			kd = 1.1;
 			ki = 0.0;
 			dist_to_maintain = 0.7;

 		}else if(abs(steer_angle_1) > (PI/18) && abs(steer_angle_1) <= (PI/9)){
 			velocity = 1.0;
 			lookahead_dist = 0.1;
            dist_to_maintain = 0.7;
 			kp = 0.5;
 			kd = 10.0;
 			ki = 0.0;
 		}else{
 			velocity = 0.5;
 			lookahead_dist = 0.01;
 			dist_to_maintain = 0.7;
 			kp = 0.5;
 			kd = 10.0;
 			ki = 0.0;
 		}

 		float dist_to_leftwall_la = dist_to_leftwall + lookahead_dist*sin(steer_angle_1);

 		float p_error = dist_to_maintain - dist_to_leftwall_la;
 		float d_error = p_error - p_error_last; 
 		float i_error = i_error + p_error;

 		bool is_special = detect_special_crosssection(steer_input, scan_msg);

 		steer_input = -(kp*p_error - kd*d_error + ki*i_error);
 		if(steer_input>= 0.43){
 			steer_input = 0.43;
 		}
 		if(steer_input <= -0.43){
 			steer_input = -0.43;
 		}
 		drive_val.drive.steering_angle = steer_input;
 		drive_val.drive.speed = velocity;
        dist.data = is_special;
 		ack_pub.publish(drive_val);
 		gen_pub.publish(dist);
 		p_error_last = p_error;
	}

	bool detect_special_crosssection(float steer_angle, const sensor_msgs::LaserScan::ConstPtr &scan_msg){
		bool special_case = false;
		float inc = scan_msg->angle_increment;

		int extra_ind = floor(steer_angle/inc);
		int front_ind = floor(PI/inc);
		int back_ind = 0;
		int left_ind = floor((3*PI/2)/inc);
		int right_ind = floor((PI/2)/inc);

		float front_range = scan_msg->ranges[front_ind + extra_ind];
		float back_range = scan_msg->ranges[back_ind + extra_ind];
		float left_range = scan_msg->ranges[left_ind + extra_ind];
		float right_range = scan_msg->ranges[right_ind + extra_ind];

		if(front_range > 30 || isinf(front_range)){
			if(back_range > 5 || isinf(back_range)){
				if(left_range > 1 && right_range > 1.5){
					special_case = true;
				}
			}
		}

		return special_case;
	}
 
};

int main(int argc, char **argv){
	ros::init(argc, argv, "wallfollow");
	WallFollow w;
	ros::spin();
    return 0;
}