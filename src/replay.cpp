#include "ros/ros.h"
#include "Event.h"
#include "Episodes.h"
#include "ParticleFilter.h"
#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h> 
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "raspimouse_ros_2/LightSensorValues.h"
#include "raspimouse_ros_2/TimedMotion.h"
#include "raspimouse_ros_2/ButtonValues.h"
#include "sensor_msgs/LaserScan.h"
#include "raspimouse_gamepad_teach_and_replay/Event.h"
#include "ParticleFilter.h"
#include "raspimouse_gamepad_teach_and_replay/PFoEOutput.h"
using namespace ros;

Episodes ep;
ParticleFilter pf(1000,&ep);

Observation sensor_values;

NodeHandle *np;
int sum_forward = 0;

bool on = false;
bool bag_read = false;

const double pi = 3.141592;

/*void pfoe_outCallback(const raspimouse_gamepad_teach_and_replay::PFoEOutput::ConstPtr& msg)
{
	string filename = "/home/ubuntu/catkin_ws/src/raspimouse_gamepad_teach_and_replay/src/log_replay.txt";
	ofstream writing_file;
	writing_file.open(filename, ios::app);//out
	writing_file << msg->left_side << "," << msg->left_forward << "," << msg->right_forward << "," << msg->right_side << "," << msg->linear_x << "," << msg->angular_z << "," << msg->eta << "," << msg->lh[1] << endl;
	writing_file.close();
}*/
void buttonCallback(const raspimouse_ros_2::ButtonValues::ConstPtr& msg)
{
	on = msg->mid_toggle;
}

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int lf = (-pi*3.0/180 - msg->angle_min)/msg->angle_increment; 
	int rf = (pi*3.0/180 - msg->angle_min)/msg->angle_increment; 
	int ls = (-pi*45.0/180 - msg->angle_min)/msg->angle_increment; 
	int rs = (pi*45.0/180 - msg->angle_min)/msg->angle_increment; 

	double lfv = isnan(msg->ranges[lf]) ? 500.0 : msg->ranges[lf]*1000;
	double rfv = isnan(msg->ranges[rf]) ? 500.0 : msg->ranges[rf]*1000;
	double rsv = isnan(msg->ranges[rs]) ? 500.0 : msg->ranges[rs]*1000;
	double lsv = isnan(msg->ranges[ls]) ? 500.0 : msg->ranges[ls]*1000;

	sensor_values.setValues(lfv,lsv,rsv,rfv);
}

void on_shutdown(int sig)
{
	ros::ServiceClient motor_off = np->serviceClient<std_srvs::Trigger>("motor_off");
	std_srvs::Trigger t;
	motor_off.call(t);

	shutdown();
}

void readEpisodes(string file)
{
	ep.reset();

	rosbag::Bag bag1(file, rosbag::bagmode::Read);

	vector<std::string> topics;
	topics.push_back("/event");
	
	rosbag::View view(bag1, rosbag::TopicQuery(topics));

	double start = view.getBeginTime().toSec() + 5.0; //discard first 5 sec
	double end = view.getEndTime().toSec() - 5.0; //discard last 5 sec
	for(auto i : view){
	        auto s = i.instantiate<raspimouse_gamepad_teach_and_replay::Event>();

		Observation obs(s->left_forward,s->left_side,s->right_side,s->right_forward);
		Action a = {s->linear_x,s->angular_z};
		Event e(obs,a,0.0);
		e.time = i.getTime();

		if(e.time.toSec() < start)
			continue;

		ep.append(e);

		if(e.time.toSec() > end)
			break;
	}
}

int main(int argc, char **argv)
{
	cout << "1" << endl;
	init(argc,argv,"go_around");
	NodeHandle n;
	np = &n;

	cout << "2" << endl;
	//Subscriber sub = n.subscribe("lightsensors", 1, sensorCallback);
	Subscriber sub = n.subscribe("/scan", 1, sensorCallback);
	Subscriber sub_b = n.subscribe("buttons", 1, buttonCallback);
	//Subscriber sub_p = n.subscribe("pfoe_out",1,pfoe_outCallback);
	Publisher cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	Publisher pfoe_out = n.advertise<raspimouse_gamepad_teach_and_replay::PFoEOutput>("pfoe_out", 100);
	ros::ServiceClient motor_on = n.serviceClient<std_srvs::Trigger>("motor_on");
	ros::ServiceClient tm = n.serviceClient<raspimouse_ros_2::TimedMotion>("timed_motion");
	cout << "3" << endl;

	signal(SIGINT, on_shutdown);
	cout << "4" << endl;

	motor_on.waitForExistence();
	std_srvs::Trigger t;
	motor_on.call(t);
	cout << "5" << endl;

	geometry_msgs::Twist msg;
	cout << "6" << endl;
	//pf.init();
  cout << "7" << endl;
	Rate loop_rate(10);
	Action act = {0.0,0.0};


	while(ok()){
		//cout << "wowow" << endl;
		if(not on){
			cout << "idle" << endl;
			bag_read = false;
			spinOnce();
			loop_rate.sleep();
			continue;
		}else if(not bag_read){
			cout << "not bag_read" << endl;
			string bagfile;
			//bagfile = "/home/ubuntu/.ros/20220911_180007.bag"; //ゼミ後の動画撮りのため、一時的に記述
			bagfile = "/home/ubuntu/catkin_ws/src/raspimouse_gamepad_teach_and_replay/log/v3.0/20220911_184708.bag";
			//n.setParam("/current_bag_file", bagfile);
			//n.getParam("/current_bag_file", bagfile);//20220911_180007.bag //比較的きれいな教示データ
			readEpisodes(bagfile);
			bag_read = true;
			pf.init();
			spinOnce();
			loop_rate.sleep();
			continue;
		}
		raspimouse_gamepad_teach_and_replay::PFoEOutput out;
		
		cout << "1" << endl;

		act = pf.sensorUpdate(&sensor_values, &act, &ep, &out);
		msg.linear.x = act.linear_x;
		out.linear_x = act.linear_x;
		msg.angular.z = act.angular_z;
		out.angular_z = act.angular_z;

		cout << "2" << endl;
		out.left_forward = sensor_values.lf;
		out.left_side = sensor_values.ls;
		out.right_forward = sensor_values.rf;
		out.right_side = sensor_values.rs;
		
		cmdvel.publish(msg);
		pfoe_out.publish(out);
		pf.motionUpdate(&ep);

		spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
