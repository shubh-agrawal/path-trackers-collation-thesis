#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>
#include "prius_msgs/Control.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <chrono>
//#include "matplotlibcpp.h"

using namespace std;

float wheelbase = 1.983; // wheel base for the vehicle
float ep = 0;
float cp = 0;

//state parameters of car
float x_bot ;
float y_bot ;
float yaw_bot ;
float vel_bot  ;
float prev_acc = 0;
float prev_steer = 0;
float cte = 0;
float epsi = 0;
float initial = 1;
float max_index = 5;
float sampling_length = 30;
//printf("start");

geometry_msgs::Twist cmd;
prius_msgs::Control prius_vel;

typedef struct Point 
{
	float x;
	float y;
} Point ;

//Plot CTE, Delta and Speed
//Requires matplotlib
bool plotting = false;
int max_iters = 100;
//namespace plt = matplotlibcpp;

std::vector<double> ptsx = {0, 1, 2, 3, 4, 5};
std::vector<double> ptsy = {0, 1, 2, 3, 4, 5};

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x*pi()/180; }
double rad2deg(double x) { return x*180/pi(); }



double polyeval (Eigen::VectorXd coeffs, double x) 
{
	// Evaluate a polynomial.
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++)
	{
		result += coeffs[i]*pow(x, i);
	}

}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
	// Fit a polynomial.
	// Adapted from
	// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 	// printf("entered \n");
 	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
	A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
	for (int i = 0; i < order; i++) {
	  A(j, i + 1) = A(j, i) * xvals(j);
	}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
	// printf("exit \n");
}

float dist(geometry_msgs::PoseStamped a,float x,float y)
{	
	// Calculates distance between two points.
	// :param a [float]
	// :param x [float]
	// :param y [float]
	//calculate distance
	return sqrt(pow((a.pose.position.x - x),2) + pow((a.pose.position.y - y),2));
}

void callback_path(const nav_msgs::Path::ConstPtr& data)
{	
	// calculates target path point and the steering angle
	// :param data [Path]
	// :param ep [float]
	// calculate minimum distance
	// printf("entered path_callback \n"); 

	max_index=(data->poses).size();
	ptsx.clear();
	ptsy.clear();
	// std::vector<double> ptsx = {};
	// std::vector<double> ptsy = {};
	
	// for(unsigned int i=0; i < max_index; i++)
	// {
	// 	//shift car reference angle to 90 degrees
	// 	ptsx.push_back(0);
	// 	ptsy.push_back(0);	
 //    }
	
	std::vector<double> distances;
	for(int i=0; i<max_index; i++)
	{
		distances.push_back(dist((data->poses[i]),x_bot,y_bot));
	}

	int nearest_index = min_element(distances.begin(),distances.end()) - distances.begin(); 
	for(int i=0; i<sampling_length; i++)
	{
		ptsx.push_back(data->poses[nearest_index + i].pose.position.x);
		ptsy.push_back(data->poses[nearest_index + i].pose.position.y);
	}

	// vector <float> distances;
	// for(int i=0;i<max_index;i++)
	// {
	// 	//distances.push_back( dist((data->poses[i]),x_bot,y_bot));
	// 	ptsx.push_back(data->poses[i].pose.position.x);
	// 	// printf("path x: %f \n", ptsx[i]);
	// 	// printf("path x: %f \n", data->poses[i].pose.position.x);
	// 	ptsy.push_back(data->poses[i].pose.position.y);
		// printf("path y: %f \n", ptsy[i]);
		// printf("path y: %f \n", data->poses[i].pose.position.y);

	// }
	// printf("size of path: %d \n",ptsx.size() );

	//ep = *min_element(distances.begin(), distances.end()); //min element value
	//cp = min_element(distances.begin(),distances.end()) - distances.begin(); //index of min element

	//cout<<"old index:"<< cp<<endl;
	// calculate index of target point on path
	

	// float L=0;
	// float Lf = k * max_vel + d_lookahead;
	// float dx,dy;
	
	// while (Lf > L && (cp + 1) < max_index)
	// {
	// 	dx = data->poses[cp + 1].pose.position.x - data->poses[cp].pose.position.x;
	// 	dy = data->poses[cp + 1].pose.position.y - data->poses[cp].pose.position.y;
	// 	L += sqrt(dx*dx + dy*dy);
	// 	cp ++;
	// }

	// cout << max_index << endl;
	// cout <<"new index is:"<< cp<< endl;

	// Point goal_point;
	// goal_point.x = data->poses[cp].pose.position.x;
	// goal_point.y = data->poses[cp].pose.position.y;

	// cout<<"current goal is:"<< goal_point.x<<" "<<goal_point.y<<endl;

	// Point error;
	// error.x = goal_point.x - x_bot;
	// error.y = goal_point.y - y_bot;
}


void callback_feedback(const nav_msgs::Odometry::ConstPtr& data)
{
	// Assigns the position of the robot to global variables from odometry.
	// :param x_bot [float]
	// :param y_bot [float]
	// :param yaw_bot [float]
	// :param vel_bot [float]
	// printf("entered odom callback \n");
	x_bot = data->pose.pose.position.x;
	y_bot = data->pose.pose.position.y;
	
	// quarternion to euler conversion
	float siny = 2.0 * (data->pose.pose.orientation.w *
				   data->pose.pose.orientation.z +
				   data->pose.pose.orientation.x *
				   data->pose.pose.orientation.y);
	float cosy = 1.0 - 2.0 * (data->pose.pose.orientation.y *
						 data->pose.pose.orientation.y +
						 data->pose.pose.orientation.z *
						 data->pose.pose.orientation.z);

	yaw_bot = atan2(siny, cosy) ;//yaw in radians

	vel_bot = (data->twist.twist.linear.x * cos(yaw_bot) +
                data->twist.twist.linear.y * sin(yaw_bot));

	// cout<<"x of car:"<<x_bot<<endl;
	// cout<<"y of car:"<< y_bot<<endl;
	cout<<"angle of car:"<< yaw_bot << endl;
	// cout<<"c"<<endl;
}

/*

def prius_pub(data):
	'''
	publishes the velocity and steering angle
	published on topic : ackermann_cmd_topic
	'''
	global prius_vel
	prius_vel = Control()

	if(data.linear.x > 0):
		prius_vel.throttle = data.linear.x / 100
		prius_vel.brake = 0
		print ("acc")
		print (prius_vel.throttle)

	if(data.linear.x < 0):
		prius_vel.brake = -data.linear.x / 100
		prius_vel.throttle = 0
		print ("brake")
		print (prius_vel.brake)

	prius_vel.steer = data.angular.z / 30

	pub.publish(prius_vel)

*/

int main(int argc, char** argv)
{
	// printf("s \n");
	ros::init(argc, argv, "main0");
	ros::NodeHandle nh;
	ros::Publisher prius_pub = nh.advertise<prius_msgs::Control>("/prius", 10);
	ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("cmd_delta", 10);
  	ros::Subscriber odom_sub = nh.subscribe("base_pose_ground_truth",10, callback_feedback);
  	ros::Subscriber path_sub = nh.subscribe("astroid_path",10, callback_path);
  	
  	ros::Rate r(100);
  	// printf("start \n");
	//MPC is initialised here
	MPC mpc; 

	//For plotting
	int iters = 0;

	std::vector<double> x_vals = {};
	std::vector<double> y_vals = {};
	std::vector<double> psi_vals = {};
	std::vector<double> v_vals = {};
	std::vector<double> cte_vals = {};
	std::vector<double> epsi_vals = {};
	std::vector<double> delta_vals = {};
	std::vector<double> a_vals = {};


	while(ros::ok())
    {
		//get the values from simulator
		//vector<double> ptsx = 
		//vector<double> ptsy = 
		// printf("entered ros loop \n");
		double px = x_bot;
		double py = y_bot;
		double psi = yaw_bot;
		double v = vel_bot;
		double steer_value = prev_steer;
		double throttle_value = prev_acc;
		// printf("x: %f \n", px);
		
		//Adjust plain path to car coordinates
		//Set x, y and psi to zero
		for(unsigned int i=0; i < ptsx.size(); i++)
		{
			//shift car reference angle to 90 degrees
			double shift_x = ptsx.at(i) -px;
			// printf("path x: %f \n", ptsx[i]);
			// printf("shift_x: %f \n", shift_x);
			double shift_y = ptsy.at(i) -py;
			// printf("path y: %f \n", ptsy[i]);
			// printf("shift_y: %f \n", shift_y);

			ptsx.at(i) = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
			ptsy.at(i) = (shift_x * sin(0-psi) + shift_y*cos(0-psi));	
	    }
	    // printf("2 \n");
	    // printf("value: %f \n", ptsx[0]);
	    float path_length = ptsx.size();
		double* ptrx = &ptsx[0];
		Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, path_length);

		double* ptry = &ptsy[0];
		Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, path_length);
		//Fit the Polynomial
		// cout << "value" << ptsx_transform[0] << "\n";
		// printf("3 \n");
		// ptsx_transform << 0, 0, 0, 0, 0;
		// ptsy_transform << 0, 0, 0, 0, 0;
		auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
		for(int i = 0; i < coeffs.size(); i++)
		{
			printf("coefficient of x^ %d : %f", i, coeffs[i] );
		}
		// printf("4 \n");
		//Get cte and epsi
		double cte = polyeval(coeffs, 0);
		printf("cte: %f \n", cte );
		double epsi = atan(coeffs[1]);
		printf("epsi: %f \n", epsi );

		// cte = cp; 
		// epsi = 0; 

		//Plotting
		if(plotting)
		{
		//If set, save data for plotting
			iters++;
			x_vals.push_back(px);
			y_vals.push_back(py);
			psi_vals.push_back(psi);
			v_vals.push_back(v);
			cte_vals.push_back(cte);
			epsi_vals.push_back(epsi);	  
			delta_vals.push_back(steer_value);
			a_vals.push_back(throttle_value);
		}

		//predict next state to avoid Latency problems
		//Latency of .1 seconds
		double dt = 0.1;
		const double Lf = 1.983;
		double x1=0, y1=0,  psi1=0, v1=v, cte1=cte, epsi1=epsi;	  
		x1 += v * cos(0) * dt;
		y1 += v * sin(0) * dt;
		//steer_value is negative
		psi1 +=  v/Lf * steer_value * dt;
		v1 += throttle_value * dt;	    
		cte1 +=   v * sin(epsi1) * dt;
		//steer_value is negative
		epsi1 = -epsi1 + v * steer_value / Lf * dt;	  
		Eigen::VectorXd state(6);	  
		state << x1,y1,psi1,v1,cte1,epsi1;
		// printf("happy \n");
		auto vars = mpc.Solve(state, coeffs);
		// cout << "throttle" << vars[1] << endl;
		// cout << "steering" << vars[0] << endl;
		// printf("super happy \n");
		// std::vector<double> vars = {0,0};
		//print polynomial back to simulator
		vector<double> way_x_vals;
		vector<double> way_y_vals;  
		//x value distance
		const double poly_inc = Lf;
		//number of points to print
		const int num_points = 25;

		for(unsigned int i=1; i<num_points; i++)
		{
			way_x_vals.push_back(poly_inc*i);
			way_y_vals.push_back(polyeval(coeffs, poly_inc*i));
	    }

	    vector<double> mpc_x_vals;
		vector<double> mpc_y_vals;
		
		for(unsigned int i=2; i<vars.size(); i++)
		{
			if(i%2==0)
			{
			  mpc_x_vals.push_back(vars[i]);
			} 
			else 
			{
			  mpc_y_vals.push_back(vars[i]);
			}
		}

	      	  
		// Calculate steering angle and throttle using MPC.

		prius_vel.steer = vars[0]/(deg2rad(30)*Lf);
		
		if(vars[1] > 0)
		{
			prius_vel.throttle = vars[1];
			prius_vel.brake = 0;
			//print ("acc");
			//print (prius_vel.throttle);
		}
		
		if(vars[1] < 0)
		{
			prius_vel.brake = -vars[1];
			prius_vel.throttle = 0;
			//print ("brake");
			//print (prius_vel.brake);	
		}
		cout << "throttle:" << prius_vel.throttle << endl;
		cout << "steering:" << prius_vel.steer << endl;
		prev_steer = prius_vel.steer;
		prev_acc = vars[1];

		prius_pub.publish(prius_vel);

		/*
		//Display the MPC predicted trajectory           
		msgJson["mpc_x"] = mpc_x_vals;
		msgJson["mpc_y"] = mpc_y_vals;

		//Display the waypoints/reference line
		msgJson["next_x"] = way_x_vals;
		msgJson["next_y"] = way_y_vals;
		*/

		
		// Latency
		// The purpose is to mimic real driving conditions where
		// the car does actuate the commands instantly.
		//
		// Feel free to play around with this value but should be to drive
		// around the track with 100ms latency.
		//        
		//

		//this_thread::sleep_for(chrono::milliseconds(100));
		

		// if(iters > max_iters)
		// {
		// 	//Plot Graph for analysis of the first 100 iterations
		// 	plt::subplot(3, 1, 1);
		// 	plt::title("CTE");
		// 	plt::plot(cte_vals);
		// 	plt::subplot(3, 1, 2);
		// 	plt::title("Delta (Radians)");
		// 	plt::plot(delta_vals);
		// 	plt::subplot(3, 1, 3);
		// 	plt::title("Velocity");
		// 	plt::plot(v_vals);	    
		// 	plt::show();
		// 	iters = 0;
		// 	exit(1);
		// }

        // pub1.publish();
		r.sleep();
        ros::spinOnce();

    }

	return 0;   

}
