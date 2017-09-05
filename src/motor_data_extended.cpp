/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file masterBoard.cpp
 * @author Cyril Jourdan
 * @date Mar 10, 2017
 * @version 2.0.0
 * @brief Implementation file for the serial communication between ROS and the Master Board
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 10, 2017
 */

/*! Includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <osa_msgs/MotorDataExtendedMultiArray.h>
#include <stdio.h>

/*! Defines */
#define LOOP_RATE	15 //HEART_BEAT

/*! Variables */
//ROS publisher
ros::Publisher pub_motorDataExtended;
osa_msgs::MotorDataMultiArray motorData_ma, prev1MotorData_ma, prev2MotorData_ma;
osa_msgs::MotorDataExtendedMultiArray prev1MotorDataExt_ma, prev2MotorDataExt_ma; //motorDataExt_ma

ros::Time currTime, prev1Time, prev2Time; //for 1st and 2nd derivative
ros::Duration diffTime;

bool motorData_ma_arrived = false;

int cycleIdx = 0;

/*! functions */
void initMotorDataExtended()
{
	//create the data multi array
	prev1MotorDataExt_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	prev1MotorDataExt_ma.layout.dim[0].size = motorData_ma.layout.dim[0].size;
	prev1MotorDataExt_ma.layout.dim[0].stride = motorData_ma.layout.dim[0].stride;
	prev1MotorDataExt_ma.layout.dim[0].label = "slavesExt";
	prev1MotorDataExt_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	prev1MotorDataExt_ma.layout.dim[1].size = motorData_ma.layout.dim[1].size;
	prev1MotorDataExt_ma.layout.dim[1].stride = motorData_ma.layout.dim[1].stride;
	prev1MotorDataExt_ma.layout.dim[1].label = "motorsExt";
	prev1MotorDataExt_ma.layout.data_offset = 0;
	prev1MotorDataExt_ma.motorDataExtended.clear();
	prev1MotorDataExt_ma.motorDataExtended.resize(motorData_ma.layout.dim[0].stride);

	prev2MotorDataExt_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	prev2MotorDataExt_ma.layout.dim[0].size = motorData_ma.layout.dim[0].size;
	prev2MotorDataExt_ma.layout.dim[0].stride = motorData_ma.layout.dim[0].stride;
	prev2MotorDataExt_ma.layout.dim[0].label = "slavesExt";
	prev2MotorDataExt_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	prev2MotorDataExt_ma.layout.dim[1].size = motorData_ma.layout.dim[1].size;
	prev2MotorDataExt_ma.layout.dim[1].stride = motorData_ma.layout.dim[1].stride;
	prev2MotorDataExt_ma.layout.dim[1].label = "motorsExt";
	prev2MotorDataExt_ma.layout.data_offset = 0;
	prev2MotorDataExt_ma.motorDataExtended.clear();
	prev2MotorDataExt_ma.motorDataExtended.resize(motorData_ma.layout.dim[0].stride);
}

/*! Callback functions */
void motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	currTime = ros::Time::now();

	motorData_ma = *data;
	motorData_ma_arrived = true;

	cycleIdx++;
}

/*! \fn int main(int argc, char** argv)
 *  \brief main function.
 *  \param argc
 *  \param argv
 *  \return int
 */
int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "osa_motorDataExtended_node");
	ros::NodeHandle nh;
	ros::Rate r(LOOP_RATE);


	//Subscribers
	ros::Subscriber sub_motorDataArray = nh.subscribe ("/motor_data_array", 10, motorDataArray_cb);

	//Publishers
	pub_motorDataExtended = nh.advertise<osa_msgs::MotorDataExtendedMultiArray>("/motor_data_extended", 100, true);


	prev1Time = ros::Time::now();

	//set diffTime as the inverse of the cycle period as a first estimation
/*
	double period_d = LOOP_RATE;
	period_d = 1/period_d;
	//ROS_INFO("diffTime = %f", period_d);
	period_d *= 1000000;
	int period_i = (int)period_d;

	diffTime = ros::Duration(0, period_i);
	ROS_INFO("diffTime = %d", period_i);
*/

	//skip the first and second cycles to get 3 points to calculate the derivative.
	while(ros::ok())
	{
		ros::spinOnce();

		//initialization, wait for 2 cycles, to measure the 1st and 2nd derivatives.
		if(cycleIdx >= 5)
		{
			//calculate first and second derivatives.

			//publish extended motor data, with 2 cycles delay because they are needed to calculate the 2nd derivative.
			pub_motorDataExtended.publish(prev2MotorDataExt_ma);
		}
		else if(cycleIdx == 4)
		{
			//calculate first derivatives.
		}
		else if(cycleIdx == 3)
		{
			//calculate first derivatives
			for(int i=0; i<motorData_ma.layout.dim[0].size; i++)
			{
				int32_t d = motorData_ma.motorData[i].position - prev2MotorData_ma.motorData[i].position;
				diffTime = currTime - prev2Time;

				//TODO calculate velocity in rpm

				prev1MotorDataExt_ma.motorDataExtended[i].velocity = d/diffTime.nsec;
			}
		}
		else if(cycleIdx == 2) //(cycleIdx < 4) && (cycleIdx >= 3)
		{
			//do nothing
		}
		else if(cycleIdx == 1)
		{
			initMotorDataExtended();
		}

		motorData_ma_arrived = false;

		//save previous data.
		prev2MotorData_ma = prev1MotorData_ma;
		prev1MotorData_ma = motorData_ma;

		prev2MotorDataExt_ma = prev1MotorDataExt_ma;
		//prev1MotorDataExt_ma = motorDataExt_ma;

		//save previous times, shifted by one cycle.
		prev2Time = prev1Time;
		prev1Time = currTime;

		r.sleep();
	}

	return 0;
}
