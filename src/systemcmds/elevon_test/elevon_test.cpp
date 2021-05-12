/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Holger Steinhaus <hsteinhaus@gmx.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file elevon_test.c
 *
 * Tool for drive testing
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>
#include <px4_platform_common/posix.h>
#include "drivers/drv_pwm_output.h"
#include <math.h>

extern "C" __EXPORT int elevon_test_main(int argc, char *argv[]);



#define LEFT_ELEVATOR_SERVO		4
#define RIGHT_ELEVATOR_SERVO		5
#define SLOW_TEST_TIME			5.0f
#define TOTAL_TEST_TIME			10.0f

#define SHAKING_START_DELAY		300000
#define SHAKING_END_DELAY		100000
#define SHAKING_NUMBER_OF_STEPS		25
#define ELEVON_START_POSITION		1500

static bool _elevonTestInProgFlag = false;

//returns true if interrupted
static bool abortOnUserInterruption(pollfd fds){

	int ret;
	char c;

	ret = poll(&fds, 1, 0);

	if (ret > 0) {

		ret = read(0, &c, 1);

		if (c == 0x03 || c == 0x63 || c == 'q') {

			return true;
		}
	}
	return false;
}

static uint32_t elevon_test(float slowTestTime, float totalTestTime, int fd){
	_elevonTestInProgFlag = true;
	int ret;
	int ret2;

	/* Open console directly to grab CTRL-C signal */
	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	PX4_INFO("test procedure started");
	PX4_INFO("Press CTRL-C or 'c' to abort.");

	struct pwm_output_values pwm_values {};

	uint32_t servo_count;

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);



	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return 1;
	}
	pwm_values.channel_count = servo_count;

	ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_MIN_PWM");
		return 1;
	}

	uint16_t minLeft = pwm_values.values[LEFT_ELEVATOR_SERVO];
	uint16_t minRight = pwm_values.values[RIGHT_ELEVATOR_SERVO];

	ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_MAX_PWM");
		return 1;
	}

	uint16_t maxLeft = pwm_values.values[LEFT_ELEVATOR_SERVO];
	uint16_t maxRight = pwm_values.values[RIGHT_ELEVATOR_SERVO];

	if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
		PX4_ERR("PWM_SERVO_SET_MODE -> PWM_SERVO_ENTER_TEST_MODE");
		return 1;
	}

	PX4_INFO("minLeft: %d", minLeft);
	PX4_INFO("minRight: %d", minRight);
	PX4_INFO("maxLeft: %d", maxLeft);
	PX4_INFO("maxRight: %d", maxRight);

	px4_usleep(1000);

	float LeftDifference = maxLeft - minLeft;
	float RightDifference = maxRight - minRight;

	uint32_t LeftStartPosition = ELEVON_START_POSITION;
	uint32_t RightStartPosition = ELEVON_START_POSITION;

	float LeftStartRelativePosition = ((float)LeftStartPosition - (float)minLeft)/LeftDifference;
	float RightStartRelativePosition = ((float)RightStartPosition - (float)minRight)/RightDifference;;

	float LeftElevatorPosition = LeftDifference * LeftStartRelativePosition + minLeft;
	float RightElevatorPosition = RightDifference * RightStartRelativePosition + minRight;

	hrt_abstime start = hrt_absolute_time();
	float elapsedTime;

	float outWeightLeft = 0.0;
	float outWeightRight = 0.0;

	//do the slow elevon test
	while(_elevonTestInProgFlag){

		//up_pwm_servo_set(motorNr,elevatorPosition);
		ret = px4_ioctl(fd, PWM_SERVO_SET(LEFT_ELEVATOR_SERVO), (long)LeftElevatorPosition);
		ret2 = px4_ioctl(fd, PWM_SERVO_SET(RIGHT_ELEVATOR_SERVO), (long)RightElevatorPosition);

		elapsedTime = (float)hrt_elapsed_time(&start) * 1e-6f;

		outWeightLeft = 0.5f * (LeftStartRelativePosition * 2.0f + sinf(M_TWOPI_F * elapsedTime / slowTestTime));
		outWeightRight = 0.5f * (RightStartRelativePosition * 2.0f + sinf(M_TWOPI_F * elapsedTime / slowTestTime));

		LeftElevatorPosition = (LeftDifference * outWeightLeft) + minLeft;
		RightElevatorPosition = (RightDifference * outWeightRight) + minRight;

		px4_usleep(1000);


		if (ret != OK) {
			PX4_ERR("PWM_SERVO_SET(%d), value: (%d)", LEFT_ELEVATOR_SERVO, (int)LeftElevatorPosition);
			break;
		}

		if (ret2 != OK) {
			PX4_ERR("PWM_SERVO_SET(%d), value: (%d)", RIGHT_ELEVATOR_SERVO,(int)RightElevatorPosition);
			break;
		}

		if((elapsedTime / SLOW_TEST_TIME) > 1){
			break;
		}

		if(abortOnUserInterruption(fds))
		{
			_elevonTestInProgFlag = false;
			break;
		}
	}

	//do the shaking elevon test
	uint32_t differenceBetweenDelays = SHAKING_START_DELAY - SHAKING_END_DELAY;
	uint32_t delayDecreasePerStep = differenceBetweenDelays/SHAKING_NUMBER_OF_STEPS;
	uint32_t currentDelay = SHAKING_START_DELAY;

	if(_elevonTestInProgFlag){
		for(int i = 0; i < SHAKING_NUMBER_OF_STEPS;i++){
			if(i%2){
				ret = px4_ioctl(fd, PWM_SERVO_SET(LEFT_ELEVATOR_SERVO), minLeft);
				ret2 = px4_ioctl(fd, PWM_SERVO_SET(RIGHT_ELEVATOR_SERVO), minRight);
			}
			else{
				ret = px4_ioctl(fd, PWM_SERVO_SET(LEFT_ELEVATOR_SERVO), maxLeft);
				ret2 = px4_ioctl(fd, PWM_SERVO_SET(RIGHT_ELEVATOR_SERVO), maxRight);
			}

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_SET(%d)", LEFT_ELEVATOR_SERVO);
				break;
			}

			if (ret2 != OK) {
				PX4_ERR("PWM_SERVO_SET(%d)", RIGHT_ELEVATOR_SERVO);
				break;
			}

			px4_usleep(currentDelay);
			currentDelay -= delayDecreasePerStep;


			elapsedTime = (float)hrt_elapsed_time(&start) * 1e-6f;
			if(elapsedTime > totalTestTime){
				break;
			}

			if(abortOnUserInterruption(fds)){
				_elevonTestInProgFlag = false;
				break;
			}

		}
	}



	if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
		PX4_ERR("PWM_SERVO_SET_MODE -> PWM_SERVO_EXIT_TEST_MODE");
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_CLEAR_ARM_OK, 0) != PX4_OK) {
		PX4_ERR("PWM_SERVO_CLEAR_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_DISARM, 0) != PX4_OK) {
		PX4_ERR("PWM_SERVO_DISARM");
		return 1;
	}

	PX4_INFO("end of test");
	return 0;
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
	R"DESCR_STR(
	Utility to test elevons.

	WARNING: remove all props before using this command.
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("elevont_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the elevon test");


}

int elevon_test_main(int argc, char *argv[])
{


	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);



	int myoptind = 1;



	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != PX4_OK) {
		PX4_ERR("PWM_SERVO_SET_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != PX4_OK) {
		PX4_ERR("PWM_SERVO_ARM");
		return 1;
	}

	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_FAILSAFE, 0) != PX4_OK) {
		PX4_ERR("PWM_SERVO_SET_FORCE_FAILSAFE");
		return 1;
	}


	const char *command = argv[myoptind];

	if (!strcmp(command, "start")){

		if(_elevonTestInProgFlag){
			//recursion protection
			PX4_INFO("elevon test already in progress");
		}

		if(elevon_test(SLOW_TEST_TIME, TOTAL_TEST_TIME, fd)){
			_elevonTestInProgFlag = false;
			return 1;
		}
		_elevonTestInProgFlag = false;
	}
	else if(!strcmp(command, "stop")){
		if(_elevonTestInProgFlag){
			PX4_INFO("elevon test aborted");
		}
		else{
			PX4_INFO("elevon test not in progress");
		}
		_elevonTestInProgFlag = false;
	}
	else{
		usage(nullptr);
		return 0;
	}


	close(fd);


	return 0;
}
