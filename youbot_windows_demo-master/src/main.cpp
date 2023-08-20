// Copyright (c) 2014 Locomotec
//

#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"


#include <SFML/Window/Joystick.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Time.hpp>

#define	_USE_MATH_DEFINES
#include <math.h>
#include <float.h>


#define JOYSTICK_ID 1	//joystick id
#define X0 0			//in mm
#define Y0 200			//in mm
#define Z0 375			//in mm
#define A0 0			//in deg
#define B0 0			//in deg
#define GR0 0			//in 0/1 (open/close)

#define L_SPEED 150			//in mm/s
#define R_SPEED	60			//in deg/s
#define GR_SPEED	0.01	//in m/s
#define DEAD_ZONE	5		//in %

#define D1	33
#define L1	147
#define L2	155
#define L3	135
#define L4	217.5

using namespace youbot;
//using namespace std;

void setAngles(YouBotManipulator* manipulator, double a1, double a2, double a3, double a4, double a5) {
	JointAngleSetpoint desiredJointAngle;

	desiredJointAngle.angle = M_PI / 180.0 * (169.0 - a1) * radian;
	manipulator->getArmJoint(1).setData(desiredJointAngle);

	desiredJointAngle.angle = M_PI / 180.0 * (65.0 + a2) * radian;
	manipulator->getArmJoint(2).setData(desiredJointAngle);

	desiredJointAngle.angle = -M_PI / 180.0 * (146.0 - a3) * radian;
	manipulator->getArmJoint(3).setData(desiredJointAngle);

	desiredJointAngle.angle = M_PI / 180.0 * (102.5 - a4) * radian;
	manipulator->getArmJoint(4).setData(desiredJointAngle);

	desiredJointAngle.angle = M_PI / 180.0 * (167.5 + a5) * radian;
	manipulator->getArmJoint(5).setData(desiredJointAngle);

	SLEEP_MILLISEC(10);
}

bool inverseKinematics(YouBotManipulator* manipulator, double x, double y, double z, double phi, double psi) {

	double L = sqrt(y * y + x * x) - D1;
	double aa = z - L4 * sin(phi / 180.0 * M_PI) - L1;
	double bb = L - L4 * cos(phi / 180.0 * M_PI);
	double cc = sqrt(aa * aa + bb * bb);

	double a1 = atan2(-x, y) / M_PI * 180.0;
	double a2 = 90.0 - atan2(aa, bb) / M_PI * 180.0 - acos((cc * cc + L2 * L2 - L3 * L3) / (2 * cc * L2)) / M_PI * 180.0;
	double a3 = 180.0 - acos((L2 * L2 + L3 * L3 - cc * cc) / (2 * L2 * L3)) / M_PI * 180.0;
	double a4 = a2 + a3 + phi - 90.0;
	double a5 = psi;

	if (cc > L2 + L3 ||
		cc < L2 - L3 ||

		abs(a1) > 164 ||
		a2 > 85 || a2 < -60 ||
		a3 > 141 || a3 < -155 ||
		abs(a4) > 97.5 ||
		abs(a5) > 155)
	{
		//stop if out of reach
		return false;
	}
	else {
		//move joints if able to reach target
		setAngles(manipulator, a1, a2, a3, a4, a5);
	}
	return true;
}

int sign(double x) {
	if (x > 0.0) return 1;
	if (x < 0.0) return -1;
	return 0;
}

int main() {

	bool youBotHasArm = false;
	YouBotManipulator* myYouBotManipulator = 0;
	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();
		myYouBotManipulator->calibrateGripper();

		youBotHasArm = true;
	}
	catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}

	double x = X0, y = Y0, z = Z0, a = A0, b = B0, gr = GR0;
	double sp = 20.0;
	sf::Clock clock;

	long long ilyaDickSpace = 1ll << 62;
	short SergeyMoskalevDickSpace = 1 - (1 << 15);
	sf::Clock globalTimer;
	globalTimer.restart();
	double gripperLastCall = -1;

	bool startTask = false;
	bool taskPause = false;
	double taskTimer = -1;
	double taskPauseTimer = -1;
	double taskPauseTime = -1;
	double lastTaskTime = -1;
	double AttemptTime = -1;
	double StartTime = -1;
	double StopTime = -1;


	if (youBotHasArm) {
		//home position
		x = X0, y = Y0, z = Z0, a = A0, b = B0, gr = GR0;
		std::cout << "x:" << x << " y:" << y << " z:" << z << " a:" << a << " b:" << b;

		setAngles(myYouBotManipulator, 0, -63, 144, -40, 0);
		myYouBotManipulator->getArmGripper().open();

		SLEEP_MILLISEC(2000);
	}

	bool gripperFlag = false;

	while (!sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)){
		sf::Joystick::update();

		try {
			if (youBotHasArm) {
				JointAngleSetpoint desiredJointAngle;
				GripperBarSpacingSetPoint desiredBarSpacing;

				bool hasJoystick = false;
				
				double jx = 0, jy = 0, jz = 0, jr = 0, ju = 0, jv = 0, jgr = 0;
					
				for (int joystick_id = 1; joystick_id < 8; joystick_id++) {
					if (hasJoystick |= sf::Joystick::isConnected(joystick_id)) {
						hasJoystick = true;

						double jxi = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::X);
						double jyi = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::Y);
						double jzi = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::Z);
						double jri = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::R);
						double jui = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::U);
						double jvi = sf::Joystick::getAxisPosition(joystick_id, sf::Joystick::V);


						if (abs(jxi) > DEAD_ZONE) jx += jxi;
						if (abs(jyi) > DEAD_ZONE) jy += jyi;
						if (abs(jzi) > DEAD_ZONE) jz += jzi;
						if (abs(jri) > DEAD_ZONE) jr += jri;
						if (abs(jui) > DEAD_ZONE) ju += jui;
						if (abs(jvi) > DEAD_ZONE) jv += jvi;

						if (sf::Joystick::isButtonPressed(joystick_id, 5)) {
							gr = 0;
							gripperFlag = true;
						}
						if (sf::Joystick::isButtonPressed(joystick_id, 4)) {
							gr = 1;
							gripperFlag = true;
						}

						//Start task
						if (sf::Joystick::isButtonPressed(joystick_id, 1) || sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
							startTask = true;
							gripperFlag = true, gr = 1;
							taskTimer = globalTimer.getElapsedTime().asSeconds();
							taskPauseTime = 0;
							StartTime = globalTimer.getElapsedTime().asSeconds();
						}
						//Stop task
						if (sf::Joystick::isButtonPressed(joystick_id, 2) || sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
							startTask = false;
							lastTaskTime = globalTimer.getElapsedTime().asSeconds() - taskTimer;
							StopTime = globalTimer.getElapsedTime().asSeconds();
						}
						//Pause task
						if (sf::Joystick::isButtonPressed(joystick_id, 0) || sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
							taskPause = true;
							taskPauseTimer = globalTimer.getElapsedTime().asSeconds();
						}
						//Resume task
						if (taskPause && (sf::Joystick::isButtonPressed(joystick_id, 3) || sf::Keyboard::isKeyPressed(sf::Keyboard::R))) {
							taskPauseTime = globalTimer.getElapsedTime().asSeconds() - taskPauseTimer;
							taskTimer += taskPauseTime;
							taskPause = false;
						}
					}
				}

				if(hasJoystick){
					std::cout << "Joystick connected:    ";

					double xPrev = x;
					double yPrev = y;
					double zPrev = z;
					double aPrev = a;
					double bPrev = b;
					
					if (startTask) {
						
						double time = globalTimer.getElapsedTime().asSeconds() - taskTimer;
						std::cout << "Task in progress. Time: " << time << "  ";

						//cube coordinates
						double x1 = 74.0, y1 = 185.0;
						double b1 = 90.0 + atan2(x1, y1) / M_PI * 180.0;
						double x2 = 0.0, y2 = 185.0;
						double b2 = 90.0 + atan2(x2, y2) / M_PI * 180.0;
						double x3 = -74.0, y3 = 185.0;
						double b3 = 90.0 + atan2(x3, y3) / M_PI * 180.0;
						double x4 = 37.0, y4 = 268.0;
						double b4 = 90.0 + atan2(x4, y4) / M_PI * 180.0;
						double x5 = -37.0, y5 = 268.0;
						double b5 = 90.0 + atan2(x5, y5) / M_PI * 180.0;
						//common cube height
						double cubeZ1 = 70.0;
						double cubeZ2 = 30.0;
						//slide coordinates
						double slideX = 195.0, slideY = 195.0;
						double slideZ1 = 210.0;
						double slideZ2 = 190.0;
						//Period
						double period = 12.0;

						if (!taskPause) {
						gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;

						
							if (time > 1.0) gripperFlag = false, x = x1, y = y1, z = cubeZ1, a = -90, b = b1;
							if (time > 2.5) gripperFlag = false, x = x1, y = y1, z = cubeZ2, a = -90, b = b1;
							if (time > 3.0) gripperFlag = true, gr = 0;
							if (time > 5.5) gripperFlag = false, x = x1, y = y1, z = 100, a = -80, b = b1;
							if (time > 6.5) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = -90;
							if (time > 8.0) gripperFlag = false, x = slideX, y = slideY, z = slideZ1, a = -30, b = -90;
							if (time > 10.0) gripperFlag = true, gr = 1;
							if (time > 10.5) gripperFlag = false, x = slideX, y = slideY, z = slideZ2, a = -30, b = -90;
							if (time > 12.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;

							if (time > period + 1.0) gripperFlag = false, x = x2, y = y2, z = cubeZ1, a = -90, b = b2;
							if (time > period + 2.5) gripperFlag = false, x = x2, y = y2, z = cubeZ2, a = -90, b = b2;
							if (time > period + 3.0) gripperFlag = true, gr = 0;
							if (time > period + 5.5) gripperFlag = false, x = x2, y = y2, z = 100, a = -80, b = b2;
							if (time > period + 6.5) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = -90;
							if (time > period + 8.0)  gripperFlag = false, x = slideX, y = slideY, z = slideZ1, a = -30, b = -90;
							if (time > period + 10.0) gripperFlag = true, gr = 1;
							if (time > period + 10.5) gripperFlag = false, x = slideX, y = slideY, z = slideZ2, a = -30, b = -90;
							if (time > period + 12.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;

							if (time > 2 * period + 1.0) gripperFlag = false, x = x3, y = y3, z = cubeZ1, a = -90, b = b3;
							if (time > 2 * period + 2.5) gripperFlag = false, x = x3, y = y3, z = cubeZ2, a = -90, b = b3;
							if (time > 2 * period + 3.0) gripperFlag = true, gr = 0;
							if (time > 2 * period + 5.5) gripperFlag = false, x = x3, y = y3, z = 100, a = -80, b = b3;
							if (time > 2 * period + 6.5) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = -90;
							if (time > 2 * period + 8.0)  gripperFlag = false, x = slideX, y = slideY, z = slideZ1, a = -30, b = -90;
							if (time > 2 * period + 10.0) gripperFlag = true, gr = 1;
							if (time > 2 * period + 10.5) gripperFlag = false, x = slideX, y = slideY, z = slideZ2, a = -30, b = -90;
							if (time > 2 * period + 12.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;

							if (time > 3 * period + 1.0) gripperFlag = false, x = x4, y = y4, z = cubeZ1, a = -85, b = b4;
							if (time > 3 * period + 3.5) gripperFlag = false, x = x4, y = y4, z = cubeZ2, a = -90, b = b4;
							if (time > 3 * period + 4.0) gripperFlag = true, gr = 0;
							if (time > 3 * period + 5.5) gripperFlag = false, x = x4, y = y4, z = 100, a = -80, b = b4;
							if (time > 3 * period + 7.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = -90;
							if (time > 3 * period + 8.0)  gripperFlag = false, x = slideX, y = slideY, z = slideZ1, a = -30, b = -90;
							if (time > 3 * period + 10.0) gripperFlag = true, gr = 1;
							if (time > 3 * period + 10.5) gripperFlag = false, x = slideX, y = slideY, z = slideZ2, a = -30, b = -90;
							if (time > 3 * period + 12.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;

							if (time > 4 * period + 1.0) gripperFlag = false, x = x5, y = y5, z = cubeZ1, a = -85, b = b5;
							if (time > 4 * period + 3.5) gripperFlag = false, x = x5, y = y5, z = cubeZ2, a = -90, b = b5;
							if (time > 4 * period + 4.0) gripperFlag = true, gr = 0;
							if (time > 4 * period + 5.5) gripperFlag = false, x = x5, y = y5, z = 100, a = -80, b = b5;
							if (time > 4 * period + 7.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = -90;
							if (time > 4 * period + 8.0)  gripperFlag = false, x = slideX, y = slideY, z = slideZ1, a = -30, b = -90;
							if (time > 4 * period + 10.0) gripperFlag = true, gr = 1;
							if (time > 4 * period + 10.5) gripperFlag = false, x = slideX, y = slideY, z = slideZ2, a = -30, b = -90;
							if (time > 4 * period + 12.0) gripperFlag = false, x = 0, y = 250, z = 300, a = 0, b = 0;
						}
						else
							std::cout << "Pause.  ";

					}
					else {
						AttemptTime = StopTime - StartTime;
						lastTaskTime = AttemptTime;
						std::cout << "Manual controll. Last task time was: " << (int)lastTaskTime / 60 << ":" << (int)lastTaskTime % 60 + (lastTaskTime - (int)lastTaskTime) << "   ";

						sf::Time elapsed = clock.restart();
						double dt = elapsed.asSeconds();

						x += dt * L_SPEED * jz / 100.0;
						y -= dt * L_SPEED * jr / 100.0;
						z -= dt * L_SPEED * ju / 200.0;
						z += dt * L_SPEED * jv / 200.0;
						a += dt * R_SPEED * jy / 100.0;
						b -= dt * R_SPEED * jx / 100.0;
						//gr += dt * GR_SPEED * jgr;
					}

					if (!inverseKinematics(myYouBotManipulator, x, y, z, a, b)) {
						//if unable to reach point, use previous coordinates;
						std::cout << "Impossible coordinates! ";
						x = xPrev;
						y = yPrev;
						z = zPrev;
						z = zPrev;
						a = aPrev;
						b = bPrev;
					}

					std::cout << "x:" << x << "  y:" << y << "  z:" << z << "  a:" << a << "  b:" << b << " gr:" << gr;


					if (gripperFlag && globalTimer.getElapsedTime().asSeconds() - gripperLastCall > 0.5) {
						gripperLastCall = globalTimer.getElapsedTime().asSeconds();
						
						if (gr) myYouBotManipulator->getArmGripper().open();
						else myYouBotManipulator->getArmGripper().close();

						gripperFlag = false;
						std::cout << "Sent data to Gripper. ";
					}
					
				}
				else {
					std::cout << "Waiting. No joystick connected";
				}

				std::cout << "\n\r";
			}

		} catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			std::cout << "unhandled exception" << std::endl;

			//delete myYouBotManipulator;
			//myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		}
		
	}

	if (youBotHasArm) {
		//home position
		setAngles(myYouBotManipulator, 0, -63, 144, -40, 0);
		myYouBotManipulator->getArmGripper().open();

		SLEEP_MILLISEC(500);
	}

	
		// clean up
		if (myYouBotManipulator) {
			//myYouBotManipulator->~YouBotManipulator();
			delete myYouBotManipulator;
			myYouBotManipulator = 0;
		}

	return 0;
}

