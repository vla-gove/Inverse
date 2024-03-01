// Inverse.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "InverseKinematics.h"


int main() {
	LinkLengths linkLengths = { 1.0, 1.0, 1.0 };
	EndEffectorPosition position = { 1.0, 1.2, 1.2 };

	JointAngles angles = calculateInverseKinematics(linkLengths, position);

	std::cout << "Joint 1 angle: " << angles.theta1 << " radians" << std::endl;
	std::cout << "Joint 2 angle: " << angles.theta2 << " radians" << std::endl;
	std::cout << "Joint 3 angle: " << angles.theta3 << " radians" << std::endl;
}

