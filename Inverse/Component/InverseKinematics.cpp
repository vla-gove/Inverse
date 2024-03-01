// simple geometric approach assuming zero twists and offsets, neglecting ee orientation

// function for calculating inverse kinematics of a 3dof robot manipulator with all revolute joints
// first joint rotates around the z axis, changing x and y coordinates
// second two joints are perpendicular to the first, changing the height of the end effector (z value), and also x and y due to geometric constraints
JointAngles calculateInverseKinematics(const LinkLengths& linkLengths, const EndEffectorPosition& position) {
    JointAngles angles;

    double x = position.x;
    double y = position.y;
    double z = position.z;

    // first joint angle calculation (rotates around z axis, changes x and y value)
    angles.theta1 = atan2(y, x);

    // distance from the first joint to the end effector projection on the xy plane
    double d = sqrt(x * x + y * y) - linkLengths.a3;

    // third joint angle calculation based on link lengths, first joint distance from xy projection, and z value
    angles.theta3 = acos((d * d + z * z - linkLengths.a1 * linkLengths.a1 - linkLengths.a2 * linkLengths.a2) / (2 * linkLengths.a1 * linkLengths.a2));

    // second joint angle calculation based on link lengths, first joint distance from xy projection, and z value
    double alpha = atan2(z, d);
    double beta = acos((linkLengths.a1 * linkLengths.a1 + d * d + linkLengths.a2 * linkLengths.a2 - z * z) / (2 * linkLengths.a1 * sqrt(d * d + z * z)));
    angles.theta2 = alpha + beta;

    return angles;
}