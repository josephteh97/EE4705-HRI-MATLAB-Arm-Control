function cost = stompConstraintCost(robot_struct, theta)

nBodies = robot_struct.NumBodies;
endEffectorName = robot_struct.Bodies{nBodies}.Name;

theta_cell = num2cell(theta);
tConfiguration = robot_struct.homeConfiguration;
[tConfiguration.JointPosition] = theta_cell{:};

% TODO: hardcoded transformation
T_d = axang2tform([1 0 0 pi/2])*axang2tform([0 1 0 pi/2]); 
T_e = getTransform(robot_struct, tConfiguration, endEffectorName, robot_struct.BaseName);

R_d = T_d(1:3,1:3);
R_e = T_e(1:3,1:3);

A = eye(3) - transpose(R_d) * R_e;
cost = norm(A, "fro") ^ 2;

end