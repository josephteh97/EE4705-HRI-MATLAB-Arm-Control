%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)
% url: https://www.mathworks.com/help/robotics/ref/rigidbodytree.gettransform.html
% >> showdetails(robot_struct)
% --------------------
% Robot: (8 bodies)

%  Idx               Body Name         Joint Name         Joint Type               Parent Name(Idx)   Children Name(s)
%  ---               ---------         ----------         ----------               ----------------   ----------------
%    1           Shoulder_Link          Actuator1           revolute                   base_link(0)   HalfArm1_Link(2)  
%    2           HalfArm1_Link          Actuator2           revolute               Shoulder_Link(1)   HalfArm2_Link(3)  
%    3           HalfArm2_Link          Actuator3           revolute               HalfArm1_Link(2)   ForeArm_Link(4)  
%    4            ForeArm_Link          Actuator4           revolute               HalfArm2_Link(3)   Wrist1_Link(5)  
%    5             Wrist1_Link          Actuator5           revolute                ForeArm_Link(4)   Wrist2_Link(6)  
%    6             Wrist2_Link          Actuator6           revolute                 Wrist1_Link(5)   Bracelet_Link(7)  
%    7           Bracelet_Link          Actuator7           revolute                 Wrist2_Link(6)   EndEffector_Link(8)  
%    8        EndEffector_Link        Endeffector              fixed               Bracelet_Link(7)   
% --------------------

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
    

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
nJoints = length(theta);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 

M = eye(4);
ws = [
    [0 0 -1];
    [0 1 0];
    [0 0 -1];
    [0 1 0];
    [0 0 -1];
    [0 1 0];
    [0 0 -1];
];

Ms = cell(1, nJoints);
Slist = zeros(6, nJoints);
thetalist = zeros(nJoints, 1);
for k=1:nJoints
    M_ = robot_struct.Bodies{k}.Joint.JointToParentTransform;
    M = M * M_;

    homo_q = M(:,4);
    q = homo_q(1:3);
    w = ws(k, :);
    v = -cross(w, q);
    s = cat(2, w, v);

    Slist(:, k) = s;
    thetalist(k,:) = tConfiguration(k).JointPosition;
    Ms{k} = M;
    % T_matrix = FKinSpace(M, Slist, thetalist, k);
    % T{k} = T_matrix;
    % homogenousCoordinates = T_matrix(:,4);
    % X(k,:) = homogenousCoordinates;
end

Is = FkinSpaceIntermediates(Slist, thetalist, nJoints);

for k=1:nJoints
    T_ = Is{k} * Ms{k};
    T{k} = T_;
    homo_q = T_(:, 4);
    X(k,:) = homo_q;
end

end