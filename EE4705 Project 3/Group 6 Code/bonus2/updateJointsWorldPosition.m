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

% TODO: get the body names
baseLinkName = robot_struct.BaseName;
bodyLinkNames = robot_struct.BodyNames;

for k=1:nJoints
    % get the homegeneous transformation from kth joint's frame to the
    % base frame
    % % getTransform can only takes in structure array Configuration
    % TODO: get current body frame
    bodyLinkName = bodyLinkNames{k};
    % TODO: get transformation matrix between base link and body link
    T_matrix = getTransform(robot_struct, tConfiguration, bodyLinkName, baseLinkName);
    T{k} = T_matrix;
    % % Get joint's world coordinates
    homogenousCoordinates = T_matrix(:,4);
    X(k,:) = homogenousCoordinates;
end
    
end