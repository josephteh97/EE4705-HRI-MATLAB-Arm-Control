%Parameters
% T = 5;
nDiscretize = 20; % number of discretized waypoint
nPaths = 50; % number of sample paths
convergenceThreshold = 0.01; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5*Rinv/sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    % Sample noisy trajectories
    [theta_paths, em] = stompSamples(nPaths, Rinv, theta);

    % Calculate Local trajectory cost
    Stheta = zeros(nPaths, nDiscretize);
    for i=1:nPaths
        theta_path = theta_paths{i};
        [local_trajectory_cost, ~] = stompTrajCost(robot_struct, theta_path,  R, voxel_world);
        Stheta(i,:) = local_trajectory_cost;
    end

    % Given the local traj cost, update local trajectory probability
    trajProb = stompUpdateProb(Stheta);

    % Compute delta theta (aka gradient estimator)
    dtheta = stompDTheta(trajProb, em);

    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);

    % Compute the cost of the new trajectory
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
 
    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Stop iteration criteria:
    if iter > 100 || sum(dtheta_smoothed,'all') == 0
        disp('Maximum iteration (30) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
        disp('Estimated gradient is 0.')
        break
    end

end

disp('STOMP Finished.');


%% Plot path
% axis tight manual % this ensures that getframe() returns a consistent size
for t=1:size(theta,2)
    show(robot, theta(:,t),'PreservePlot', true, 'Frames', 'off');
    drawnow;
    pause(1/50);
end


%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')

