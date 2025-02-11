% Input: 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory 

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma));

for m = 1 : nJoints
    % Each joint is sampled independently
    % The starting q0 and final qT are fixed, so set the sample to 0
    % sample from multivariable Gaussian distribution
    gau_dis = mvnrnd(mu, sigma, nSamplePaths);
    zero_dis = zeros(size(gau_dis(:,1)),'like', gau_dis); % 20 by 1 
    dis = cat(2,zero_dis, gau_dis, zero_dis); % 20 by 20
    em{m} = dis;
end

% regroup it by samples
emk = [em{:}];
for k=1:nSamplePaths
    % emk(k,:): 1 by 140; nDiscretize = 20; nJoints = 7;
    ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
end
% Columns 1 through 12

% 0   -0.0037   -0.0218   -0.0494   -0.0668   -0.0639   -0.0698   -0.0862   -0.0973   -0.1147   -0.1252   -0.1462
% 0    0.0030    0.0030    0.0120   -0.0032   -0.0046   -0.0169   -0.0170    0.0085    0.0315    0.0395    0.0728
% 0    0.0165    0.0366    0.0668    0.0748    0.0839    0.0900    0.0938    0.1114    0.1445    0.1830    0.1951
% 0   -0.0105   -0.0200   -0.0356   -0.0499   -0.0599   -0.0420   -0.0332   -0.0021   -0.0087   -0.0075    0.0035
% 0    0.0342    0.0734    0.1082    0.1089    0.1306    0.1600    0.1717    0.1883    0.2194    0.2324    0.2212
% 0    0.0153    0.0169    0.0124   -0.0013   -0.0406   -0.0906   -0.1052   -0.1131   -0.1241   -0.1414   -0.1479
% 0   -0.0097   -0.0186   -0.0392   -0.0502   -0.0232   -0.0265   -0.0333   -0.0372   -0.0500   -0.0564   -0.0899

% Columns 13 through 20

% -0.1467   -0.1151   -0.0608   -0.0229   -0.0053   -0.0025    0.0071         0
% 0.0790    0.0798    0.0559    0.0152   -0.0051   -0.0210   -0.0186         0
% 0.1961    0.1963    0.1758    0.1379    0.1031    0.0706    0.0314         0
% 0.0208    0.0395    0.0249   -0.0008   -0.0316   -0.0300   -0.0112         0
% 0.1883    0.1496    0.1013    0.0357   -0.0114   -0.0156   -0.0098         0
% -0.1270   -0.0766   -0.0589   -0.0395   -0.0184   -0.0179   -0.0233         0
% -0.1126   -0.1196   -0.1040   -0.1088   -0.1037   -0.0674   -0.0327         0