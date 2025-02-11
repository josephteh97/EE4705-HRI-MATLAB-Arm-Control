% update theta 
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);

dtheta = zeros(nJoints, nDiscretize);

for i=1:nJoints
    em_i = em{i};
    T = trajProb .* em_i;
    T_sum = sum(T, 1);
    dtheta(i,:) = T_sum;
end
