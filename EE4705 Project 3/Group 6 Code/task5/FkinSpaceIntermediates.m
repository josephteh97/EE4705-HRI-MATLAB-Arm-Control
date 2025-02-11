function Is = FkinSpaceIntermediates(Slist, thetalist, nJoints)
I = eye(4);
for i = 1:nJoints
    I = I * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
    Is{i} = I;
end
end