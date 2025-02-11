%% Plot path
% axis tight manual % this ensures that getframe() returns a consistent size
for t=1:size(theta,2)
    show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'off');
    drawnow;
    pause(1/50);
end