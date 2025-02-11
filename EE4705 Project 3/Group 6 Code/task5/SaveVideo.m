v = VideoWriter("video.avi");
v.FrameRate = 10;
open(v);

for t=1:size(theta,2)
    show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
    pause(1/50);
end

close(v);