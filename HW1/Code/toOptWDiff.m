function score = toOptWDiff(angles)
%Optimization criteria for inverse kinematics

    %Define variables
    global goal boundaries lengths;
    
    pose = kinWDiff(angles);
    num_links = numel(lengths);
    
    %Initial score based on distance from robot tip to goal
    score = sum(pose(end,:) - goal).^2;
        
    %Add a smaller score rewarding distance from joint limits
    for i = 1:num_links
       midAngles = [(boundaries(i,2)-boundaries(i,1))/2 (boundaries(i,4)-boundaries(i,3))/2 ...
           (boundaries(i,6)-boundaries(i,5))/2];
       
       score = score + sum(0.1*(pose(i,4:end)-midAngles).^2);
    end
end