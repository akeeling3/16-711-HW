function [] = roboDraw(angles)
%Draw the result of the optimization in 3D
    
    %Initialize variables
    global objects;
    
    pos = kin(angles);
    
    %Plot the robot
    plot3(pos(:,1), pos(:,2), pos(:,3));
    hold on;
    
    %Plot each obstacle
    for i = 1:numel(objects)/4
       [X,Y,Z] = sphere;
       r = objects(i,4);
       surf(X+(r*objects(i,1)), Y+(r*objects(i,2)), Z+(r*objects(i,3)));
       hold on;        
    end

end