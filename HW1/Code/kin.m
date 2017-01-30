function pos = kin(angles)
%Forward kinematics for a snake robot with given angles

    %Initialize variables
    global lengths;
   
    num_links = numel(lengths);
    pos = [0 0 0 0 0 0];
    
    x_angle = 0;
    y_angle = 0;
    z_angle = 0;
    
    %Define the endpoints of each link given the length and rotation
    for i = 1:num_links
        x_angle = x_angle + angles(i, 1);
        y_angle = y_angle + angles(i, 2);
        z_angle = z_angle + angles(i, 3);
        link = [lengths(i); 0; 0];
        
        Rx = [1 0 0; 0 cos(x_angle) -sin(x_angle); 0 sin(x_angle) cos(x_angle)];
        Ry = [cos(y_angle) 0 sin(y_angle); 0 1 0; -sin(y_angle) 0 cos(y_angle)];
        Rz = [cos(z_angle) -sin(z_angle) 0; sin(z_angle) cos(z_angle) 0; 0 0 1];
        
        link_pos = transpose(Rx*Ry*Rz*link);
        
        if i ~= 1
            link_pos = pos(i-1, 1:3) + link_pos;
        end
        
        %Record the endpoints and rotated angles of each link
        pos = [pos; link_pos x_angle y_angle z_angle];
    end
    
    pos = pos(2:end,:);
end