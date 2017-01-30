function [roll, pitch, yaw] = part1(target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles)
%Perform inverse kinematics optimization for a given target and snake robot    

    %Define variables
    global lengths goal boundaries objects;
    
    target = target';
    
    lengths = link_length;
    goal = [target(1,1:3) toEuler(target(1,4:end))];
    boundaries = [min_roll max_roll min_pitch max_pitch min_yaw max_yaw];
    objects = obstacles;
    
    %Set initial guess and options for optimization
    a0 = [min_roll min_pitch min_yaw];
    options = optimset('Display','iter','MaxFunEvals',1000000,'Algorithm','interior-point');
    
    %Perform optimization for joint angles
    [angles, ~] = fmincon(@toOpt, a0, [], [], [], [], [], [], @const, options);
    
    %Output roll, pitch, and yaw vectors
    roll = angles(:, 1);
    pitch = angles(:, 2);
    yaw = angles(:, 3);
    
    %Draw result
    roboDraw(angles)
end