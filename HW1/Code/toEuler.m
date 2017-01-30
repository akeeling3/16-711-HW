function euler = toEuler(q)
%Convert a given quaternion to XYZ Euler angles

    roll = atan2(2*((q(1)*q(2))+(q(3)*q(4))), 1-(2*(q(2).^2+q(3).^2)));
    pitch = asin(2*((q(1)*q(3))-(q(4)*(q(2)))));
    yaw = atan2(2*((q(1)*q(4))+(q(2)*q(3))), 1-(2*(q(3).^2+q(4).^2)));
    
    euler = [roll pitch yaw];
end