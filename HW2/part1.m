%gravity
syms g real
%position of cart
syms pos real
%angle between cart and pole
syms theta real
%cart mass
syms mc real
%pole mass, length, and moment of inertia
syms mp lp Ip real
% linear and angular velocities
syms dpos dtheta real

%define gravity
gravity = [0; -g];

%define rotation matrix
Rcp = [cos(theta) -sin(theta); sin(theta) cos(theta)];
Rpc = Rcp';

%find center of mass location for pole
lcm = Rcp*[(lp/2); 0];

%find the velocity and acceleration of the center of mass of the pole
pvel = fulldiff(lcm, {theta, dtheta});
pacc = fulldiff(pvel, {theta, dtheta});

pacc = simplify(expand(pacc));

%find the acceleration of the cart
cacc = fulldiff(dpos, {pos, dpos});

%compute total force on pole
force = mp*pacc - mp*gravity +(mp+mc)*cacc;

%find angular acceleration
d2theta = fulldiff(dtheta, {theta, dtheta});

%find torque at base of pole
t = Ip*d2theta;
tau = t + cross2(lcm, force);

tau = simplify(expand(tau));
