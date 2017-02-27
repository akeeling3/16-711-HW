%Use LQR to find gains for cart-pole sim

mc = 1;
mp = 0.1;
l = 0.5;

H = [mc+mp mp*l; mp*l mp*l.^2];
dG = [0; mp*9.81*l];
C = [0 -mp*l; 0 0];
D = [1; 0];

A = [0 0 1 0; 0 0 0 1];
B = [0; 0];
T = [[0; 0] inv(H)*dG inv(H)*C];

A = [A; -T];
B = [B; inv(H)*D];

Q = [1 0 0 0; 0 0 0 0; 0 0 1 0; 0 0 0 0];
R = 1;

K = dlqr(A,B,Q,R);