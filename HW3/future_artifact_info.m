load problem_2_0.dat
load problem_2_1.dat
load problem_2_2.dat

oldPos = problem_2_0(end,1:3);
oldQuat = problem_2_0(end,4:7);
w = problem_2_1(end,:);
dw = problem_2_2(end,:);

velocity = [0.0200 -0.0500 0.0100];
inertia = [0.5199, 0, 0; 0, 0.4359, 0; 0, 0, 0.7346];


%part 2e
position = oldPos + 0.01*velocity;

for n = 2:10000
    newPos = position(n-1,:) + 0.01*velocity;
    position = [position; newPos];
end

oldR = quat2rotm(oldQuat);
quaternion = oldQuat;

for n = 1:10000
   S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
   
   newR = oldR + S*oldR*0.01;
   quaternion = [quaternion; rotm2quat(newR)];
   oldR = newR;
   
   oldW = w;
   w = oldW + dw*0.01;
   
   syms dwx dwy dwz
   acc = [dwx; dwy; dwz];
   
   eqn = inertia*acc + cross(w', inertia*w') == 0;
   temp = solve(eqn,acc);
   
   dw = [double(temp.dwx), double(temp.dwy), double(temp.dwz)]; 
end

quaternion = quaternion(2:end,:);

comFutureData = [position quaternion];
save -ascii 'problem_2_3.dat' comFutureData