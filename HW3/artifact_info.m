[data, ~, ~, ~] = mrdplot_convert('d00059');
markerData = data(:, 81:104);


%part 2a
position = [-11.0000 0.0002 0.0000];
velocity = [0.0200 -0.0500 0.0100];
quaternion = [1 0 0 0];

o = (markerData(1,1:3))';
vec1 = (markerData(1,4:6))'-o;
vec1 = vec1/norm(vec1);
vec2 = (markerData(1,7:9))'-o;
vec2 = vec2/norm(vec2);
vec3 = (markerData(1,13:15))'-o;
vec3 = vec3/norm(vec3);
p0 = [vec1 vec2 vec3];

for n = 2:10000
    %Find position
    position = [position; position(n-1,:)+0.01*velocity];
    
    %Find rotation
    o = (markerData(n,1:3))';
    vec1 = (markerData(n,4:6))' - o - 0.01*velocity';
    vec1 = vec1/norm(vec1);
    vec2 = (markerData(n,7:9))' - o - 0.01*velocity';
    vec2 = vec2/norm(vec2);
    vec3 = (markerData(n,13:15))' - o - 0.01*velocity';
    vec3 = vec3/norm(vec3);
    p1 = [vec1 vec2 vec3];
    
    rot = p1/p0;
    quaternion = [quaternion; rotm2quat(rot)];
end

comData = [position quaternion];
save -ascii 'problem_2_0.dat' comData


%part 2b
zyx = quat2eul(quaternion(1,:));
prevAngle = flip(zyx);
angularVel = [0, 0, 0];

for n = 2:10000
   zyx = quat2eul(quaternion(n,:));
   angle = flip(zyx);
   w = (angle - prevAngle)*100;
   
   prevAngle = angle;
   angularVel = [angularVel; w(1) w(2) w(3)];
end

save -ascii 'problem_2_1.dat' angularVel


%part 2c
angularAcc = [0, 0, 0; 0, 0, 0];

for n = 2:9999
    acc = (angularVel(n,:)-angularVel(n-1,:))*100;
    angularAcc = [angularAcc; acc];
end

save -ascii 'problem_2_2.dat' angularAcc


%part 2d
i = floor(linspace(1,10000));

wx = angularVel(i,1);
wy = angularVel(i,2);
wz = angularVel(i,3);

dwx = angularAcc(i,1);
dwy = angularAcc(i,2);
dwz = angularAcc(i,3);

A1 = [dwx(1) dwy(1)-(wx(1)*wz(1)) dwz(1)+(wx(1)*wy(1)) wy(1)*wz(1) wy(1)^2-wz(1)^2 wy(1)*wz(1)];
A2 = [wx(1)*wz(1) dwx(1)+(wy(1)*wz(1)) wx(1)^2+wz(1)^2 dwy(1) dwz(1)-(wx(1)*wy(1)) wx(1)*wz(1)];
A3 = [wx(1)*wy(1) wx(1)^2-wy(1)^2 dwx(1)-(wy(1)*wz(1)) -(wx(1)*wy(1)) dwy(1)+(wx(1)*wz(1)) dwz(1)];

A = [A1; A2; A3];

for n = 2:100
    A1 = [dwx(n) dwy(n)-(wx(n)*wz(n)) dwz(n)+(wx(n)*wy(n)) wy(n)*wz(n) wy(n)^2-wz(n)^2 wy(n)*wz(n)];
    A2 = [wx(n)*wz(n) dwx(n)+(wy(n)*wz(n)) wx(n)^2+wz(n)^2 dwy(n) dwz(n)-(wx(n)*wy(n)) wx(n)*wz(n)];
    A3 = [wx(n)*wy(n) wx(n)^2-wy(n)^2 dwx(n)-(wy(n)*wz(n)) -(wx(n)*wy(n)) dwy(n)+(wx(n)*wz(n)) dwz(n)];

    A = [A; A1; A2; A3];   
end

[~,~,V] = svd(A);

moment = abs([V(1,6), V(2,6), V(3,6); V(3,6), V(4,6), V(5,6); V(3,6), V(5,6), V(6,6)]);

pWorld = [1 0 0; 0 1 0; 0 0 1];
inertRot = pWorld/p0;
inertQuat = rotm2quat(inertRot);