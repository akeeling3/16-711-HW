load problem_2_0.dat;

rotGlobal = [0 0 1 0; 1 0 0 11; 0 -1 0 4; 0 0 0 1];
rotQuat = [0.6479 -0.7563 -0.0883 -0.0201];

lander = zeros(1,7);

for n = 1:10000
    artifactPos = [problem_2_0(n,1:3)'; 1];  
    worldPos = rotGlobal*artifactPos;
    
    landerPos = worldPos(1:3)';
    
    currentQuat = problem_2_0(n,4:end);
    landerRot = quatMult(currentQuat, rotQuat);
    
    if n>1
        if abs(sum(landerRot-lander(n-1,4:end))) > 0.5
            landerRot = -landerRot;
        end
    end
    
    landerRot = landerRot/norm(landerRot);
    
    lander = [lander; landerPos landerRot];
end

lander = lander(2:end,:);

save -ascii 'problem_3.dat' lander