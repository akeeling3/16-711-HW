function [ineq, eq] = const(angles)
%Define constraints for inverse kinematics optimization

   %Initialize variables 
   global boundaries objects lengths;
    
   pos = kin(angles);
    
   eq = [];
   ineq = [];
   testPoints = [];
   
   num_links = numel(lengths);
   
   %Set joint limits
   for i = 1:num_links
      ineq = [ineq; boundaries(i,1)-angles(i,1); angles(i,1)-boundaries(i,2); ...
          boundaries(i,3)-angles(i,2); angles(i,2)-boundaries(i,4);...
          boundaries(i,5)-angles(i,3); angles(i,3)-boundaries(i,6)];
      
      %Define a vector of link end- and midpoints
      testPoints = [testPoints; pos(i,1:3)];
      if i<num_links
          midPoint = (pos(i+1,1:3)-pos(i,1:3))/2;
          testPoints = [testPoints; midPoint];
      end
   end
   
   %Define obstacles and use vector of end- and midpoints to limit robot to
   %obstacle-free space
   for n = 1:numel(testPoints)/3
       point = testPoints(n,:);
       
       for m = 1:numel(objects)/4
          distance = sqrt((point(1,1)-objects(m,1)).^2+(point(1,2)-objects(m,2)).^2+...
              (point(1,3)-objects(m,3)).^2);
          ineq = [ineq; objects(m,4)-distance];
       end
   end
end