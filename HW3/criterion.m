%%% criterion for optimizing initial center of mass location and velocity
function score=criterion(p)

global markerData
global numSamples

numMarkers = 8;

% pull out parameters
com = [ p(1) p(2) p(3) ];
vel = [ p(4) p(5) p(6) ];

score = 0;

% calculate distances from com to each marker.
d2(numMarkers) = 0;
for j = 1:numMarkers
 v = markerData(1,(3*(j-1)+2):(3*(j-1)+4)) - com;
 d2(j) = v*v';
 end

for i = 2:numSamples
 for j = 1:numMarkers
  v = markerData(i,(3*(j-1)+2):(3*(j-1)+4)) - com - vel*0.01*markerData(i,1);
  dist = v*v';
  score = score + (d2(j) - dist)*(d2(j) - dist);
 end
end



end
