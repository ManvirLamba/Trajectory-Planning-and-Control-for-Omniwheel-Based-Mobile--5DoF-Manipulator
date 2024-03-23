%% NextState
function  [newState] = NextState(currState, vel ,delT ,maxVel)

% checking if velocity is inside the limit
vel(vel > maxVel) = maxVel; % Set elements greater than 10 to 10
vel(vel < -maxVel) = -maxVel; % Set elements less than -10 to -10

l = 0.47/2;
w = 0.3/2; 
r = 0.0475;
%initializing new state to zero
newState = zeros(size(currState));
%new arm joint angles = (old arm joint angles) + (joint speeds)∆t
newState(4:8)=currState(4:8)+vel(5:9)*delT;
%new wheel angles = (old wheel angles) + (wheel speeds)∆t
newState(9:12)=currState(9:12)+vel(1:4)*delT;

phiK = currState(1);  %first element of current state is phi
delTheta = (newState(9:12) - currState(9:12)); %calculate delta theta
F = (r/4).*[-1/(l + w) 1/(l + w) 1/(l+ w) -1/(l + w);1 1 1 1;-1 1 -1 1  ]; 
Vb = F*vel(1:4)'*delT; 
% Odometry
wbz = Vb(1);
vbx = Vb(2);
vby = Vb(3);
if wbz == 0
    delQb = [0;vbx;vby];
else
    delQb = [wbz; (vbx * sin(wbz) + vby * (cos(wbz) - 1)) / wbz;...
        (vby * sin(wbz) + vbx * (1 - cos(wbz))) / wbz];
end

delQ = [1 0 0; 0 cos(phiK) -sin(phiK);0 sin(phiK) cos(phiK)]*delQb;
newState(1:3) = currState(1:3) + delQ';

end