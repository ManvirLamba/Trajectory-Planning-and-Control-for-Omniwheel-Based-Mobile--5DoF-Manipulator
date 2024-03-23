%% FeedbackControl

function [V,vel,Xerr,Xerrt] = FeedbackControl(currState,X,Xd,Xdn,Kp,Ki,Xerrt,maxVelo,maxJoint)

delT = 0.01;
Blist = [[0;  0; 1;       0; 0.033; 0], ...
         [0; -1; 0; -0.5076;     0; 0], ...
         [0; -1; 0; -0.3526;     0; 0], ...
         [0; -1; 0; -0.2176;     0; 0],...
         [0;  0; 1;       0;     0; 0]];
l = 0.47/2;
w = 0.3/2; 
r = 0.0475;
F = (r/4).*[-1/(l + w) 1/(l + w) 1/(l+ w) -1/(l + w);1 1 1 1;-1 1 -1 1  ]; 
% 6x6 version of F
F6  = [zeros(1,size(F,2));zeros(1,size(F,2));F;zeros(1,size(F,2))];
Tb0 = [[eye(3,3); 0 0 0 ] [0.1662;0;0.0026;1] ];
M0e = [[eye(3,3); 0 0 0 ] [0.033;0;0.6546;1] ];
thetalist = currState(4:8)';
T0e = FKinBody(M0e,Blist,thetalist);
Jarm = JacobianBody(Blist, thetalist);
Jb = Adjoint(inv(T0e)*inv(Tb0) )*F6;
Je = [Jb Jarm];

Vd = (se3ToVec(Xd\Xdn)/delT);
Xerr = se3ToVec(MatrixLog6(X\Xd));
Xerrt = Xerrt()+Xerr;
a = Adjoint(TransInv(X)*Xd)*Vd;
% Control
V = a + Kp*Xerr + Ki*Xerrt*delT;
vel = pinv(Je,1e-3)*V; 

% check jacobian
checkJe=checkJointLimit(Je,vel',delT,currState,maxVelo,maxJoint);
vel = pinv(checkJe,1e-3)*V;  %wheel vel then theta dots

end

%% Define function to check joint limits
function Je = checkJointLimit(Je,velo,dt,currState,maxVelo,maxJoint)
   testState = NextState(currState, velo, dt, maxVelo);
   itr = size(maxJoint,2);
   testTheta=testState(4:8);
   for j = 2:3
       if testTheta(j)>maxJoint(1,j)||testTheta(j)<maxJoint(2,j)
           Je(:,j+4)=zeros(6,1);
       end
   end
end