%% WRAPPER CODE

function [stateCat,plotX]=wrapperCode(Tsci,Tscf,maxVel,kp,ki,num,text)
%% Initializations
%Home postion of robot manipulator (arm) for the Forward Kinematics
M0e = [[eye(3,3); 0 0 0 ] [0.033;0;0.6546;1] ];
% Screw Axis wrt to body frame
Blist = [[0;  0; 1;       0; 0.033; 0], ...
         [0; -1; 0; -0.5076;     0; 0], ...
         [0; -1; 0; -0.3526;     0; 0], ...
         [0; -1; 0; -0.2176;     0; 0],...
         [0;  0; 1;       0;     0; 0]];
% Definition between body and manipulator base frame
Tb0= [[eye(3,3); 0 0 0 ] [0.1662;0;0.0026;1] ];

% Tsei - The initial configuration of the end-effector wrt space frame
Tsei = [1 0 0 0; 0 1 0 0; 0 0 1 0.5;0 0 0 1];

% Tceg - The configuration of the end-effector relative to the cube while grasping: Tce,grasp
th=180-45;  %th temp variable for theta
Tceg =[ [cosd(th) 0 sind(th); 0 1 0;-sind(th) 0 cosd(th); 0 0 0] [0 ;0; 0.025;1]];

% Tces -  The standoff configuration of the end-effector above the cube, before and after grasping, relative to the cube: Tce,standof f
Tces = [0 0 1 0; 0 1 0 0 ;-1 0 0 0.25;0 0 0 1];

%Current State of the robot, initialize at zero
currState = [0 0 0 0 0 0 0 0 0 0 0 0]; 

delT= 0.01; %Change in Time


%% Reference Trajectory Generation
%Component 2: Reference Trajectory Generator (TrajectoryGenerator)
%~Inputs
%The initial configuration of the block is at (x, y, θ) = (1 m, 0 m, 0 rad) and the
%final configuration is (x, y, θ) = (0 m, −1 m, −π/2 rad)

% k - The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer with a value qof 1 or greater. 1
k=10;  %k=1

% 6 inputs, 2 outputs.
[tr,trac1] = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k);

%% Loop
% N is number of iterations, 1600
N = size(tr,3)-1;
stateCat = zeros(N,12);
%Changing initial postion to add some error
stateCat(1,:)=[0 -0.9 0 0.177 0.502 -0.760 -0.636 0 0 0 0 0 ];

% Initilization for the for loop
ns = currState;
Xerr  = zeros(6,1);
plotX = [];     % initialize empty list for holding error, to plot later
Xerrt = zeros(6,1);
vel = zeros(N,9);
pil = pi;  % temporary variable for joint limit
maxJ = [[pil,-pil]',[0.8,-pil]', [2,-2]',[pil,-pil]',[pil,-pil]'];

for i=1:N
    % Transformation matrix from s-frame to body frame 
    Tsb=[cos(stateCat(i,1)) -sin(stateCat(i,1)) 0 stateCat(i,2) ;...
         sin(stateCat(i,1)) cos(stateCat(i,1)) 0 stateCat(i,3);...
         0 0 1 0.0963;...
         0 0 0 1 ] ;
    %Transformation matrix from base of manipulator to e-e of manipulator
    T0e=FKinBody(M0e,Blist,stateCat(i,4:8)');    
    %Current actual position of the end effector wrt space frame
    X = Tsb*Tb0*T0e;
    thetalist = stateCat(i,:);
    % calculate the control law using FeedbackControl,
    % and generate the wheel and joint controls using J†e(θ).
    Xd = tr(:,:,i);    % X desired
    Xdn = tr(:,:,i+1); % X desired, next
    %% Call FeedbackControl function
    % 9 inputs, 4 outputs
    [Ve,vel(i,:),Xerr,Xerrt] = FeedbackControl(thetalist,X,Xd,Xdn,kp,ki,Xerrt,maxVel,maxJ);
    plotX = [plotX; Xerr'];
    %% use NextState to calculate the new configuration
    stateCat(i+1,:) = NextState(ns, vel(i,:) ,delT ,maxVel);
    % store every k th Xerr 6-vector to later plot the evolution of the error over time
    % store every k th configuration for later animation

    ns = stateCat(i+1,:);
    % if mod(i, 100) == 0
    %     disp(i)
    % end
end


%% Export csv
g = trac1(:,13);   % g is gripper state
stateCat = [stateCat g];  % concatenate gripper state
filePath = strcat(text,'.csv');
writematrix(stateCat,filePath)
%% Plot Error
figure(num)
t = linspace(0, (N-1)*delT, N);
hold on
plot(t,plotX(:,1))
plot(t,plotX(:,2))
plot(t,plotX(:,3))
plot(t,plotX(:,4))
plot(t,plotX(:,5))
plot(t,plotX(:,6))
legend('w1 error','w2 error','w3 error','v1 error','v2 error','v3 error') 
xlabel('Time (sec)')
ylabel('Error')
title(text)
hold off
end