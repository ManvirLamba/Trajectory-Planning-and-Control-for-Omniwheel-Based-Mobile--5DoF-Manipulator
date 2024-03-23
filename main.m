%% MAIN CODE
% Includes Wrapper

close all
clc
clear all
addpath('mr')

%% Best Case
%Initial Postion of cube
Tsci = [1 0 0 1; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
%Final Postion of cube
thetaFinal = -pi()/2;
Tscf = [cos(thetaFinal) -sin(thetaFinal) 0 0;...
        sin(thetaFinal) cos(thetaFinal) 0 -1;...
        0 0 1 0 ;...
        0 0 0 1 ];
%Maximum Joint Velocity limit
maxVel = 50 ; 
%Controller Gains
kp= eye(6,6)*9 ;
ki= eye(6,6)*5 ;

%calling wrapper function 
wrapperCode(Tsci,Tscf,maxVel,kp,ki,1,'BestCase');


%% Overshoot Case
%Controller Gains
kp= eye(6,6)*1;
ki= eye(6,6)*5;

%calling function 
wrapperCode(Tsci,Tscf,maxVel,kp,ki,2,'OvershootCase');

%% Performance with low max Velocity
maxVel = 5 ; 
%Controller Gains
kp= eye(6,6)*9 ;
ki= eye(6,6)*0 ;
wrapperCode(Tsci,Tscf,maxVel,kp,ki,3,'LowMaxVelocity');


%% New Task
%Maximum Joint Velocity limit
maxVel = 50 ; 
%Controller Gains
kp= eye(6,6)*9 ;
ki= eye(6,6)*5 ;
Tsci = [1 0 0 1; 0 1 0 1 ; 0 0 1 0; 0 0 0 1];
Tscf = [cos(thetaFinal) -sin(thetaFinal) 0 1;...
        sin(thetaFinal) cos(thetaFinal) 0 -1;...
        0 0 1 0 ;...
        0 0 0 1 ];
wrapperCode(Tsci,Tscf,maxVel,kp,ki,4,'NewTask');


%% Test Functions


% Trajectory Generator Test


% Tsei - The initial configuration of the end-effector
Tsei = [1 0 0 0; 0 1 0 0; 0 0 1 0.5;0 0 0 1];
% Tsci - The initial configuration of the cube: Tsc,initial
Tsci = [1 0 0 1; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
% Tscf -  The desired final configuration of the cube: Tsc,f inal
thetaFinal = -pi()/2;
Tscf = [cos(thetaFinal) -sin(thetaFinal) 0 0;...
        sin(thetaFinal) cos(thetaFinal) 0 -1;...
        0 0 1 0 ;...
        0 0 0 1 ];
% Tceg - The configuration of the end-effector relative to the cube while grasping: Tce,grasp
th=180-45;
Tceg =[ [cosd(th) 0 sind(th); 0 1 0;-sind(th) 0 cosd(th); 0 0 0] [0 ;0; 0.025;1]];
% Tces -  The standoff configuration of the end-effector above the cube, before and after grasping, relative to the cube: Tce,standof f
Tces = [0 0 1 0; 0 1 0 0 ;-1 0 0 0.25;0 0 0 1];
% k - The number of trajectory reference configurations per 0.01 seconds: k.
% The value k is an integer with a value qof 1 or greater. 1
k=10;
[tr,trac1] = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k);
writematrix(trac1,'EETrajectory.csv')


%% Next State test

%1x 2y 3phi 4t1 5t2 6t3 7t4 8t5 9wt1 10wt2 11wt3 12wt4
currState = [0 0 0 0 0 0 0 0 0 0 0 0]; %Current State
%To move Forward
vel = [10 10 10 10 0 0 0 0 0 ]; %o1 o2 o3 o4 o5 u1 u2 u3 u4
%To move along Y
%vel = [-10 10 -10 10 0 0 0 0 0 ]; %o1 o2 o3 o4 o5 u1 u2 u3 u4
% To move rotate about Z axis
%vel = [-10 10 10 -10 0 0 0 0 0 ]; %o1 o2 o3 o4 o5 u1 u2 u3 u4
delT= 0.01; %Change in Time
maxVel = 15 ; %maxVel
stateCat = zeros(100,12);
ns=currState;
%Move along X
for i=1:100
    stateCat(i,:)= NextState(ns, vel, delT ,maxVel);
    ns = stateCat(i,:);
end
stateCat = [stateCat zeros(size(stateCat,1),1)];% Setting gripper state  to 0
%[newState] = NextState(currState, vel, delT ,maxVel);
writematrix(stateCat,'NextStateTest.csv') % Convert to csv


%% Feedback Controller Test

X =[0.170 0 0.985 0.387;0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1 ]  ;
Xd=[0 0 1 0.5; 0 1 0 0;-1 0 0 0.5; 0 0 0 1]  ;
Xdn =[0 0 1 0.6; 0 1 0 0;-1 0 0 0.3; 0 0 0 1]  ;
kp= zeros(6,6) ;
ki= zeros(6,6) ;
currState = [0,0,0, 0, 0, 0.2, -1.6, 0,0, 0,0, 0]; %theta then wheel angles
Xerrt = 0;
pil = 3.14;
maxJ = [[pil,-pil]',[0.8,-pil]', [2,-2]',[pil,-pil]',[pil,-pil]'];

[Ve,vel,Xerr,Xerrt] = FeedbackControl(currState,X,Xd,Xdn,kp,ki,Xerrt,50,maxJ);
% uncomment to display velocity
%disp(vel)
