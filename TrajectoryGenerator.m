%% TrajectoryGenerator

function [finalTraj,catt] = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k) 

% multiply T's together such that all become s-->e transformations
% w is for waypoint
w1=Tsei;        % s - e initial
w2=Tsci*Tces;   % standby
w3=Tsci*Tceg;   % move gripper to cube position
w4=w2;          % standby
w5=Tscf*Tces;   % move to standby pos above final position
w6=Tscf*Tceg;   % move to ground
w7=w5;          % standby again
tf =  [0.75 0.1 0.1 0.1 0.25 0.1 0.1 0.1];  %time between waypoints

N = tf*k/0.01;    % number of iterations
finalTraj = zeros(4,4,sum(N)); 

% Move the gripper from its initial configuration to a "standoff” configuration a few cm above the block.
traj1 = ScrewTrajectory(w1, w2, tf(1),N(1),5);
t1=rowVector(traj1,N(1),0);
finalTraj1= reshape(cell2mat(traj1),[4,4,N(1)]);

% Move the gripper down to the grasp position.
traj2= ScrewTrajectory(w2, w3,  tf(2),N(2),5);
t2=rowVector(traj2,N(2),0);
finalTraj2 = reshape(cell2mat(traj2),[4,4,N(2)]);

% Close the gripper2
traj3 = ScrewTrajectory(w3, w3, tf(3),N(3),5);
finalTraj3 = reshape(cell2mat(traj3),[4,4,N(3)]);
t3=rowVector(traj3,N(3),1);

% Move the gripper back up to the "standoff” configuration.
traj4 = ScrewTrajectory(w3, w4, tf(4),N(4),5);
t4=rowVector(traj4,N(4),1);
finalTraj4 = reshape(cell2mat(traj4),[4,4,N(4)]);

% Move the gripper to a "standoff” configuration above the final configuration.
traj5=ScrewTrajectory(w4, w5, tf(5),N(5),5);
t5=rowVector(traj5,N(5),1);
finalTraj5 = reshape(cell2mat(traj5),[4,4,N(5)]);

% Move the gripper to the final configuration of the object.
traj6=ScrewTrajectory(w5, w6, tf(6),N(6),5);
t6=rowVector(traj6,N(6),1);
finalTraj6 = reshape(cell2mat(traj6),[4,4,N(6)]);

% Open the gripper.
traj7 = ScrewTrajectory(w6, w6, tf(7),N(7),5);
t7=rowVector(traj7,N(7),0);
finalTraj7 = reshape(cell2mat(traj7),[4,4,N(7)]);

% Move the gripper back to the "standoff” configuration.
traj8=ScrewTrajectory(w6, w7, tf(8),N(8),5);
t8=rowVector(traj8,N(8),0);
finalTraj8 = reshape(cell2mat(traj8),[4,4,N(8)]);

% combine all 8 steps (4x4 vectors) into 1 stack, dim 3
finalTraj= cat(3,finalTraj1,finalTraj2,finalTraj3,finalTraj4,finalTraj5,finalTraj6,finalTraj7,finalTraj8);
catt = [t1 ;t2;t3;t4;t5;t6;t7;t8];  % concatenate all the t's
end

%% convert stack of 4x4 matrices into rows for csv and include gripper state
function [convtTr]=rowVector(traj1,N,gp)
convtTr=0;
temp = reshape(cell2mat(traj1),[4,4,N]); % Convert cells to matrix
% Reshape and transpose rotation matrices
row_vector=reshape(permute(temp(1:3,1:3,:), [2 1 3]), [], size(temp, 3))';
% Reshape translation matrices
translation_vector = reshape(temp(1:3,4,:), [], size(temp, 3))';
    % Define gripper postion 
    if gp == 1
        % Gripper open
        convtTr = [row_vector, translation_vector, ones(size(temp, 3), 1)];
    elseif gp == 0
        % Gripper closed
        convtTr = [row_vector, translation_vector, zeros(size(temp, 3), 1)];
    end
end

