close all;
clear all;
clc;

%% This script computes the forward kinmatics of stewart platform 
%   using newton raphson method for error control, screw based kinematic
%   jacobian and inverse kinematics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs (1):
% leg joint points for base and end effector
% points b: base joints
% points p: end-effector points
% both 6x3 matrices are stacked to
% legs = [p;b]
%
% inputs (2):
% measured leg lengths
% InLegLn = [L1;L2;L3;L3;L4;L5;L6];
% outputs (1):
% homogeneous transform from end-effector coordinate system to base
% GuessPlatPose
% outputs (2): 
% optional output
% estimated leg lengths
% ThLegLn
    
%% addon Nureddin: Leg joint points
r1 = 150;
r2 = 150;
n1 = 12;
n2 = 12;
kp = [1,2,5,6,9,10];
kb = [1,4,5,8,9,12];
%kp = [1,4,5,8,9,12];
%kb = [1,2,5,6,9,10];
kp = kp-1;
kb = kb-1;
for k=1:6
    p(:,k) = [r1*cos(2/n1*pi*kp(k));r1*sin(2/n1*pi*kp(k));0];
    b(:,k) = [r2*cos(2/n2*pi*kb(k));r2*sin(2/n2*pi*kb(k));0];
end;    

legs = [p;b];
%% Nureddin Addon: actual leg lengths
InLegLn = [167.02;167.03;190.61;194.32;170.91;126];
%% addon Nureddin: initial platform position
GuessPlatPose = [1,0,0,0;0,1,0,0;0,0,1,max(abs(InLegLn));0,0,0,1];

%% forward kinematics calculation
% newton raphson method 
% guessing the pose, then computing the leglengths corresponding to the
% guesses pose, then updating the guessed pose and so on till the
% difference between the actual leg lengths and leg length corresponding to
% the updated guessed pose is small. If the loop converges, the guessed
% pose will give the forward kinematics corresponding to the actualleg
% lengths 
%
% AcLegLn = Actual Leg Lengths [L1,L2,L3,L4,L5,L6]
AcLegLn = InLegLn;

for i = 1:100                       
    ThLegLn = zeros(6,1);
    jac = zeros(6,6);               % with screws stacked horizontally   
    for ji = 1:6
        PlatPt = GuessPlatPose*[legs(1:3,ji);1];          r = PlatPt(1:3) - GuessPlatPose(1:3,4);
        BasePt = legs(4:6,ji);
        L = PlatPt(1) - BasePt(1);  M = PlatPt(2) - BasePt(2);  N = PlatPt(3) - BasePt(3) ;
        ThLegLn(ji) = sqrt(L^2+M^2+N^2);
        jac(ji,:) = [L,M,N,cross(r,[L;M;N])']/ThLegLn(ji);
    end

    error = AcLegLn - ThLegLn;
    if norm(error) < 1e-10
        break
        % breaking the loop in case the error in leg lengths is extremely
        % small
    end

    if rank(jac) < 6
        disp('Jacobian turned out to be singular....  Stopping')
        break
    end

    dlta = jac\error;
    GuessPlatPose = NewOri(dlta, GuessPlatPose);
    %disp(['iteration: ',num2str(i)]);
end
disp(GuessPlatPose)
disp(ThLegLn)



%% eval 
sqrt(sum(GuessPlatPose(1:3,4).^2))
GuessPlatPose * [150;0;0;1]
