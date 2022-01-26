function [GuessPlatPose, ThLegLn] = fk_stewart_6_6(b,p,InLegLn)
% function [GuessPlatPose, ThLegLn] = fk_stewart_6_6(b,p,InLegLn)
%
% This script computes the forward kinmatics of stewart platform 
%   using newton raphson method for error control, screw based kinematic
%   jacobian and inverse kinematics
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs (1):
%  leg joint points for base and end effector
%  points b: base joints
%  points p: end-effector points
%  both 6x3 matrices are stacked to
%  legs = [p;b]
%
% inputs (2):
%  measured leg lengths
%  InLegLn = [L1;L2;L3;L3;L4;L5;L6];
% outputs (1):
%  homogeneous transform from end-effector coordinate system to base
%  GuessPlatPose
% outputs (2): 
%  optional output
%  estimated leg lengths
%  ThLegLn
    
%% Leg joint points
legs = [p;b];
%% initial platform position
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
    GuessPlatPose = NewOriN(dlta, GuessPlatPose);
end
%disp(GuessPlatPose)
%disp(ThLegLn)
% nested functions
    function [ Pfnew ] = NewOriN( dlta, Pfold )
    % takes delta (dx,dx,dz,dthx,dthy,dthz) and Pfold (platform old position) 
    % and returns updated position of platform Pfnew

    th = norm(dlta(4:6));                       % amount of rotation

    kBase = dlta(4:6)/th;                       % vector of direction cosines represented in base frame
    kx = kBase(1); ky = kBase(2); kz = kBase(3);   % direction cosines
    %%
    % kNew = Pfold(1:3,1:3)*kBase;                
    % kx = kNew(1); ky = kNew(2); kz = kNew(3);   % direction cosines
    %%

    c = cos(th); s = sin(th); v = 1-c;

    R = [kx^2*v+c      kx*ky*v-kz*s    kx*kz*v+ky*s   ;
    kx*ky*v+kz*s   ky^2*v+c        ky*kz*v-kx*s   ;
    kx*kz*v-ky*s   ky*kz*v+kx*s    kz^2*v+c    ];

    Rot = R*Pfold(1:3,1:3);

    Pfnew = [Rot,           Pfold(1:3,4)+dlta(1:3);
        zeros(1,3),     1];
    end
end