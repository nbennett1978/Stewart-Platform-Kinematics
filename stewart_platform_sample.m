% script calculating 6-6 steward platform using fk_stewart_6_6
% 
clear
clc
%% leg joint positions
% radius of base ring:
r1 = 150;
% radius of end effector ring:
r2 = 150;
% number of holes in base ring:
n1 = 12;
% number of holes in end effector ring:
n2 = 12;
% holes, in which joints are mounted in base:
kp = [1,2,5,6,9,10];
% holes, in which joints are mounted in end effector:
kb = [1,4,5,8,9,12];
% correct indices to start from zero
kp = kp-1;
kb = kb-1;
% calculate positions with respect to their own frames:
for k=1:6
    p(:,k) = [r1*cos(2/n1*pi*kp(k));r1*sin(2/n1*pi*kp(k));0];
    b(:,k) = [r2*cos(2/n2*pi*kb(k));r2*sin(2/n2*pi*kb(k));0];
end;  
clear kp kb n1 n2 k r1 r2
%% leg lengths
InLegLn = [167.02;167.03;190.61;194.32;170.91;126];


%% calc
[GuessPlatPose, ThLegLn] = fk_stewart_6_6(b,p,InLegLn);



%%
disp(['absolute distance of effector frame to base frame: ',...
    num2str(sqrt(sum(GuessPlatPose(1:3,4).^2)))])

GuessPlatPose * [0;150;0;1]
%%
% whos
