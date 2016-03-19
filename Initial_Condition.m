clear
clc
x=0:0.001:1;
y=0:0.001:1;
X=zeros(length(x),length(y));
Y=zeros(length(x),length(y));
p_en=[0.2 0.4];
phi_en=zeros(length(x),length(y));
phi=zeros(length(x),length(y));
% M=zeros(5,1);
% L=zeros(5,2);
% C=zeros(5,2);
%% set robot initial condition
Robot1_position=[0.5 0.8];
Robot2_position=[0.6 0.4];
R_p1=0.1;
Robots=Robot1_position;
SensingR=R_p1;
phi_robot=zeros(length(x),length(y));
Kappa=1;%vehicle velocity
%% set environment static obstacle

Obstacle1_position=[0.3 0.55];
Obstacle2_position=[0.4 0.7];
Obstacle3_position=[0.3 0.5];
Obstacle4_position=[0.45 0.5];
ObstaclesPoint=[Obstacle1_position;Obstacle2_position;Obstacle3_position;Obstacle4_position];

