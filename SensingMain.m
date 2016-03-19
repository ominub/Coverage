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

% set robot initial condition
Robot1_position=[0.5 0.8];
Robot2_position=[0.6 0.4];
R_p1=0.1;
Robots=Robot1_position;
SensingR=R_p1;
phi_robot=zeros(length(x),length(y));
Kappa=2;%vehicle velocity

% set environment static obstacle

Obstacle1_position=[0.3 0.55];
Obstacle2_position=[0.4 0.7];
Obstacle3_position=[0.3 0.5];
Obstacle4_position=[0.45 0.5];
ObstaclesPoint=[Obstacle1_position;Obstacle2_position;Obstacle3_position;Obstacle4_position];


% Density Function Design

for i=1:length(x)
    for j=1:length(y)
         X(i,j)=(i-1)*0.001;
         Y(i,j)=(j-1)*0.001;
         q=[x(i) y(j)];
         phi_en(i,j)=(exp(-norm(q-p_en)^2/(2*0.12^2))+0.01);
         if norm(q-Robots)<0.5*SensingR && norm(q-Robots)>0.25*SensingR
               phi_robot(i,j)=1-8/SensingR*abs(norm(q-Robots)-3/8*SensingR);
         else 
              phi_robot(i,j)=0;
         end
         phi(i,j)=phi_en(i,j)*phi_robot(i,j);
        
     end
end
% phi=phi/phi_robot(round(Robots(1)*1000+1),round(Robots(2)*1000+1));

% Voronoi Diagram

t = 0:0.1:2*pi;
X_hm =Robots(1,1)+SensingR(1)*cos(t);
Y_hm =Robots(1,2)+SensingR(1)*sin(t);


plot(Robots(:,1),Robots(:,2),'c*');hold on

plot(ObstaclesPoint(:,1),ObstaclesPoint(:,2),'ro');hold on
plot(X_hm,Y_hm,'c');hold on
plot(p_en(1),p_en(2),'bx');
axis([0 1 0 1])

% show density function
% figure
% surf(X,Y,phi,'LineStyle','none');
% title('f_{en}')
%  axis([0 1 0 1])
%  view([0 0 1])

%% Controller

for xs=1:1:120
%% �M�w�n�p�⪺����

SensedObjects=10*ones(length(ObstaclesPoint),2);
SensedObjectsNum=0;
for i=1:1:length(ObstaclesPoint)
if norm(ObstaclesPoint(i,:)-Robots(1,:))<SensingR %�O�_���P�����ê��
   SensedObjectsNum=SensedObjectsNum+1;
   SensedObjects(SensedObjectsNum,:)= ObstaclesPoint(i,:);%�⦳�P���쪺��ê���O���U��
end
end
% Lloyd_Controller_func(SensedObjects)
if SensedObjectsNum>0%�Y���P�����ê���h�eVoronoi
%% Lloyd
L=[0 0];
M=0;
robot=0;
Partition=1;%�M��ê���������Z�����
for r=0:0.01:SensingR %
    for theta=0:0.01:2*pi
        i=round(1000*(Robots(1,1)+r*cos(theta)))+1;
        j=round(1000*(Robots(1,2)+r*sin(theta)))+1;
        q=[x(i) y(j)];
     for k=1: SensedObjectsNum  %�P���d�򤺪���ê���e�XVoronoi         
        [maxDistance,robot]=min([norm(q-Robots(1,:)),Partition*norm(q-SensedObjects(k,:))]);
     end
    if robot==1 %�e���b�����HVoronoi�ϰ쪺���ӭp��Voronoi��ߦ�m
     M=M+phi(i,j);
     L=L+phi(i,j)*q; 
    end
    end    
end
C=L/M;
     Robots=Robots+Kappa*(C-Robots);%�e�XVoronoi�᩹���Ӥ�V�e�iKappa
  

else
L=[0 0];
M=0;
robot=0;
Partition=1;%�M��ê���������Z�����
for r=0:0.01:SensingR %
    for theta=0:0.01:2*pi
        i=round(1000*(Robots(1,1)+r*cos(theta)))+1;
        j=round(1000*(Robots(1,2)+r*sin(theta)))+1;
        q=[x(i) y(j)];
       M=M+phi(i,j);
     L=L+phi(i,j)*q; 
    end    
end
C=L/M;
     Robots=Robots+Kappa*(C-Robots);%�e�XVoronoi�᩹���Ӥ�V�e�iKappa
%   Robots=Robots+Kappa*(p_en-Robots)/norm(p_en-Robots);%�S�P����N�������ؼЦ�m����
%   C=Robots;
end
%% Patrol Position Update
% ObstaclesPoint(4,1)=ObstaclesPoint(4,1)-0.01*(heaviside(xs)-2*heaviside(xs-10)+2*heaviside(xs-20)-2*heaviside(xs-30)+2*heaviside(xs-40));

%% Density Function Redesign

for i=1:length(x)
    for j=1:length(y)
         X(i,j)=(i-1)*0.001;
         Y(i,j)=(j-1)*0.001;
         q=[x(i) y(j)];
         if norm(q-Robots)<0.5*SensingR && norm(q-Robots)>0.25*SensingR
              phi_robot(i,j)=1-8/SensingR*abs(norm(q-Robots)-3/8*SensingR);
         else 
              phi_robot(i,j)=0;
         end
          phi(i,j)=phi_en(i,j)*phi_robot(i,j);
%          if phi(i,j)<0.012%%%%%%%%%%%%%
%         phi(i,j)=0;%%%%%%%%%%%%%%%%%%%%%
%          end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     end
end
% phi=phi/phi_robot(round(Robots(1)*1000+1),round(Robots(2)*1000+1));

%Threshold
for i=1:length(x)
    for j=1:length(y)
       if phi(i,j)<phi(round(Robots(1)*1000+1),round(Robots(2)*1000+1))%%%%%%%%%%%%%%%%
        phi(i,j)=0;%%%%%%%%%%%%%%%%%%%%%
       end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end


%% plot
figure
t = 0:0.1:2*pi;
%Voronoi��ߦ�m
plot(C(:,1),C(:,2),'r*');hold on
%�����H���ߦ�m
plot(Robots(:,1),Robots(:,2),'c*');hold on


% %�P���b�|
X_hm =Robots(1,1)+SensingR(1)*cos(t);
Y_hm =Robots(1,2)+SensingR(1)*sin(t);
plot(X_hm,Y_hm,'c');hold on
%��ê����m
plot(ObstaclesPoint(:,1),ObstaclesPoint(:,2),'ko');hold on
plot(p_en(1),p_en(2),'bx');
axis([0 1 0 1])


%show density function
% figure
% surf(X,Y,phi,'LineStyle','none');
% title('f_{en}')
%  axis([0 1 0 1])
%  view([0 0 1])

%% Save Pic
picname=sprintf('%d.jpg',xs);
saveas(gcf,picname);
 end 