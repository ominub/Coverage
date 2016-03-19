Initial_Condition;
Density_Function_Design;
for xs=1:1:100
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

Lloyd_Controller;

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
     Robots=Robots+Kappa*(C-Robots)/norm(C-Robots);%�e�XVoronoi�᩹���Ӥ�V�e�iKappa
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

         phi_robot(i,j)=exp(-(norm(q-Robots)-SensingR)^4/(2*0.02^4));
          phi(i,j)=phi_en(i,j)*phi_robot(i,j);
%          if phi(i,j)<0.012%%%%%%%%%%%%%
%         phi(i,j)=0;%%%%%%%%%%%%%%%%%%%%%
%          end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     end
end
phi=phi/phi_robot(round(Robots(1)*1000+1),round(Robots(2)*1000+1));

for i=1:length(x)
    for j=1:length(y)
       if phi(i,j)<phi(round(Robots(1)*1000+1),round(Robots(2)*1000+1))%%%%%%%%%%%%%%%%
        phi(i,j)=0;%%%%%%%%%%%%%%%%%%%%%
       end%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end


%% plot
% figure
% t = 0:0.1:2*pi;
% %Voronoi��ߦ�m
% plot(C(:,1),C(:,2),'r*');hold on
% %�����H���ߦ�m
% plot(Robots(:,1),Robots(:,2),'c*');hold on
% 
% 
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