%% Density Function Design

for i=1:length(x)
    for j=1:length(y)
         X(i,j)=(i-1)*0.001;
         Y(i,j)=(j-1)*0.001;
         q=[x(i) y(j)];
         phi_en(i,j)=(exp(-norm(q-p_en)^2/(2*0.12^2))+0.01);
         phi_robot(i,j)=exp(-(norm(q-Robots)-SensingR/2)^4/(2*0.02^4));
         phi(i,j)=phi_en(i,j)*phi_robot(i,j);
        
     end
end
phi=phi/phi_robot(round(Robots(1)*1000+1),round(Robots(2)*1000+1));

%% Voronoi Diagram

t = 0:0.1:2*pi;
X_hm =Robots(1,1)+SensingR(1)*cos(t);
Y_hm =Robots(1,2)+SensingR(1)*sin(t);


plot(Robots(:,1),Robots(:,2),'c*');hold on

plot(ObstaclesPoint(:,1),ObstaclesPoint(:,2),'ro');hold on
plot(X_hm,Y_hm,'c');hold on
plot(p_en(1),p_en(2),'bx');
axis([0 1 0 1])

%% show density function
% figure
% surf(X,Y,phi,'LineStyle','none');
% title('f_{en}')
%  axis([0 1 0 1])
%  view([0 0 1])
