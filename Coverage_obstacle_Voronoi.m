clear
clc
x=0:0.001:1;
y=0:0.001:1;
X=zeros(length(x),length(y));
Y=zeros(length(x),length(y));
phi_en=zeros(length(x),length(y));
phi=zeros(length(x),length(y));
phi_ind=zeros(length(x),length(y));



%% set initial condition
%robot
r1=[0.5 0.8];
r2=[0.7 0.5];
r3=[0.6 0.6];
r4=[0.4 0.8];
r5=[0.4 0.5];
R=[r1;r2;r3;r4;r5];
%goal
g1=[0.2 0.5];

M=zeros(length(R),1);
L=zeros(length(R),2);
C=zeros(length(R),2);
%% set environment static obstacle

XCorridor=0:0.1:0.4;
XCorridor=XCorridor';
YCorridor1=0.6*ones(length(XCorridor),1);
YCorridor2=0.4*ones(length(XCorridor),1);


P=[R;[XCorridor,YCorridor1];[XCorridor,YCorridor2]];
Q=zeros(length(P),1);
%% Density Function Design

for i=1:length(x)
    for j=1:length(y)
         X(i,j)=(i-1)*0.001;
         Y(i,j)=(j-1)*0.001;
         q=[x(i) y(j)];
         for k=1:length(P)
         Q(k)=norm(P(k,:)-q);
         end
         phi_en(i,j)=(exp(-norm(q-g1)^2/(2*0.12^2)));
         phi(i,j)=phi_en(i,j);
                 
         [maxDistance,robot]=min(Q);
         if robot<length(R)+1
         M(robot)=M(robot)+phi(i,j);
         L(robot,:)=L(robot,:)+phi(i,j)*q;
         end
     end
end

for i=1:1:length(R)
    if M(i)~=0
    C(i,:)=L(i,:)/M(i);
    end
end

%% Voronoi Diagram


[vx ,vy]= voronoi(P(:,1),P(:,2));
plot(vx,vy);hold on
plot(R(:,1),R(:,2),'*');hold on
plot(C(:,1),C(:,2),'o');hold on
plot(XCorridor,YCorridor1); hold on;
plot(XCorridor,YCorridor2); hold on;
axis([0 1 0 1])

%% show density function
% figure(2)
% surf(X,Y,phi,'LineStyle','none');
% title('f_{en}')
%  axis([0 1 0 1])
%  view([0 0 1])
%  hold on;


%% Controller Design
count=1;
while(norm(R-[g1;g1;g1;g1;g1]) > 0.2)
    
plot(C(:,1),C(:,2),'o');hold on%畫出經過點

R=C;
P(1:5,:)=C;
M=zeros(length(R),1);
L=zeros(length(R),2);
C=zeros(length(R),2);
X=zeros(length(x),length(y));
Y=zeros(length(x),length(y));

for i=1:length(x)
    for j=1:length(y)
         X(i,j)=(i-1)*0.001;
         Y(i,j)=(j-1)*0.001;
         q=[x(i) y(j)];
         for k=1:length(P)
         Q(k)=norm(P(k,:)-q);
         end          
         [maxDistance,robot]=min(Q);
         if robot<length(R)+1
         M(robot)=M(robot)+phi(i,j);
         L(robot,:)=L(robot,:)+phi(i,j)*q;
         end
     end
end

for i=1:1:length(R)
    if M(i)~=0
    C(i,:)=L(i,:)/M(i);
    end
end
count=count+1;
end 
plot(C(:,1),C(:,2),'x');hold on%畫出終點

