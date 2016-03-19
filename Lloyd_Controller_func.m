function [ C ] = Lloyd_Controller_func(Robots,ObstaclesPoint, SensedObjectsNum )
L=[0 0];
M=0;
robot=0;
Partition=1;%和障礙物之間的距離比例
for r=0:0.01:SensingR %
    for theta=0:0.01:2*pi
        i=round(1000*(Robots(1,1)+r*cos(theta)))+1;
        j=round(1000*(Robots(1,2)+r*sin(theta)))+1;
        q=[x(i) y(j)];
    if SensedObjectsNum>0
     for k=1: SensedObjectsNum  %感測範圍內的障礙物畫出Voronoi         
        [maxDistance,robot]=min([Partition*norm(q-Robots(1,:)),norm(q-ObstaclesPoint(k,:))]);
     end
    if robot==1 %畫分在機器人Voronoi區域的拿來計算Voronoi質心位置
     M=M+phi(i,j);
     L=L+phi(i,j)*q; 
    end
    else 
     M=M+phi(i,j);
     L=L+phi(i,j)*q;  
    end
    
    end    
end
C=L/M;
end

