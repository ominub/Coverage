L=[0 0];
M=0;
robot=0;
Partition=SensedObjectsNum;%�M��ê���������Z�����
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
     Robots=Robots+Kappa*(C-Robots)/norm(C-Robots);%�e�XVoronoi�᩹���Ӥ�V�e�iKappa
    

