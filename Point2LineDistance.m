function d = Point2LineDistance( P,A,B,C )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%Line function Ax+By+C=0
%Point
x=P(1,1);
y=P(1,2);
d=abs(A*x+B*y+C)/sqrt(A^2+B^2);

end

