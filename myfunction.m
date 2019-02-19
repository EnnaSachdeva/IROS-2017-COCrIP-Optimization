function [ F] = myfunction(theta1,phi1,x_c,y_c,rc,d,x1,y1)
 F(1) = x_c-(rc+d)*cos(pi/2+theta1+phi1)-x1+sqrt((d^2+l2_1^2)/4)*cos(theta1+phi1-pi/4);
 F(2) = y_c+(rc+d)*sin(pi/2+theta1+phi1)-y1+sqrt((d^2+l2_1^2)/4)*sin(theta1+phi1-pi/4);  

end

