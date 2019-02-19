close all;
clear all;
clc;

theta=2*pi;
th = 0:theta/100:theta;
% th = wrapTo2Pi(th);

rc=2;
figure

x_c=0;
y_c=0;

  xtheta=[-0.0945   -0.1202;
           -0.0982   -0.1172;
           -0.1018   -0.1140;
           -0.1054   -0.1108;
           -0.1088   -0.1074;
           -0.1121   -0.1040;
           -0.1153   -0.1004;
           -0.1184   -0.0967;
           -0.1214   -0.0929;
           -0.1243   -0.0891;
           -0.1270   -0.0851;
           -0.1296   -0.0811;
           -0.1321   -0.0770;
           -0.1344   -0.0728;
           -0.1367   -0.0685;
           -0.1388   -0.0642;
           -0.1407   -0.0598;
           -0.1425   -0.0554;
           -0.1442   -0.0509;
           -0.1457   -0.0463;
           -0.1471   -0.0417;
           -0.1483   -0.0371;
           -0.1494   -0.0324;
           -0.1504   -0.0277;
           -0.1512   -0.0230;
           -0.1518   -0.0182;
           -0.1523   -0.0134;
           -0.1527   -0.0086;
           -0.1528   -0.0038;
           -0.1529    0.0010;
           -0.1528    0.0058;
           -0.1525    0.0106;
           -0.1521    0.0154;
           -0.1516    0.0201;
           -0.1509    0.0249];

d=0.075;  %%%%% 3 inch diameter of the pipe
rc=0.127; %%%%% radius of the pipe 
x_c=0;    %%%%% center x coordinate of the pipe 
y_c=0;    %%%%% center x coordinate of the pipe 
dm=0.0480;
L1 = 0.050 ;

rc_out=rc+d;

 x= x_c-rc * cos(th) ;
 y= y_c+rc * sin(th) ;
 h= plot(x,y,'r');
thetafmin=[];
thetafsolve=[];
valfmin=[];
valfsolv=[];

 hold on
    
 for i=1:1:size(xtheta,1)
     x0=-1;
     x1=xtheta(i,1);
     y1=xtheta(i,2);

   a= 2*(x1-x_c)*L1;
   b= 2*(y1-y_c)*L1;
   c= d^2-2*d*dm-L1^2;
   
%    theta1=acos(2*a*c+sqrt(4*(a^2)*(c^2)-4*(a^2+b^2)*(c^2-b))/2*(a^2+b^2));
%    
%    theta1 = wrapTo2Pi(theta1);
%     [theta1,fval] = fminunc(@(theta1)(a*cos(theta1)+b*sin(theta1)-c),x0);
    [theta1,fminval] = fmincon(@(theta1)((x1-L1*cos(theta1)-x_c).^2+(y1-L1*sin(theta1)-y_c).^2-(rc_out-dm/2).^2),x0);
    [theta2,fsolval] = fsolve(@(theta1)((x1-L1*cos(theta1)-x_c).^2+(y1-L1*sin(theta1)-y_c).^2-(rc_out-dm/2).^2),x0);
    
    
    x21=x1+L1*cos(theta2);
    y21=y1+L1*sin(theta2);
    h=plot(x1,y1,'r*');
    h=plot(x21,y21,'b*');
    hold on
    pause (0.1)
    
    theta1=wrapTo2Pi(theta2)*180/pi;
    theta2=wrapTo2Pi(theta2)*180/pi;
    
    
    valfmin=[valfmin;fminval];
    valfsolv=[valfsolv;fsolval];
    
    thetafmin=[thetafmin;theta1];
    thetafsolve=[thetafsolve;theta2];
    
 end
 
 [xtheta,thetafmin,thetafsolve,thetafmin-thetafsolve,valfmin,valfsolv]
 