function [contact_points, angles ] = get_data(data,in_curv_points,steps )

in_curv_points,
l1_1=data(1);
l1_2=data(2);
l1_3=data(3);
l2_1=data(4);
l2_2=data(5);
l2_3=data(6);
l3_1=data(7);
l3_2=data(8);
l3_3=data(9);
L1=data(10);
L2=data(11);
dm=data(12);
d=data(13);
rc=data(14);
x_c=data(15);
y_c=data(16);


thetaMod1=[];
thetaMod2=[];
thetaMod3=[];

thetalink1=[];
thetalink2=[];

pointsMod1=[];
pointsMod2=[];
pointsMod3=[];

rc_in=rc;
rc_out=rc_in+d;

%%%%% theta initialization for fsolve 
theta1=0;
theta2=0;
theta1_2=-5;
theta1_3=-5;

theta2_2=-5;
theta2_3=-5;

theta3_2=-5;
theta3_3=-5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:1:steps
   
    %%%%############ For module 1: in the inner curve of the pipe
    x1_1=in_curv_points(1,i); %%%% sub module 1 x point of contact with pipe
    y1_1=in_curv_points(2,i); %%%% sub module 1 y point of contact with pipe
    
    
    %%%%  For submodule 1_1
    theta1_1= pi-atan2((y1_1-y_c),(x_c-x1_1)); %%%% sub module 1 angle w.r.t horizontal
    theta1_1=wrapTo2Pi(theta1_1);
%     theta1_1*180/pi
    x1_1=x_c-rc_in * cos(pi-theta1_1); %%%% sub module 1 x point of contact with pipe
    y1_1=y_c+rc_in * sin(pi-theta1_1); %%%% sub module 1 y point of contact with pipe
    
    xc1_1=(x1_1*(rc+dm/2)-x_c*dm/2)/rc; %%%% sub module 1 center point x coordinate
    yc1_1=(y1_1*(rc+dm/2)-y_c*dm/2)/rc; %%%% sub module 1 center point y coordinate
   
    %%%%  For submodule 1_2
    theta1_1=theta1_1-pi/2;
    x0=theta1_2; 
    theta1_2 = fsolve(@(theta1_2)((xc1_1-(dm/2)*cos(theta1_1+theta1_2)-(dm/2)*cos(theta1_1)-x_c).^2+(yc1_1-(dm/2)*sin(theta1_1+theta1_2)-(dm/2)*sin(theta1_1)-y_c).^2-(rc+dm/2)^2),x0); %%%%% relative angle between 1 and 2 module
    theta1_2 = wrapTo2Pi(theta1_2);
    theta1_2*180/pi
    xc1_2=xc1_1-(dm/2)*cos(theta1_1+theta1_2)-(dm/2)*cos(theta1_1); %%%% sub module 2 x 
    yc1_2=yc1_1-(dm/2)*sin(theta1_1+theta1_2)-(dm/2)*sin(theta1_1); %%%% sub module 2 y 
    
    x1_2=(xc1_2*rc+x_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point x coordinate
    y1_2=(yc1_2*rc+y_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point y coordinate
       
     
    %%%%  For submodule 1_3
%     theta1_2=pi/2-theta1_2;
    
    x0=theta1_3; 
    theta1_3=fsolve(@(theta1_3)((xc1_2-(dm/2)*cos(theta1_1+theta1_2+theta1_3)-(dm/2)*cos(theta1_1+theta1_2)-x_c).^2+(yc1_2-(dm/2)*sin(theta1_1+theta1_2+theta1_3)-(dm/2)*sin(theta1_1+theta1_2)-y_c).^2-(rc+dm/2).^2),x0); %%%%% relative angle between 2 and 3 module
    theta1_3=wrapTo2Pi(theta1_3);
    
    xc1_3 = xc1_2-(dm/2)*cos(theta1_1+theta1_2+theta1_3)-(dm/2)*cos(theta1_1+theta1_2);
    yc1_3 = yc1_2-(dm/2)*sin(theta1_1+theta1_2+theta1_3)-(dm/2)*sin(theta1_1+theta1_2);   
    
    x1_3=(xc1_3*rc+x_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point x coordinate
    y1_3=(yc1_3*rc+y_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point y coordinate
     
     
    x1=x1_3-(l1_3/2)*cos(theta1_1+theta1_2+theta1_3)-dm/2*sin(theta1_1+theta1_2+theta1_3); %%%%% x coordinate of link connection to 1st module
    y1=y1_3-(l1_3/2)*sin(theta1_1+theta1_2+theta1_3)+dm/2*cos(theta1_1+theta1_2+theta1_3); %%%%% y coordinate of link connection to 1st module
   
    
    thetaMod1=[thetaMod1;theta1_1*180/pi ,(theta1_1+theta1_2)*180/pi ,(theta1_1+theta1_2+theta1_3)*180/pi]; %% angle of submodules w.r.t horizontal
    pointsMod1=[pointsMod1;x1_1,y1_1,x1_2,y1_2,x1_3,y1_3;];


    %%%%%############ For link 1
 
%  x0=0; with 0 as the initialization, it converges to any random point
   x0=theta1; 
   theta1 = fsolve(@(theta1)((x1+L1*cos(theta1)-x_c).^2+(y1+L1*sin(theta1)-y_c).^2-(rc_out-dm/2).^2),x0);
   theta1 = wrapTo2Pi(theta1);
   x21=x1+L1*cos(theta1);
   y21=y1+L1*sin(theta1);
   thetalink1=[thetalink1;(theta1-pi)*180/pi];

  
   %%%%%############ For module 2: in the outer curve of the pipe
 
   %%%  For submodule 2_1
   theta = pi-atan2((y21-y_c),(x_c-x21));
   theta = wrapTo2Pi(theta);
   
 
   alpha = asin((dm/2)/(rc_out-dm/2));
   alpha = wrapTo2Pi(alpha);
   
  theta2_1=theta+alpha;%%%% sub module 1 angle w.r.t horizontal
   
  
   x2_1=x_c-rc_out * cos(pi-theta2_1); %%%% sub module 1 x point of contact with pipe
   y2_1=y_c+rc_out * sin(pi-theta2_1); %%%% sub module 1 y point of contact with pipe
   
   xc2_1=(x2_1*(rc+d-dm/2)+x_c*dm/2)/(rc+d); %%%% sub module 3 center point x coordinate
   yc2_1=(y2_1*(rc+d-dm/2)+y_c*dm/2)/(rc+d); %%%% sub module 3 center point y coordinate
    

  
   %%%% For submodule 2_2
    theta2_1=theta2_1-pi/2;
    x0=theta2_2; 
    theta2_2 = fsolve(@(theta2_2)((xc2_1-(dm/2)*cos(theta2_1+theta2_2)-(dm/2)*cos(theta2_1)-x_c).^2+(yc2_1-(dm/2)*sin(theta2_1+theta2_2)-(dm/2)*sin(theta2_1)-y_c).^2-(rc_out-dm/2)^2),x0); %%%%% relative angle between 1 and 2 module
    theta2_2 = wrapTo2Pi(theta2_2);
    theta2_2*180/pi
    xc2_2=xc2_1-(dm/2)*cos(theta2_1+theta2_2)-(dm/2)*cos(theta2_1); %%%% sub module 2 x 
    yc2_2=yc2_1-(dm/2)*sin(theta2_1+theta2_2)-(dm/2)*sin(theta2_1); %%%% sub module 2 y 
    
    x2_2=(xc2_2*rc_out+x_c*dm/2)/(rc_out-dm/2); %%%% sub module 2 center point x coordinate
    y2_2=(yc2_2*rc_out+y_c*dm/2)/(rc_out-dm/2); %%%% sub module 2 center point y coordinate
    
   
    %%%%  For submodule 2_3
    x0=theta2_3; 
    theta2_3=fsolve(@(theta2_3)((xc2_2-(dm/2)*cos(theta2_1+theta2_2+theta2_3)-(dm/2)*cos(theta2_1+theta2_2)-x_c).^2+(yc2_2-(dm/2)*sin(theta2_1+theta2_2+theta2_3)-(dm/2)*sin(theta2_1+theta2_2)-y_c).^2-(rc_out-dm/2).^2),x0); %%%%% relative angle between 2 and 3 module
    theta2_3=wrapTo2Pi(theta1_3);
    
    xc2_3 = xc2_2-(dm/2)*cos(theta2_1+theta2_2+theta2_3)-(dm/2)*cos(theta2_1+theta2_2);
    yc2_3 = yc2_2-(dm/2)*sin(theta2_1+theta2_2+theta2_3)-(dm/2)*sin(theta2_1+theta2_2);   
    
    x2_3=(xc2_3*rc_out+x_c*dm/2)/(rc_out-dm/2); %%%% sub module 2 center point x coordinate
    y2_3=(yc2_3*rc_out+y_c*dm/2)/(rc_out-dm/2); %%%% sub module 2 center point y coordinate
     
    theta = pi-atan2((y2_3-y_c),(x_c-x2_3));
    theta = wrapTo2Pi(theta);
    alpha = asin((dm/2)/(rc_out-dm/2));
    alpha = wrapTo2Pi(alpha);
    theta=theta+alpha;%%%% sub module 1 angle w.r.t horizontal
   
  
   x22=x_c-(rc_out-dm/2) * cos(pi-theta); %%%% sub module 1 x point of contact with pipe
   y22=y_c+(rc_out-dm/2) * sin(pi-theta); %%%% sub module 1 y point of contact with pipe
               
          
   thetaMod2=[thetaMod2; (theta2_1+pi/2)*180/pi ,(theta2_1+theta2_2+pi/2)*180/pi ,(theta2_1+theta2_2+theta2_3+pi/2)*180/pi];
   pointsMod2=[pointsMod2;x2_1,y2_1,x2_2,y2_2,x2_3,y2_3;];

   %%%%%############ For link 2
   %  x0=0; with 0 as the initialization, it converges to any random point
   x0=theta2; 
   theta2 = fsolve(@(theta2)((x22+L1*cos(theta2)-x_c).^2+(y22+L1*sin(theta2)-y_c).^2-(rc+dm/2).^2),x0);
   theta2 = wrapTo2Pi(theta2);
   x31=x22+L2*cos(theta2);
   y31=y22+L2*sin(theta2);
  
   thetalink2=[thetalink2;(theta2+pi)*180/pi];
         
    
   %%%%############ For module 3: in the inner curve of the pipe
   
%   theta = atan2((y31-x_c),(x31-x_c));
    theta= pi-atan2((y31-y_c),(x_c-x31)); %%%% sub module 1 angle w.r.t horizontal
    theta = wrapTo2Pi(theta);
    alpha = asin((dm/2)/(rc+dm/2));
    alpha = wrapTo2Pi(alpha);
    theta3_1=theta+alpha;%%%% sub module 1 angle w.r.t horizontal
    
    x3_1=x_c+rc_in * cos(theta3_1); %%%% sub module 1 x point of contact with pipe
    y3_1=y_c+rc_in * sin(theta3_1); %%%% sub module 1 y point of contact with pipe
           
   %%%%  For submodule 31
    theta3_1= pi-atan2((y3_1-y_c),(x_c-x3_1)); %%%% sub module 1 angle w.r.t horizontal
    theta3_1=wrapTo2Pi(theta3_1);
  
 
%     x3_1=x_c-rc_in * cos(pi-theta3_1); %%%% sub module 1 x point of contact with pipe
%     y3_1=y_c+rc_in * sin(pi-theta3_1); %%%% sub module 1 y point of contact with pipe
%     
    xc3_1=(x3_1*(rc+dm/2)-x_c*dm/2)/rc; %%%% sub module 1 center point x coordinate
    yc3_1=(y3_1*(rc+dm/2)-y_c*dm/2)/rc; %%%% sub module 1 center point y coordinate
   
   
   %%%%  For submodule 3_2
    theta3_1=theta3_1-pi/2;
    x0=theta3_2; 
    theta3_2 = fsolve(@(theta3_2)((xc3_1-(dm/2)*cos(theta3_1+theta3_2)-(dm/2)*cos(theta3_1)-x_c).^2+(yc3_1-(dm/2)*sin(theta3_1+theta3_2)-(dm/2)*sin(theta3_1)-y_c).^2-(rc+dm/2)^2),x0); %%%%% relative angle between 1 and 2 module
    theta3_2 = wrapTo2Pi(theta3_2);
    theta3_2*180/pi
    xc3_2=xc3_1-(dm/2)*cos(theta3_1+theta3_2)-(dm/2)*cos(theta3_1); %%%% sub module 2 x 
    yc3_2=yc3_1-(dm/2)*sin(theta3_1+theta3_2)-(dm/2)*sin(theta3_1); %%%% sub module 2 y 
    
    x3_2=(xc3_2*rc+x_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point x coordinate
    y3_2=(yc3_2*rc+y_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point y coordinate
     
    %%%%  For submodule 3_3
    x0=theta3_3; 
    theta3_3=fsolve(@(theta3_3)((xc3_2-(dm/2)*cos(theta3_1+theta3_2+theta3_3)-(dm/2)*cos(theta3_1+theta3_2)-x_c).^2+(yc3_2-(dm/2)*sin(theta3_1+theta3_2+theta3_3)-(dm/2)*sin(theta3_1+theta3_2)-y_c).^2-(rc+dm/2).^2),x0); %%%%% relative angle between 2 and 3 module
    theta3_3=wrapTo2Pi(theta3_3);
    
    xc3_3 = xc3_2-(dm/2)*cos(theta3_1+theta3_2+theta3_3)-(dm/2)*cos(theta3_1+theta3_2);
    yc3_3 = yc3_2-(dm/2)*sin(theta3_1+theta3_2+theta3_3)-(dm/2)*sin(theta3_1+theta3_2);   
    
    x3_3=(xc3_3*rc+x_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point x coordinate
    y3_3=(yc3_3*rc+y_c*dm/2)/(rc+dm/2); %%%% sub module 2 center point y coordinate
     
    
    thetaMod3=[thetaMod3;(theta3_1)*180/pi ,(theta3_1+theta3_2)*180/pi ,(theta3_1+theta3_2+theta3_3)*180/pi];
    pointsMod3=[pointsMod3;x3_1,y3_1,x3_2,y3_2,x3_3,y3_3;];   
      
   %%%%%####################### GUI
   link1_x=[x1,x21];
   link1_y=[y1,y21];
   link2_x=[x22,x31];
   link2_y=[y22,y31];
   
   
   centres=[xc1_1,xc1_2,xc1_3,xc2_1,xc2_2,xc2_3,xc3_1,xc3_2,xc3_3;
            yc1_1,yc1_2,yc1_3,yc2_1,yc2_2,yc2_3,yc3_1,yc3_2,yc3_3];   %%%%%% centres of each module        
      
   h2 = draw_modules(centres,dm );
   
 
end

contact_points=[pointsMod2,pointsMod2,pointsMod2];
angles=[thetaMod1,thetaMod2,thetaMod3,thetalink1,thetalink2];

end

