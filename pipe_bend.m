function [ out_curv_points, in_curv_points,mid_in_curv_points,mid_out_curv_points] = pipe_bend(d,dm,rc,x_c,y_c,theta,steps )

%% All Units are SI ###########

% d: diameter of the pipe
% rc: center coordinates and radius of curvature
% x_c: x coordinate of centre;
% y_c: y coordinate of centre;
% theta : angle in degrees
% steps: steps in which the points on the curve are distributed

theta=theta*pi/180; %%%%% angle in radians

th = 0:theta/steps:theta;
th = wrapTo2Pi(th);

rc_in=rc;
rc_out=rc_in+d;

rc_mid_in=rc_in+dm/2;
rc_mid_out=rc_out-dm/2;

x_in= x_c-rc_in * cos(th) ;
y_in= y_c+rc_in * sin(th) ;

x_out= x_c-rc_out * cos(th) ;
y_out= y_c +rc_out * sin(th) ;

x_mid_in= x_c-rc_mid_in * cos(th) ;
y_mid_in= y_c +rc_mid_in * sin(th) ;


x_mid_out= x_c-rc_mid_out * cos(th) ;
y_mid_out= y_c +rc_mid_out * sin(th) ;


out_curv_points=[x_out;y_out];
in_curv_points=[x_in;y_in];
mid_in_curv_points=[x_mid_in;y_mid_in];
mid_out_curv_points=[x_mid_out;y_mid_out]; 




end

