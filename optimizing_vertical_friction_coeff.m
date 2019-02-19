close all;
clear all;
clc;
%% Design parameters (All Units are SI) ###########

m_m1 = 0.160; %module_1 mass
m_m2 = 0.180; %module_2 mass
m_m3 = 0.160; %module_3 mass

m_l1 = 0.020; %link_1 mass
m_l2 = 0.020; %link_2 mass

g = 9.8; %acceleration due to gravity

w_m1 = m_m1*g; %module_1 mass
w_m2 = m_m2*g; %module_1 mass
w_m3 = m_m3*g; %module_1 mass

w_l1 = m_l1*g; %link_1 mass
w_l2 = m_l2*g; %link_2 mass

mu = 1; %coefficient of friction

l_m=0.120; % length of module
l_p=0.024; % length of rolling pololu motor

l1 = l_m+l_p ; %length of module+pololu
l2 = l_m+(2*l_p); %length of module+pololu
l3 = l_m+l_p ; %length of module+pololu

L1 = 0.060 ; %length of link1
L2 = 0.060 ; %length of link2

d = 0.050; % diameter of the module

theta=[114.6 65.4];  % theta in degrees

%%%% convert theta into radians
theta_1= theta(1)*pi/180;
theta_2= theta(2)*pi/180;

theta1=pi-theta_1;
theta2=theta_2;

%%%%%%%%%%%%%%%% input points on the pipe curvature
d=0.075;  %%%%% 3 inch diameter of the pipe
rc=0.127; %%%%% radius of the pipe 
x_c=0;    %%%%% center x coordinate of the pipe 
y_c=0;    %%%%% center x coordinate of the pipe 
theta_pipe=360; %%%%% in degrees
steps=360; %%%%% steps in which the points on the curve are distributed
% [out_curv, in_curv] = pipe_bend(d,rc,x_c,y_c,theta_pipe,steps )

%  

 tau1=0.1897;
 tau2=0.2872;
 tau3=0.1948;
 tau4=0.0846;

%  tau1= 0.2359;
%  tau2= 0.3683;
%  tau3=0.2760;
%  tau4=0.1310;
% 

 eps=0;
 
 %######################## Using free body diagram of individual parts ##############################################

%%%% Formulating Ax=b denotes the static equilibrium conditions in a paritcular configuration
%% matrix A
%%%%% F1      F2      F3       N1      N2      N3     
Aeq=[0  ,     0   ,   0   ,    1   ,   -1   ,   1 ;...
     1    ,   1   ,   1   ,    0   ,    0   ,   0 ;...
    d/2  ,   0   ,   0   ,   l1/2 ,    0   ,   0   ;...
    L1*cos(theta1)   ,   0    ,   0   ,   L1*sin(theta1)  ,    0    ,   0  ;...
     0    ,  -d/2  ,   0   ,   l2   ,  -l2/2   ,   0    ;...
    -L2*cos(theta2)     ,   -L2*cos(theta2)  ,  0   ,   L2*sin(theta2)   ,  -L2*sin(theta2)  ,  0 ;...
     0                  ,    0              ,   d/2 ,       0           ,     0            ,  -l3/2 ];
   
beq=[0 ; w_m1+w_m2+w_m3+w_l1+w_l2 ; tau1 ; w_m1*L1*cos(theta1)+w_l1*(L1/2)*cos(theta1)-tau1+tau2 ; -tau2+tau3 ; -(w_m2+w_m1+w_l1)*L2*cos(theta2)-w_l2*(L2/2)*cos(theta2)-tau3+tau4  ; -tau4  ];

%%%%###########################################################################################################################

%%% Formulating Ax<=b denote the F<=mu*N 
%% matrix A
%%% F1      F2      F3       N1      N2      N3    

% A=[ 1    ,   0   ,   0   ,   0    ,   0    ,   0   ;
%      0    ,   1   ,   0   ,   0    ,   0    ,   0   ;
%      0    ,   0   ,   1   ,   0    ,   0    ,   0   ];
% b=[ 4*0.07/d; 4*0.07/d; 4*0.07/d];


A=[  1    ,   0   ,   0   ,  -mu   ,   0    ,   0   ;
     0    ,   1   ,   0   ,   0    ,  -mu  ,    0   ;
     0    ,   0   ,   1   ,   0    ,   0    ,  -mu  ;
     1    ,   0   ,   0   ,   0    ,   0    ,   0   ;
     0    ,   1   ,   0   ,   0    ,   0    ,   0   ;
     0    ,   0   ,   1   ,   0    ,   0    ,   0   ];

b=[eps ; eps ; eps; 4*0.07/d; 4*0.07/d; 4*0.07/d];

% % 
% A=[  1    ,   0   ,   0   ,  -mu   ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
%      0    ,   1   ,   0   ,   0    ,   -mu  ,   0    ,  0  ,   0   ,   0   ,   0;
%      0    ,   0   ,   1   ,   0    ,   0    ,  -mu   ,  0  ,   0   ,   0   ,   0];
% 
% b=[0 ; 0 ; 0];
% 
%  
%  
%  
% x=fsolve(fun,x0)

%  opts = optimoptions(@lsqnonlin,'Display','iter','Algorithm','levenberg-marquardt', 'MaxIterations',100000,'MaxFunctionEvaluations',100000);
% opts = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-40);
opts = optimset('Display','iter','Algorithm','interior-point');
% opts = optimset('Display','iter','Algorithm','interior-point', 'MaxIter',Inf);%, 'MaxFunEvals', 10000);

%  opts= optimoptions(@fmincon,'Algorithm','sqp','Display','iter-detailed');   %%% searches for global minima
% opts = optimset('Algorithm','sqp','MaxIter',5000,'MaxFunEvals',100000,'Display','iter');


%%%%%% optimization technique
lb = [ 0.01, 0.01, 0.01, 0.01, 0.01, 0.01];  %LOWER BOUNDS
% lb = [ 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0];  %LOWER BOUNDS
% lb = [];%LOWER BOUNDS
ub = [5,5,5,5,5,5]; %UPPER BOUNDS
% ub = [10,10,10,10,10,10,10,10,10,10]; %UPPER BOUNDS
% ub = [15,15,15,15,15,15,15,15,15,15]; %UPPER BOUNDS
% % 
% lb = [];
% ub = [inf, inf, inf, inf, inf, inf , inf, inf, inf, inf];

%  x0= [0.1  0.1  0.1  0.1  0.1  0.1]'; % [F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [0 0 0 0 0 0]'; %[F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [1.6128, 1.1723, 0.4405, 2.3446, 1.4637, 0.0001]' %% considering F=uN equality  
%  x0= [0.1085 0.1033 0.0947 0.1171 0.1067 0.0895]';
%  x0= [0.3655  0.2938  0.1595  0.2667 0.1745 0.0001]';
%  x0= [1 1 1 1 1 1 ]';
% x0=[1.3555 2.6460 1.2905 1.6944 3.3075 1.6131]';
x0=[1.9825 2.7136 1.9843 1.8174 3.2806 1.8264 ]';
ceq=[];
c=[];
% 
% x=zeros(10,1);
% A=[A;Aeq];
% b=[b;beq];
% 
% 
% x=fsolve(@(x)double(A)*x-double(b),x0);

%%%%%%%%%%%%%%%%%%%%%%% using CVX optimization
% n=size(Aeq,2);
% cvx_begin
%     variable x(n);
% minimize(norm(x(1),2)+norm(x(2),2)+norm(x(3),2));
% %   minimize(-abs(x(7))-abs(x(8))-abs(x(9))-abs(x(10)));
%     subject to
%     Aeq*x == beq;
%     A*x <= b;
% %     x(1)==x(3);
% %     x(4)==x(6);
% 
%     cvx_end
% % 
% 
% x = fmincon(@(x)(-norm(x(4),2)-norm(x(5),2)-norm(x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% maximizing Normal forces
% x = fmincon(@(x)(norm(x(7),1)+norm(x(8),1)+norm(x(9),1)+norm(x(10),1)),x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts); %%%%% minimizing moments 
% x = fmincon(@(x)(norm(x(1),2)+norm(x(2),2)+norm(x(3),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction forces;
x = fmincon(@(x)(norm(x(1)/x(4),2)+norm(x(2)/x(5),2)+norm(x(3)/x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction/normal force ratio;
% x = fmincon(@(x)(x(7)+x(8)+x(9)+x(10)) ,x0,A,b,Aeq,beq,lb,ub)
%%%%%%%%%%%%%% to check which constraint is not being satisfied

ang1=90-(theta1*180/pi);
ang2=theta2*180/pi;

x

%% ratio of F/N
x(1)/x(4)
x(2)/x(5)
x(3)/x(6)
% 
% spring_moments=[x(7);x(8);x(9);x(10)]
% spring_constants=[x(7)/ang1;x(8)/ang2;x(9)/ang2;x(10)/ang1]

%%%%%%%%%%%%%% to check which constraint is not being satisfied


% x_new=inv([Aeq;A])*([beq;b])

 
