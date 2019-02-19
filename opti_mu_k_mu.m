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

mu_start = 0.55; %coefficient of friction

mu=mu_start;
l_m=0.120; % length of module
l_p=0.024; % length of rolling pololu motor

l1 = l_m+l_p ; %length of module+pololu
l2 = l_m+(2*l_p); %length of module+pololu
l3 = l_m+l_p ; %length of module+pololu

L1 = 0.060 ; %length of link1
L2 = 0.060 ; %length of link2

d = 0.050; % diameter of the module

theta=[114.6 65.4];  % theta in degrees
mu_lim=[];
eps=-0;
%%%% convert theta into radians
theta_1= theta(1)*pi/180;
theta_2= theta(2)*pi/180;

theta1=pi-theta_1;
theta2=theta_2;
spring_constants=[];
spring_constants_1=[];
%%%%%%%%%%%%%%%% input points on the pipe curvature
d=0.075;  %%%%% 3 inch diameter of the pipe
 
mu_designed=[];
add=0.01;
k1=figure(01); 
k2=figure(02); 
% k3=figure(03); 
% k4=figure(04); 

mu_lim_array=[];
while mu<=1
    mu = mu+0.01; %coefficient of friction


%######################## Using free body diagram of individual parts ##############################################
%%%% Formulating Ax=b denotes the static equilibrium conditions in a paritcular configuration
%% matrix A
%%%%% F1      F2      F3       N1      N2      N3     tau1    tau2    tau3     tau4
Aeq=[0  ,     0   ,   0   ,    1   ,   -1   ,   1   ,   0   ,   0   ,   0   ,   0;...
     1    ,   1   ,   1   ,    0   ,    0   ,   0   ,   0   ,   0   ,   0   ,   0;...
    d/2  ,   0   ,   0   ,   l1/2 ,    0   ,   0   ,  -1  ,   0   ,   0   ,   0;...
    L1*cos(theta1)   ,   0    ,   0   ,   L1*sin(theta1)  ,    0    ,   0   ,   1   ,   -1   ,   0   ,   0;...
     0    ,  -d/2  ,   0   ,   l2   ,  -l2/2   ,   0   ,   0   ,   1    ,   -1   ,   0;...
     -L2*cos(theta2)     ,   -L2*cos(theta2)  ,  0   ,   L2*sin(theta2)   ,  -L2*sin(theta2)  ,  0  ,   0   ,   0   ,   1   ,   -1;...
     0                  ,    0              ,   d/2 ,       0           ,     0            ,  -l3/2  , 0  ,   0   ,   0   ,    1];
   
beq=[0 ; w_m1+w_m2+w_m3+w_l1+w_l2 ; 0 ; w_m1*L1*cos(theta1)+w_l1*(L1/2)*cos(theta1) ; 0 ; -(w_m2+w_m1+w_l1)*L2*cos(theta2)-w_l2*(L2/2)*cos(theta2)  ; 0  ];

%%%%###########################################################################################################################

%%% Formulating Ax<=b denote the F<=mu*N 
%% matrix A
%%% F1      F2      F3       N1      N2      N3     tau1    tau2    tau3     tau4


A=[  1    ,   0   ,   0   ,  -mu   ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   1   ,   0   ,   0    ,   -mu  ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   0   ,   1   ,   0    ,   0    ,  -mu   ,  0  ,   0   ,   0   ,   0;
     1    ,   0   ,   0   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   1   ,   0   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   0   ,   1   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0];

% b=[eps ; eps ; eps; 4*0.07/d; 4*0.07/d; 4*0.07/d];
b=[eps ; eps ; eps; 50*0.07/d; 50*0.07/d; 50*0.07/d];

% b=[0 ; 0 ; 0; 0.3; 0.3; 0.3; 0.3];
%  

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


%%%%%%%%%%%%%%%%%%%%%%% using CVX optimization
% n=size(Aeq,2);
% cvx_begin
%     variable x(n);
% minimize(norm(x(7),1)+norm(x(8),1)+norm(x(9),1)+norm(x(10),1));
% %   minimize(-abs(x(7))-abs(x(8))-abs(x(9))-abs(x(10)));
%     subject to
%     Aeq*x == beq;
%     A*x < b;
% %     x(1)==x(3);
% %     x(4)==x(6);
% 
%     cvx_end
% % 
% % x
%  opts = optimoptions(@lsqnonlin,'Display','iter','Algorithm','levenberg-marquardt', 'MaxIterations',100000,'MaxFunctionEvaluations',100000);
% opts = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-40);
opts = optimset('Display','iter','Algorithm','interior-point');
% opts = optimset('Display','iter','Algorithm','interior-point', 'MaxIter',Inf);%, 'MaxFunEvals', 10000);

%  opts= optimoptions(@fmincon,'Algorithm','sqp','Display','iter-detailed');   %%% searches for global minima
% opts = optimset('Algorithm','sqp','MaxIter',5000,'MaxFunEvals',100000,'Display','iter');


%%%%%% optimization technique
% lb = [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 , 0.1, 0.1, 0.1, 0.1];  %LOWER BOUNDS
lb = [ 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0];  %LOWER BOUNDS
%lb = [];%LOWER BOUNDS
ub = [5,5,5,5,5,5,5,5,5,5]; %UPPER BOUNDS
% ub = [inf,inf,inf,inf,inf,inf,inf,inf,inf,inf]; %UPPER BOUNDS

% ub = [10,10,10,10,10,10,10,10,10,10]; %UPPER BOUNDS
% ub = [15,15,15,15,15,15,15,15,15,15]; %UPPER BOUNDS
% % 
% lb = [];
% ub = [];

% x0= [0.1  0.1  0.1  0.1  0.1  0.1  0.01  0.01  0.01  0.01]'; % [F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [0 0 0 0 0 0 0 0 0 0]'; %[F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [1.6128, 1.1723, 0.4405, 2.3446, 1.4637, 0.0001, 1.0047, 1.0982, 0.7961, 0.8286]' %% considering F=uN equality  
%  x0= [0.1085 0.1033 0.0947 0.1171 0.1067 0.0895 0.0025 0.0036 0.0004 0.0000]';
%  x0= [0.3655  0.2938  0.1595  0.2667 0.1745 0.0001 0.0196 0.0311 0.0001 0.8910]';
%  x0= [1 1 1 1 1 1 1 1 1 1]';
 x0=[1.3555 2.6460 1.2905 1.6944 3.3075 1.6131 0.1559 0.2406 0.1813 0.0839]';

ceq=[];
c=[];
% 
% x=zeros(10,1);
% A=[A;Aeq];
% b=[b;beq];
% 
% 
% x=fsolve(@(x)double(A)*x-double(b),x0);

x = fmincon(@(x)(-norm(x(4),2)-norm(x(5),2)-norm(x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% maximizing Normal forces
% x = fmincon(@(x)(norm(x(7),1)+norm(x(8),1)+norm(x(9),1)+norm(x(10),1)),x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts); %%%%% minimizing moments 
% x = fmincon(@(x)(norm(x(1),2)+norm(x(2),2)+norm(x(3),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction forces;
% x = fmincon(@(x)(norm(x(1)/x(4),2)+norm(x(2)/x(5),2)+norm(x(3)/x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction/normal force ratio;
% x = fmincon(@(x)(x(7)+x(8)+x(9)+x(10)) ,x0,A,b,Aeq,beq,lb,ub)
%%%%%%%%%%%%%% to check which constraint is not being satisfied

ang1=90-(theta1*180/pi);
ang2=theta2*180/pi;

x
spring_moments=[x(7);x(8);x(9);x(10)]
spring_constants=[spring_constants; x(7)/ang1,x(8)/ang2,x(9)/ang2,x(10)/ang1]

% x_new=inv([Aeq;A])*([beq;b])
x(1)/x(4)
x(2)/x(5)
x(3)/x(6)
 

 tau1=spring_moments(1);
 tau2=spring_moments(2);
 tau3=spring_moments(3);
 tau4=spring_moments(4);
 
 
mu_max = 1; %coefficient of friction


 eps=0;

 %%%%% F1      F2      F3       N1      N2      N3     
Aeq=[0  ,     0   ,   0   ,    1   ,   -1   ,   1 ;...
     1    ,   1   ,   1   ,    0   ,    0   ,   0 ;...
    d/2  ,   0   ,   0   ,   l1/2 ,    0   ,   0   ;...
    L1*cos(theta1)   ,   0    ,   0   ,   L1*sin(theta1)  ,    0    ,   0  ;...
     0    ,  -d/2  ,   0   ,   l2   ,  -l2/2   ,   0    ;...
    -L2*cos(theta2)     ,   -L2*cos(theta2)  ,  0   ,   L2*sin(theta2)   ,  -L2*sin(theta2)  ,  0 ;...
     0                  ,    0              ,   d/2 ,       0           ,     0            ,  -l3/2 ];
   
beq=[0 ; w_m1+w_m2+w_m3+w_l1+w_l2 ; tau1 ; w_m1*L1*cos(theta1)+w_l1*(L1/2)*cos(theta1)-tau1+tau2 ; -tau2+tau3 ; -(w_m2+w_m1+w_l1)*L2*cos(theta2)-w_l2*(L2/2)*cos(theta2)-tau3+tau4  ; -tau4  ];



A=[  1    ,   0   ,   0   ,  -mu_max   ,   0    ,   0   ;
     0    ,   1   ,   0   ,   0    ,  -mu_max  ,    0   ;
     0    ,   0   ,   1   ,   0    ,   0    ,  -mu_max  ;
     1    ,   0   ,   0   ,   0    ,   0    ,   0   ;
     0    ,   1   ,   0   ,   0    ,   0    ,   0   ;
     0    ,   0   ,   1   ,   0    ,   0    ,   0   ];

b=[eps ; eps ; eps; 4*0.07/d; 4*0.07/d; 4*0.07/d];

opts = optimset('Display','iter','Algorithm','interior-point');
% opts = optimset('Display','iter','Algorithm','interior-point', 'MaxIter',Inf);%, 'MaxFunEvals', 10000);

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
x = fmincon(@(x)(-norm(x(4),2)-norm(x(5),2)-norm(x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% maximizing Normal forces
% x = fmincon(@(x)(norm(x(1),2)+norm(x(2),2)+norm(x(3),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction forces;
% x = fmincon(@(x)(norm(x(1)/x(4),2)+norm(x(2)/x(5),2)+norm(x(3)/x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction/normal force ratio;
% x = fmincon(@(x)(x(7)+x(8)+x(9)+x(10)) ,x0,A,b,Aeq,beq,lb,ub)
%%%%%%%%%%%%%% to check which constraint is not being satisfied

ang1=90-(theta1*180/pi);
ang2=theta2*180/pi;

x

%% ratio of F/N
x(1)/x(4)
x(2)/x(5)
x(3)/x(6)

mu_lim= max([x(1)/x(4),x(2)/x(5),x(3)/x(6)]);
mu_lim_array=[mu_lim_array;mu_lim];
mu_designed=[mu_designed;mu];




%######################## Using free body diagram of individual parts ##############################################
%%%% Formulating Ax=b denotes the static equilibrium conditions in a paritcular configuration
%% matrix A
%%%%% F1      F2      F3       N1      N2      N3     tau1    tau2    tau3     tau4
Aeq=[0  ,     0   ,   0   ,    1   ,   -1   ,   1   ,   0   ,   0   ,   0   ,   0;...
     1    ,   1   ,   1   ,    0   ,    0   ,   0   ,   0   ,   0   ,   0   ,   0;...
    d/2  ,   0   ,   0   ,   l1/2 ,    0   ,   0   ,  -1  ,   0   ,   0   ,   0;...
    L1*cos(theta1)   ,   0    ,   0   ,   L1*sin(theta1)  ,    0    ,   0   ,   1   ,   -1   ,   0   ,   0;...
     0    ,  -d/2  ,   0   ,   l2   ,  -l2/2   ,   0   ,   0   ,   1    ,   -1   ,   0;...
     -L2*cos(theta2)     ,   -L2*cos(theta2)  ,  0   ,   L2*sin(theta2)   ,  -L2*sin(theta2)  ,  0  ,   0   ,   0   ,   1   ,   -1;...
     0                  ,    0              ,   d/2 ,       0           ,     0            ,  -l3/2  , 0  ,   0   ,   0   ,    1];
   
beq=[0 ; w_m1+w_m2+w_m3+w_l1+w_l2 ; 0 ; w_m1*L1*cos(theta1)+w_l1*(L1/2)*cos(theta1) ; 0 ; -(w_m2+w_m1+w_l1)*L2*cos(theta2)-w_l2*(L2/2)*cos(theta2)  ; 0  ];

%%%%###########################################################################################################################

%%% Formulating Ax<=b denote the F<=mu*N 
%% matrix A
%%% F1      F2      F3       N1      N2      N3     tau1    tau2    tau3     tau4


A=[  1    ,   0   ,   0   ,  -mu_lim   ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   1   ,   0   ,   0    ,   -mu_lim  ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   0   ,   1   ,   0    ,   0    ,  -mu_lim   ,  0  ,   0   ,   0   ,   0;
     1    ,   0   ,   0   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   1   ,   0   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0;
     0    ,   0   ,   1   ,   0    ,   0    ,   0    ,  0  ,   0   ,   0   ,   0];

% b=[eps ; eps ; eps; 4*0.07/d; 4*0.07/d; 4*0.07/d];
b=[eps ; eps ; eps; 50*0.07/d; 50*0.07/d; 50*0.07/d];

% b=[0 ; 0 ; 0; 0.3; 0.3; 0.3; 0.3];
%  

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


%%%%%%%%%%%%%%%%%%%%%%% using CVX optimization
% n=size(Aeq,2);
% cvx_begin
%     variable x(n);
% minimize(norm(x(7),1)+norm(x(8),1)+norm(x(9),1)+norm(x(10),1));
% %   minimize(-abs(x(7))-abs(x(8))-abs(x(9))-abs(x(10)));
%     subject to
%     Aeq*x == beq;
%     A*x < b;
% %     x(1)==x(3);
% %     x(4)==x(6);
% 
%     cvx_end
% % 
% % x
%  opts = optimoptions(@lsqnonlin,'Display','iter','Algorithm','levenberg-marquardt', 'MaxIterations',100000,'MaxFunctionEvaluations',100000);
% opts = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-40);
opts = optimset('Display','iter','Algorithm','interior-point');
% opts = optimset('Display','iter','Algorithm','interior-point', 'MaxIter',Inf);%, 'MaxFunEvals', 10000);

%  opts= optimoptions(@fmincon,'Algorithm','sqp','Display','iter-detailed');   %%% searches for global minima
% opts = optimset('Algorithm','sqp','MaxIter',5000,'MaxFunEvals',100000,'Display','iter');


%%%%%% optimization technique
% lb = [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 , 0.1, 0.1, 0.1, 0.1];  %LOWER BOUNDS
lb = [ 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0];  %LOWER BOUNDS
%lb = [];%LOWER BOUNDS
ub = [5,5,5,5,5,5,5,5,5,5]; %UPPER BOUNDS
% ub = [inf,inf,inf,inf,inf,inf,inf,inf,inf,inf]; %UPPER BOUNDS

% ub = [10,10,10,10,10,10,10,10,10,10]; %UPPER BOUNDS
% ub = [15,15,15,15,15,15,15,15,15,15]; %UPPER BOUNDS
% % 
% lb = [];
% ub = [];

% x0= [0.1  0.1  0.1  0.1  0.1  0.1  0.01  0.01  0.01  0.01]'; % [F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [0 0 0 0 0 0 0 0 0 0]'; %[F1 F2 F3 N1 N2 N3 tau1 tau2 tau3 tau4]
%  x0= [1.6128, 1.1723, 0.4405, 2.3446, 1.4637, 0.0001, 1.0047, 1.0982, 0.7961, 0.8286]' %% considering F=uN equality  
%  x0= [0.1085 0.1033 0.0947 0.1171 0.1067 0.0895 0.0025 0.0036 0.0004 0.0000]';
%  x0= [0.3655  0.2938  0.1595  0.2667 0.1745 0.0001 0.0196 0.0311 0.0001 0.8910]';
%  x0= [1 1 1 1 1 1 1 1 1 1]';
 x0=[1.3555 2.6460 1.2905 1.6944 3.3075 1.6131 0.1559 0.2406 0.1813 0.0839]';

ceq=[];
c=[];
% 
% x=zeros(10,1);
% A=[A;Aeq];
% b=[b;beq];
% 
% 
% x=fsolve(@(x)double(A)*x-double(b),x0);

x = fmincon(@(x)(-norm(x(4),2)-norm(x(5),2)-norm(x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% maximizing Normal forces
% x = fmincon(@(x)(norm(x(7),1)+norm(x(8),1)+norm(x(9),1)+norm(x(10),1)),x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts); %%%%% minimizing moments 
% x = fmincon(@(x)(norm(x(1),2)+norm(x(2),2)+norm(x(3),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction forces;
% x = fmincon(@(x)(norm(x(1)/x(4),2)+norm(x(2)/x(5),2)+norm(x(3)/x(6),2)) ,x0,A,b,Aeq,beq,lb,ub,[ceq,c],opts);  %%%%% minimize traction/normal force ratio;
% x = fmincon(@(x)(x(7)+x(8)+x(9)+x(10)) ,x0,A,b,Aeq,beq,lb,ub)
%%%%%%%%%%%%%% to check which constraint is not being satisfied

ang1=90-(theta1*180/pi);
ang2=theta2*180/pi;

x
% spring_moments_1=[x(7);x(8);x(9);x(10)]
spring_constants_1=[spring_constants_1; x(7)/ang1,x(8)/ang2,x(9)/ang2,x(10)/ang1]

% x_new=inv([Aeq;A])*([beq;b])
x(1)/x(4)
x(2)/x(5)
x(3)/x(6);



end
% plot(theta,trac_force(:,2),'g-','LineWidth',1);hold on
% plot(theta,trac_force(:,3),'b-','LineWidth',1);hold on
% 
figure(k1); 
plot(mu_lim_array, spring_constants(:,1),'y-','LineWidth',2);hold on
plot(mu_lim_array, spring_constants_1(:,1),'g-','LineWidth',2);hold on

plot(mu_lim_array, spring_constants(:,2),'b-','LineWidth',2);hold on
plot(mu_lim_array, spring_constants_1(:,2),'y-','LineWidth',2);hold on

plot(mu_lim_array, spring_constants(:,3),'black-','LineWidth',2);hold on
plot(mu_lim_array, spring_constants_1(:,3),'b-','LineWidth',2);hold on

plot(mu_lim_array, spring_constants(:,4),'c-','LineWidth',2);hold on
plot(mu_lim_array, spring_constants_1(:,4),'r-','LineWidth',2);hold on

figure(k2)
plot(mu_designed,mu_lim_array,'r-','LineWidth',2);hold on


