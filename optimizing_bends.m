%%%%%%%%%% This involves all nonlinear equations and solved using fsolve 

close all;
clear all;
clc;
%% Design parameters (All Units are SI) ###########
m_m1 = 0.160; %module_1 mass
m_m2 = 0.180; %module_2 mass
m_m3 = 0.160; %module_3 mass

m_m11 = m_m1/3; %module_1 mass
m_m12 = m_m1/3; %module_2 mass
m_m13 = m_m1/3; %module_3 mass

m_m21 = m_m2/3; %module_1 mass
m_m22 = m_m2/3; %module_2 mass
m_m23 = m_m2/3; %module_3 mass

m_m31 = m_m3/3; %module_1 mass
m_m32 = m_m3/3; %module_2 mass
m_m33 = m_m3/3; %module_3 mass

m_l1 = 0.020; %link_1 mass
m_l2 = 0.020; %link_2 mass

g = 9.8; %acceleration due to gravity

w11 = m_m11*g; %module_1 mass
w12 = m_m12*g; %module_1 mass
w13 = m_m13*g; %module_1 mass

wl1 = m_l1*g; %link_1 mass
wl2 = m_l2*g; %link_2 mass

w21 = m_m21*g; %module_1 mass
w22 = m_m22*g; %module_1 mass
w23 = m_m23*g; %module_1 mass

w31 = m_m31*g; %module_1 mass
w32 = m_m32*g; %module_1 mass
w33 = m_m33*g; %module_1 mass

mu = 0.8; %coefficient of friction

l_m=0.120; % length of module
l_p=0.024; % length of rolling pololu motor

l1 = l_m+l_p ; %length of module+pololu
l2 = l_m+(2*l_p); %length of module+pololu
% l2 = l_m+l_p; %length of module+pololu
l3 = l_m+l_p; %length of module+pololu
% l3 = l_m+l_p ; %length of module+pololu

l11= l1/3;
l12= l1/3;
l13= l1/3;

l21= l2/3;
l22= l2/3;
l23= l2/3;

l31= l3/3;
l32= l3/3;
l33= l3/3;

L1 = 0.060 ; %length of link1
L2 = 0.060 ; %length of link2

% dm = 0.050; % diameter of the module
dm = l11; % diameter of the module

%%%%%%%%%%%%%%%% input points on the pipe curvature
d=0.075;  %%%%% 3 inch diameter of the pipe
rc=0.127; %%%%% radius of the pipe 
xc=0;    %%%%% center x coordinate of the pipe 
yc=0;    %%%%% center x coordinate of the pipe 
theta_pipe=360; %%%%% in degrees
steps=360; %%%%% steps in which the points on the curve are distributed
initialization=[0, 0, -5 , -5, -5, -5, -5, -5];


% initialization_opt=ones(28, 1) * 0.1;
% initialization_opt=ones(28, 1) * 0;

%%%%%%% worked with initialization using lsqnonneg

% initialization_opt=[0.5209 0 1.1264  0 0 0 0 0 0 1.5910 0 0.3841 0  0.1353 5.2691 0 0 0 0.0507 0.0390 0 0.0686 0 0 0.1574 0.1017 0.06000 0.0118];
% initialization_opt= [0.0940 0.0000 0.0804 0.0246 0.0006 3.5228 0.0049 0.0811 0.5772 0.0910 0.0769 0.0717 0.0000 0.0039 5.0425,...
%                      4.9591 0.1931 3.2271 0.0262 0.0582 0.0200 0.0573 0.0117 5.0418 0.3207 0.1135 0.0802 0.0974];
                 
%  initialization_opt=[0 0.0006 0.5095 0.4023 0.1956 0.0000 3.4077 0.0000 0.0000 0 0.0008 0.6369 0.5029 0.2445 0 4.2597 0 0 0.0000,...
%                      0.0357 0.0124  0.0155  0  0  0.0672  0.1025 0.0813 0.0118];
%                  
%  initialization_opt=[0.1515 0 0 0 0 0 0 0 0 0.4600 0 0 0.0229 0.1350 5.2688 0 0 0 0.0147 0.0367 -0.0212 0.0485 0 0 0.1250 0.0693 0.0276 0.0118]; % initialization=[];
%  initialization_opt=ones(28, 1) * 0.01;

% initialization_opt=[0.6558 0.2302 0.7234 0 0 1.3465 0 0 1.9001 0.8198 0.2877 0.9042 0 0 1.6831 0 0 2.3751 0.0354 0.0415 0.0151 0.0382 0 0.0876 0.0103 0 0.0134 0.0232];

%  initialization_opt=[-0.1319 -0.0471 0.1274 -0.1084 -0.0868 0.9377 -1.2664 -0.0134 1.3781 -0.1649 -0.0589 0.1592 -0.1355 -0.1085 1.1721 -1.5830 -0.0168,...
%                         1.7226 -0.0071 0.0346 -0.0064 0.0241 -0.0025 0.0186 -0.0894 -0.0296 -0.0246 0.0201];
% 
% initialization_opt=[0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.1 0.1 0.1 0.1  0.1 0.1 0.1 0.1 0.1 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02 0.02];

 initialization_opt=[1.0337 1.2168 1.0552 0.1327 0.0671 0.2198 0.1638 0.1853 0.1716 1.4292 2.6079 4.3223 0.7127 3.9866 4.8345 1.4923 4.4848,...
                     4.6704 0.0591 0.2124 0.5055 1.0005 3.1969 2.0270 1.8999 1.2385 0.7716 0.1068];
% 
%                  
%   initialization_opt=[0.0023 0.0000 0.0000 0.0000 0.0001 0.0004 0.0004 0.0007 0.0009 0.0001 0.0006 0.0021 0.0282 0.0407 5.2212 0.0024 0.0001 1.3454 0.1112,...
%                       0.1216 0.1571 0.2055 0.6162 0.7206 0.3939 0.2907 0.2116 0.1429];
% 
%  initialization_opt=[0.1000  0.1000 0.2607 0.1000 0.1000 0.1000 0.1000 0.1000 0.1000 0.1000 0.2268 0.5886 0.1000 0.5086 5.2349 0.1000 0.1000 2.3680,...
%                      0.1228  0.1440 0.2098 0.3023 0.6369 0.0100 0.6369 0.4636 0.3343 0.1711];
% 
% 



%%%%% bent pipe points 
[out_curv_points, in_curv_points, mid_in_curv_points, mid_out_curv_points] = pipe_bend(d,dm,rc,xc,yc,theta_pipe,steps );

h1=figure(01);
figure(h1);
h11=plot(in_curv_points(1,:),in_curv_points(2,:),'r',out_curv_points(1,:),out_curv_points(2,:),'b',mid_in_curv_points(1,:),mid_in_curv_points(2,:),'g--',mid_out_curv_points(1,:),mid_out_curv_points(2,:),'c--');
hold on

lb = [ 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 , 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 , 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 , 0.01, 0.01];  %LOWER BOUNDS
%  lb = [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 , 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 , 0.01, 0.01];  %LOWER BOUNDS
%  lb=[];
%         % lb = [];%LOWER BOUNDS
        % ub = [5,5,5,5,5,5,5,5,5,5]; %UPPER BOUNDS
%  ub = [10, 10, 10, 10, 10, 10 , 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 , 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 , 10, 10];  %LOWER BOUNDS
 ub = [ 5, 5, 5, 5, 5, 5 , 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 , 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5];  %LOWER BOUNDS
%  ub=[];
 %         
%     
output=[];
spring_constants=[];
torsion_angles_1=[];
spring_moments=[];
thetas=[];

for_plot=[];
% m=figure(03);
pipe_angle= 0:theta_pipe/steps:theta_pipe;  %%%%% in degrees

theta1=[];
theta2=[];
theta3=[];

normal_force=[];

h2=figure(02); 

moments=figure(03); 
springs=figure(04);

moments_prob=figure(05); 
springs_prob=figure(06); 
normal=figure(06); 
torsion_angles=[];
fric_coeff=[];
max_fric_coeff=[];
trac_force=[];
% for i=1:1:size(in_curv_points,2)
iter=steps;
% iter=50;
for i=1:1:iter
        data_for_traj=[l11,l12,l13,l21,l22,l23,l31,l32,l33,L1,L2,dm,d,rc,xc,yc];
        %%%% Finding trajectory contact points and thetas of submodules from geometry
        % format of contact_points = [modl (x,y,x,y,x,y),mod2 (x,y,x,y,x,y),mod3 (x,y,x,y,x,y)];
        % format of angles = [mod11,mod12,mod13,mod21,mod22,mod23,mod31,mod32,mod33,link1,link2];
        
       [contact_points,centres,angles,init,points ] = get_data_from_traj(data_for_traj,initialization,in_curv_points(:,i),i,steps );
        
        initialization=init;
       
        figure(h1);
        h12=plot([points(1),points(2)],[points(3),points(4)],[points(5),points(6)],[points(7),points(8)],...
               points(9), points(10),'b*',points(11), points(12),'r*',points(13), points(14),'c*', points(15), points(16),'y--o',points(17), points(18),'g--o',...
               points(19),points(20),'b*',points(21),points(22),'r*',points(23),points(24),'c*',points(25),points(26),'g--o',points(27),points(28),'b--o',...
               points(29),points(30),'b*',points(31), points(32),'r*',points(33), points(34),'c*'); 

      hold on;
      h13= draw_modules( centres,dm );
      hold on;
  
      M(i)=getframe(gcf);
      
      angles=rem((angles+2*pi),2*pi);
        
        th11=angles(1);
        th12=angles(2);
        th13=angles(3);
        th1=angles(4);
        th21=angles(5);
        th22=angles(6);
        th23=angles(7);
        th2=angles(8);
        th31=angles(9);
        th32=angles(10);
        th33=angles(11);
       
        
%         theta1=[theta1;th1];
%         theta2=[theta2;th2];
%                
        % angles*180/pi
%         thetas=[thetas;angles];
        %%%% Finding normal forces, moments, torques and spring constant from optimization
        data_for_optim=[l11,l12,l13,l21,l22,l23,l31,l32,l33,L1,L2,...
                        th11,th12,th13,th21,th22,th23,th31,th32,th33,th1,th2,...
                        w11,w12,w13,wl1,w21,w22,w23,wl2,w31,w32,w33,dm];
       
        [optimized_output,init_opt]= optimization(data_for_optim,initialization_opt,lb,ub,mu);
        initialization_opt=init_opt;
        output = [output; optimized_output'];
        
        
        fric_coeff=[optimized_output(1)/optimized_output(10),optimized_output(2)/optimized_output(11),optimized_output(3)/optimized_output(12),...
                    optimized_output(4)/optimized_output(13),optimized_output(5)/optimized_output(14),optimized_output(6)/optimized_output(15),...
                    optimized_output(7)/optimized_output(16),optimized_output(8)/optimized_output(13),optimized_output(17)/optimized_output(18)];
        
                
                
        max_fric_coeff=[max_fric_coeff;max(fric_coeff)];        % x=[x(1);x(2);x(3);x(4);x(5);x(6);x(7)/ang1;x(8)/ang2;x(9)/ang2;x(10)/ang1]
       
%        if (max(fric_coeff)>0.9)
%            display('Friction coeff > 0.8');
%            return;
%        end
%        th23*180/pi
        
%         torsion_angles=[(th11+th12+th13-th1)*180/pi, (th21-th1)*180/pi, (th2-th21-th22-th23)*180/pi, (th2-th31)*180/pi]  %%%% in degrees
           torsion_angles= [5.8256   41.6183   40.7858    7.8004];
        torsion_angles_1=[torsion_angles_1;torsion_angles];
        
        spring_moments=[spring_moments;optimized_output(21),...
                                       optimized_output(22),...
                                       optimized_output(25),...
                                       optimized_output(26)];                 
               
        normal_force=[normal_force;optimized_output(10),...  %%%% N11
                                   optimized_output(11),...  %%%% N12
                                   optimized_output(12),...  %%%% N13
                                   optimized_output(13),...  %%%% N21
                                   optimized_output(14),...  %%%% N22
                                   optimized_output(15),...  %%%% N23
                                   optimized_output(16),...  %%%% N31
                                   optimized_output(17),...  %%%% N32
                                   optimized_output(18)];    %%%% N33
        
                               
        trac_force=[trac_force;optimized_output(1),...  %%%% N11
                                   optimized_output(2),...  %%%% N12
                                   optimized_output(3),...  %%%% N13
                                   optimized_output(4),...  %%%% N21
                                   optimized_output(5),...  %%%% N22
                                   optimized_output(6),...  %%%% N23
                                   optimized_output(7),...  %%%% N31
                                   optimized_output(8),...  %%%% N32
                                   optimized_output(9)];    %%%% N33
               






%         for_plot=[for_plot; optimized_output(10),th11*180/pi];
%         
        spring_constants=[spring_constants; optimized_output(21)/torsion_angles(1),...
                                            optimized_output(22)/torsion_angles(2),...
                                            optimized_output(25)/torsion_angles(3),...
                                            optimized_output(26)/torsion_angles(4)];
                                        
     
                           
%     figure(mm);
%     plot(pipe_angle(i),optimized_output(21),'b*'); hold on; 
%     
%     figure(nn);
%     plot(pipe_angle(i),optimized_output(22),'r*'); hold on; 
% 
%     figure(oo);
%     plot(pipe_angle(i),optimized_output(25),'g*'); hold on; 
% 
%     figure(pp);
%     plot(pipe_angle(i),optimized_output(26),'c*'); hold on;  
% 

 
%    pause(0.1);
   
   if(i~=iter)
       delete(h12);
       delete(h13);
   end
   
  
  
end

output=output';
    
max_fric_coeff

% %     n=size(spring_constants,1);
% %     h = histogram(spring_constants(:,1),n);
     %  output %=[ output(:,24)
%     mean_spring_moments=[max(spring_moments(1,:));mean(spring_moments(2,:));mean(spring_moments(3,:));mean(spring_moments(4,:))]
%     mean_spring_constants=[mean(spring_constants(1,:));mean(spring_constants(2,:));mean(spring_constants(3,:));mean(spring_constants(4,:))]
%     
%     max_spring_moments=[max(spring_moments(1,:));max(spring_moments(2,:));max(spring_moments(3,:));max(spring_moments(4,:))]
%     max_spring_constants=[max(spring_constants(1,:));max(spring_constants(2,:));max(spring_constants(3,:));max(spring_constants(4,:))]
% 
%     mean_sea_moments=[mean(output(19,:));mean(output(20,:));mean(output(23,:));mean(output(24,:));mean(output(27,:));mean(output(28,:))]
%     max_sea_moments=[max(output(19,:));max(output(20,:));max(output(23,:));max(output(24,:));max(output(27,:));max(output(28,:))]
% 
%   
  
theta=0:theta_pipe/(steps-1):theta_pipe;



spring_moments(:,1) = smooth(spring_moments(:,1));
spring_moments(:,2) = smooth(spring_moments(:,2));
spring_moments(:,3) = smooth(spring_moments(:,3));
spring_moments(:,4) = smooth(spring_moments(:,4));



spring_constants(:,1) = smooth(spring_constants(:,1));
spring_constants(:,2) = smooth(spring_constants(:,2));
spring_constants(:,3) = smooth(spring_constants(:,3));
spring_constants(:,4) = smooth(spring_constants(:,4));

%     cs = csapi(theta,spring_moments(1,:));
%     fnplt(cs,2);


    figure(moments);
    plot(theta-(th13*180/pi),spring_moments(:,1),'b--o','LineWidth',2);hold on
    plot(theta-(th21*180/pi),spring_moments(:,2),'r-','LineWidth',2);hold on
    plot(theta-(th22*180/pi),spring_moments(:,3),'g-','LineWidth',2);hold on
    plot(theta-(th31*180/pi),spring_moments(:,4),'c-','LineWidth',2);hold on
    title(' Joint moment profile w.r.t bending angle')
    xlabel('angle') % x-axis label
    ylabel('joint moment') % y-axis label

    l{1}='Joint J1'; l{2}='Joint J2';  l{3}='Joint J3';  l{4}='Joint J4';  
    legend(l); 
    hold on
     
     figure(springs);
     plot(theta,spring_constants(:,1),'b--o','LineWidth',2);hold on
     plot(theta,spring_constants(:,2),'r-','LineWidth',2);hold on
     plot(theta,spring_constants(:,3),'g-','LineWidth',2);hold on
     plot(theta,spring_constants(:,4),'c-','LineWidth',2);hold on
     title(' Estimated spring stiffness')
     xlabel('angle') % x-axis label
     ylabel('spring stiffness') % y-axis label
     l{1}='Joint J1'; l{2}='Joint J2';  l{3}='Joint J3';  l{4}='Joint J4';  
     legend(l); 
     hold on

     figure(normal);     
     plot(theta,normal_force(:,1),'r-','LineWidth',1);hold on
     plot(theta,normal_force(:,2),'g-','LineWidth',1);hold on
     plot(theta,normal_force(:,3),'b-','LineWidth',1);hold on
     plot(theta,normal_force(:,4),'c-','LineWidth',1);hold on
     plot(theta,normal_force(:,5),'black','LineWidth',1);hold on
     plot(theta,normal_force(:,6),'y-','LineWidth',1);hold on
     plot(theta,normal_force(:,7),'r*-','LineWidth',1);hold on
     plot(theta,normal_force(:,8),'g*-','LineWidth',1);hold on
     plot(theta,normal_force(:,9),'b*-','LineWidth',1);hold on
     
     title('Normal force w.r.t angle');hold on
     xlabel('angle');hold on % x-axis label
     ylabel('Normal force') ;hold on% y-axis label
     l{1}='N11'; l{2}='N12';  l{3}='N13';
     l{4}='N21', l{5}='N22'; l{6}='N23';
     l{7}='N31'; l{8}='N32', l{9}='N33';  
     legend(l); 
     hold on
     
x_values = 0.001:0.0001:0.05;
pd_1= fitdist(spring_constants(:,1),'Normal');
spring_constant_1 = pdf(pd_1,x_values);

pd_2= fitdist(spring_constants(:,2),'Normal');
spring_constant_2 = pdf(pd_2,x_values);

pd_3= fitdist(spring_constants(:,3),'Normal');
spring_constant_3 = pdf(pd_3,x_values);

pd_4= fitdist(spring_constants(:,4),'Normal');
spring_constant_4 = pdf(pd_4,x_values);


spring_constant_1=spring_constant_1/max(spring_constant_1);
spring_constant_2=spring_constant_2/max(spring_constant_2);
spring_constant_3=spring_constant_3/max(spring_constant_3);
spring_constant_4=spring_constant_4/max(spring_constant_4);


     figure(springs_prob);
     plot(x_values,spring_constant_1,'b-','LineWidth',2);hold on
     plot(x_values,spring_constant_2,'r-','LineWidth',2);hold on
     plot(x_values,spring_constant_3,'g-','LineWidth',2);hold on
     plot(x_values,spring_constant_4,'c-','LineWidth',2);hold on
     l{1}='Joint J1'; l{2}='Joint J2';  l{3}='Joint J3';  l{4}='Joint J4';  
     legend(l);   hold on
     xlabel('Spring stiffness');hold on % x-axis label
     ylabel('Probability distribution') % y-axis label
     hold on

    
     
     figure
     plot(theta,trac_force(:,1),'r-','LineWidth',1);hold on
     plot(theta,trac_force(:,2),'g-','LineWidth',1);hold on
     plot(theta,trac_force(:,3),'b-','LineWidth',1);hold on
     plot(theta,trac_force(:,4),'c-','LineWidth',1);hold on
     plot(theta,trac_force(:,5),'black','LineWidth',1);hold on
     plot(theta,trac_force(:,6),'y-','LineWidth',1);hold on
     plot(theta,trac_force(:,7),'r*-','LineWidth',1);hold on
     plot(theta,trac_force(:,8),'g*-','LineWidth',1);hold on
     plot(theta,trac_force(:,9),'b*-','LineWidth',1);hold on
     title('Trcation force w.r.t angle');hold on
     xlabel('angle');hold on % x-axis label
     ylabel('Traction force') ;hold on% y-axis label
     l{1}='F11'; l{2}='F12';  l{3}='F13';
     l{4}='F21', l{5}='F22'; l{6}='F23';
     l{7}='F31'; l{8}='F32', l{9}='F33';  
     legend(l); 
     hold on
     
% m=figure(02);
% x=[1:1:11]; %%%% number of joints 
% y=angles-pi;
% p = polyfit(x,y,5);
%  
% x1=[1:1:11];
% y1 = polyval(p,x1);
% plot(x,y,'o')
% hold on
% plot(x1,y1)
% hold off
 
figure;
% movie(M,5);
video = VideoWriter('quasi_static.avi','Uncompressed AVI');
open(video);
writeVideo(video,M);
