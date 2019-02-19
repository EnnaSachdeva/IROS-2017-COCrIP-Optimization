function F = fun(x)
Aeq=[0  ,     0   ,   0   ,    1   ,   -1   ,   1   ,   0   ,   0   ,   0   ,   0;
     1    ,   1   ,   1   ,    0   ,    0   ,   0   ,   0   ,   0   ,   0   ,   0;
    -d/2  ,   0   ,   0   ,   -l1/2 ,    0   ,   0   ,   1  ,   0   ,   0   ,   0;
    -L1*cos(theta1)   ,   0    ,   0   ,   -L1*sin(theta1)  ,    0    ,   0   ,   -1   ,   1   ,   0   ,   0;
     0    ,  d/2  ,   0   ,   l2   ,  l2/2   ,   0   ,   0   ,   -1    ,   1   ,   0;
     L2*cos(theta2)     ,   L2*cos(theta2)  ,  0   ,   -L2*sin(theta2)   ,  L2*sin(theta2)  ,  0  ,   0   ,   0   ,   -1   ,   1;
     0                  ,    0              ,   -d/2 ,       0           ,     0            ,   l3/2  , 0  ,   0   ,   0   ,    -1];
   
beq=[0 ; w_m1+w_m2+w_m3+w_l1+w_l2 ; 0 ; -w_m1*L1*cos(theta1)-w_l1*(L1/2)*cos(theta1) ; 0 ; (w_m2+w_m1+w_l1)*L2*cos(theta2)+w_l2*(L2/2)*cos(theta2)  ; 0];


F = Aeq*[x(1);x(2);x(3);x(4);x(5);x(6);x(7);x(8);x(9);x(10)];
