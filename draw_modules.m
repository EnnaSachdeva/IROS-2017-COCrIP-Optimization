function [ h ] = draw_modules( centres,dm)

th = 0:2*pi/200:2*pi;
th = wrapTo2Pi(th);


rm=dm/2;
l=rm;

cent_mod_11=centres(:,1);
cent_mod_12=centres(:,2);
cent_mod_13=centres(:,3);
cent_mod_21=centres(:,4);
cent_mod_22=centres(:,5);
cent_mod_23=centres(:,6);
cent_mod_31=centres(:,7);
cent_mod_32=centres(:,8);
cent_mod_33=centres(:,9);

x_mod_11= cent_mod_11(1)+ rm* cos(th) ;
y_mod_11= cent_mod_11(2)+ l* sin(th) ;

x_mod_12= cent_mod_12(1)+ rm* cos(th) ;
y_mod_12= cent_mod_12(2)+ l* sin(th) ;

x_mod_13= cent_mod_13(1)+ rm* cos(th) ;
y_mod_13= cent_mod_13(2)+ l* sin(th) ;

x_mod_21= cent_mod_21(1)+ rm* cos(th) ;
y_mod_21= cent_mod_21(2)+ l* sin(th) ;

x_mod_22= cent_mod_22(1)+ rm* cos(th) ;
y_mod_22= cent_mod_22(2)+ l* sin(th) ;

x_mod_23= cent_mod_23(1)+ rm* cos(th);
y_mod_23= cent_mod_23(2)+ l* sin(th) ;

x_mod_31= cent_mod_31(1)+ rm* cos(th) ;
y_mod_31= cent_mod_31(2)+ l* sin(th) ;

x_mod_32= cent_mod_32(1)+ rm* cos(th) ;
y_mod_32= cent_mod_32(2)+ l* sin(th) ;

x_mod_33= cent_mod_33(1)+ rm* cos(th) ;
y_mod_33= cent_mod_33(2)+ l* sin(th) ;

 h= plot(x_mod_11,y_mod_11,'blue',x_mod_12,y_mod_12,'red',x_mod_13,y_mod_13,'cyan',...
         x_mod_21,y_mod_21,'blue',x_mod_22,y_mod_22,'red',x_mod_23,y_mod_23,'cyan',...
         x_mod_31,y_mod_31,'blue',x_mod_32,y_mod_32,'red',x_mod_33,y_mod_33,'cyan');
hold on

end

