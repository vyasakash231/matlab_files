function [T_new,R,O] = Transformation_matrix(n,alpha,a,d,theta) 
% All angles in radian
I = eye(4);
 for i = 1:(n)
     T = [cosd(theta(i))                             -sind(theta(i))                    0                   a(i)      ;
         sind(theta(i))*cosd(alpha(i))      cosd(theta(i))*cosd(alpha(i))     -sind(alpha(i))     -d(i)*sind(alpha(i));
         sind(theta(i))*sind(alpha(i))      cosd(theta(i))*sind(alpha(i))      cosd(alpha(i))      d(i)*cosd(alpha(i));
                       0                                 0                           0                      1         ];
     T_new = I*T;
     R(1:3,1:3,i) = T_new(1:3,1:3);  
     O(1:3,i) = T_new(1:3,4);
     I = T_new;
 end
end