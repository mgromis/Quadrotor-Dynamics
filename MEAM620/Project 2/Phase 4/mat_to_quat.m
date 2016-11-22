function [ q ] = mat_to_quat( rot )
%MAT_TO_QUAT Summary of this function goes here
%   Detailed explanation goes here

if(isequal(rot,eye(3)))
   q = [1 0 0 0]';
   return
end
  phi = acos((rot(1,1)+rot(2,2)+rot(3,3)-1)/2);
    w = 1/(2*sin(phi))*[rot(3,2)-rot(2,3);
        rot(1,3)-rot(3,1);
        rot(2,1)-rot(1,2)];
   q = [cos(phi/2); 
       w(1,1)*sin(phi/2); 
       w(2,1)*sin(phi/2); 
       w(3,1)*sin(phi/2)];
    
   


end

