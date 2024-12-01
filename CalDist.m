 
%% 计算目标点与其他机器人之间最近距离D
% 若D < safe_distance  这调整目标点
function [targetx, targety]  = CalDist(targetx , targety,x,y,safe_distance)

     D = sqrt((x - targetx)^2 + (y - targety)^2);
  
     if safe_distance > D
           targetx = x + safe_distance * (targetx - x)/D;
           targety = y + safe_distance * (targety - y)/D;
         
     end



end



