function [ traveled_distance  ,traveled_time ] = validate( xposition, yposition, time , pxdata , pydata, ptime)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
sum = 0.0 ; 
for i=2:length(xposition)
    sum = sum + sqrt((xposition(i)-xposition(i-1))^2 + (yposition(i)-yposition(i-1))^2) ;
end
traveled_distance = sum 
traveled_time = time(end) - time(1) 



end
% 

% Y_x = diff ( pxdata )
% Y_x2 = diff ( Y_x) 
% Y_x3 = diff(Y_x2)
% Y_x4 = diff(Y_x3)
% Y_y = diff ( pydata )
% Y_y2 = diff ( Y_y) 
% close all
% figure(1)
% plot(Y_x)
% figure(2)
% plot(Y_x2 ,'r')
% figure(3)
% plot(Y_y)
% figure(4)
% plot(Y_y2 , 'r')
