%% mapping 
[oo,index_max_for]=max(abs(pxdata));
position_time = stime(index_max_for) ; 
%%
close all

eps = 0.001;
aux = [] ;
a = [] ; 
%% Algorithm to make two arrays with the same size pased on the closest time
%start the loop with the smaller array
for i =1:length(s2time)
    a = [a s2time(i) - position_time] ; % it will generate an array with the differencs between the elemnt and the largest array
end
[c,j]=min(abs(a)); % find the index of the min 'j'
% j is the index of the time for the position that we need 
    
%% timing for the markers with the maximum force 
b = [] ; 
for i = 1:length(s5time)
    b = [b s5time(i) - position_time ]; 
end 
[c,w]=min(abs(b)); % find the index of the min 'w'
% w is the index for the array of markers that are acting on the vecile at
% that position and that time 
%%
 
o_point = sqrt(sxdata(j)^sxdata(j) + sydata(j)+sydata(j))
unit_vector = [sxdata(j)/o_point , sydata(j)/o_point, 0] 
mmmmm =  size ( x_pose , 2 ) ; 
force_in_R=zeros(2, mmmmm);
force_norm=zeros(2, mmmmm);
orgin_arrow=zeros(2,mmmmm);
end_arrow=zeros(2, mmmmm);



%% Transform the forces from the world frame to the robot frame ( use quaterinian) using a pre defind function called quat to dec
% we have the quar data from the bag file 

 q = [owdata(j) oxdata(j) oydata(j) ozdata(j)]; % j = the index of the position with the highest force
    dcm = quat2dcm (q/norm(q)); % call the function by sending the Quatr data over the norm it will generate 4 by 4 
    dcm2D = dcm(1:2,1:2); % we want only the data related to x and y  ????
    force_in_R(:,:) = dcm2D * [x_pose(w,:) ; y_pose(w,:)]; % all the resk vectors at that position 
    %force_norm(:,:) = force_in_R(:,1:length(x_pose))/norm(force_in_R(:,1:length(x_pose))); % ??? 
    orgin_arrow(:,:) = [x_pose(w,:) ; y_pose(w,:)] ; % the start of the arraw is the position in robot frame  
    %end_arrow(:,:) = orgin_arrow(:,1:length(x_pose)) + force_in_R(:,1:length(x_pose)); % the end of the arraw is the force in robot frame 
   

%% Plotting the data 
figure(1)
%hold on
plot(cx_mat (w,:),cy_mat (w,:),'*')
hold on 
plot(sxdata(j) , sydata(j),'*')
hold on
plot (sxdata(j) , sydata(j) , 'o' )
hold on
plot([orgin_arrow(1,:),orgin_arrow(1,:)] , [orgin_arrow(2,:),orgin_arrow(2,:)],'r')

for l = 1 : mmmmm
orgin_arrow(1,l)
orgin_arrow(2,l)
%plot3(sxdata(aux(l)),sydata(aux(l)),s2time(aux(l)),'*')
 %plot([orgin_arrow(1,l),end_arrow(1,l)] , [orgin_arrow(2,l),end_arrow(2,l)],'r')
 %plot([orgin_arrow(1,l),end_arrow(1,l)] , [orgin_arrow(2,l),end_arrow(2,l)],'r')
%quiver(orgin_arrow(1,l),orgin_arrow(2,l),unit_vector(1),unit_vector(2),0.5)
end
 %plot([orgin_arrow(1,1:end),end_arrow(1,1:end)] , [orgin_arrow(2,1:end),end_arrow(2,1:end)],'r')

%% OVERLAYING THE MAP




%%%%%%%%%%%%%%
% FIRST WALL %
%%%%%%%%%%%%%%

wall_origin_x=0.0;
wall_origin_y=0.0;
wall_gazebo_offset_x=0.0;
wall_gazebo_offset_y=0.0;
wall_length=7.5;
thickness=0.2;
theta=90.0; %degrees
%convert to rad
theta_rad=theta * (pi/180.0);
% DRAW A WALL
%drawing_map(wall_origin_x+wall_gazebo_offset_x,wall_origin_y+wall_gazebo_offset_y,theta_rad, wall_length,thickness);



