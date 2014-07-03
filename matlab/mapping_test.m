close all

eps = 0.001;
aux = [] ;

%% Algorithm to make two arrays with the same size pased on the closest time
%start the loop with the smaller array
for i =1:length(stime)
    a = stime(i) - s2time ; % it will generate an array with the differencs between the elemnt and the largest array
    [c,j]=min(abs(a)); % find the index of the min 'j'
    
    if a(j)/1000000000 < eps % if the value of the min different  is less than eps 
        aux = [aux ; i,j]; % create 2D array with the indises from the small one and the large one ( the large one is j)  
    end 
end

%%

force_in_R=zeros(2, length(aux));
force_norm=zeros(2, length(aux));
orgin_arrow=zeros(2, length(aux));
end_arrow=zeros(2, length(aux));

%% Transform the forces from the world frame to the robot frame ( use quaterinian) using a pre defind function called quat to dec
% we have the quar data from the bag file 
%
for k =1: length(aux)
    q = [owdata(aux(k,2)) oxdata(aux(k,2)) oydata(aux(k,2)) ozdata(aux(k,2))]; % 2 == j which means the index for the data related to the state of the robot 
    dcm = quat2dcm (q/norm(q)); % call the function by sending the Quatr data over the norm it will generate 4 by 4 
    dcm2D = dcm(1:2,1:2); % we want only the data related to x and y  ????
    force_in_R(:,k) = dcm2D * [pxdata(aux(k,1)) ; pydata(aux(k,1))]; % for frame transformation  (1 == i) indies for the data related to the force 
    force_norm(:,k) = force_in_R(:,k)/norm(force_in_R(:,k)); % ??? 
    orgin_arrow(:,k) = [sxdata(aux(k,2)), sydata(aux(k,2))] ; % the start of the arraw is the position in robot frame  
    end_arrow(:,k) = orgin_arrow(:,k) + force_norm(:,k); % the end of the arraw is the force in robot frame 
end

%% Plotting the data 
figure(1)
hold on
plot(sxdata,sydata,'*')


for l = 1 : length(aux)
 %plot3(sxdata(aux(l)),sydata(aux(l)),s2time(aux(l)),'*')
 plot([orgin_arrow(1,l),end_arrow(1,l)] , [orgin_arrow(2,l),end_arrow(2,l)],'r')
end

%% OVERLAYING THE MAP


%%%%%%%%%%%%%%%%%%
% Starting point %
%%%%%%%%%%%%%%%%%%

starting_x=10.0;
starting_y=0.0;

plot(starting_x,starting_y,'ko','LineWidth',2,'markers',20);


%%%%%%%%%%%%%
% Goal Area %
%%%%%%%%%%%%%

starting_x=-12.0;
starting_y=0.0;

plot(starting_x,starting_y,'rx','LineWidth',2,'markers',20);
plot(starting_x,starting_y,'ro','LineWidth',2,'markers',20);

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
drawing_map(wall_origin_x+wall_gazebo_offset_x,wall_origin_y+wall_gazebo_offset_y,theta_rad, wall_length,thickness);


%%%%%%%%%%%%%%
% SECOND WALL %
%%%%%%%%%%%%%%

wall_origin_x=-10.0;
wall_origin_y=-2.5;
wall_gazebo_offset_x=0.0;
wall_gazebo_offset_y=0.0;
wall_length=7.5;
thickness=0.2;
theta=90.0; %degrees
%convert to rad
theta_rad=theta * (pi/180.0);
% DRAW A WALL
drawing_map(wall_origin_x+wall_gazebo_offset_x,wall_origin_y+wall_gazebo_offset_y,theta_rad, wall_length,thickness);

