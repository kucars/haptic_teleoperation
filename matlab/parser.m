clear rosbag_wrapper;
clear ros.Bag;
clear all
close all


epsilon=0.0001;
starting_sample=2;


%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
%bag = ros.Bag.load('example.bag');
addpath ~/Downloads/matlab_rosbag-0.4-linux64/
%bag = ros.Bag.load('/home/kuri/Desktop/testing_PF/Ground/sim_2w_Sp.bag');
bag = ros.Bag.load('/home/kuri/Desktop/results/24_sept/2014-09-24-14-13-15.bag');


%bag = ros.Bag.load('/home/kuri/Desktop/results//obj1/bag_files/testing_the_marekers.bag');
%bag = ros.Bag.load('/home/kuri/Desktop/results/new_results/5_9_2014/2014-09-06-14-37-45.bag');
bag.info()

%% Read all messages on a few topics
topic1 = '/haptic_teleoperation/pf_force_feedback';
%topic2= '/Pioneer3AT/pose' ;
topic2 = '/ground_truth/state';
topic3 = '/haptic_teleoperation/cloud';
topic4 = '/haptic_teleoperation/haptic_position_pub';
topic5 = '/haptic_teleoperation/force_field_markers';
topic6 = '/husky_ns/odom';

% 
% msgs = bag.readAll({topic5});
% 
%fprintf('Read %i messages\n', length(msgs));

%% Re-read msgs on topic1 and get their metadata
%[msgs, meta] = bag.readAll(topic2);
%fprintf('Got %i messages, first one at time %f\n', length(msgs), meta{1}.time.time);

bag.resetView(topic1);
stime = [] ;
pxdata = [] ;
pydata = [] ;
pzdata = [] ;
i=0;
while bag.hasNext();
    
    [msg, meta] = bag.read();
    
    stime = [ stime msg.header.stamp.time] ;
    pxdata = [ pxdata msg.pose.position(1)] ;
    pydata = [ pydata msg.pose.position(2)] ;
    pzdata = [ pzdata msg.pose.position(3)] ;
end


bag.resetView(topic2);
s2time = [] ;
sxdata = [] ;
sydata = [] ;
szdata = [] ;
oxdata = [] ;
oydata = [] ;
ozdata = [] ;
owdata = [];
Txdata = [] ;
Tydata = [] ;
Tzdata = [] ;
i=0;
while bag.hasNext();
    i=i+1;
    [msg, meta] = bag.read();
    if i<starting_sample
        continue
    end
    s2time = [ s2time msg.header.stamp.time] ;
    sxdata = [ sxdata msg.pose.pose.position(1)] ;
    sydata = [ sydata msg.pose.pose.position(2)] ;
    szdata = [ szdata msg.pose.pose.position(3)] ;
    oxdata = [ oxdata msg.pose.pose.orientation(1)] ;
    oydata = [ oydata msg.pose.pose.orientation(2)] ;
    ozdata = [ ozdata msg.pose.pose.orientation(3)] ;
    owdata = [ owdata msg.pose.pose.orientation(4)] ;
    Txdata = [ Txdata msg.twist.twist.linear(1)] ;
    Tydata = [ Tydata msg.twist.twist.linear(2)] ;
    Tzdata = [ Tzdata msg.twist.twist.linear(3)] ;
    
end
%%

bag_topic_3 = bag.readAll({topic3});
nofcells = length(bag_topic_3);
c=0;
for i = 1:nofcells
    
    c_ = length(bag_topic_3{1,i}.points(1,:));
    
    if (c_>c)
        c=c_;
    end
end
cx_mat = zeros(nofcells,c);
cy_mat = zeros(nofcells,c);
cz_mat = zeros(nofcells,c);
s3time = zeros; 

for i = 1:nofcells
    s3time(i) = bag_topic_3{1,i}.header.stamp.time;
    LengOfCol = length (bag_topic_3{1,i}.points(1,:));
    cx_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(1,:);
    cy_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(2,:);
    cz_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(3,:);
    
end
%%
bag.resetView(topic4);
s4time = [] ;
hxdata = [] ;
hydata = [] ;
hzdata = [] ;

i=0;
while bag.hasNext();
    i=i+1;
    [msg, meta] = bag.read();
    if i<starting_sample
        continue
    end
    s4time = [ s4time msg.header.stamp.time] ;
    hxdata = [ hxdata msg.pose.pose.position(1)] ;
    hydata = [ hydata msg.pose.pose.position(2)] ;
    hzdata = [ hzdata msg.pose.pose.position(3)] ;
end

%%
bag.resetView(topic6);
s6time = [] ;
husky_pose_x = [] ;
husky_pose_y = [] ;
husky_pose_z = [] ;

i=0;
while bag.hasNext();
    i=i+1;
    [msg, meta] = bag.read();
    if i<starting_sample
        continue
    end
    s6time = [ s6time msg.header.stamp.time] ;
    husky_pose_x = [ husky_pose_x msg.pose.pose.position(1)] ;
    husky_pose_y = [ husky_pose_y msg.pose.pose.position(2)] ;
    husky_pose_z = [ husky_pose_z msg.pose.pose.position(3)] ;
end
%% markeres

% bag.resetView(topic5);
% s5time = [] ;
% mxdata = [] ;
% mydata = [] ;
% mzdata = [] ;
% moxdata = [] ;
% moydata = [] ;
% mozdata = [] ;
% mowdata = [] ;
%
% i=0;
% while bag.hasNext();
%     i=i+1;
%     %    s5time = [ s5time msg.header.stamp.time] ;
%     for i=1:100
%         mxdata = [ mxdata msg.markers.pose.position(1)] ;
%         mydata = [ mydata msg.markers.pose.position(2)] ;
%         mzdata = [ mzdata msg.markers.pose.position(3)] ;
%         moxdata = [ moxdata msg.markers.pose.orientation(1)] ;
%         moydata = [ moydata msg.markers.pose.orientation(2)] ;
%         mozdata = [ mozdata msg.markers.pose.orientation(3)] ;
%         mowdata = [ mowdata msg.markers.pose.orientation(4)] ;
%     end
% end

%

%% remove the data that far away from the real data 
nsxdata = [] ;
nsydata = [] ;
nszdata = [] ;
nxodata = [] ;
nyodata = [] ;
nzodata = [] ;
ns2time = [] ;
for i=2:length(sxdata)
    if sqrt((sxdata(i) - sxdata(i-1))^2 + (sydata(i) - sydata(i-1))^2 ) <= epsilon
        continue ;
    else
        nsxdata = [nsxdata sxdata(i)];
        nsydata = [nsydata sydata(i)];
        nszdata = [nszdata szdata(i)];
        nxodata = [nxodata oxdata(i)];
        nyodata = [nyodata oydata(i)];
        nzodata = [nzodata ozdata(i)];
        ns2time = [ns2time s2time(i)];
        
    end
    
end

sxdata = nsxdata ;
sydata = nsydata ;
szdata = nszdata ;
oxdata = nxodata ;
oydata = nyodata ;
ozdata = nzodata ;
s2time = ns2time ;



%% See definitions of messages contained within the bag
% twist_definition = bag.definition('geometry_msgs/Twist')
%
% % Setting 'raw' to true shows the original message definition with comments
% raw = true;
% raw_twist_definition = bag.definition('geometry_msgs/Twist', raw)
%
% % When it's unambiguous, you can drop the package name.  You can also get
% % definitions for messages defined in other messages;
% % geometry_msgs/Quaternion comes from geometry_msgs/Twist
% quaternion_definition = bag.definition('Quaternion', true)
%
% %% Convert velocity messages to a matrix to plot linear speed
% [msgs, meta] = bag.readAll(topic1); % Messages are structs
% accessor = @(twist) twist.linear;
% [xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear velcoity
% times = cellfun(@(x) x.time.time, meta); % Get timestamps
% % Plot linear speed over time
% plot(times, xyz(1, :));
% ylim([-2.5 2.5]);
%
% %% Learn more
% doc ros.Bag
