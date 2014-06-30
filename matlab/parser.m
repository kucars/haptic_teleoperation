clear rosbag_wrapper;
clear ros.Bag;
clear all

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
%bag = ros.Bag.load('example.bag');
bag = ros.Bag.load('/home/kuri/Desktop/testing_PF/pf_sim_two_wall.bag');
bag.info()

%% Read all messages on a few topics
topic1 = '/haptic_teleoperation/pf_force_feedback';
topic2 = '/ground_truth/state';
topic3 = '/haptic_teleoperation/cloud';
topic4 = '/haptic_teleoperation/haptic_position_pub';
msgs = bag.readAll({topic1, topic2});

%fprintf('Read %i messages\n', length(msgs));

%% Re-read msgs on topic1 and get their metadata
%[msgs, meta] = bag.readAll(topic1);
%fprintf('Got %i messages, first one at time %f\n', length(msgs), meta{1}.time.time);

%% Read messages incrementally
% bag.resetView(topic1);
% count = 0;
% while bag.hasNext();
%     [msg, meta] = bag.read();
%     count = count + 1;
% end

bag.resetView(topic1);
count = 0;
stime = [] ;
pxdata = [] ;
pydata = [] ;
pzdata = [] ;
while bag.hasNext();
[msg, meta] = bag.read();
count = count + 1;
stime = [ stime msg.header.stamp.time] ;
pxdata = [ pxdata msg.pose.position(1)] ;
pydata = [ pydata msg.pose.position(2)] ;
pzdata = [ pzdata msg.pose.position(3)] ;
end


bag.resetView(topic2);
count = 0;
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
while bag.hasNext();
[msg, meta] = bag.read();
count = count + 1;
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

bag.resetView(topic3);
count = 0;
s3time = [] ;
cxdata = [] ;
cydata = [] ;
czdata = [] ;

while bag.hasNext();
[msg, meta] = bag.read();
count = count + 1;
s3time = [ s3time msg.header.stamp.time] ;
cxdata = [ cxdata msg.points(1)] ;
cydata = [ cydata msg.points(2)] ;
czdata = [ czdata msg.points(3)] ;
end

%%

bag.resetView(topic4);
count = 0;
s4time = [] ;
hxdata = [] ;
hydata = [] ;
hzdata = [] ;

while bag.hasNext();
[msg, meta] = bag.read();
count = count + 1;
s4time = [ s4time msg.header.stamp.time] ;
hxdata = [ hxdata msg.pose.pose.position(1)] ;
hydata = [ hydata msg.pose.pose.position(2)] ;
hzdata = [ hzdata msg.pose.pose.position(3)] ;
end


nsxdata = [] ;
nsydata = [] ;
nszdata = [] ;
noxdata = [] ;
noydata = [] ;
nozdata = [] ;
ns2time = [] ;



for i=2:length(sxdata)
	if (sqrt(sxdata(i)-sxdata(i-1))^2 + (sydata(i) -sydata(i-1)^2) <= epsilon
		continue 
	else
		nsxdata = [ nsxdata sxdata] ;
		nsydata = [ nsydata sydata] ;
		nszdata = [ nszdata szdata] ;
		noxdata = [ noxdata oxdata] ;
		noydata = [ noydata oydata] ;
		nozdata = [ nozdata ozdata] ;
		ns2time = [ ns2time s2time] ;
	end
end 

sxdata = nsxdata;
sydata = nsydata;
szdata = nszdata;
oxdata = noxdata;
oydata = nozdata;
ozdata = nozdata;
s2time = ns2time;


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
