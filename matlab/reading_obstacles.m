
nofcells = length(bag_topic_3);
c=0;
for i = 1:nofcells
    
    c_ = length(bag_topic_3{1,i}.points(1,:));

    if (c_>c)
        c=c_;
    end
end

x_mat = zeros(nofcells,c);
y_mat = zeros(nofcells,c);
z_mat = zeros(nofcells,c);

s3time = zeros; 

for i = 1:nofcells
s3time(i) = bag_topic_3{1,i}.header.stamp.time; 

LengOfCol = length (bag_topic_3{1,i}.points(1,:));
x_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(1,:);
y_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(2,:);
z_mat(i,1:LengOfCol) = bag_topic_3{1,i}.points(3,:);

end