function [x1] = drawing_map( x,y,th, length, width)



line_x=[x-(length/2.0)*cos(th) , x+(length/2.0)*cos(th)]
line_y = [y-(length/2.0)*sin(th) , y+(length/2.0)*sin(th)]
plot(line_x+width/2.0,line_y,'b');
plot(line_x-width/2.0,line_y,'b');

line_x =[x-(width/2.0)*cos(th+(pi/2.0)) , x+(width/2.0)*cos(th+(pi/2.0))]
line_y = [y-(width/2.0)*sin(th+(pi/2.0)) , y+(width/2.0)*sin(th+(pi/2.0))]
plot(line_x,line_y+length/2.0,'b');
plot(line_x,line_y-length/2.0,'b');

end

