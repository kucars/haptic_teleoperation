x = [1, 0 , 0 ] 
y = [ 0 , 1, 0 ] 
arrow(x,y) ; 

ind = [] ; 
for i = 1 : length(stime)
    for j = 1 : length(s2time)
        if (  stime(i)==s2time(j)) 
            ind = [ ind i ] ;

        end
    end
    
        
end


newsxdata = sxdata(ind)
newsydata = sydata(ind)
newpxdata = pxdata(ind)
newpydata = pydata(ind)

sxy = [ newsxdata ; newsydata]
pxy = [newpxdata ; newpydata] 
arrow( sxy , pxy ) 

eps = 0.00001;
aux = [] ;
for i =1000 : 1100
    j=0
    a = stime(i) - s2time ;
    [c,j]=min(a) ;
    if a(j) < eps 
    aux = [aux ; i,j]; 
    end 
end
