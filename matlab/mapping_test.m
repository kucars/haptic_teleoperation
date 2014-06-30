eps = 0.001;
aux = [] ;

for i =1:length(stime)
    a = stime(i) - s2time ;
    [c,j]=min(abs(a));
    
    if a(j)/1000000000 < eps
        aux = [aux ; i,j]; 
    end 
end
force_in_R=zeros(2, length(aux));
force_norm=zeros(2, length(aux));
arrow_vector=zeros(4,length(aux));
 orgin_arrow=zeros(2, length(aux));
 end_arrow=zeros(2, length(aux));

for k =1: length(aux)
    q = [ owdata(aux(k,2)) oxdata(aux(k,2)) oydata(aux(k,2)) ozdata(aux(k,2))];
   
    dcm = quat2dcm (q/norm(q)); 
    dcm2D = dcm(1:2,1:2);
    force_in_R(:,k) = dcm2D * [pxdata(aux(k,1)) ; pydata(aux(k,1))];
    force_norm(:,k) = force_in_R(:,k)/norm(force_in_R(:,k));
    orgin_arrow(:,k) = [sxdata(aux(k,2)), sydata(aux(k,2))] ;
    end_arrow(:,k) = orgin_arrow(:,k) + force_norm(:,k);
end

figure(1)
hold on
 plot(sxdata,sydata,'*')

for l = 1 : length(aux)
 %plot3(sxdata(aux(l)),sydata(aux(l)),s2time(aux(l)),'*')
 plot([orgin_arrow(1,l),end_arrow(1,l)] , [orgin_arrow(2,l),end_arrow(2,l)],'r')
end