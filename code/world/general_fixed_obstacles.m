function  [obstacles,min_obs_radius] =  general_fixed_obstacles()
    %centers = [100,100,100;50,50,50;100,40,60;150,100,100;60,130,50];
    %r=[50;20;20;15;15];%°ë¾¶
    centers =[125,125,50;];
    r=[100];
    obstacles=zeros(length(r),4);
    for i = 1:length(r)
        obstacles(i,:)=[centers(i,:) r(i)];
    end
    min_obs_radius = min(r);
end

