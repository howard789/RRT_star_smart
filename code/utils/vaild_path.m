function valid = vaild_path(point1,point2,obstacles,min_obs_radius)
    valid = 1;
    num = floor(norm(point1-point2)/(min_obs_radius/2));
    x_step = (point2(1)-point1(1))/num;
    y_step = (point2(2)-point1(2))/num;
    z_step = (point2(3)-point1(3))/num;
    done = 0;
    for i = 1:num
        if ~done
            point = [point1(1)+x_step*i point1(2)+y_step*i point1(3)+z_step*i] ;
            if will_collide(point,0,obstacles)
                valid=0;
                done =1;
            end
        end
     end
        
end