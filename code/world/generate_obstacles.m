function obstacles = generate_obstacles(obstacle_num,min_obs_radius,search_range,start_point,goal)
    max_iterate=500;
    %创建圆形障碍物
    obstacles=[];
    % if min_world_range=250
    % x+r>0 => x>r
    % x+r<250 => x<250-r
    % x = rand*(250-r)+r
    iterate = 0;
    
    while 1
        r=rand*(min(search_range)/2);
        if r<min_obs_radius
            r=min_obs_radius;
        end
        
        
        xyz_edge=min(search_range)-r;
        x=rand*xyz_edge;
        y=rand*xyz_edge;
        z=rand*xyz_edge;
        point=[x,y,z];
        if ~will_collide(point,r,obstacles) && ~will_collide(start_point,0,obstacles) && ~will_collide(goal,0,obstacles)
 
            new_data=[x,y,z,r];
            obstacles=[obstacles;new_data];
            if length(obstacles(:,1))==obstacle_num
                break
            end
            
        end
        
        iterate = iterate+1;
        if iterate>max_iterate
           break
        end

    end
end

