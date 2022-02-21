function result = will_collide(point,r,obstacles)
    
    
    if r>0
        test_num=19;
        points = [];
        % generate a boll 创建一个以point为中心的球体
        [x,y,z]=sphere(test_num);
        for i = 1:length(x(1,:))
             for j = 1:length(x(:,1))
                 new_point=[x(i,j)*r+point(1) y(i,j)*r+point(2) z(i,j)*r+point(3)];
                 points= [points;new_point];
             end
        end
    else
        points=point;
    end

    result = 0;
    if ~isempty(obstacles)
        for obstacle_idx =1:length(obstacles(:,1))
            obstacle = obstacles(obstacle_idx,:);
            for row = 1:length(points(:,1))
                point=points(row,:);  
                distance_to_center = norm(obstacle(1:3)-point);
                if distance_to_center <= obstacle(4)
                    result = 1;
                    break;
                end   
            end
        end
    end
end
