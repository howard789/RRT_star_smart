function [path_RRTstar,path_RRTsmart,tree,treeS] = rrt_start_smart(start_point,goal,search_range,obstacles,min_obs_radius,step_length,max_fail_attemps,target_path_num)
     
     % (1) x  
     % (2) y
     % (3) z
     % (4) parent_idx
     % (5) total_distance_to_start_node
     % (6) is_lastest_node (one step before before_goal)
     % (7) is_via_node
     % (8) id 
     
     % a point contains only (x,y,z) 
     %a node contains all data including a point
     
     start_node=[start_point 0 0 0 0 1];
     end_node=[goal -1 -1 0 0 -1];
     tree =[start_node];
     if norm(start_point-goal)<=step_length
         if link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
            path=[1;2];
         else
             path = [];
         end
     else
         fail_attemps = 0;
         path_num=0;
         % extend tree
         while 1

             [valid,is_last,tree]=extend_tree(tree,goal,min_obs_radius,obstacles,step_length,search_range);
             if is_last
                 path_num = path_num+1;             
             end
             
             if valid
                 fail_attemps =0;
             else
                fail_attemps = fail_attemps+1;      
             end

             %end
             if fail_attemps > max_fail_attemps
                 break
             end

             if path_num >= target_path_num
                %update end_node id
                 end_node(8)=size(tree,1)+1;
                 tree=[tree;end_node];
                 break
             end
         end

         % update_end_node_parent
         tree = update_end_node_parent (tree);
         path_RRTstar = get_path_from_tree(tree);
         

         
         % straighten best path
         treeS=tree(:,:);
         treeS = straighten_path(treeS,obstacles,min_obs_radius);
         path_RRTsmart = get_path_from_tree(treeS);

     end
end


function [valid,is_last,tree]=extend_tree(tree,goal,min_obs_radius,obstacles,step_length,search_range)
    is_last=0;

    % find new point and check whether it is valid
    [valid,new_point,parent_idx,min_total_dist] = find_new_point(goal,tree,obstacles,min_obs_radius,step_length,search_range);

    % update its parent and near points' (major difference between rrt* and rrt)
    if valid 
        [parent_idx,min_total_dist,near_points] = find_better_parent_idx(new_point,min_total_dist,parent_idx,min_obs_radius,obstacles,step_length,tree);
    end
    
    % add new point to the tree if valid
    if valid
        [new_node_idx,is_last,tree] = add_node(new_point,parent_idx,min_total_dist,tree,goal,step_length);
    end
    
    if valid
        tree = update_new_node_parent_idx(tree,new_node_idx,step_length,min_total_dist,parent_idx,min_obs_radius,obstacles,near_points);
    end
    

    
    
    
end

function [valid,new_point,parent_idx,min_total_dist] = find_new_point(goal,tree,obstacles,min_obs_radius,step_length,search_range)
    %random point
     if rand <0.5
        sample_point = rand(1,3).*search_range;
     else
        sample_point =goal;
     end

    %find the node closest to new_point_tmp
     distance = cal_leafs_to_point_distance(tree,sample_point);
     [~,parent_idx]=min(distance);
     closest_point = tree(parent_idx,1:3);
     
     % get new_point with step_length
     moving_directions = [sample_point(1)-closest_point(1) sample_point(2)-closest_point(2) sample_point(3)-closest_point(3)];
     moving_directions = moving_directions/sqrt(sum(moving_directions.^2));
     new_point = closest_point + step_length*moving_directions;
     new_point_line_dist = norm(new_point-closest_point);
     
     
     if new_point_valid(new_point,obstacles,search_range) && link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
         min_total_dist=tree(parent_idx,5)+new_point_line_dist;
         valid=1;
     else
         min_total_dist=0;
         valid=0;
     end

end


function [parent_idx,min_total_dist,near_points] = find_better_parent_idx(new_point,min_total_dist,parent_idx,min_obs_radius,obstacles,step_length,tree)
    %find near points
    search_radius=step_length*2;
    
    dist_mat = cal_leafs_to_point_distance(tree,new_point);
    near_points = find(dist_mat <= search_radius);
    
    for i = 1:length(near_points)
        idx = near_points(i);
        
        leaf_point = tree(idx,1:3);
        total_dist = tree(idx,5) + norm(leaf_point-new_point);
       if total_dist<min_total_dist && link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
           min_total_dist=total_dist;
           parent_idx=idx;
       end
        
    end  


end



function tree=update_new_node_parent_idx(tree,new_node_idx,step_length,min_total_dist,parent_idx,min_obs_radius,obstacles,near_points)
    % nodes close to the new node could get a shorter path with new node as its parent
    total_dist_of_new_point=tree(new_node_idx,5);
    new_point = tree(new_node_idx,1:3);
    for i = 1:length(near_points)
        idx = near_points(i);
        near_point = tree(idx,1:3);
        pre_total_dist = tree(idx,5);
        new_total_dist = norm(near_point-new_point)+total_dist_of_new_point;
        
       if new_total_dist<pre_total_dist && link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
           tree(idx,4)=new_node_idx; %parent
           tree(idx,5)=new_total_dist; %total_dist
           tree(idx,6)=0; % it is not the lastest anymore if it was
       end
    end

end

function [idx,is_last,tree] = add_node(new_point,parent_idx,min_total_dist,tree,goal,step_length)
  %add new node to tree
    if found_last_node_before_goal(new_point,goal,step_length)
        is_last = 1;
    else
        is_last =0;
    end
    id = size(tree,1)+1;
    new_node = [new_point parent_idx min_total_dist is_last 0 id];
    tree = [tree;new_node];  
    idx=size(tree,1);
end




function valid = new_point_valid(new_point,obstacles,search_range)
    valid = 1;
    done = 0;
    
    if valid && out_of_range(new_point,search_range)
        valid=0;
        done =1;
    end
    
    if valid && will_collide(new_point,0,obstacles)
        valid=0;
        done =1;
    end
end
    
    
function valid = link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
    done =0;
    valid =1;

  parent_point = tree(parent_idx,1:3);     
  if ~done && ~vaild_path(new_point,parent_point,obstacles,min_obs_radius)
        valid=0;             
        done =1;
  end
  
   
end




function distance = cal_leafs_to_point_distance(tree,point)
    a = tree(:,1:3);
    b = ones(size(tree,1),1)*point;
    diff = a-b;
    sqrt_diff = diff.*diff;
    sum=zeros(size(sqrt_diff,1),1);
    
    for i =1:3
        sum=sum+sqrt_diff(:,i);
    end
    
    distance=sum.^0.5;
   
end

function result = found_last_node_before_goal(new_point,goal,step_length)
    distance = norm(new_point-goal);
    if distance <= step_length
        result = 1;
    else
        result = 0;
    end
end

function tree = update_end_node_parent (tree)
    candidates=[];
    for idx = 1:size(tree,1)
        if tree(idx,6)==1
            candidates = [candidates;tree(idx,:)];
        end
    end
    
    if size(candidates,1)>0    
        %find the last_node (not goal)  with min cost
        [~,idx]=min(candidates(:,5),[],2);
        winner_id=candidates(idx,8);
        
        %update end_node data
        end_node_id=size(tree,1);
        tree(end_node_id,4) = winner_id;
        tree(end_node_id,5) = norm(tree(end_node_id,1:3)-tree(winner_id,1:3)) + tree(winner_id,5);
    end
end

function path = get_path_from_tree(tree)
    end_node_idx=size(tree,1);
    %put last_node to path
        path=[end_node_idx];
        parent_idx=tree(end_node_idx,4);
        % add points
        while parent_idx > 0
            path=[parent_idx;path];
            parent_idx=tree(parent_idx,4);
        end

end


function tree = straighten_path(tree,obstacles,min_obs_radius)

    end_node_idx=size(tree,1);

    while 1
        jump_step=2;
        
        current_idx=end_node_idx;
        success_num=0;
        while 1 
           parent_idx = get_parent_idx(tree,current_idx,jump_step);
           if parent_idx ==0
                break;
           end 
           
           new_point=tree(current_idx,1:3);
           if link_points_valid(new_point,parent_idx,tree,obstacles,min_obs_radius)
               tree(current_idx,4)=parent_idx;
               current_idx=parent_idx;
               jump_step=jump_step*2;
               success_num=success_num+1;
           elseif jump_step>2
               jump_step=max(floor(jump_step/2),2);
           else
               current_idx = tree(current_idx,4);
           end

        end
        
        if success_num ==0
            break;
        end
        
    end
    
    
    



end


function idx = get_parent_idx(tree,idx,jump_step)
    if idx > 0
        for i = 1:jump_step
           idx= tree(idx,4);
           if idx ==0
               break;
           end
        end
    end
    
end