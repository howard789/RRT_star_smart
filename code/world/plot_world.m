function plot_world(obstacles,start_point,goal,path_RRTstar,path_RRTsmart,tree,treeS)
    %obstacle_centers 是障碍物的中心点 x y z r(radius)
    [x,y,z]=sphere(); %创造一个球面,中心点是0
    for i = 1:length(obstacles(:,1))
        %调整位置和大小
        X=x*obstacles(i,4)+obstacles(i,1);
        Y=y*obstacles(i,4)+obstacles(i,2);
        Z=z*obstacles(i,4)+obstacles(i,3);
        mesh(X,Y,Z);
        hold on;
    end
    scatter3(start_point(1),start_point(2),start_point(3),'filled','g');
    scatter3(goal(1),goal(2),goal(3),'filled','b');
    xlabel('x-axis'),ylabel('y-axis'),zlabel('z-axis');
    title('rrt*-smart');
    grid on;
    axis equal;
    
    %plot tree
    if size(tree,1)>1
       plot_tree(tree);
    end
    
    %plot best path
    
    if length(path_RRTstar)>0
        end_node=tree(size(tree,1),:);
        plot_path(end_node,tree,3,'b');
    end
    
    if length(path_RRTsmart)>0
        end_node=treeS(size(treeS,1),:);
        plot_path(end_node,treeS,3,'r');
    end
    
    
end


function plot_tree(tree)
    idx = size(tree,1);
    while idx>1        
        node = tree(idx,:);
        plot_path(node,tree,0.5,'g');
        idx = idx - 1;
    end
end


function plot_path(end_node,tree,line_width,line_color)
    %end_node
    branch = [end_node];
    parent_idx=end_node(4);
    while parent_idx>0
       node = tree(parent_idx,:);
       branch = [node;branch];
       parent_idx=node(4);
    end
    if size(branch,1)>1
        X = branch(:,1);
        Y = branch(:,2);
        Z = branch(:,3);
        p = plot3(X,Y,Z);
        set(p,'Color',line_color,'LineWidth',line_width,'Marker','.','MarkerEdgeColor',line_color);
        hold on;
    end
    
end


