
clc;clear all;close all;
addpath(genpath('utils'),genpath('world'))
fixed_env=1;

%% set search range 
search_range=[250 250 250];

%% set start_point and goal

start_point=[10 10 10];
goal=[150 150 150];

%% create world with obstacle
if fixed_env
    
    [obstacles,min_obs_radius] =  general_fixed_obstacles();
    
else
    obstacle_num=5;
    min_obs_radius=30;
    obstacles = generate_obstacles(obstacle_num,min_obs_radius,search_range,start_point,goal);
end
%% find best path with rrt*-smart
step_length=10;
max_fail_attemps=500;
target_path_num=1;
[path_RRTstar,path_RRTsmart,tree,treeS] = rrt_start_smart(start_point,goal,search_range,obstacles,min_obs_radius,step_length,max_fail_attemps,target_path_num);

%% plot world
plot_world(obstacles,start_point,goal,path_RRTstar,path_RRTsmart,tree,treeS);











