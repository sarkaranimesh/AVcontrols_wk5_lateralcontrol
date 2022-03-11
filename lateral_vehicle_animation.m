clc; clear all; close all;
%%
sim_time = 10;
sample_time = 0.01;
time = 0:sample_time:sim_time;
sim_options = ["one lane", "3-lane","reference track"];
vehicle_no = ['1','2','3'];
number_vehicles = vehicle_no(1);
plot_mode = sim_options(1);
fig_trajectory_result = figure(1);
rad2deg = 180 / pi;
deg2rad = pi / 180;
kmh2ms = 1000 / 3600;
%% import data
fig_trajectory_result; hold on;
path = lateral_path;
n = size(path);
for ii= 1: n(2)
    ref_x(ii) = path(ii).ActorPoses.Position(1);
    ref_y(ii) = path(ii).ActorPoses.Position(2);
    ref_yaw(ii) = path(ii).ActorPoses.Yaw;
end
x_pos = ref_x + 0.1;
y_pos = ref_y - 0.1;
yaw_pos = ref_yaw + 2;

%% create reference road lines
road_end1 = [];
road_end2 = [];

for ii = 1:length(ref_x)-1
    A = [ref_x(ii) ref_y(ii)]; %[x y] for first point in slope
    B = [ref_x(ii+1) ref_y(ii+1)];%[x y] for second point in slope
    track = 3.16; % width of the road
    
    % do the math
    slope = (B(2)-A(2)) / (B(1)-A(1)); % slope of line AB
    yinit = B(2) - slope*B(1); % line equation for AB
    
    % get perpendicular line at A and its equation
    perpslope = -1/slope
        if perpslope == -Inf
            perpslope = 5729
        else
            perpslope = -1/slope
        end
    
    perp_yinit = A(2) - perpslope*A(1);
%     perpslope = abs(perpslope);


    % road end points on the perpendicular line
    if perpslope > 0
        x = A(1) + (track*sqrt(1/(1+perpslope^2)))*[-1,1]; 
        y= A(2) + (perpslope*track*sqrt(1/(1+perpslope^2)))*[-1,1];
    else
        x = A(1) + (track*sqrt(1/(1+perpslope^2)))*[1,-1];
        y= A(2) + (perpslope*track*sqrt(1/(1+perpslope^2)))*[1,-1];        
    end 
    
    road_end1(ii,:) = [x(1) y(1)];
    road_end2(ii,:) = [x(2) y(2)];
end

% for last reference point
    if perpslope > 0
        x = B(1) + (track*sqrt(1/(1+perpslope^2)))*[-1,1]; 
        y= B(2) + (perpslope*track*sqrt(1/(1+perpslope^2)))*[-1,1];
    else
        x = B(1) + (track*sqrt(1/(1+perpslope^2)))*[1,-1];
        y= B(2) + (perpslope*track*sqrt(1/(1+perpslope^2)))*[1,-1];        
    end 
road_end1(ii+1,:) = [x(1) y(1)];
road_end2(ii+1,:) = [x(2) y(2)];
%% plot the reference path and road lines
fig_trajectory_result; hold on;
% plot(ref_x,ref_y,'*');
plot(road_end1(:,1), road_end1(:,2));plot(road_end2(:,1), road_end2(:,2));
%% Plot the car motion
z_axis = [0 0 1];
fig_trajectory_result; hold on;
body =[];
for ii = 1: length(x_pos)

L = 3;
rear_x = x_pos(ii);
rear_y = y_pos(ii);
yaw = yaw_pos(ii);
rear_length = 1.6;
front_length = 1.6;
side_width = 1;
front_x = rear_x + L;
front_y = rear_y;
delete(body);

plot(x_pos(ii),y_pos(ii),'Linewidth',1.5);
drawnow;
 
body = plot([rear_x-rear_length, front_x+front_length, front_x+front_length, rear_x-rear_length, rear_x-rear_length], ...
    [rear_y-side_width, front_y-side_width, front_y+side_width, rear_y+side_width, rear_y-side_width],'k','Linewidth',2);
rear_origin = [rear_x, rear_y, 0];
front_origin = [rear_x + L*cos(yaw), rear_y + L*sin(yaw), 0];
rotate(body, z_axis, yaw , rear_origin);

drawnow;

pause(0.1)
end