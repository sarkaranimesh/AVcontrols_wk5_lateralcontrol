clc; clear all ; close all
%%
x= 15 ; y=24; r = 50;
hold on
th = 0:pi/50:2*pi;
xunit = r*cos(th) + x;
yunit = r*sin(th) + y;
h = plot(xunit,yunit);

xunit = (r+3.5)*cos(th) + x;
yunit = (r+3.5)*sin(th) + y;
h = plot(xunit,yunit);


xunit = (r)*cos(th) + x;
yunit = (r)*sin(th) + y + 3.5+2*r;
h = plot(xunit,yunit);


xunit = (r+3.5)*cos(th) + x;
yunit = (r+3.5)*sin(th) + y;
h = plot(xunit,yunit);

xunit = (r+3.5)*cos(th) + x;
yunit = (r+3.5)*sin(th) + y + 3.5+r*2;
h = plot(xunit,yunit);

%% waypoint generation
crc = @(r,p,a) [r*cosd(a + p) ;  r*sind(a + p)]; % r- radius a- time p  - initial position
% t = linspace(0, 360, 361);                              % Time
t = linspace(0 , 180 ,100); % quater time
waypoints = [];
% first quater for figure eight
init_pos = 271;                                          % Initial Position (°)
radius = r+1.75;
locus_1 = crc(radius, init_pos, t); % anit clokwise
b = locus_1(2,:)+y;
a = locus_1(1,:) +x;
locus_1 = [a ; b];

figure(1)
for k1 = 2:length(t)
    plot(locus_1(1,k1),  locus_1(2,k1), '*'); hold on
    grid
    axis square
    drawnow    
end
waypoints = [locus_1']; % 100x2

% second quater for figure eight

init_pos = 181;                                          % Initial Position (°)
locus_2 = crc(radius, init_pos, t);
locus_2 = [ locus_2(2,:)+ x; locus_2(1,:)+y+3.5+2*r ]';

% locus  = locus*[0 1;1 0]; % swaping sine and cos terms for clockwise
figure(1)
for k1 = 2:length(t)
    a  = locus_2(k1,:); % swaping sine and cos terms for clockwise
    plot(a(1),  a(2), '*'); hold on
    grid
    axis square
    drawnow    
end

waypoints = [locus_1'; locus_2]; % 200x2

% third quater for figure eight

init_pos = 0.5;                                          % Initial Position (°)
locus_3 = crc(radius, init_pos, t);
locus_3 = [ locus_3(2,:) + x; locus_3(1,:)+3.5+2*r + y]';
figure(1)
for k1 = 2:length(t)
    a  = locus_3(k1,:); % swaping sine and cos terms for clockwise
    plot(a(1),  a(2), '*'); hold on
    grid
    axis square
    drawnow    
end

waypoints = [locus_1'; locus_2; locus_3]; % 300x2

% fourth quater for figure eight
init_pos = 91;                                          % Initial Position (°)
locus_4 = crc(radius, init_pos, t); % anit clokwise
b = locus_4(2,:)+y;
a = locus_4(1,:) +x;
locus_4 = [a ; b];
figure(1)
for k1 = 2:length(t)
    plot(locus_4(1,k1),  locus_4(2,k1), '*'); hold on
    grid
    axis square
    drawnow    
end
waypoints = [locus_1'; locus_2; locus_3; locus_4']; % 400x2
z_points = zeros(400,1);
waypoints = [waypoints  z_points];
%% road centers 
% take evey 4 th point from the waypoints
n = size(waypoints)
road_centers = [];
for ii = 1:n(1)-4
    road_centers(ii,:) = waypoints(ii + 4,:)
end
 %%  
plot(road_centers(:,1),road_centers(:,2), 'o'); grid on
