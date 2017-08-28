%
% PotentialFieldScript.m
%

%% Generate some points

nrows = 400;
ncols = 600;

obstacle = false(nrows, ncols);

[x, y] = meshgrid (1:ncols, 1:nrows);   % note that x refers to the horizental axis (1:ncols), y to vertical (1:nrows)
% [X,Y] = meshgrid(xgv,ygv) replicates the grid vectors xgv and ygv to
%     produce the coordinates of a rectangular grid (X, Y). The grid vector
%     xgv is replicated numel(ygv) times to form the rows of X. The grid
%     vector ygv is replicated numel(xgv) times to form the columns of Y.
%% Generate some obstacle

obstacle (300:end, 100:250) = true;             % rectangular
obstacle (150:200, 400:500) = true;             % rectangular

t = ((x - 200).^2 + (y - 50).^2) < 50^2;        % a circle centered at (200, 50), radius = 50
obstacle(t) = true;

t = ((x - 400).^2 + (y - 300).^2) < 100^2;      % a circle, (400, 300), r=100
obstacle(t) = true;

%% Compute distance transform

d = bwdist(obstacle);
%  D = bwdist(BW) computes the Euclidean distance transform of the
%  binary image BW. For each pixel in BW, the distance transform assigns
%  a number that is the distance between that pixel and the nearest
%  nonzero pixel of BW. bwdist uses the Euclidean distance metric by
%  default.  BW can have any dimension.  D is the same size as BW.

% Rescale and transform distances
d2 = (d/100) + 1;

d0 = 2;
nu = 800;

repulsive = nu*((1./d2 - 1/d0).^2); % every point (x, y) on 2D space has a repulsive value. repulsive is a matrix.

repulsive (d2 > d0) = 0;            % d0 is like a threshold and repulsive force is 0 when distance from obstacle < d0


%% Display repulsive potential

figure;

% mesh(Z) and mesh(Z,C) use x = 1:n and y = 1:m.  In this case, the height,
% Z, is a single-valued function, defined over a geometrically rectangular
% grid.
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;

title ('Repulsive Potential');

%% Compute attractive force

goal = [400, 50];

xi = 1/700;

attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );  % every point on 2D space has an attractive force value

figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;

title ('Attractive Potential');

%% Display 2D configuration space

figure;
imshow(~obstacle);
%  imshow(BW) displays the binary image BW. imshow displays pixels with the
%  value 0 (zero) as black and pixels with the value 1 as white.
hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold off;

axis ([0 ncols 0 nrows]);
axis xy;
axis on;

xlabel ('x');
ylabel ('y');

title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;

figure;
m = mesh (f);
m.FaceLighting = 'phong';
axis equal;

title ('Total Potential');

%% Plan route
start = [50, 350];

route = GradientBasedPlanner (f, start, goal, 1000);

%% Plot the energy surface

figure;
m = mesh (f);
axis equal;

%% Plot ball sliding down hill

[sx, sy, sz] = sphere(20);

scale = 20;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
p = mesh(sx, sy, sz);
p.FaceColor = 'red';
p.EdgeColor = 'none';
p.FaceLighting = 'phong';
hold off;

for i = 1:size(route,1)
    P = round(route(i,:));
    z = f(P(2), P(1));
    
    p.XData = sx + P(1);
    p.YData = sy + P(2);
    p.ZData = sz + f(P(2), P(1));
    
    drawnow;
    
    drawnow;
    
end

%% quiver plot
[gx, gy] = gradient (-f);
skip = 20;

figure;

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 ncols 1 nrows]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
