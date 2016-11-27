%
% TestScript for Assignment 1
%

%% Define a small map
input_map = false(10);

% Add an obstacle
input_map (1:5, 6) = true;
input_map (4:10, 3) = true;
input_map (7:10, 5) = true;

start_coords = [2, 2];
dest_coords  = [8, 9];

%%
close all;
[route, numExpanded] = DijkstraGrid (input_map, start_coords, dest_coords);
% Uncomment following line to run Astar
pause();
[route, numExpanded] = AStarGrid (input_map, start_coords, dest_coords);
