function n = DiNeighbors (G, I, J)
% find the neighbors of cell (I, J) on a 2D grid for Dijkstra's Algorithm
% Input: G - the 2D grid presented as a matrix
%       (I, J) - the coordinate of the cell on the grid, measured from the top-left
%                corner as (1, 1), I = #row, J = #column
% Output: a list of linear positions of the neighbors on the grid
nrows = size(G, 1);
ncols = size(G, 2);
n = [];
i = 1;
if I - 1 > 0 && (G(I-1, J) ~= 2 && G(I-1, J) ~= 3 && G(I-1, J) ~= 5)         % if the upper cell is inside of the grid and it is not an obstacle and not the visited cell and not the start node 
    n(i) = sub2ind(size(G), I-1, J);
    i = i+1;
end
if I + 1 <= nrows && (G(I+1, J) ~= 2 && G(I+1, J) ~= 3 && G(I+1, J) ~= 5)    % if the lower cell is inside the grid and it is not ...
    n(i) = sub2ind(size(G), I+1, J);
    i = i+1;
end
if J - 1 > 0 && (G(I, J-1) ~= 2 && G(I, J-1) ~= 3 && G(I, J-1) ~= 5)
    n(i) = sub2ind(size(G), I, J-1);
    i = i+1;
end
if J + 1 <= ncols && (G(I, J+1) ~= 2 && G(I, J+1) ~= 3 && G(I, J+1) ~= 5)
    n(i) = sub2ind(size(G), I, J+1);
    i = i+1;
end
end 
