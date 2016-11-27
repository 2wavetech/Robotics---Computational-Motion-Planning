function n = neighbors (G, I, J)
% find the neighbors of cell (I, J) on a 2D grid
% Input: G - the 2D grid presented as a matrix
%       (I, J) - the coordinate of the cell on the grid, measured from the top-left
%                corner as (1, 1), I = #row, J = #column
% Output: a list of linear positions of the neighbors on the grid
nrows = size(G, 1);
ncols = size(G, 2);
i = 1;
n = [];
if I - 1 > 0 && (G(I-1, J) == 1 || G(I-1, J) == 6)    % if the upper cell is still inside of the grid and it is a clear cell or destination
    n(i) = sub2ind(size(G), I-1, J);
    i = i+1;
end
if I + 1 <= nrows && (G(I+1, J) == 1 || G(I+1, J) == 6)    % if the lower cell is within the grid and it is a clear cell or destination
    n(i) = sub2ind(size(G), I+1, J);
    i = i+1;
end
if J - 1 > 0 && (G(I, J-1) == 1 || G(I, J-1) == 6)
    n(i) = sub2ind(size(G), I, J-1);
    i = i+1;
end
if J + 1 <= ncols && (G(I, J+1) == 1 || G(I, J+1) == 6)
    n(i) = sub2ind(size(G), I, J+1);
    i = i+1;
end
end 
