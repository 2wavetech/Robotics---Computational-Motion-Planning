%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SideOfLine computes the side of a point C on the line decided by the
% point A and B. It returns 2 if C is on the line, 1 if C is above or right
% to the line, and 0 otherwise. A, B, C has the form of (x, y).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function side = SideOfLine (A, B, C)
    if A(1) == B(1)             % this is a verticle line
        if C(1) == A(1)
            side = 2;           % C is on the line
        else
            if C(1) > A(1)
                side = 1;       % C is on the right of the line
            else
                side = 0;       % C is on the left of the line
            end
        end
    else                        % if the line is not verticle
        Y = (A(2) - B(2))/(A(1) - B(1))*C(1) + (A(1)*B(2) - A(2)*B(1))/(A(1) - B(1));
        if C(2) == Y
            side = 2;           % C is on the line
        else
            if C(2) > Y
                side = 1;       % C is above the line
            else
                side = 0;       % C is under the line
            end
        end
    end
end