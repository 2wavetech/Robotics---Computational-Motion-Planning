function out = LocalPlannerSixLink (x1, x2, obstacle)
% Generate a random freespace configuration for the robot x1 and x2 are
% parameter vectors. Their size is the degree of freedom: A function that
% determines whether there is a collision-free straight line path between
% two points (x1, x2) in configuration space given obstacle as input.

delta = x2 - x1;

t = delta > 180;
delta(t) = delta(t) - 360;

t = delta < -180;
delta(t) = delta(t) + 360;

nsamples = ceil(sum(abs(delta)) / 10);

for i = 1:nsamples
    
    x = mod(x1 + (i/nsamples)*delta, 360);
    
    fv = SixLinkRobot (x);
    
    if (CollisionCheck(fv, obstacle))
        out = false;
        return
    end
end

out = true;
