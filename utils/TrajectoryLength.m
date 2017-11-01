function length = TrajectoryLength(positions)
%TRAJECTORYLENGTH Returns the length of the trajectory
    length = 0;
    assert(size(positions, 1) == 3);
    N = size(positions, 2);
    for i = 2:N
        delta = positions(:, i) - positions(:, i - 1);
        length = length + norm(delta);
    end
end

