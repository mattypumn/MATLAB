function h = PlotPoints(pts, varargin)
%PLOTPOINTS Summary of this function goes here
%   Detailed explanation goes here
    h = plot3(pts(1,:), pts(2,:), pts(3, :), varargin{:});
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end

