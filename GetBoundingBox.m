function [top_left_corner, height, width] = GetBoundingBox(points2d, radius)
    if isempty(points2d)
        top_left_corner = [0;0]; height = 0; width = 0; return;
    end
    minx = min(points2d(1,:));
    maxx = max(points2d(1,:));
    miny = min(points2d(2,:));
    maxy = max(points2d(2,:));
    top_left_corner = [minx - radius; miny - radius];
    height = maxy - miny + 2 * radius;
    width = maxx - minx + 2 * radius;
end