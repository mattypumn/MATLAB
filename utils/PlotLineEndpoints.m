function PlotLineEndpoints(lp1, lp2, color_option_str)
%PLOTLINEENDPOINTS Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 3 || isempty(color_optio_str)
        color_option_str = '-';
    end

    for i = 1:size(lp1, 2)
        hold on;
%         if norm(lp1(:, i)- lp2(:, i)) > 20
%             continue;
%         end
        PlotPoints([lp1(:, i), lp2(:, i)], color_option_str);
    end

end

