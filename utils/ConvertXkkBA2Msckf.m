function ConvertXkkBA2Msckf(output_filename, xkk)
%CONVERTXKKBA2MSCKF Summary of this function goes here
%   Detailed explanation goes here

    state_size = 16;
    xkk = reshape(xkk, state_size, numel(xkk)/state_size)';

    dlmwrite(output_filename, [size(xkk, 1) 17], ' ');
    dlmwrite(output_filename, [(1:size(xkk,1))', xkk], ...
        'delimiter', ' ', '-append');
    
end

