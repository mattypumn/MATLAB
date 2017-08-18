function [g_p_f] = interp_position(g_p_f, reference_times, target_times) 
    N = size(g_p_f, 2);



    for k = 1:3
        g_p_c_(k,:) =  interp1(c_t',g_p_c(k,:),r_t,'spline','extrap');
    end

end