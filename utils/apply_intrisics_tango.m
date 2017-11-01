function [x_d] = apply_intrisics_tango(x_n,fc,cc,kc)
    r = norm(x_n);
    if(r>(1e-3))
        distortion_factor = (1.0 / kc(1)) * atan(2 * r * tan(kc(1) / 2));
        x_d = distortion_factor / r * x_n;
    else
        x_d = x_n;
    end
    
    x_d(1,1) =  x_d(1,1) * fc(1);
    x_d(1,1) =  x_d(1,1) + cc(1);
    x_d(2,1) =  x_d(2,1) * fc(2);
    x_d(2,1) =  x_d(2,1) +  cc(2);
end