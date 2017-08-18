%   const double u = undistorted_pt[0]; 
%   const double v = undistorted_pt[1]; 
%   const double r = std::sqrt(u * u + v * v);
%   const double theta = atan(r);
%   const double theta2 = theta * theta;
%   const double theta4 = theta2 * theta2;
%   const double theta6 = theta2 * theta4;
%   const double theta8 = theta4 * theta4;
%   const double theta_d = theta * (1.0 + kc[0] * theta2 + kc[1] * theta4 +
%                           kc[2] * theta6 + kc[3] * theta8);
%   const double scaling = (r > 1e-8) ? (theta_d / r) : 1;
%   x_d[0] = fc[0] * scaling * u + cc[0];
%   x_d[1] = fc[1] * scaling * v + cc[1];
%   return true ;



function pixel = DistortFisheye(ray, fc, cc, kc)
    if lenght(ray) == 3
        ray = ray / ray(3);
    end
    r = sqrt(ray(1:2)' * ray(1:2));
    theta = atan(r);
    theta2 = theta * theta;
    theta4 = theta2 * theta2;
    theta6 = theta2 * theta4;
    theta8 = theta4 * theta4;
    theta_d = theta * (1 + kc(1) * theta2 + kc(2) * theta4 + ...
              kc(3) * theta6 + kc(4) * theta8);
    scaling = 1;
    if r > 1e-8 
        scaling = theta_d / r;
    end
    
    pixel(1,1) = fc(1) * scaling * ray(1) + cc(1);
    pixel(2,1) = fc(2) * scaling * ray(2) + cc(2);
end