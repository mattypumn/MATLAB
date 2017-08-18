% bool CameraModelRadial::StaticDistort(const Eigen::Vector2d& undistorted_pt,
%                              const Eigen::Vector2d& fc,
%                              const Eigen::Vector2d& cc,
%                              const Vector5d& kc,
%                              Eigen::Vector2d& x_d)
% {
%   const double u = undistorted_pt[0];
%   const double v = undistorted_pt[1];
%   const double r_sq = u * u + v * v;
%   const double r_four = r_sq * r_sq;
%   const double pix_term2 = 1.0 + kc[0] * r_sq + kc[1] * r_four
%         + kc[4] * r_four * r_sq + 2.0 * kc[2] * v + 2.0 * kc[3] * u;
%   x_d[0] = fc[0] * (pix_term2 * u + kc[3] * r_sq) + cc[0];
%   x_d[1] = fc[1] * (pix_term2 * v + kc[2] * r_sq) + cc[1];
%   return true;
% }

function pixel = DistortRadial(ray, fc, cc, kc)
    if length(ray) == 3 
        ray = ray / ray(3);
    end
    r_sq = ray(1:2)' * ray(1:2);
    r_four = r_sq * r_sq;
    pix_term2 = 1.0 + kc(1) * r_sq + kc(2) * r_four + ...
                kc(5) * r_four * r_sq + 2.0 * kc(3) * ray(2) + ...
                2.0 * kc(4) * ray(1);
    pixel(1,1) = fc(1) * (pix_term2 * ray(1) + kc(4)*r_sq) + cc(1);
    pixel(2,1) = fc(2) * (pix_term2 * ray(2) + kc(3)*r_sq) + cc(2);
end
