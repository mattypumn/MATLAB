function [g_p_c_, g_q_c_] = interp_poses(g_p_c, g_q_c, c_t, r_t)

g_q_c_ = zeros(4,length(r_t));
g_p_c_ = zeros(3,length(r_t));
x_axes_interp = zeros(3,length(r_t));
y_axes_interp = zeros(3,length(r_t));
z_axes_interp = zeros(3,length(r_t));

x_axes = zeros(3,length(c_t));
y_axes = zeros(3,length(c_t));
z_axes = zeros(3,length(c_t));

for k = 1:length(c_t)
    I_R_G = quat2rot(g_q_c(:,k));
    x_axes(:,k) = I_R_G(:,1);
    y_axes(:,k) = I_R_G(:,2);
    z_axes(:,k) = I_R_G(:,3);
end

for k = 1:3
    x_axes_interp(k,:) =  interp1(c_t',x_axes(k,:),r_t,'spline','extrap');
    y_axes_interp(k,:) =  interp1(c_t',y_axes(k,:),r_t,'spline','extrap');
    z_axes_interp(k,:) =  interp1(c_t',z_axes(k,:),r_t,'spline','extrap');
end

for k = 1:3
    g_p_c_(k,:) =  interp1(c_t',g_p_c(k,:),r_t,'spline','extrap');
end

for k=1:length(r_t)
    I_R_G_before_projection = [x_axes_interp(:,k) y_axes_interp(:,k) z_axes_interp(:,k)];
    %% Project to SO(3).
    I_R_G_projected =  I_R_G_before_projection*(I_R_G_before_projection'*I_R_G_before_projection)^(-.5);
    g_q_c_(:,k) = rot2quat(I_R_G_projected);
end