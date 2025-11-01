function [K, L_sum] = model_LQR(l_r , l_l , l_w_r , l_w_l , l_b_r , l_b_l , I_l_r , I_l_l)

syms theta_r(t) theta_l(t) theta_l_l(t) theta_l_r(t) theta_w_r(t) theta_w_l(t) theta_b(t) phi(t) S(t) T_r(t) T_l(t) Tp_r(t) Tp_l(t)
syms theta_dot_r theta_dot_dot_r theta_dot_l theta_dot_l_l theta_dot_dot_l theta_dot_dot_l_l theta_dot_l_r theta_dot_dot_l_r theta_dot_b theta_dot_dot_b S_dot S_dot_dot 
syms theta_dot_w_r theta_dot_dot_w_r theta_dot_w_l theta_dot_dot_w_l
% syms Rw Rl L_r L_l Lw_r Lw_l Lb_r Lb_l lc Mw Ml Mb Iw Il_r Il_l Ib Iz 


g = 9.81;
R_w = 0.064;
R_l = 0.225;
l_c = 0.01;
M_w = 0.3;
M_l = 2;
M_b = 3;
I_w = 0.0006144;
I_b = 0.000001;
I_z = 0.000001;


F1 = (I_w * l_l/R_w + M_w*R_w*l_l + M_l*R_w*l_b_l)*diff(theta_w_l , t , t) + (M_l*l_w_l*l_b_l - I_l_l)*diff(theta_l_l , t , t) + (M_l*l_w_l + 0.5*M_b*l_l)*g*theta_l_l + Tp_l - T_l(1 + l_l/R_w) == 0; 
F2 = (I_w * l_r/R_w + M_w*R_w*l_r + M_l*R_w*l_b_r)*diff(theta_w_r , t , t) + (M_l*l_w_r*l_b_r - I_l_r)*diff(theta_l_r , t , t) + (M_l*l_w_r + 0.5*M_b*l_r)*g*theta_l_r + Tp_r - T_r(1 + l_r/R_w) == 0;
F3 = -(M_w*R_w*R_w + I_w + M_l*R_w*R_w + 0.5*M_b*R_w*R_w)*diff(theta_w_l , t , t) + (M_w*R_w*R_w + I_w + M_l*R_w*R_w + 0.5*M_b*R_w*R_w)*diff(theta_w_r ,t ,t) - (M_l*R_w*l_w_l + 0.5*M_b*R_w*l_l)*diff(theta_l_l ,t ,t) - (M_l*R_w*l_w_r + 0.5*M_b*R_w*l_r)*diff(theta_l_r ,t ,t) + T_l + T_r == 0;
F4 = (M_w*R_w*l_c + I_w*l_c/R_w + M_l*R_w*l_c)*diff(theta_w_l ,t ,t) + (M_w*R_w*l_c + I_w*l_c/R_w + M_l*R_w*l_c)*diff(theta_w_r ,t ,t) + M_l*l_w_l*l_c*diff(theta_l_l ,t ,t) + M_l*l_w_r*l_c*diff(theta_l_r ,t ,t) - I_b*diff(theta_l_r ,t) + M_b*g*l_c*theta_b - (T_l + T_r)*l_c/R_w - (Tp_l + Tp_r) == 0;
F5 = (0.5*I_z*R_w/R_l + I_w*R_l/R_w)*diff(theta_w_l ,t ,t) - (0.5*I_z*R_w/R_l + I_w*R_l/R_w)*diff(theta_w_r ,t ,t) + 0.5*I_z*l_l/R_l*diff(theta_l_l ,t ,t) - 0.5*I_z*l_r/R_l*diff(theta_l_r ,t ,t) - T_l*R_l/R_w + T_r*R_l/R_w == 0;

F1 = subs(F1 , [diff(theta_w_l , t , t) , diff(theta_l_l , t , t)] , [theta_dot_dot_w_r , theta_dot_dot_l_l]);
F2 = subs(F2 , [diff(theta_w_r , t , t) , diff(theta_l_r , t , t)] , [theta_dot_dot_w_l , theta_dot_dot_l_r]);
F3 = subs(F3 , [diff(theta_w_l , t , t) , diff(theta_w_r ,t ,t) , diff(theta_l_l ,t ,t) , diff(theta_l_r ,t ,t)] , [theta_dot_dot_w_l , theta_dot_dot_w_r ,theta_dot_dot_l_l ,theta_dot_dot_l_r]);
F4 = subs(F4 , [diff(theta_w_l ,t ,t) , diff(theta_w_r ,t ,t) , diff(theta_l_l ,t ,t) , diff(theta_l_r ,t ,t) , diff(theta_l_r ,t)] , [theta_dot_dot_w_l , theta_dot_dot_w_r , theta_dot_dot_l_l , theta_dot_dot_l_r ,theta_dot_l_r]);
F5 = subs(F5 , [diff(theta_w_l ,t ,t) ,diff(theta_w_r ,t ,t) ,diff(theta_l_l ,t ,t) ,diff(theta_l_r ,t ,t)] , [theta_dot_dot_w_l ,theta_dot_dot_w_r ,theta_dot_dot_l_l ,theta_dot_dot_l_r]);

eqa = [F1;F2;F3;F4;F5];
var = [theta_dot_dot_l_l , theta_dot_dot_l_r , theta_dot_dot_l_l , theta_dot_dot_l_r];
[theta_dot_dot_l_l , theta_dot_dot_l_r , theta_dot_dot_w_l , theta_dot_dot_w_r] = solve(eqa , var);

end

