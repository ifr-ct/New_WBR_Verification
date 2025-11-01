function [I,x,y] = I_simplify(Xa, Ya, Xb, Yb, Xc, Yc, Xd, Yd, Xf, Yf, Xg, Yg, Xh, Yh)
% global l1 l2 l3 l4 l5;
global mlae mlcg mldc mlad mlhf mlgh;
global Ilae Ilcg Ildc Ilad Ilhf Ilgh;
global K1;
syms mx_ae my_ae mx_cg my_cg mx_l3 my_l3 mx_l4 my_l4 mx_ef my_ef mx_gh my_gh mx_hf my_hf;

mx_ae = 0.5 * (K1*Xb-Xa) + Xa;
my_ae = 0.5 * (K1*Yb-Ya) + Ya;
mx_cg = 0.5 * (Xc-Xg) + Xg;
my_cg = 0.5 * (Yc-Yg) + Yg;
mx_l3 = 0.5 * (Xc-Xd) + Xd;
my_l3 = 0.5 * (Yc-Yd) + Yd;
mx_l4 = 0.5 * (Xd-Xa) + Xa;
my_l4 = 0.5 * (Yd-Ya) + Ya;
mx_hf = 0.5 * (Xf-Xh) + Xh;
my_hf = 0.5 * (Yf-Yh) + Yh;
mx_gh = 0.5 * (Xh-Xg) + Xg;
my_gh = 0.5 * (Yh-Yg) + Yg;
x = (mx_ae*mlae + mx_cg*mlcg + mx_l3*mldc + mx_l4*mlad + mx_hf*mlhf + mx_gh*mlgh)/(mlae+mlcg+mldc+mlad+mlhf+mlgh);
y = (my_ae*mlae + my_cg*mlcg + my_l3*mldc + my_l4*mlad + my_hf*mlhf + my_gh*mlgh)/(mlae+mlcg+mldc+mlad+mlhf+mlgh);
dae = sqrt((x-mx_ae).^2 + (y-my_ae).^2);
dcg = sqrt((x-mx_cg).^2 + (y-my_cg).^2);
d3 = sqrt((x-mx_l3).^2 + (y-my_l3).^2);
d4 = sqrt((x-mx_l4).^2 + (y-my_l4).^2);
dhf = sqrt((x-mx_hf).^2 + (y-my_hf).^2);
dgh = sqrt((x-mx_gh).^2 + (y-my_gh).^2);

I = (Ilae + mlae*dae.^2) + (Ilcg + mlcg*dcg.^2) + (Ildc + mldc*d3.^2) + (Ilad + mlad*d4.^2) + (Ilhf + mlhf*dhf.^2) + (Ilgh + mlgh*dgh.^2);
end