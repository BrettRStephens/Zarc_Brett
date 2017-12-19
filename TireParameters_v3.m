% Plot lateral tire force vs slip angle 
clc;
clear;
close all;

cd '/Users/BrettStephens/Documents/Zarc_Brett/measurements_18Dec2017';

%vehicle parameters
Lf = 0.1741105; %distance from cg to front axle [m]
Lr = 0.1498895; %distance from cg to rear axle [m]
L = Lf+Lr; %wheelbase [m]
m = 2.792; %vehicle mass [kg]
Fz_f = 12.69414; % [N] measured normal force on front tires
Fz_r = 14.745411; % [N] measured normal force on rear tires
mu_flag = true;
if mu_flag == true
    mu = 0.2865; %measured kinetic friction coefficient in Vicon room
else mu_flag = false;
end

folder = dir();
folder = folder(3:end);

alpha_f_tot = [];
alpha_r_tot = [];
Fy_f_tot = [];
Fy_r_tot = [];

files = dir();
files = files(3:end);
alpha_fs = [];
alpha_rs = [];
Fy_fs = [];
Fy_rs = [];
debug = false;

%parse each file 
for j = 1:length(files)

    str_file = files(j).name;
    str = strcat(str_file);
    data = csvread(str,1);
    file = fopen(str,'r');
    header = fgets(file);
    fclose(file);
    labels = getLabels(header);

    %extract labels
    for i=1:length(labels)
        str = [labels{i} '=data(:,i);'];
       eval(str); 
    end

    N = length(t);

    Q = [qw qx qy qz];

    eulerAngles = quat2eul(Q);
    head = unwrap(eulerAngles(:,1)); %heading angle [rad]
    
    dxW = zeros(N-1,1);
    dyW = zeros(N-1,1);
    dxB = zeros(N-1,1);
    dyB = zeros(N-1,1);
    yaw = zeros(N-1,1);
    for i=2:N
        dxW(i-1) = (x(i) - x(i-1)) / (t(i) - t(i-1)); %world frame v_x
        dyW(i-1) = (y(i) - y(i-1)) / (t(i) - t(i-1)); %world frame v_y
        yaw(i-1) = (head(i) - head(i-1)) / (t(i) - t(i-1)); %[rad/s]
        R = [cos(head(i)) -sin(head(i)); sin(head(i)) cos(head(i))]; %rotation matrix
        temp = R' * [dxW(i-1); dyW(i-1)];
        dxB(i-1) = temp(1); % body frame v_x
        dyB(i-1) = temp(2); % body frame v_y
    end

    d1 = designfilt('lowpassiir','FilterOrder',12, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');
    dxB_f = filtfilt(d1,dxB);
    dyB_f = filtfilt(d1,dyB);

    d2 = designfilt('lowpassiir','FilterOrder',12, ...
        'HalfPowerFrequency',0.05,'DesignMethod','butter');
    yaw_f = filtfilt(d2,yaw);

    %need to get idea of when we swept through steering angles 
    %find indceces when forward velocity is above some value
    idx_s = find(dxB_f > 0.014,1);
    t_s = t(idx_s);
    t_e = t_s + 8; %8 seconds later, when steering sweep is done
    idx_e = find(t >= t_e,1);
    
    %sanity check
    figure()
    subplot(2,1,1)
    plot(t(1:end-1), dxB_f);hold on;
    plot(t(idx_s),dxB_f(idx_s),'r*');
    plot(t(idx_s:idx_e),dxB_f(idx_s:idx_e),'LineWidth',2)
    subplot(2,1,2)
    plot(t(1:end-1), yaw_f);hold on;
    plot(t(idx_s),yaw_f(idx_s),'r*');
    plot(t(idx_s:idx_e),yaw_f(idx_s:idx_e),'LineWidth',2)
    
    %clip the data
    t = t(idx_s:idx_e);
    dxB_f = dxB_f(idx_s:idx_e);
    dyB_f = dyB_f(idx_s:idx_e);
    yaw_f = yaw_f(idx_s:idx_e);

    %linearly "increasing" pwm 
    pwm_s = 90;
    pwm_e = 60;
    pwm_range = pwm_s - 60;
    res = idx_e-idx_s;
    pwm_scale = pwm_range/res;
    pwm = (pwm_s:-pwm_scale:pwm_e);
    
    %use steering map to get delta from pwm
    delta_f = -0.0051266*pwm + 0.55592;  
    
    %want to get a "static" v_x
    %tunable parameters
    threshold = 0.1;
    horizon = 150;

    indeces = [];

    for i = (horizon+1):length(dxB_f)
        %find when v_x stops changing
        if dxB_f(i) > 0.2 && (abs(dxB_f(i) - dxB_f(i-horizon))) < threshold
            indeces = [indeces,i]; 
        end
    end
    
    %further clip data - only a portion of the data was gathered during
    %constant v_x
    t_const = t(indeces);
    v_x = dxB_f(indeces);
    v_y = dyB_f(indeces);
    pwm = pwm(indeces);
    yaw_f_in = yaw_f(indeces);
    delta_f = delta_f(indeces);
    v_x_avg = mean(v_x);
    v_y_avg = mean(v_y);
    yaw_f_avg = mean(yaw_f_in);
    v_x_avg_array = v_x_avg*ones(1,length(t));

%     figure();
%     subplot(2,1,1);
%     plot(t, dxB_f);hold on;
%     plot(t_const,v_x,'LineWidth',2);
%     plot(t, v_x_avg_array,'LineWidth',2);
%     xlabel('time(s)','Interpreter','latex','fontsize',16);
%     ylabel('$V_W$','Interpreter','latex','fontsize',16)
%     text(18.5,0.75,'$V_xavg$','Interpreter','latex','fontsize',12);

%     subplot(2,1,2)
%     plot(t, yaw_f);hold on;
%     %plot(t_const,yaw_f_in,'LineWidth',2);
%     plot(t, yaw_f_avg_array,'LineWidth',2);
%     xlabel('time(s)','Interpreter','latex','fontsize',16);
%     ylabel('$\dot{\Psi}$','Interpreter','latex','fontsize',16)
% %     text(18.5,0.14,'$\dot{\Psi}avg$','Interpreter','latex','fontsize',12);

    for i = 1:length(delta_f)
        %calcuate front slip angle
%         beta(i) = v_y(i)/v_x(i);
        alpha_f(i) = atan((v_y(i)+Lf*yaw_f_in(i))/(v_x(i))) - delta_f(i);
%         alpha_f(i) = atan((v_y_avg+Lf*yaw_f_avg)/(v_x_avg)) - delta_f(i);

%         alpha_f(i) = atan(beta(i) + ((Lf*yaw_f_in(i))/v_x(i))) - delta_f(j);
        alpha_r(i) = atan((v_y(i)-Lr*yaw_f_in(i))/(v_x(i))); 
%         alpha_r(i) = atan((v_y_avg-Lr*yaw_f_avg)/(v_x_avg));                
%         alpha_r(i) = atan(beta(i) - ((Lr*yaw_f_in(i))/v_x(i)));
        %calculate lateral tire forces
        ay_ss(i) = yaw_f_in(i)*v_x(i);
%         ay_ss(i) = yaw_f_avg*v_x_avg;        
        Fy_f(i) = (Lr/L)*m*ay_ss(i);        
        Fy_r(i) = (Lf/L)*m*ay_ss(i);
    end
    
    alpha_fs = [alpha_fs,alpha_f];
    alpha_rs = [alpha_rs,alpha_r];
    Fy_fs = [Fy_fs,Fy_f];
    Fy_rs = [Fy_rs,Fy_r];
    
    if debug == true
        figure(1)
        plot(alpha_f,Fy_f,'b*');hold on;
        figure(2)
        plot(alpha_r,Fy_r,'b*');hold on; 
    end
     
    end
alpha_f_tot = [alpha_f_tot,alpha_fs];
alpha_r_tot = [alpha_r_tot,alpha_rs];
Fy_f_tot = [Fy_f_tot,Fy_fs];
Fy_r_tot = [Fy_r_tot,Fy_rs];

%curve fit with thesis
if mu_flag == true
    %x_f = [C_alpha];
    fun_f_t = @(x_f,alpha_f_tot) ( -x_f(1)*tan(alpha_f_tot) ) + ( ((x_f(1)^2)/(3*mu*Fz_f))...
    * abs(tan(alpha_f_tot)) .* tan(alpha_f_tot) ) - ( ((x_f(1)^3) / (27*(mu^2)*(Fz_f^2)))...
    * (tan(alpha_f_tot)).^3);
    x0_f_t = 1;
    x_f_t = lsqcurvefit(fun_f_t,x0_f_t,alpha_f_tot,Fy_f_tot);
    alpha_sl_f = atan((3*mu*Fz_f)/(x_f_t));
    
    %x_r = [C_alpha]
    fun_r_t = @(x_r,alpha_r_tot) ( -x_r(1)*tan(alpha_r_tot) ) + ( ((x_r(1)^2)/(3*mu*Fz_r))...
    * abs(tan(alpha_r_tot)) .* tan(alpha_r_tot) ) - ( ((x_r(1)^3) / (27*(mu^2)*(Fz_r^2)))...
    * (tan(alpha_r_tot)).^3);
    x0_r_t = 1;
    x_r_t = lsqcurvefit(fun_r_t,x0_r_t,alpha_r_tot,Fy_r_tot);
    alpha_sl_r = atan((3*mu*Fz_r)/(x_r_t));
    
else
    %x_f = [C_alpha_f,mu_f];
    fun_f_t = @(x_f,alpha_f_tot) ( -x_f(1)*tan(alpha_f_tot) ) + ( ((x_f(1)^2)/(3*x(2)*Fz_f))...
        * abs(tan(alpha_f_tot)) .* tan(alpha_f_tot) ) - ( ((x_f(1)^3) / (27*(x_f(2)^2)*(Fz_f^2)))...
        * (tan(alpha_f_tot)).^3);
    x0_f_t = [1,1];
    x_f_t = lsqcurvefit(fun_f_t,x0_f_t,alpha_f_tot,Fy_f_tot);
    alpha_sl_f = atan((3*x_f_t(2)*Fz_f)/(x_f_t(1)));
    
    %x_r = [C_alpha_r,mu_r];
    fun_r_t = @(x_r,alpha_r_tot) ( -x_r(1)*tan(alpha_r_tot) ) + ( ((x_r(1)^2)/(3*x(2)*Fz_r))...
    * abs(tan(alpha_r_tot)) .* tan(alpha_r_tot) ) - ( ((x_r(1)^3) / (27*(x_r(2)^2)*(Fz_r^2)))...
    * (tan(alpha_r_tot)).^3);
    x0_r_t = [1,1];
    x_r_t = lsqcurvefit(fun_r_t,x0_r_t,alpha_r_tot,Fy_r_tot);
    alpha_sl_r = atan((3*x_r_t(2)*Fz_r)/(x_r_t(1)));
end

%curve fit with Pacejka
if mu_flag == true
    %x_f = [D,C,B]
    x0_f_p = [1.2,1.4,10];
    fun_f_p = @(x_f,alpha_f_tot) -Fz_f*mu*x_f(1)*sin(x_f(2)*atan(x_f(3)*alpha_f_tot));
    x_f_p = lsqcurvefit(fun_f_p,x0_f_p,alpha_f_tot,Fy_f_tot);
    
    %x_r = [D,C,B]
    fun_r_p = @(x_r,alpha_r_tot) -Fz_r*mu*x_r(1)*sin(x_r(2)*atan(x_r(3)*alpha_r_tot));
    x0_r_p = [1.2,1.4,10];
    x_r_p = lsqcurvefit(fun_r_p,x0_r_p,alpha_r_tot,Fy_r_tot);
else
    %x_f = [D,C,B,mu]
    x0_f_p = [1,1,1,1];
    fun_f_p = @(x_f,alpha_f_tot) -Fz_f*x_f(4)*x_f(1)*sin(x_f(2)*atan(x_f(3)*alpha_f_tot));
    x_f_p = lsqcurvefit(fun_f_p,x0_f_p,alpha_f_tot,Fy_f_tot);

    %x_r = [D,C,B,mu]
    x0_r_p = [1,1,1,1];
    fun_r_p = @(x_r,alpha_r_tot) -Fz_r*x_r(4)*x_r(1)*sin(x_r(2)*atan(x_r(3)*alpha_r_tot));
    x_r_p = lsqcurvefit(fun_r_p,x0_r_p,alpha_r_tot,Fy_r_tot);
end

%%
%make a range of alphas
alpha_f = (0:-0.01:-0.6);
alpha_r = (0:-0.01:-0.6);
alpha_f_1 = (0:-0.0001:-alpha_sl_f);
alpha_f_2 = (-alpha_sl_f:-0.0001:-0.6);
alpha_r_1 = (0:-0.0001:-alpha_sl_r);
alpha_r_2 = (-alpha_sl_r:-0.0001:-0.6);

%plot results
figure()
%x_f_p = [1.2,1.4,10];
%alpha_f  = [0:-0.01:-1];
%fun = @(alpha)-Fz_f*mu*x_f_p(1)*sin(x_f_p(2)*atan(x_f_p(3)*alpha))
plot(alpha_f_tot,Fy_f_tot,'b*');hold on;

plot(alpha_f_1, fun_f_t(x_f_t,alpha_f_1),'r','LineWidth',2);
plot(alpha_f_2, -mu*Fz_f*sign(alpha_f_2),'r','LineWidth',2);

plot(alpha_f, fun_f_p(x_f_p, alpha_f),'g','LineWidth',2);
xlabel('$\alpha_F$','Interpreter','latex','fontsize',16);
ylabel('$F_{yF}$','Interpreter','latex','fontsize',16)

figure()
plot(alpha_r_tot,Fy_r_tot,'b*');hold on;

plot(alpha_r_1, fun_r_t(x_r_t,alpha_r_1),'r','LineWidth',2);
plot(alpha_r_2, -mu*Fz_r*sign(alpha_r_2),'r','LineWidth',2);

plot(alpha_r, fun_r_p(x_r_p,alpha_r),'g','LineWidth',2);
xlabel('$\alpha_R$','Interpreter','latex','fontsize',16);
ylabel('$F_{yR}$','Interpreter','latex','fontsize',16)

