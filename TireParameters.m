% Plot lateral tire force vs slip angle 
clc;
clear;
close all;

cd '/Users/BrettStephens/Documents/Zarc_Brett/measurements_17Dec2017/';

%vehicle parameters
Lf = 0.1741105; %distance from cg to front axle [m]
Lr = 0.1498895; %distance from cg to rear axle [m]
L = Lf+Lr; %wheelbase [m]
m = 2.792; %vehicle mass [kg]
Fz_f = 12.69414; % [N] measured normal force on front tires
Fz_r = 14.745411; % [N] measured normal force on rear tires

%make a vector of steering pwm signals
folder = dir();
folder = folder(3:end);
 
for k = 1:length(folder)
   
    if strcmp(folder(k).name,'motorPwm_115');
        pwm = (68:-3:59);
    else pwm = (80:-3:59);
    end

%use steering map to get delta from pwm
delta_f = -0.0051266*pwm + 0.55592;
delta_f_deg = delta_f*(180/pi);

files = dir();
files = files(3:end);
alpha_fs = [];
alpha_rs = [];
Fy_fs = [];
Fy_rs = [];
debug = false;

str_folder = folder(k).name;
files = dir(str_folder);
files = files(3:end);

%parse each file 
for j = 1:length(delta_f)

    str_file = files(j).name;
    str = strcat(str_folder,'/',str_file);
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

    t_const = t(indeces);
    v_x = dxB_f(indeces);
    v_y = dyB_f(indeces);
    yaw_f_in = yaw_f(indeces);
    v_x_avg = mean(v_x);
    v_y_avg = mean(v_y);
    v_x_avg_array = v_x_avg*ones(1,length(t(1:end-1)));
    yaw_f_avg = mean(yaw_f(indeces));
    yaw_f_avg_array = yaw_f_avg*ones(1,length(t(1:end-1)));
    
%     figure();
%     subplot(2,1,1);
%     plot(t(1:end-1), dxB_f);hold on;
%     plot(t_const,v_x,'LineWidth',2);
%     plot(t(1:end-1), v_x_avg_array,'LineWidth',2);
%     xlabel('time(s)','Interpreter','latex','fontsize',16);
%     ylabel('$V_W$','Interpreter','latex','fontsize',16)
% %     text(18.5,0.75,'$V_xavg$','Interpreter','latex','fontsize',12);
% 
%     subplot(2,1,2)
%     plot(t(1:end-1), yaw_f);hold on;
%     %plot(t_const,yaw_f_in,'LineWidth',2);
%     plot(t(1:end-1), yaw_f_avg_array,'LineWidth',2);
%     xlabel('time(s)','Interpreter','latex','fontsize',16);
%     ylabel('$\dot{\Psi}$','Interpreter','latex','fontsize',16)
% %     text(18.5,0.14,'$\dot{\Psi}avg$','Interpreter','latex','fontsize',12);
   
    %get n closest values to the average    
    diff = abs(dxB_f-v_x_avg);
    n = 1;
    [diff_sort,i] = sort(diff);
    indeces = i(1:n);
    v_x = dxB_f(indeces);
    v_y = dyB_f(indeces);
    yaw_f_in = yaw_f(indeces);
    
    alpha_f = [];
    alpha_r = [];
    ay_ss = [];
    Fy_f = [];
    Fy_r = [];
%     ay_ss = yaw_f_avg*v_x_avg;

    for i = 1:n
        %calcuate front slip angle
%         beta(i) = v_y(i)/v_x(i);
        alpha_f(i) = atan((v_y(i)+Lf*yaw_f_in(i))/(v_x(i))) - delta_f(j);
%         alpha_f(i) = atan(beta(i) + ((Lf*yaw_f_in(i))/v_x(i))) - delta_f(j);
        alpha_r(i) = atan((v_y(i)-Lr*yaw_f_in(i))/(v_x(i)));        
%         alpha_r(i) = atan(beta(i) - ((Lr*yaw_f_in(i))/v_x(i)));
        %calculate lateral tire forces
        ay_ss(i) = yaw_f_in(i)*v_x(i);
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

end
%lateral force model curve

alpha_sl_f = 0.06; %[rad] emperical estimate
alpha_sl_r = 0.04;
muFz_f = 5; %[N] emperical estimate
muFz_r = 5.5;
mu_f = muFz_f/Fz_f;
mu_r = muFz_r/Fz_r;
C_alpha_f = (3*mu_f*Fz_f)/(tan(alpha_sl_f)); %emperical estimate
C_alpha_r = (3*mu_r*Fz_r)/(tan(alpha_sl_r)); 

%make a range of alphas
alpha_f = (0:-0.001:-0.15);
alpha_r = (0:-0.001:-0.15);

for i = 1:length(alpha_f)
    if abs(alpha_f(i)) <= abs(alpha_sl_f)
        Fy_f_e(i) = ( -C_alpha_f*tan(alpha_f(i)) ) + ( ((C_alpha_f^2)/(3*mu_f*Fz_f)) * abs(tan(alpha_f(i))) *...
            tan(alpha_f(i)) ) - ( ((C_alpha_f^3) / (27*(mu_f^2)*(Fz_f^2))) * (tan(alpha_f(i)))^3);
    else Fy_f_e(i) = -mu_f*Fz_f*sign(alpha_f(i));  
    end
end

for i = 1:length(alpha_r)
    if abs(alpha_r(i)) <= abs(alpha_sl_r)
        Fy_r_e(i) = ( -C_alpha_r*tan(alpha_r(i)) ) + ( ((C_alpha_r^2)/(3*mu_r*Fz_r)) * abs(tan(alpha_r(i))) *...
            tan(alpha_r(i)) ) - ( ((C_alpha_r^3) / (27*(mu_r^2)*(Fz_r^2))) * (tan(alpha_r(i)))^3);
    else Fy_r_e(i) = -mu_r*Fz_r*sign(alpha_r(i));  
    end
end

%plot results
figure(1)
plot(alpha_fs,Fy_fs,'b*');hold on;
%plot(alpha_f, Fy_f_e,'r','LineWidth',2);
xlabel('$\alpha_F$','Interpreter','latex','fontsize',16);
ylabel('$F_{yF}$','Interpreter','latex','fontsize',16)

figure(2)
plot(alpha_rs,Fy_rs,'b*');hold on;
%plot(alpha_r, Fy_r_e,'r','LineWidth',2);
xlabel('$\alpha_R$','Interpreter','latex','fontsize',16);
ylabel('$F_{yR}$','Interpreter','latex','fontsize',16)


