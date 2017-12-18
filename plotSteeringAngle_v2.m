% Plot steering angle vs PWM input using Encoder data 
clc;
clear;
cd '/Users/BrettStephens/Documents/Zarc_Brett/measurements_17Dec2017/';

deltas = [];

files = dir();
files = files(3:end);


%plot steering angle vs. uPWM
pwm = (80:-3:59);
% pwm = (80:-3:20);

    %parse each file 
    for j = 1:length(pwm)

        str = files(j).name;
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

        %want to get a "static" v_x to calc. the steering angle (delta) using 
        %the equation: delta = atan( (yaw*(Lf+Lr)) / (v_x) )

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
        yaw_f_in = yaw_f(indeces);
        v_x_avg = mean(v_x);
        v_x_avg_array = v_x_avg*ones(1,length(t(1:end-1)));
        yaw_f_avg = mean(yaw_f(indeces));
        yaw_f_avg_array = yaw_f_avg*ones(1,length(t(1:end-1)));

        figure();
        subplot(2,1,1);
        plot(t(1:end-1), dxW);hold on;
        plot(t(1:end-1), dxB_f);hold on;
        plot(t_const,v_x,'LineWidth',2);
        plot(t(1:end-1), v_x_avg_array,'LineWidth',2);
        xlabel('time(s)','Interpreter','latex','fontsize',16);
        ylabel('$V_x$','Interpreter','latex','fontsize',16)
        text(18.5,0.75,'$V_xavg$','Interpreter','latex','fontsize',12);
        
        subplot(2,1,2)
        plot(t(1:end-1), yaw_f);hold on;
        %plot(t_const,yaw_f_in,'LineWidth',2);
        plot(t(1:end-1), yaw_f_avg_array,'LineWidth',2);
        xlabel('time(s)','Interpreter','latex','fontsize',16);
        ylabel('$\dot{\Psi}$','Interpreter','latex','fontsize',16)
        text(18.5,0.14,'$\dot{\Psi}avg$','Interpreter','latex','fontsize',12);
        
        figure();
        subplot(2,1,1);
        plot(t,head);hold on;
        subplot(2,1,2);
        plot(t(1:end-1),yaw_f);
    
        %steering angle calc for each file
        Lf = 0.1698;
        Lr = 0.1542;
        delta = atan((yaw_f_avg*(Lf+Lr))/(v_x_avg));
        deltas = [deltas,delta];
        v_x_avgs(j) = v_x_avg;
        yaw_f_avgs(j) = yaw_f_avg;
    end

figure()
plot(pwm,deltas,'b*');hold on;

xlabel('$U_{pwm}$','Interpreter','latex','fontsize',16);
ylabel('$\delta_f$','Interpreter','latex','fontsize',16);

%fit a line to the data
P = polyfit(pwm,deltas,1);
yfit = P(1)*pwm+P(2);
plot(pwm,yfit,'k-');

text(65,0.24,strcat('$\delta_f$ = ',num2str(P(1)),' $* U_{pwm}$',' + ',...
    num2str(P(2))),'Interpreter','latex','fontsize',14);
%title('$\hat{\psi}$','Interpreter','latex')

%title('Steering Angle Mapping','Interpreter','latex','fontsize',18);




