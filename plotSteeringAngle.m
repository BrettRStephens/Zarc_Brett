% Plot steering angle vs PWM input using Encoder data 
clc;
clear;
cd '/Users/BrettStephens/Documents/Zarc_Brett/VICON Measurements/';

folder = dir();
folder = folder(4:6);

delta = zeros(3,10);
v_x_avgs = zeros(3,10);
yaw_f_avgs = zeros(3,10);

%parse each folder
for m = 1:length(folder)
    
str_folder = folder(m).name;
%str_folder = 'POsitive PWM (inc5, start 95)';

files = dir(str_folder);

    %parse each file 
    for j = 1:length(files)-2

        str_file = files(j+2).name;
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

        %want to get a "static" v_x to calc. the steering angle (delta) using 
        %the equation: delta = atan( (yaw*(Lf+Lr)) / (v_x) )

        %tunable parameters
        threshold = 0.03;
        horizon = 400;
        
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
        plot(t(1:end-1), dxB_f);hold on;
        plot(t_const,v_x,'LineWidth',2);
        plot(t(1:end-1), v_x_avg_array,'LineWidth',2);
        
        subplot(2,1,2)
        plot(t(1:end-1), yaw_f);hold on;
        %plot(t_const,yaw_f_in,'LineWidth',2);
        plot(t(1:end-1), yaw_f_avg_array,'LineWidth',2);

        %steering angle calc for each file
        Lf = 0.1698;
        Lr = 0.1542;
        delta(m,j) = atan((yaw_f_avg*(Lf+Lr))/(v_x_avg));
        v_x_avgs(m,j) = v_x_avg;
        yaw_f_avgs(m,j) = yaw_f_avg;
    end

end

%plot steering angle vs. uPWM
pos_pwm = delta(1,:);
pos_pwm = pos_pwm(pos_pwm~=0);
pos_pwm_x = [95,100,105,110,115,120];
neg_pwm = delta(2,:);
neg_pwm = neg_pwm(neg_pwm~=0);
neg_pwm_x = [87,84,81,78,75,72,69,66,63,60];
pwm_90_93 = delta(3,:);
pwm_90_93 = pwm_90_93(pwm_90_93~=0);
pwm_90_93_x = [90,93];

%bring data together
pwm_data_x = [pos_pwm_x,neg_pwm_x,pwm_90_93_x];
pwm_data_y = [pos_pwm,neg_pwm,pwm_90_93];

figure()
plot(pos_pwm_x,pos_pwm,'b*');hold on;
plot(neg_pwm_x,neg_pwm,'r*');
plot(pwm_90_93_x,pwm_90_93,'g*');
xlabel('uPWM');ylabel('delta');

%fit a line to the data
P = polyfit(pwm_data_x,pwm_data_y,1);
yfit = P(1)*pwm_data_x+P(2);
plot(pwm_data_x,yfit,'r-');
text(90,0.1,strcat('delta = ',num2str(P(1)),' * uPWM',' + ',num2str(P(2))));




