% Plot steering angle vs PWM input using Encoder data 
clc;
clear;
cd '/Users/BrettStephens/Documents/Zarc_Brett/VICON Measurements/';


str_folder = 'negative PWM (inc3, start 87)';
files = dir(str_folder);

%parse each file 
for j = 3:length(files)
    delta = zeros(1,length(files)-2);
    
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
    head = eulerAngles(:,1); %heading angle [rad]

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
       
    %threshold = 0.03;
    %res = 150; 
    threshold = 0.17;
    horizon = 20;
    indeces = [];
    
    for i = (horizon+1):length(dxB_f)
        %find when v_x stops changing
        if dxB_f(i) > 0.7 && (abs(dxB_f(i) - dxB_f(i-horizon))) < threshold
            index_start = i; %starting index
            for k = index_start:length(dxB_f)-1
                if abs(dxB_f(k+1)-dxB_f(k)) < 0.2
                    indeces = [indeces,k];
                end
            end
        end
    end
    
    %indeces of "constant" v_x
    indeces = (index_start:1:index_end);
    
    t_const = t(indeces);
    v_x = dxB_f(indeces);

    figure();
    plot(t(1:end-1), dxB_f);hold on;
    plot(t_const,v_x,'LineWidth',2);
    
    v_x_avg = mean(v_x);
    yaw_f_avg = mean(yaw_f(indeces));

    %steering angle calc
    Lf = 0.1698;
    Lr = 0.1542;
    delta(j) = atan((yaw_f_avg*(Lf+Lr))/(v_x_avg));
    
end
