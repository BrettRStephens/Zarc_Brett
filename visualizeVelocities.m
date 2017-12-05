%vextract velocities

str_folder = '/Users/BrettStephens/Documents/Zarc_Brett/VICON Measurements/pwm_90,93/';
str_file = 'viconExampleOutput_23-11-2017 03:07:06.csv'; %heading ok
%str_file = 'viconExampleOutput_23-11-2017 02:59:55.csv'; %heading wierd 

%str_file = [str_file_1;str_file_2];

for j = 1:1%length(str_file(:,1))
    
    str = strcat(str_folder,str_file);%(j,:));
    data = csvread(str,1);
    file = fopen(str,'r');
    header = fgets(file);
    fclose(file);
    labels = getLabels(header);


    for i=1:length(labels)
        str = [labels{i} '=data(:,i);'];
       eval(str); 
    end

    %% plot positins and velocities

    N = length(t);



%     plot(x,y, 'Linewidth', 2);
%     xlabel('x');
%     xlabel('y');

    Q = [qw qx qy qz];

    eulerAngles = quat2eul(Q);
    yaw = unwrap(eulerAngles(:,1)); %heading angle [rad]

    dxW = zeros(N-1,1);
    dyW = zeros(N-1,1);
    dxB = zeros(N-1,1);
    dyB = zeros(N-1,1);
    r = zeros(N-1,1);
    for i=2:N
        dxW(i-1) = (x(i) - x(i-1)) / (t(i) - t(i-1)); %world frame v_x
        dyW(i-1) = (y(i) - y(i-1)) / (t(i) - t(i-1)); %world frame v_y
        r(i-1) = (yaw(i) - yaw(i-1)) / (t(i) - t(i-1)); %[rad/s]
        R = [cos(yaw(i)) -sin(yaw(i)); sin(yaw(i)) cos(yaw(i))]; %rotation matrix
        temp = R' * [dxW(i-1); dyW(i-1)];
        dxB(i-1) = temp(1); % body frame v_x
        dyB(i-1) = temp(2); % body frame v_y
    end

    d1 = designfilt('lowpassiir','FilterOrder',12, ...
        'HalfPowerFrequency',0.15,'DesignMethod','butter');

    %filter the data

    yaw_f = filtfilt(d1,yaw);
    dxB_f = filtfilt(d1,dxB);
    dyB_f = filtfilt(d1,dyB);
    d2 = designfilt('lowpassiir','FilterOrder',12, ...
        'HalfPowerFrequency',0.05,'DesignMethod','butter');
    r_f = filtfilt(d2,r);
    
    figure()
    subplot(3,1,1)
    plot(t,yaw,'Linewidth', 2);
    ylabel('heading');
    hold on
    subplot(3,1,2)
    plot(t(1:end-1),dxB_f,'Linewidth', 2);
    hold on
    plot(t(1:end-1),dyB_f,'Linewidth', 2);
    legend('V_x', 'V_y')
    subplot(3,1,3)
    plot(t(1:end-1), r_f);
    ylabel('yaw rate')
    hold on

    figure()
    subplot(3,1,1)
    plot(t,yaw,'Linewidth', 2);
    hold on
    plot(t,yaw_f,'Linewidth', 2);
    subplot(3,1,2)
    plot(t(1:end-1),dxB,'Linewidth', 2);
    hold on
    plot(t(1:end-1),dyB,'Linewidth', 2);
    plot(t(1:end-1),dxB_f,'Linewidth', 2);
    grid on
    plot(t(1:end-1),dyB_f,'Linewidth', 2);
    legend('V_x', 'V_y')
    subplot(3,1,3)
    plot(t(1:end-1), r);
    hold on
    plot(t(1:end-1), r_f);
    
end
