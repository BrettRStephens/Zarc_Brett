%% Visualize Encoder data
close;
clear;

%cd /Users/BrettStephens/Documents/Zarc_Brett;
str_folder = '/Users/BrettStephens/Documents/Zarc_Brett/Encoder,IMU Measurements/PWM_90,93/';
str_file = 'encOutput_23-11-2017 02:07:09.csv';
str = strcat(str_folder,str_file);
data = csvread(str,1);
file = fopen(str,'r');
header = fgets(file);
fclose(file);
labels = getLabels(header);

% parse csv headers
for i=1:length(labels)
    str = [labels{i} '=data(:,i);'];
    eval(str); 
end

vel_avg = zeros(length(time),1);

% avg velocity (without BL encoder since it drops to 0 often, BR looks off)
for i = 1:length(vel_avg)
    vel_avg(i) = (velFL(i)+velFR(i)+velBR(i))/3;
end

%max of 3 velocities
vel_conc = [velFL, velFR, velBR];
vel_max = max(vel_conc,[], 2);

% most frequent velocity
%vel_mode = mode(vel_avg(vel_avg ~= 0));

%% Plot Encoder data

figure(1);
subplot(2,1,1);
plot(time,velFL,time,velFR,time,velBL,time,velBR);hold on;
plot(time, vel_avg,'Linewidth', 2);
legend('FL','FR','BL','BR','avg(-BL)');
xlabel('time[s]');ylabel('v_x[m/s]');
%text(0.5,vel_mode,strcat({'velocity mode = '},num2str(vel_mode)));

%% Visualize VICON velocity

str_folder = '/Users/BrettStephens/Documents/Zarc_Brett/VICON Measurements/negative PWM (inc3, start 87)/';
str_file = 'viconExampleOutput_23-11-2017 02:54:53.csv';
str = strcat(str_folder,str_file);
data = csvread(str,1);
file = fopen(str,'r');
header = fgets(file);
fclose(file);
labels = getLabels(header);


for i=1:length(labels)
    str = [labels{i} '=data(:,i);'];
   eval(str); 
end

N = length(t);

Q = [qw qx qy qz];

eulerAngles = quat2eul(Q);
yaw = eulerAngles(:,1); %heading angle [rad]

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

%% plot VICON data

figure(1);
subplot(2,1,2);
plot(time, velFL,'Linewidth', 2);hold on;
plot(time, velFR,'Linewidth', 2)
plot(t(1:end-1),dxB_f,'Linewidth', 2);
legend('Encoder', 'VICON')
xlabel('time[s]');ylabel('v_x[m/s]');

