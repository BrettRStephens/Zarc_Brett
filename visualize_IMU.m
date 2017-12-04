%% Visualize IMU data
close;
clear;

str_folder = '/Users/BrettStephens/Documents/Zarc_Brett/Encoder,IMU Measurements/Negative PWM, (inc3, start 87)/';
str_file = 'imuOutput_23-11-2017 01:54:53.csv';
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

N = length(t);
Q = [qw qx qy qz];

eulZYX = quat2eul(Q);
heading = eulZYX(:,1); %Z euler angle (heading) [rad]

% adjust for heading offset, which is negative here
offset = heading(1);
heading_offset = heading - offset;

% dxW = zeros(N-1,1);
% dyW = zeros(N-1,1);
% dxB = zeros(N-1,1);
% dyB = zeros(N-1,1);
yaw = zeros(N-1,1); %yaw [rad/s]

for i=2:N
    %dxW(i-1) = (x(i) - x(i-1)) / (t(i) - t(i-1));
    %dyW(i-1) = (y(i) - y(i-1)) / (t(i) - t(i-1));
    yaw(i-1) = (heading_offset(i) - heading_offset(i-1)) / (t(i) - t(i-1)); %yaw 
    %R = [cos(eulZ(i)) -sin(eulZ(i)); sin(eulZ(i)) cos(eulZ(i))]; %rotation matrix
    %temp = R' * [dxW(i-1); dyW(i-1)];
    %dxB(i-1) = temp(1);
    %dyB(i-1) = temp(2);
end

d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.15,'DesignMethod','butter');

%filter the data

heading_f = filtfilt(d1,heading_offset);
%dxB_f = filtfilt(d1,dxB);
%dyB_f = filtfilt(d1,dyB);
d2 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.05,'DesignMethod','butter');
yaw_f = filtfilt(d2,yaw);

%% plot IMU

figure(1)
subplot(2,1,1)
plot(t,rad2deg(heading_f),'Linewidth', 2);
ylabel('heading');
hold on
% subplot(3,1,2)
% plot(t(1:end-1),dxB_f,'Linewidth', 2);
% hold on
% plot(t(1:end-1),dyB_f,'Linewidth', 2);
% legend('V_x', 'V_y')
subplot(2,1,2)
plot(t(1:end-1), yaw_f);
ylabel('yaw rate');
hold on

%% VICON Data

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

figure(1)
subplot(2,1,1)
plot(t,rad2deg(yaw),'Linewidth', 2);
ylabel('heading');
legend('IMU','VICON');

subplot(2,1,2)
plot(t(1:end-1), r_f);
ylabel('yaw rate')
hold on
legend('IMU','VICON');
