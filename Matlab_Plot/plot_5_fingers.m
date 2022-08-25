clc
clear all
close all

%% DATA EXTRACTING | phi = roll | theta = pitch | psi = yaw
% acc_x_1 | acc_y_1 | acc_z_1 | g_x_1 | g_y_1 | g_z_1 | acc_x_2 | acc_y_2 | % acc_z_2 | g_x_2 | g_y_2 | g_z_2 | roll_1 | pitch_1 | yaw_1 | roll_2 | pitch_2 | yaw_2 | pressure | position | closed | time | object
%   1           2       3         4       5        6      7         8           9         10     11       12      13       14       15        16       17       18      19          20        21      102       23   
fileID = fopen('data.txt','r');
DATA_SI = fscanf(fileID,'%f', [102 Inf]);
N = size(DATA_SI);
Nsamples = N(2)-1; %length of DATA
fclose(fileID);

PhiSaved   = DATA_SI(13, :) ;   % roll
ThetaSaved = DATA_SI(14, :) ;   % pitch
PsiSaved   = DATA_SI(15, :) ;   % yaw

unit_transform_gyro = pi/180;

for k = 1:Nsamples
  
    %Gyro LSB -> deg/s -> rad/s
    DATA_SI(4,k)= (unit_transform_gyro)*DATA_SI(4,k);
    DATA_SI(5,k)= (unit_transform_gyro)*DATA_SI(5,k);
    DATA_SI(6,k)= (unit_transform_gyro)*DATA_SI(6,k);   
    %Time ms -> s
    
end

    DATA_SI(102,:) = (DATA_SI(102,:)-DATA_SI(102,1))/1000;

x = [0 0];
y = [-1000,1000];

%% Plot the angle graph
figure()
P1=plot(DATA_SI(102,:), PhiSaved, 'r'); % roll
hold on
P2=plot(DATA_SI(102,:), ThetaSaved, 'b'); % pitch
P3=plot(DATA_SI(102,:), PsiSaved, 'g'); % yaw
refline([0 0])
title('Euler Angle (degree)')
Timeline_1 = line('XData',x,'YData',y);
TimeValue_1= xlabel('');
legend([P1 P2 P3],{'Phi', 'Theta', 'Psi'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(102,Nsamples) -360 360])



%% Plot the acceleration graph
figure()
subplot(1,2,1)
plot(DATA_SI(102,:),DATA_SI(1,:),'r',DATA_SI(102,:),DATA_SI(2,:),'g',DATA_SI(102,:),DATA_SI(3,:),'b'); % ax ay az
refline([0 0])
title('Acceleration (m/s^2)');
Timeline_2 = line('XData',x,'YData',y);
TimeValue_2= xlabel('');
legend({'AccX', 'AccY', 'AccZ'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(102,Nsamples) -2.1 2.1])

%% Plot the angular velocity graph
subplot(1,2,2)
plot(DATA_SI(102,:),DATA_SI(4,:),'r',DATA_SI(102,:),DATA_SI(5,:),'g',DATA_SI(102,:),DATA_SI(6,:),'b'); % gx gy gz
refline([0 0])
title('Angular velocity (rad/s)')
Timeline_3 = line('XData',x,'YData',y);
TimeValue_3= xlabel('');
legend({'GyroX', 'GyroY', 'GyroZ'},'Location','northwest','AutoUpdate','off');
axis([0 DATA_SI(102,Nsamples) -3 3])

%% Plot the magnetic flux graph
% subplot(1,3,3)
% plot(DATA_SI(13,:),DATA_SI(7,:),'r',DATA_SI(13,:),DATA_SI(8,:),'g',DATA_SI(13,:),DATA_SI(9,:),'b');
% refline([0 0])
% title('Magnetic flux density (uT)')
% Timeline_4 = line('XData',x,'YData',y);
% TimeValue_4= xlabel('');
% legend({'MagX', 'MagY', 'MagZ'},'Location','northwest','AutoUpdate','off');
% axis([0 DATA_SI(13,Nsamples) -40 40])

%% Graphical Plot & Dynamic Plot
%set IMU Object's vertex
Cuboid_1=[-2,-3,-0.5];
Cuboid_2=[2,-3,-0.5];
Cuboid_3=[2,3,-0.5];
Cuboid_4=[-2,3,-0.5];
Cuboid_5=[-2,-3,0.5];
Cuboid_6=[2,-3,0.5];
Cuboid_7=[2,3,0.5];
Cuboid_8=[-2,3,0.5];
Cuboid=[Cuboid_1; Cuboid_2; Cuboid_3; Cuboid_4; Cuboid_5; Cuboid_6; Cuboid_7; Cuboid_8];
% ?
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
%Plot Object
figure()
view(3)
Obejct_IMU = patch('Faces', fac, 'Vertices', [0, 0, 0]);
Obejct_IMU.FaceColor = 'g';
axis([-5 5 -5 5 -5 5])
title('IMU state')
Realtime = xlabel(' ');
%Animate
for k=1:Nsamples-1
    %Rotation matrix for vertex
    Rz = [cosd(PsiSaved(k)) -sind(PsiSaved(k)) 0; sind(PsiSaved(k)) cosd(PsiSaved(k)) 0; 0 0 1];
    Ry = [cosd(ThetaSaved(k)) 0 sind(ThetaSaved(k)); 0 1 0; -sind(ThetaSaved(k)) 0 cosd(ThetaSaved(k))];
    Rx = [1 0 0; 0 cosd(PhiSaved(k)) -sind(PhiSaved(k)); 0 sind(PhiSaved(k)) cosd(PhiSaved(k))];
    dt=(DATA_SI(102,k+1)-DATA_SI(102,k));
for j=1:8
    %Rotated vertex
    Result_1(j,:,k) = Rx*Ry*Rz*Cuboid(j,:)';
end
%Display realtime object and time
set(Obejct_IMU, 'Vertices', Result_1(:,:,k))
set(Realtime, 'String', sprintf('time = %.2f [s]', DATA_SI(102, k)))
%Display realtime sensor values
set(TimeValue_1, 'String', sprintf('Phi: %.2f  Theta: %.2f  Psi: %.2f', PhiSaved(k), ThetaSaved(k), PsiSaved(k)));
set(TimeValue_2, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(1,k), DATA_SI(2,k), DATA_SI(3,k)));
set(TimeValue_3, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(4,k), DATA_SI(5,k), DATA_SI(6,k)));
%set(TimeValue_4, 'String', sprintf('X: %.2f  Y: %.2f  Z: %.2f', DATA_SI(7,k), DATA_SI(8,k), DATA_SI(9,k)));
%Display timeline
set(Timeline_1, 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
set(Timeline_2, 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
set(Timeline_3, 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
%set(Timeline_4, 'XData', [DATA_SI(102, k) DATA_SI(102, k)]);
% pause(dt)
drawnow
end