close all;
clear;
%% connection between matlab and arduino uno
a = arduino('COM3', 'Uno', 'Libraries', 'I2C');
%% creating mpu6050 object and setting its parameters
fs = 50; % Sample Rate in Hz   
imu = mpu6050(a,'SampleRate',fs,'SamplesPerRead',5,'ReadMode','Latest');

GyroscopeNoise = 3.0462e-06; % GyroscopeNoise (variance) in units of rad/s

AccelerometerNoise = 0.0061; % AccelerometerNoise (variance) in units of m/s^2

%% for visualisation
viewer = HelperOrientationViewer('Title',{'IMU Filter'});

%%
FUSE = imufilter('SampleRate',fs, 'GyroscopeNoise',GyroscopeNoise,'AccelerometerNoise', AccelerometerNoise);

stopTimer=1000;
tic;

while(toc < stopTimer)
    [accelReadings,timestamp] = readAcceleration(imu);
    [gyroReadings,timestamp] = readAngularVelocity(imu);

%     disp(accelReadings); %% display readings on command window    
%     disp(gyroReadings);  %% display readings on command window 
     
    rotators = FUSE(accelReadings,gyroReadings);
    for j = numel(rotators)
        viewer(rotators(j)); %% returns orientation in quaternion
        eul = 180*quat2eul(rotators(j))/pi; %% returns quaternion TO euler angles
        disp(eul); %% display euler angles
    end
end