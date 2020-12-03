%MECH103 Final Project
%Collaborators Alex, Daniel, Myron
%Start Date 11/27/2020
%Last Edit 11/27/2020

%A ‘black box’ that will record flight data from a remote-controlled model airplane.
%The system will also be able to alert the ‘pilot’ if the plane is in a dive with lights and a buzzer.
%
%The board will be mounted on a plane and record sensor readings from a gyroscope,
% accelerometer, compass, and barometer to record and plot pitch,
% bank, magnetic heading, vertical speed, and pressure altitude.
% The data will need to be saved to an SD card. 
%
%The alerting system will work by constantly monitoring the pitch,
% bank, and vertical speed, and trigger LEDs and a buzzer if the values go out of range.
% The pitch and bank alerting system can be tested and demonstrated on the ground,
% and an in-flight test can be recorded using a GoPro. 

%% Code to Initiate Arduino


%% (We may need something here to call or initiate our new sensors, but we will learn more later




%% Code to name ports, sensors etc


%% A While Loop here to constantly read data from the sensors

% algorithm to calculate vertical speed
%     store altitude information for x seconds, compare to current altitude and divide by x

% put code to record date here as well

    
    


%% Logic will be placed here that set off alarms when certain peramiters are hit
% Example: If pitch is too high and speed is too low, set off alarm

% while true
%     if pitch out of limits
%         run pitch alert function
%     end
%     
%     if bank angle out of limits
%         run bank angle alert function
%     end
%     
%     if vertical speed excessive
%         run vertical speed function
%     end