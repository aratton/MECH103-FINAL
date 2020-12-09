%MECH 103 Final
%Date Created: December 8 2020
%Date Last Edited: December 8 2020
%Group: Alex, Daniel, Myron

%The following script creates graphs based on the data collected from our
%datalogger arduino script. The data collected using this script is saved
%in a .csv file and uploaded to a computer to be plotted using this script


%Tinmstamp array that counts in halfseconds since start until the total
%number of seconds run
timestamp = [0:0.5:24];


%Create Array from the csv file
Array=csvread('TEST.CSV');

t = tiledlayout(2,2);


%Altitude Graph x and y axis (x axis is time)

altitude = Array(:, 1);
nexttile
plot(timestamp, altitude);

title('Altitude vs Time');
xlabel('Time in Seconds Since Takeoff');
ylabel('Altitude');
grid ON


%Roll Graph x and y axis (x axis is time)

roll = Array(:,2);
nexttile
plot(timestamp, roll);

title('Roll vs Time');
xlabel('Time in Seconds Since Takeoff');
ylabel('Roll');
grid ON

%Pitch Graph x and y axis (x axis is time)

pitch = Array(:,3);
nexttile
plot(timestamp, pitch);

title('Pitch vs Time');
xlabel('Time in Seconds Since Takeoff');
ylabel('Pitch');
grid ON

%Yaw Graph x and y axis (x axis is time)

yaw = Array(:,4);
nexttile
plot(timestamp, yaw);

title('Yaw vs Time');
xlabel('Time in Seconds Since Takeoff');
ylabel('Yaw');
grid ON
