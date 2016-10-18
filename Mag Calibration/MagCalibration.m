%******************************************************************************************
% MAgnetometer Calibration
% Sérgio Patrão
% April 2015
% 
% Tested with:
% MPU9150
% LSM9DS0
% AltIMU
%******************************************************************************************

clc;
clear all;
close all;

% Read data file with extension *.txt and column separated by space " "
[nomedoficheiro,caminho] = uigetfile('*.txt');
try
data = dlmread([caminho,nomedoficheiro],' ');
catch me
end

% Magnetometer data is in columns 7, 8 and 9
x = data(:,7);
y = data(:,8);
z = data(:,9);
% For static tests, to know mean values on the accelerometer and gyroscope
% data, we can use this script. 
meanAccX = mean(data(:,1));
meanAccY = mean(data(:,2));
meanAccZ = mean(data(:,3));
meanGyrX = mean(data(:,4));
meanGyrY = mean(data(:,5));
meanGyrZ = mean(data(:,6));

figure;
hold on;
plot3( x, y, z, '-r' ); % original magnetometer data


% Fit magnetometer data to an ellipsoid
[offset, radius, eigenvectors, algebraic] = ellipsoid_fit([x, y, z]);

offset %Print offset values that defines the Hard-Iron Calibration
radius;
eigenvectors;
algebraic;
% compensate distorted magnetometer data
% eigenvectors is an orthogonal matrix, so it's transpose is the same as it's
% inverse

% Hard-Iron Calibration
magCal = [x - offset(1), y - offset(2), z - offset(3)]'; 

% Soft-Iron Calibration
scale = inv([radius(1) 0 0; 0 radius(2) 0; 0 0 radius(3)]) * min(radius); % scale factors
map = eigenvectors'; % transformation matrix to map ellipsoid axes to coordinate system axes
invmap = eigenvectors; % inverse of above
soft_iron = invmap*scale*map;
magCal = soft_iron * magCal; % do compensation

soft_iron % Print Soft-Iron Matrix values that defines the Soft-Iron calibration

plot3(magCal(1,:), magCal(2,:), magCal(3,:), '-b'); % compensated data
view( -70, 40 );
grid on;
axis vis3d;
axis equal;
legend ('Uncalibrated','Calibrated');


% Paint the surface of the sphere and ellipsoid
% Comment next lines for better performnce
maxd = max(radius);
step=maxd/50;
[xp, yp, zp] = meshgrid(-maxd:step:maxd + offset(1), -maxd:step:maxd + offset(2), -maxd:step:maxd + offset(3));
Ellipsoid = algebraic(1) *xp.*xp +   algebraic(2) * yp.*yp + algebraic(3)   * zp.*zp + ...
          2*algebraic(4) *xp.*yp + 2*algebraic(5) * xp.*zp + 2*algebraic(6) * yp.*zp + ...
          2*algebraic(7) *xp     + 2*algebraic(8) * yp     + 2*algebraic(9) * zp;
p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
set(p, 'FaceColor', 'r', 'EdgeColor', 'none');
alpha(0.5);
view( -70, 40 );
grid on
axis vis3d;
axis equal;
camlight;
lighting phong;

[offset, radius, eigenvectors, algebraic] = ellipsoid_fit([magCal(1,:)',magCal(2,:)', magCal(3,:)']);

maxd = max(radius);
step = maxd / 50;
[xp, yp, zp] = meshgrid(-maxd:step:maxd + offset(1), -maxd:step:maxd + offset(2), -maxd:step:maxd + offset(3));
Ellipsoid = algebraic(1) *xp.*xp +   algebraic(2) * yp.*yp + algebraic(3)   * zp.*zp + ...
          2*algebraic(4) *xp.*yp + 2*algebraic(5) * xp.*zp + 2*algebraic(6) * yp.*zp + ...
          2*algebraic(7) *xp     + 2*algebraic(8) * yp     + 2*algebraic(9) * zp;
p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
set(p, 'FaceColor', 'b', 'EdgeColor', 'none');
alpha(0.5);
view( -70, 40 );
grid on
axis vis3d;
axis equal;
camlight;
lighting phong;


x1=[0,30];
y1=[0,0];
z1=[0,0];
plot3(x1,y1,z1);

x1=[0,0];
y1=[0,30];
z1=[0,0];
plot3(x1,y1,z1);

x1=[0,0];
y1=[0,0];
z1=[0,30];
plot3(x1,y1,z1);