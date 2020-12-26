pkg load statistics
% Number of points
N = 100;
% prior state and covariance

X_0 = [0 0 14 0 0]';
P_0 = diag([10 10 2 pi/180 5*pi/180].^2);
% Sampling time
T = 1;
% Sensor positions
s1 = [-200 100]';
s2 = [-200 -100]';

% Process and Measurement Noise Covariances
sigma_V = 1;
sigma_W = pi/180;

sigma_Phi_1 = 0.5*pi/180;
sigma_Phi_2 = 0.5*pi/180;

Q = diag([0 0 T*sigma_V^2 0 T*sigma_W^2]);
R = diag([sigma_Phi_1^2 sigma_Phi_2^2]);

% The first task is to generate the "Assumed" true state sequences

% Motion model function handle. Note how sample time T is inserted into the function.
motionModel = @(x) coordinatedTurnMotion(x, T);

% generate state sequence
X = genNonLinearStateSequence(X_0, P_0, motionModel, Q, N);

% Generating the simulated measurements 

% This is the measurement model
measModel = @(X) dualBearingMeasurement(X, s1, s2);

% Generate measurements sequences
Y = genNonLinearMeasurementSequence(X, measModel, R);
% calcualte unfiltered position from sensors given angles
Xm(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
Xm(2,:) = s1(2) + tan(Y(1,:)) .* ( Xm(1,:) - s1(1) );

%p1 = plot(X(1,:),X(2,:), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position');
%hold on
%p3 = plot(Xm(1,:),Xm(2,:), 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1, 'DisplayName','Measured position');

measurementData = Y';

save matlab_data/measData.mat measurementData

groundTruth = X';

save matlab_data/grdtruth.mat groundTruth  

unfilteredPos = Xm';
save matlab_data/unfilteredPos.mat unfilteredPos  

var = load ("matlab_data/measData.mat");

meas = var.measurementData

var2 = load("matlab_data/grdtruth.mat");

gtruth = var2.groundTruth

