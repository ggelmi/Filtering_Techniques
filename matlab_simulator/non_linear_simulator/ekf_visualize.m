% Visualization script
% This octave script displays true, measurement and state_estimate data

var = load ("matlab_data/unfilteredPos.mat");
meas = var.unfilteredPos
var2 = load("matlab_data/grdtruth.mat");
gtruth = var2.groundTruth

state_estimates = load("matlab_data/stateEstimates.mat")


% Plotting the results of the x dimension
p1 = plot(gtruth(:,1),gtruth(:,2), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position');
%plot(gtruth(:,1))

hold on

% Plotting the measurements
p2 = plot(meas(:,1),meas(:,2), 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1, 'DisplayName','Measured position');

hold on

p3 = plot(state_estimates(:,1),state_estimates(:,2), 'Color', 'red', 'LineWidth',2, 'DisplayName','estimated position');

legend({' True Data','Measurement Data', 'State Estimates'})

