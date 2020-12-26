% Visualization script
% This octave script displays true, measurement and state_estimate data

% Sensor positions
s1 = [-200 100]';
s2 = [-200 -100]';

var = load ("matlab_data/unfilteredPos.mat");
meas = var.unfilteredPos;
var2 = load("matlab_data/grdtruth.mat");
gtruth = var2.groundTruth;

state_estimates = load("matlab_data/stateEstimates.mat");

figure('Color','white','Position',[500  300  603  429]);
grid on; hold on %, axis equal

sc1 = scatter (s1(1), s1(2), 100, 'b', "filled",'DisplayName','sensor 1 location');
sc2 = scatter (s2(1), s2(2), 100, 'g', "filled",'DisplayName','sensor 2 location');

% Plotting the 

% Plotting the results of the x dimension
p1 = plot(gtruth(:,1),gtruth(:,2), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position');
%plot(gtruth(:,1))

hold on

% Plotting the measurements
p2 = plot(meas(:,1),meas(:,2), 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1, 'DisplayName','Measured position');

hold on

p3 = plot(state_estimates(:,1),state_estimates(:,2), 'Color', 'red', 'LineWidth',2, 'DisplayName','estimated position');

axis manual
xlabel 'pos x', ylabel 'pos y'

legend([p1 p2 p3 sc1 sc2], 'Location','southwest')

%title(sprintf('Case %d, filter type: %s',case_i,filter_type{1}))