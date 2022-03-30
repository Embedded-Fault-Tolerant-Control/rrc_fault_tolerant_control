% Choose which log file to read
% filename = 'dist/simulation-set-1/quad-x/m1.mat';
filename = 'dist/prototype-set-1/hover/5mm-cut-hover/5mm_cut_hover_1.mat';

% Choose variables to read - you can open the .mat file and read the (upto
% 4 digit) var names. You can add the desired var names here.
varsToRead = {'AHR2','ATT','BARO','IMU','MAG','PARM','RATE','SIM'};

DATASET = load(filename, varsToRead{:});
% Change searchParam value; by replacing the 1 to any other integer in
% [1,2,3,4] for QuadCopter or [1,2,3,4,5,6] for HexaCopter
searchParam = 'SERVO1_FUNCTION';

searchParamUses = ~cellfun('isempty',strfind(cellstr(DATASET.PARM.Name),searchParam));
searchLastParamUse = find(searchParamUses);

% Index of Last usage of param stored here
lastParamUse = searchLastParamUse(end);

% Timestamp (TimeUS) variable of the above packet - this is the time in
% microseconds, when the motor is overridden. Fault occurs immediately
% after this.
lastParamTimestamp = DATASET.PARM.TimeUS(lastParamUse);

% Select the dataset array you wish to work on (Baro, Gyro, EKF, ...)
selectedArray = DATASET.RATE;

% Find closest timestamp in the BARO (Barometer) data array.
% val stores the time value in us
% key stores the key of that timestamp in BARO. i.e. this is the closest
% timestamp in BARO, when compared to the input SERVO1_FUNCTION timestamp's
% last call.
[val, key] = min(abs(selectedArray.TimeUS-lastParamTimestamp));
RATELastTimestamp=selectedArray.TimeUS(key);

% TODO: Marker
disp("RATELastTimestamp: " + RATELastTimestamp);
timestampInSeconds = RATELastTimestamp*10^-6;

% The selected array can now be observed from the selected key onwards.
% i.e. this is the timestamp after which motor fault occurs.

% Create plots
x = (selectedArray.TimeUS);
f= figure;
rateP = selectedArray.P;
ratePDes = selectedArray.PDes;
t1 = nexttile;
plot(x*10^-6, rateP, 'LineWidth', 2);
hold on;
plot(x*10^-6, ratePDes, 'Color',[1,0.7,0], 'LineStyle', '--', 'LineWidth', 2);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel(t1,'Time in seconds');
ylabel(t1,'Pitch and Pitch Desired');
legend('Pitch','PitchDes','ERR','Location','southwest');
set(gca,'FontSize', 14);

rateY = selectedArray.Y;
rateYDes = selectedArray.YDes;
t2 = nexttile;
plot(x*10^-6, rateY, 'LineWidth', 2);
hold on;
plot(x*10^-6, rateYDes, 'Color',[1,0.7,0], 'LineStyle', '--', 'LineWidth', 2);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel(t2,'Time in seconds');
ylabel(t2,'Yaw and Yaw Desired');
legend('Yaw','YawDes','ERR','Location','southwest');
set(gca,'FontSize', 14);

rateR = selectedArray.R;
rateRDes = selectedArray.RDes;
t3 = nexttile;
plot(x*10^-6, rateR, 'LineWidth', 2);
hold on;
plot(x*10^-6, rateRDes, 'Color',[1,0.7,0], 'LineStyle', '--', 'LineWidth', 2);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel(t3,'Time in seconds');
ylabel(t3,'Roll and Roll Desired');
legend('Roll','RollDes','ERR','Location','southwest');
set(gca,'FontSize', 14);

% Select the dataset array you wish to work on (Baro, Gyro, EKF, ...)
selectedArray = DATASET.BARO;

% Find closest timestamp in the BARO (Barometer) data array.
% val stores the time value in us
% key stores the key of that timestamp in BARO. i.e. this is the closest
% timestamp in BARO, when compared to the input SERVO1_FUNCTION timestamp's
% last call.
[val, key] = min(abs(selectedArray.TimeUS-lastParamTimestamp));
RATELastTimestamp=selectedArray.TimeUS(key);

timestampInSeconds = RATELastTimestamp*10^-6;

% The selected array can now be observed from the selected key onwards.
% i.e. this is the timestamp after which motor fault occurs.

% Create plots
x = (selectedArray.TimeUS);

baroAlt = selectedArray.Alt;
t4 = nexttile;
plot(x*10^-6, baroAlt, 'LineWidth', 2);
hold on;
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel(t4,'Time in seconds');
ylabel(t4,'Altitude');
legend('Alt','ERR','Location','southwest');
set(gca,'FontSize', 14);