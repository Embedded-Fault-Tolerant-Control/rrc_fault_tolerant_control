% Choose which log file to read
% filename = 'dist/simulation-set-1/quad-x/m1.mat';
% filename = 'dist/simulation-set-1/quad-x/m1.mat';
filename = 'temp/arrdata2.mat';

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
selectedArray = DATASET.ATT;

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

subplot(2,3,1);
rateP = selectedArray.DesPitch;
plot(x*10^-6, rateP, 'LineWidth', 1.25);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel('Time in seconds');
ylabel('Desired Pitch');
set(gca,'FontSize', 14);

subplot(2,3,2);
rateR = selectedArray.DesRoll;
plot(x*10^-6, rateR, 'LineWidth', 1.25);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel('Time in seconds');
ylabel('Desired Roll');
set(gca,'FontSize', 14);

subplot(2,3,3);
rateY = selectedArray.DesYaw;
plot(x*10^-6, rateY, 'LineWidth', 1.25);
line([timestampInSeconds timestampInSeconds], ylim, 'Color',[1,0,0], 'LineWidth', 1);
xlabel('Time in seconds');
ylabel('Desired Yaw');
set(gca,'FontSize', 14);


filename = 'temp/arrdata.mat';
varsToRead = {'Pitch','Roll'};

DATASET = load(filename, varsToRead{:});
PitchArr = DATASET.Pitch;
RollArr = DATASET.Roll;

subplot(2,3,4);
plot(21*10^-6, PitchArr, 'LineWidth', 1.25);
xlabel('Time in seconds');
ylabel('On-board Desired Pitch');
set(gca,'FontSize', 14);

subplot(2,3,5);
plot(21*10^-6, RollArr, 'LineWidth', 1.25);
xlabel('Time in seconds');
ylabel('On-board Desired Roll');
set(gca,'FontSize', 14);