%% NLMPC
% load('waypoints.mat')
clear all;

nx = 4;
ny = 4;
nu = 2;
nlobj = nlmpc(nx, ny, nu);

% Obstacle initial states [x, y, orientation, velocity]
obsState = [10, -2.5, 0, 0;
    30, -2.5, 0, 0;
    30, 2.5, pi, 0];

% Obstacle X and Y
obs = [obsState(1,1:2);
    obsState(2,1:2);
    obsState(3,1:2)];

% sample time
Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 2;

%% setting up the transition model, Jacobian and output functions
nlobj.Model.StateFcn = 'discreteStateEq';
nlobj.Jacobian.StateFcn = 'discretestateJacobian';

nlobj.Model.IsContinuousTime = false;

nlobj.Model.NumberOfParameters = 2;

nlobj.Model.OutputFcn = 'outputFunc';
nlobj.Jacobian.OutputFcn = 'outputJacobian';

nlobj.Weights.OutputVariables = [3 3 1 1];
nlobj.Weights.ManipulatedVariablesRate = [1 0.1];

% Defined the the custom contraints
nlobj.Optimization.CustomIneqConFcn = 'ObstacleConstraint';

%% Setting up the limits for input and outputs
% Road limits
nlobj.States(2).Min = -2.5;
nlobj.States(2).Max = 2.5;

nlobj.MV(1).Min = -10;
nlobj.MV(1).Max = 10;

nlobj.MV(2).Min = -pi/3;
nlobj.MV(2).Max = pi/3;

nlobj.MV(2).RateMin = -0.2;
nlobj.MV(2).Max = 0.2;

%% Validating the nlmpc
x0 = [0; -2.5; 0; 0];
u0 = [0; 0];
validateFcns(nlobj,x0,u0,[],{Ts, obs});

%% Initializing the model
% set the initial states
x = [0; -2.5; 0; 0];

% set the output state
y = x;

%
mv = [0; 0];

% goal location
yref = [60 -2.5 0 0];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts, obs};

% Run the simulation.
Duration = 10;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
uHistory = mv;

% obstacle state history
% Obstacle velocity
obsV = 10;
for ob=1:length(obs)
    obsHistory(ob,1:nx,1) = [obs(ob,1); obs(ob,2); obsState(ob,3); obsV];
end

% unseen obstacle - time that the obstacle should apprear
time_obs = 15;


elpt = 0;
for ct = 1:(Duration/Ts)

    tic;

    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,yref,[],nloptions);
    
    % Implement first optimal control move and update plant states.
    x = discreteStateEq(x,mv,Ts);
    
    for ob=1:length(obs)
        obsHistory(ob,1:4,ct+1) = discreteStateEq(obsHistory(ob,1:4,ct),[obsV/25 + 1; 0], Ts, obs);
        obs(ob,1:2) = obsHistory(ob,1:2,ct+1);
    end
    nloptions.Parameters = {Ts, obs};

    % Generate sensor data with some white noise.
    
    pos_noise = randn(2,1)*1;
    angle_noise = randn(1,1)*0.1;
    velocity_noise = randn(1,1)*1;
    y = x + [pos_noise;angle_noise;velocity_noise]; 
    elpt = elpt + toc;
    
    % Save plant states for display.
    xHistory = [xHistory x];
    uHistory = [uHistory mv];
    waitbar(ct*Ts/20,hbar);
end
close(hbar)
fprintf('Elapsed Time: %.4f seconds\n', elpt/ct);
lane_length = 60;
plotting(yref, obsHistory, xHistory, time_obs, lane_length, nlobj);
plot_graphs;

