%% 1-D example 
% 1. Prameter setting
dim = 1; % dimension
ts = [0 2 3]; % knots 
order = 7; % polynomial order 
optimTarget = 'poly-coeff'; % 'poly-coeff' or 'end-derivative'
maxConti = 4; % maximally imposed continuity between segment 
objWeights = [10 1 1 10];  % 1 2 3 4 th order derivatives 

% 2. Pin (FixPin or LoosePin)
pin1 = struct('t',0,'d',0,'X',0);
pin2 = struct('t',0,'d',2,'X',1);
pin3 = struct('t',0,'d',1,'X',2); 
pin4 = struct('t',0.3,'d',0,'X',[0.7 0.8]);
pin5 = struct('t',2,'d',2,'X',[0.7 0.8]);
pin6 = struct('t',2.4,'d',1,'X',[0.0 1.9]);
pin7 = struct('t',3,'d',0,'X',1);

% 3. Generate trajectory object and path
pTraj = PolyTrajGen(ts,order,optimTarget,dim,maxConti); % construct the functor
pTraj.setDerivativeObj(objWeights); % set the objective function for penalizing the derivatives 
pTraj.addPinSet([pin1 pin2 pin3 pin4 pin5 pin6 pin7]); % impose pins
pTraj.solve; % quadratic programming

% 4. Plot 
figh = figure(2);
clf
titleStr1 = sprintf('poly order : %d / max continuity: %d / ',order,maxConti);
titleStr2 = [' minimzed derivatives order: ', num2str(find(objWeights > 0))];
sgtitle(strcat(titleStr1,titleStr2))
set(figh,'Position',[193 294 1473 418]);
pTraj.showTraj(4,figh)

%% 3-D example 

% 1. Prameter setting
dim = 3;
knots = [0 2 4 7]; % knots
order = 6; % polynomial order 
optimTarget = 'poly-coeff'; % 'poly-coeff' or 'end-derivative'
maxConti = 4; % maximally imposed continuity between segment 
objWeights = [0 1 1];  % 1 2 3 th order derivative
pTraj = PolyTrajGen(knots,order,optimTarget,dim,maxConti);

% 2. Pin 
% 2.1 FixPin 
ts = [0 2 4 7]; % knots
Xs = [0 2 5 7 ; ...
        0 -1 3 -5 ; ...
        0 2 4 5]; % waypoints

Xdot = [ 0 ;
            0 ; 
            0]; % initial velocity

Xddot = [ 0 ;
              0 ; 
              0]; % initial acceleration

% Order 0 pin (waypoints)
for m = 1:size(Xs,2)
    pin = struct('t',ts(m),'d',0,'X',Xs(:,m));
    pTraj.addPin(pin);
end   
% Order 1 pin 
pin = struct('t',ts(1),'d',1,'X',Xdot);
pTraj.addPin(pin);
% Order 2 pin 
pin = struct('t',ts(1),'d',2,'X',Xddot);
pTraj.addPin(pin);

% 2.2 LoosePin 
passCube = [3 4.2 ; -3 -2 ; 1 2];
pin = struct('t',3,'d',0,'X',passCube);
pTraj.addPin(pin);

% 3. Solve 
pTraj.setDerivativeObj(objWeights); % set the objective function for penalizing the derivatives 
pTraj.solve;

% 4. Plot 
figh3 = figure(3); clf
figh4 = figure(4); clf
titleStr1 = sprintf('poly order : %d / max continuity: %d / ',order,maxConti);
titleStr2 = [' minimzed derivatives order: ', num2str(find(objWeights > 0))];
sgtitle(strcat(titleStr1,titleStr2))
set(figh3,'Position',[193 294 1473 610]);
plotOrder = 3; % Until 3rd order derivatives 
pTraj.showTraj(plotOrder,figh3) % plot element-wise trajectory 
pTraj.showPath(figh4)
view([-41 33])    
axis([-1 11 -6 10 0 8])    


    
