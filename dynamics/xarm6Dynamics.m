% deriving standard mechanical form of the xarm6

%% first, define the xarm's joints and mass characteristics

% theta (rads), d (mm), alpha (rads), a (mm), offset (rads)
% offset is the offset joint angle to the mechanical zero position
xarm.j1 = [0, 267, -pi/2, 0, 0];
xarm.j2 = [0, 0, 0, 289.48866, -1.3849179];
xarm.j3 = [0, 0, -pi/2, 77.5, 1.3849179];
xarm.j4 = [0, 342.5, pi/2, 0, 0];
xarm.j5 = [0, 0, -pi/2, 76, 0];
xarm.j6 = [0, 97, 0, 0, 0];

% these com and I tensor coordinates are in meters and kg
xarm.m1.m = 2.16;
xarm.m1.com = [-0.002 0.02692 -0.01332];
xarm.m1.inertia = diag([0.00539487 0.00519906 0.00281397]);
xarm.m2.m = 1.17;
xarm.m2.com = [0.03531 -0.21398 0.03386];
xarm.m2.inertia = diag([0.017867 0.017867 0.017867]);
xarm.m3.m = 1.384;
xarm.m3.com = [0.06781 0.10749 0.01457];
xarm.m3.inertia = diag([0.004543 0.004543 0.004543]);
xarm.m4.m = 1.115;
xarm.m4.com = [-0.00021 0.02578 -0.02538];
xarm.m4.inertia = diag([0.00440091 0.00406855 0.00103407]);
xarm.m5.m = 1.275;
xarm.m5.com = [0.05428 0.01781 0.00543];
xarm.m5.inertia = diag([0.00289757 0.0023276 0.000951789]);
xarm.m6.m = 0.65116;
xarm.m6.com = [-0.000544662 -0.00143065 0.0383418];
xarm.m6.inertia = diag([0.000818872 0.000680759 0.00033665]);

%% next, assemble the lagrangian for each input function