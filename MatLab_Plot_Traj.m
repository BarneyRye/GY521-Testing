%% ================== INIT ==================
clear all;
close all;
clc;

%% ================== LOAD DATA ==================
dataraw = readmatrix("data_log.csv");
data = dataraw(4:end,:);

time = data(:,1);
ax = data(:,2);
ay = data(:,3);
az = data(:,4);
gx = data(:,5);
gy = data(:,6);
gz = data(:,7);

dataPoints = size(data,1);

% Time step (ms -> s if needed)
dt = [0.01; diff(time)];
if max(dt) > 1
    dt = dt / 1000; % convert ms to seconds
end

%% ================== UNIT CONVERSION ==================
% MPU6050 typical units
gx = deg2rad(gx);
gy = deg2rad(gy);
gz = deg2rad(gz);

% Accelerometer should be m/s^2
accelScale = 9.81; % if logged in g
if mean(abs(ax)) < 5
    ax = ax * accelScale;
    ay = ay * accelScale;
    az = az * accelScale;
end

%% ================== CONSTANTS ==================
g = [0; 0; 9.81];

%% ================== STATE INIT ==================
q = [1; 0; 0; 0];    % quaternion (w x y z)
v = zeros(3,1);
p = zeros(3,1);
bg = zeros(3,1);
ba = zeros(3,1);

P = eye(15) * 0.01;

Q = diag([
    0.01*ones(1,3), ...   % gyro noise
    0.1*ones(1,3),  ...   % accel noise
    0.0001*ones(1,3),... % gyro bias RW
    0.001*ones(1,3)      % accel bias RW
]);

R = eye(3) * 0.5;

%% ================== STORAGE ==================
pos = zeros(dataPoints,3);
vel = zeros(dataPoints,3);

%% ================== EKF LOOP ==================
for k = 1:dataPoints
    dt_k = dt(k);

    acc = [ax(k); ay(k); az(k)];
    gyro = [gx(k); gy(k); gz(k)];

    %% ---- Prediction ----
    omega = gyro - bg;
    q = quat_integrate(q, omega, dt_k);

    Rwb = quat2rotm(q');
    a_world = Rwb * (acc - ba) - g;

    v = v + a_world * dt_k;
    p = p + v * dt_k + 0.5 * a_world * dt_k^2;

    % State transition
    F = eye(15);
    F(1:3,10:12) = -eye(3)*dt_k;
    F(4:6,13:15) = -eye(3)*dt_k;
    F(7:9,4:6)   = eye(3)*dt_k;

    G = zeros(15,12);
    G(1:3,1:3)   = eye(3);
    G(4:6,4:6)   = eye(3);
    G(10:12,7:9) = eye(3);
    G(13:15,10:12) = eye(3);

    P = F*P*F' + G*Q*G'*dt_k;

    %% ---- Update (gravity correction) ----
    z = acc / norm(acc);
    h = Rwb' * [0;0;1];
    y = z - h;

    H = zeros(3,15);
    H(:,1:3) = skew(h);

    S = H*P*H' + R;
    K = P*H'/S;

    dx = K*y;
    P = (eye(15) - K*H)*P;

    dq = [1; 0.5*dx(1:3)];
    q = quatnormalize(quatmultiply(q', dq'))';

    v = v + dx(4:6);
    p = p + dx(7:9);
    bg = bg + dx(10:12);
    ba = ba + dx(13:15);

    pos(k,:) = p';
    vel(k,:) = v';
end

%% ================== PLOTS ==================
figure;
plot3(pos(:,1), pos(:,2), pos(:,3), 'LineWidth', 1.5);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Estimated World Frame Position');

figure;
plot(time, pos);
legend('X','Y','Z');
xlabel('Time');
ylabel('Position [m]');
title('Position vs Time');

%% ================== FUNCTIONS ==================
function q = quat_integrate(q, omega, dt)
    Omega = [ 0, -omega';
              omega, -skew(omega) ];
    q = q + 0.5 * Omega * q * dt;
    q = q / norm(q);
end

function S = skew(w)
    S = [  0   -w(3)  w(2);
          w(3)   0   -w(1);
         -w(2)  w(1)   0  ];
end





