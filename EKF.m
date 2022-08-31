clc;
clear all;

%% Params
dt = 0.1;
Data = csvread('Radar_Lidar_Data1.csv',1,1);
Zradar = [];
Zlidar = [];
EKF_Path = [];
N = 150;


%% State Transition Matrix
% Jacobian of A 
F = [[1, 0, dt, 0];
     [0, 1, 0, dt];
     [0, 0, 1, 0];
     [0, 0, 0, 1]];

% Control input matrix
B = [(dt^2)/2 (dt^2)/2 dt dt]';

u = 0;

% Process noise
P = [[1, 0, 0, 0];
     [0, 1, 0, 0];
     [0, 0, 1000, 0];
     [0, 0, 0, 1000]];

% Covariance
Q = [(dt^2)/4 0 (dt^3)/2 0;
     0 (dt^2)/4 0 (dt^3)/2;
     (dt^3/2) 0 (dt^2) 0;
     0 (dt^3)/2 0 (dt^2)];

% Measurement model
H = [[1, 0, 0, 0];
     [0, 1, 0, 0]];

I = eye(4);

% Uncertainty in radar and lidar
R_l = [[0.0025, 0];
       [0, 0.0025]];
  
R_r = [[0.09, 0, 0];
      [0, 0.005, 0];
      [0, 0, 0.09]];
 
if (Data(1,1) == 1)
    x = [Data(1,2); Data(1,3); 0; 0];
else
    x = [Data(1,2); Data(1,3); Data(1,4); 0];
end


for n = 1:N
    
    if (Data(n,1) == 2)
        
        %prediction
        x = F * x + B*u;
        P = F * P * transpose(F) + Q;

        %measurement update
        Z = Data(n,2:4);
        X = Z(1)*cos(Z(2));
        Y = Z(1)*sin(Z(2));
        VX = Z(3)*cos(Z(2));
        VY = Z(3)*sin(Z(2));

        c1 = X^2 + Y^2;
        c2 = sqrt(c1);
        c3 = c1 * c2;
        if (c1==0 || c2==0 || c3==0)
            H_Jac = [[0, 0, 0, 0];
                     [0, 0, 0, 0];
                     [0, 0, 0, 0]];
        else
            H_Jac = [[X/c2, Y/c2, 0, 0];
                    [-Y/c1, X/c1, 0, 0];
                    [(Y*(VX*Y-VY*X))/c3, (X*(X*VY-Y*VX))/c3, X/c2, Y/c2]];
        end
        Z_Car = [X; Y; VX; VY];
        y = transpose(Z) - (H_Jac * Z_Car);
        S = H_Jac * P * transpose(H_Jac) + R_r;
        K = P * transpose(H_Jac) * inv(S);
        x = Z_Car + (K * y);
        P = (I - (K * H_Jac)) * P;
        EKF_Path = [EKF_Path;[x(1),x(2)]];
        Zradar = [Zradar; Data(n,2:4)];
    
    else
        
        %prediction
        x = (F * x) + B*u;
        P = F * P * transpose(F) + Q;

        %measurement update
        Z = Data(n,2:3);
        y = transpose(Z) - (H * x);
        S = H * P * transpose(H) + R_l;
        K = P * transpose(H) * inv(S);
        x = x + (K * y);
        P = (I - (K * H)) * P;
        EKF_Path = [EKF_Path;[x(1),x(2)]];
        Zlidar = [Zlidar; Data(n,2:3)];
    end
    
end

for i = 1:length(Zradar)
    Radar_Measurement_Cart(i,:) = [[Zradar(i,1),0];[0, Zradar(i,1)]]*[cos(Zradar(i,2));sin(Zradar(i,2))];
end


%% Plot
hold on;
groundTruthX = Data(:,6);
groundTruthY = Data(:,7);
groundTruthX = groundTruthX(1:N);
groundTruthY = groundTruthY(1:N);

fusedPathX = EKF_Path(:,1);
fusedPathY = EKF_Path(:,2);


plot(groundTruthX(1:N),groundTruthY(1:N),'LineWidth', 1.5, 'LineStyle','-', 'Color', 'r');
plot(fusedPathX, fusedPathY, 'LineWidth', 2, 'LineStyle' ,'-', 'Color', 'b' );
scatter(Zlidar(:,1),Zlidar(:,2), '*');
scatter(Radar_Measurement_Cart(:,1),Radar_Measurement_Cart(:,2), 'o', 'g');

legend('Ground','EKF','Lidar','Radar');
grid on;
grid minor;
axis square;
title('Sensor Fusion using EKF');
xlabel('Path [m]');
hold off;


%% Error Calculation

true = sqrt(groundTruthX.^2 + groundTruthY.^2); 
predicted = sqrt(fusedPathX.^2 + fusedPathY.^2);
RMSE = sqrt(mean(((true - predicted).^2))); 

