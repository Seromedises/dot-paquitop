%% Main code with the file path and the optimization function
function sol = Calibration_matrix_finder()
clc
clear
close all
addpath(genpath('robot-10.3.1'))

% data loading
listing = dir("*_fb.txt");

for i =1:numel(listing)
    path(i) = string(fullfile(listing(i).folder,listing(i).name));
end

% output of Robot, Camera and Known Ground Matrix
ROB = readmatrix(path(2));
CAM = readmatrix(path(1));
GND = readmatrix(string(fullfile(listing(1).folder,'GND.txt')));

% first iteration
x0 = [0, 0, pi, 0, 0.0, -0];
% x0 = [0 0 0 0 0 0];

% calibration matrix
Topt = @(alpha,beta,gamma,x,y,z)    [cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta), x;
                                    cos(beta)*sin(gamma), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha),  y;
                                    -sin(beta),                                    cos(beta)*sin(alpha),                                    cos(alpha)*cos(beta),            z;
                                    0,                                                       0,                                                       0,                     1];
                            
options = optimset("MaxFunEvals", 100000, "TolX", 1e-8);
X = fminunc(@(x) error_fct(Topt(x(1),x(2),x(3),x(4),x(5),x(6)),ROB,CAM,GND),x0,options);

disp('Calibration matrix:')
disp(Topt(X(1),X(2),X(3),X(4),X(5),X(6)))
sol = Topt(X(1),X(2),X(3),X(4),X(5),X(6));
R = sol(1:3,1:3);
disp("Control on rotation matrix of calibration matrix:")
disp(R*R')

writematrix(sol,'Calib_Matrix.txt');
  
end
%% error function
function  err = error_fct(Topt,ROB,CAM,GND)
     err = 0;
    Ropt = Topt(1:3,1:3);

   % sampling ROB and CAM in 4x4 matrix
    dim = size(GND);
    dim = dim(1);
    stop = size(ROB);
    stop = stop(1)/dim(1);
    
    row = 1;
    flag = 0;
    while flag<stop
        
        Rob = ROB(row:row+dim-1,:);
        Cam = CAM(row:row+dim-1,:);
        row = row+dim;
        flag = flag+1;
        
        while Cam(1:3,4) == 0 & flag<stop
            Rob = ROB(row:row+dim-1,:);
            Cam = CAM(row:row+dim-1,:);
            row = row+dim;
            flag = flag+1;
        end
        
        Rob_inv = inv4(Rob);
        Cam_inv = inv4(Cam);

        Tith = Rob_inv*GND*Cam_inv;

        Rith = Tith(1:3,1:3);

        delta = abs([Topt(1:3,4)-Tith(1:3,4);vex(Ropt'*Rith-eye(3))]);

        if flag == 1
            sum_delta = delta;              
        end
        
        sum_delta = sum_delta + delta;
        
        err = norm(sum_delta);
        
    end  
    
end
%% inverse function
function invT = inv4(T)
    invT = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); 0 0 0 1];
end

function T = T05(alpha,beta,gamma,x,y,z)
    T = [cos(beta)*cos(gamma), cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma), sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta), x;
         cos(beta)*sin(gamma), cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha), y;
         -sin(beta),                                    cos(beta)*sin(alpha),                                    cos(alpha)*cos(beta), z;
                 0,                                                       0,                                                       0, 1];
 
end

% Topt =[cos(beta)*cos(gamma),-cos(beta)*sin(gamma), sin(beta), x;
%        sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma), -sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), -sin(alpha)*cos(beta), y;
%        -cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma), cos(alpha)*cos(beta), z;
%        0, 0, 0, 1];


