%% initialize code
clear 
close all
clc

addpath(genpath('30cm'))
addpath(genpath('40cm'))
addpath(genpath('50cm'))

%% initialize variable

X30 = readmatrix(fullfile("30cm","X30"));
Y30 = readmatrix(fullfile("30cm","Y30"));
Z30 = readmatrix(fullfile("30cm","Z30"));
Mat_trans_30 = readmatrix(fullfile("30cm","Mat_0-c"));

X40 = readmatrix(fullfile("40cm","X40"));
Y40 = readmatrix(fullfile("40cm","Y40"));
Z40 = readmatrix(fullfile("40cm","Z40"));
Mat_trans_40 = readmatrix(fullfile("40cm","Mat_0-c"));

X50 = readmatrix(fullfile("50cm","X50"));
Y50 = readmatrix(fullfile("50cm","Y50"));
Z50 = readmatrix(fullfile("50cm","Z50"));
Mat_trans_50 = readmatrix(fullfile("50cm","Mat_0-c"));

%% correct position 
L = 0.285;
l = 0.2;
h = 0.038;
D = L/2-h/2;
d = l/2-h/2;

x_rif = 0.815*ones(9,1);
y_rif = -0.105*ones(9,1);
z_rif = 0.505*ones(9,1);

y_rif(1) = y_rif(1)-D; z_rif(1) = z_rif(1) + d;
y_rif(4) = y_rif(4)-D; z_rif(2) = z_rif(2) + d;
y_rif(7) = y_rif(7)-D; z_rif(3) = z_rif(3) + d;
y_rif(3) = y_rif(3)+D; z_rif(7) = z_rif(7) - d;
y_rif(6) = y_rif(6)+D; z_rif(8) = z_rif(8) - d;
y_rif(9) = y_rif(9)+D; z_rif(9) = z_rif(9) - d;

%% using the transform matrix to have point in function of the base

s = size(X30);

for i=1:s(2)
    
    for j=1:s(1)
        
        vect = [X30(j,i) Y30(j,i) Z30(j,i) 1]';
        sol = Mat_trans_30*vect;
        X30_tr(j,i) = sol(1);
        Y30_tr(j,i) = sol(2);
        Z30_tr(j,i) = sol(3);
    end
    
end

for i=1:s(2)
    
    for j=1:s(1)
        
        vect = [X40(j,i) Y40(j,i) Z40(j,i) 1]';
        sol = Mat_trans_40*vect;
        X40_tr(j,i) = sol(1);
        Y40_tr(j,i) = sol(2);
        Z40_tr(j,i) = sol(3);
    end
    
end

for i=1:s(2)
    
    for j=1:s(1)
        
        vect = [X50(j,i) Y50(j,i) Z50(j,i) 1]';
        sol = Mat_trans_50*vect;
        X50_tr(j,i) = sol(1);
        Y50_tr(j,i) = sol(2);
        Z50_tr(j,i) = sol(3);
    end
    
end

%% statistic elaboration

X30_mean = mean(X30_tr,2);
Y30_mean = mean(Y30_tr,2);
Z30_mean = mean(Z30_tr,2);

X40_mean = mean(X40_tr,2);
Y40_mean = mean(Y40_tr,2);
Z40_mean = mean(Z40_tr,2);

X50_mean = mean(X50_tr,2);
Y50_mean = mean(Y50_tr,2);
Z50_mean = mean(Z50_tr,2);

X30_std = std(X30_tr,0,2);
Y30_std = std(Y30_tr,0,2);
Z30_std = std(Z30_tr,0,2);

X40_std = std(X40_tr,0,2);
Y40_std = std(Y40_tr,0,2);
Z40_std = std(Z40_tr,0,2);

X50_std = std(X50_tr,0,2);
Y50_std = std(Y50_tr,0,2);
Z50_std = std(Z50_tr,0,2);

%% output

max_mean_30 = max([max(abs(X30_mean-x_rif)),max(abs(Y30_mean-y_rif)),max(abs(Z30_mean-z_rif))])*1000
max_mean_40 = max([max(abs(X40_mean-x_rif)),max(abs(Y40_mean-y_rif)),max(abs(Z40_mean-z_rif))])*1000
max_mean_50 = max([max(abs(X50_mean-x_rif)),max(abs(Y50_mean-y_rif)),max(abs(Z50_mean-z_rif))])*1000

max_std_30 = max([max(X30_std),max(Y30_std),max(Z30_std)])*1000
max_std_40 = max([max(X40_std),max(Y40_std),max(Z40_std)])*1000
max_std_50 = max([max(X50_std),max(Y50_std),max(Z50_std)])*1000

%% plot

figure()
plot(Y30_mean,Z30_mean,'b*','LineWidth',5)
hold on
plot(y_rif,z_rif,'ro','LineWidth',5)
legend('medi','coretti')
grid on
plot(Y30_tr,Z30_tr,'ko')
title('acquisizioni 30 cm')


figure()
plot(Y40_mean,Z40_mean,'b*','LineWidth',5)
hold on
plot(y_rif,z_rif,'ro','LineWidth',5)
legend('medi','coretti')
grid on
plot(Y40_tr,Z40_tr,'ko')
title('acquisizioni 40 cm')

figure()
plot(Y50_mean,Z50_mean,'b*','LineWidth',5)
hold on
plot(y_rif,z_rif,'ro','LineWidth',5)
legend('medi','coretti')
grid on
plot(Y50_tr,Z50_tr,'ko')
title('acquisizioni 50 cm')