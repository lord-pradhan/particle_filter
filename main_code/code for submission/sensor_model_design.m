%% script to design sensor model for laser range finder
clear
close all

%% params
max_range = 8200; gauss_sd = 280.0; lambda_short = 0.07;
wt_gauss=8.0; wt_short=0.8; wt_max=0.001; wt_rand=0.00001;

z_true = 2000;
%% prob distribution
z_meas = linspace(0, max_range, 10000);
p_vect = zeros(size(z_meas));
ct=1;

%% test
% p_gauss_test =  exp( -(72- z_true)*(72- z_true)/(2.0*gauss_sd*gauss_sd) ) ...
%                     / (gauss_sd*sqrt(2.0*pi))
% 
% norm_exp = 1.0 / (1.0 - exp(-lambda_short*z_true));                
% p_short_test = norm_exp*lambda_short*exp(-lambda_short*72)
%%
for i_meas = z_meas
    % Gaussian distribution
    if (i_meas > max_range) || (i_meas<=0.0)
        p_gauss=0.0;
    else
        llim = (0.0 - z_true)/gauss_sd;
        ulim = (max_range - z_true)/gauss_sd;
        norm_gauss = 1.0/( calcCDF(ulim) - calcCDF(llim) );				
%         norm_gauss = 1;

        p_gauss = norm_gauss * exp( -(i_meas - z_true)*(i_meas - z_true)/(2.0*gauss_sd*gauss_sd) ) ...
                    / (gauss_sd*sqrt(2.0*pi));        
    end

    % exp distribution
    norm_exp = 1.0 / (1.0 - exp(-lambda_short*z_true));
    if (i_meas <= z_true) && (i_meas >=0.0)
        p_short = norm_exp*lambda_short*exp(-lambda_short*i_meas);
    else
        p_short = 0.0;
    end

    % max distribution
    if abs(i_meas - max_range)<2
        p_max = 1.0;
    else
        p_max=0.0;
    end

    % random distribution
    if (i_meas >=0.0) && i_meas <= max_range
        p_rand = 1.0/max_range ;
    else
        p_rand=0.0;
    end

    wt_norm = wt_gauss + wt_short + wt_max + wt_rand;
    p_vect(ct) = (wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + wt_max/wt_norm*p_max + wt_rand/wt_norm*p_rand);
    ct = ct+1;
end

figure(1)
plot(z_meas, p_vect)
title('Sensor Model probability distribution')
xlabel('Measured range')
ylabel('Probability density function')
grid on

figure(2)
plot(z_meas, log(p_vect))
title('Sensor Model log-probability distribution')
xlabel('Measured range')
ylabel('Log probability density function')
grid on