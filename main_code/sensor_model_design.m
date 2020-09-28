%% script to design sensor model for laser range finder
clear
close all

%% params
max_range = 8183; gauss_sd = 20.0; lambda_short = 0.001;
wt_gauss=3.0; wt_short=5.0; wt_max=0.03; wt_rand=0.5;

z_true = 5000;
%% prob distribution
z_meas = linspace(0, max_range, 10000);
p_vect = zeros(size(z_meas));
ct=1;

for i_meas = z_meas
    % Gaussian distribution
    if (i_meas > max_range) || (i_meas<=0.0)
        p_gauss=0.0;
    else
        llim = (0.0 - z_true)/gauss_sd;
        ulim = (max_range - z_true)/gauss_sd;
        norm_gauss = 1.0/( calcCDF(ulim) - calcCDF(llim) );				

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
    if abs(i_meas - max_range)<1e-3
        p_max = 1.0;
    else
        p_max=0.0;
    end

    % random distribution
    if (i_meas >=0.0) && i_meas <= max_range
        p_rand = 1.0/max_range;
    else
        p_rand=0.0;
    end

    wt_norm = wt_gauss + wt_short + wt_max + wt_rand;
    p_vect(ct) = (wt_gauss/wt_norm*p_gauss + wt_short/wt_norm*p_short + wt_max/wt_norm*p_max + wt_rand/wt_norm*p_rand);
    ct = ct+1;
end

plot(z_meas, p_vect)