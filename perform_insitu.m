opti = casadi.Opti();
x = opti.variable();
y = opti.variable();
opti.minimize((1-x)^2+(y-x^2)^2);
opti.solver('ipopt');
sol = opti.solve();

load('./datasets/calib_dataset.mat')

parts = {'r_arm'};

% parts = fieldnames(dataset); % extract names of features
calibrations = struct();
for i = 1:length(parts)
    calibrations.(parts{i}) = insitu_calib(dataset.(parts{i}));
end

save("insitu_calib/calibrations.mat", "calibrations", "-v7.3")