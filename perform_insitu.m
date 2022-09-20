load('./datasets/calib2000_dataset.mat')

parts = fieldnames(dataset); % extract names of features
calibrations = struct();
for i = 1:length(parts)
    calibrations.(parts{i}) = insitu_calib(dataset.(parts{i}));
end

save("insitu_calib/calibrations.mat", "calibrations", "-v7.3")