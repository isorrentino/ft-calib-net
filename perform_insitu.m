load('./datasets/calib_dataset.mat')

parts = fieldnames(dataset); % extract names of features
calibartions = struct();
for i = 1:length(parts)
    calibartions.(parts{i}) = insitu_calib(dataset.(parts{i}));
end

save("insitu_calib/calibartions.mat", "calibartions", "-v7.3")