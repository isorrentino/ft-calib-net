function calibration = insitu_calib(dataset)
    f = waitbar(0, 'Starting');

    opti = casadi.Opti();
    
    C = opti.variable(6,6);
    o = opti.variable(6,1);
    
    num_samples = size(dataset.ft_expected,2);
    
    cost = 0;
    
    for i = 1 : num_samples
        waitbar(double(i)/double(num_samples), f, sprintf('Progress: %d %%', floor(double(i)/double(num_samples)*100.0)));
        cost = cost + sumsqr(dataset.ft_expected(:,i) - C * dataset.ft_measured(:,i) + o);
    end
    cost = cost / num_samples;
    
    lam = 1;
    cost = cost + lam * sumsqr(C - eye(6));
    
    opti.minimize(cost);
    
    opti.solver('ipopt');
    sol = opti.solve();
    
    calibration=struct();
    calibration.C = sol.value(C);
    calibration.o = sol.value(o);
    close(f)
end