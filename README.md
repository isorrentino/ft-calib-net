# FT-Calib-Net

This project contains the network and the experiments to run the FT calibration

## Dependencies
You can install all the dependencies from `conda`. Please run the following command
```
mamba install tensorflow-gpu keras matplotlib numpy h5py scikit-learn
```
## Training
Please run `train_net.py` to run the training of the network.

## Evaluate
Please run `evaluate_net.py` to evaluate the output of the network and compare it with the standard
linear regression approach
