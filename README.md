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

![image](https://user-images.githubusercontent.com/16744101/180648956-b85de55d-32b4-4887-a902-fe57ef01e511.png)
![image](https://user-images.githubusercontent.com/16744101/180648985-fc9e82f3-a491-4ac5-a623-57dc9b91fbb8.png)
