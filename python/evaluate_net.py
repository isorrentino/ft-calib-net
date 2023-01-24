import h5py
import math
import pickle
import numpy as np
from tensorflow import keras
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from keras.models import load_model
import tensorflow as tf

class LinearTransformationMatrix(keras.layers.Layer):
    def __init__(self, units=6, **kwargs):
        super(LinearTransformationMatrix, self).__init__(**kwargs)
        self.units = units

    def build(self, input_shape):
        w_init = tf.constant([[0.792646532147124, 0.129318123151594, -0.063572470609477, -0.302269249502385, 0.006396833634943, -0.360745709314269],
                              [-0.161324993407456, 0.477999165754515, -0.108633100889438, -1.891672537435563, -0.610032836606732, -0.276997568087506],
                              [0.109778543237993, -0.027385756564095, 0.944814866350217, -0.613094711563824, 0.123518812098044, -0.050988542921986],
                              [0.021737027869565, -0.008427584649784, 0.006921191435852, 0.918511780904158, 0.114673164057718, -0.026845744174192],
                              [0.000571045063613, 0.023163480169074, -0.006514886021960, -0.011156712894087, 0.935876910571665, -0.065205714255862],
                              [-0.006433199232597, -0.006632890165866, 0.005073862131363, 0.023373170226844, 0.069826769128075, 1.011361264632946]])
        self.w = tf.Variable(
            initial_value=w_init,
            trainable=False,
            name="w"
        )

    def call(self, inputs):
        return tf.matmul(inputs, tf.transpose(self.w))

    def get_config(self):
        config = super(LinearTransformationMatrix, self).get_config()
        config.update({"units": self.units})
        return config



# Fixed training params: val_size=0.33, elu, batch_size=32
model_specification = "20221026-224915"

# test_datasets = ["calib_dataset", "2022_09_22_test_dataset", "2022_09_16_test_dataset", "2022_09_09_01_test_dataset", "2022_09_09_02_test_dataset"]
test_datasets = ["r_arm_acc_vel_1_test_dataset", "filtered_2022_09_16_test_dataset", "filtered_09_09_2022_1_test_dataset", "filtered_09_09_2022_2_test_dataset"]

part = "r_arm"
dataset = dict()

model_name = "r_arm_net_" + model_specification
scaling_name = "r_arm_scaling_" + model_specification

if __name__ == "__main__":

    with open("../models/" + scaling_name + ".pickle", "rb") as handle:
        scaling = pickle.load(handle)

    for test_dataset in test_datasets:

        with h5py.File("../test_dataset/" + test_dataset + ".mat", "r") as file:
            # dataset["ang_vel"] = (np.array(file["dataset"][part]["ang_vel"]) - scaling["ang_vel"]["mean"]) / (scaling["ang_vel"]["std"])
            # dataset["lin_acc"] = (np.array(file["dataset"][part]["lin_acc"]) - scaling["lin_acc"]["mean"]) / (scaling["lin_acc"]["std"])

            # TODO: quaternion scaling
            # dataset["orientation_quat"] = np.array(file["dataset"][part]["orientation_quat"])
            # dataset["orientation_quat"] = (np.array(file["dataset"][part]["orientation_quat"]) - scaling["orientation_quat"]["mean"]) / (scaling["orientation_quat"]["std"])

            dataset["ft_measured"] = (np.array(file["dataset"][part]["ft_measured"]) - scaling["ft_measured"]["mean"]) / (scaling["ft_measured"]["std"])
            dataset["ft_temperature"] = (np.divide(np.array(file["dataset"][part]["ft_temperature"]), 10) - scaling["ft_temperature"]["mean"]) / (scaling["ft_temperature"]["std"])
            # dataset["joints"] = (np.array(file["dataset"]["joints"]) - scaling["joints"]["mean"]) / (scaling["joints"]["std"])

            dataset["ft_expected"] = np.array(file["dataset"][part]["ft_expected"])

            dataset["all_ft_measured"] = np.array(file["dataset"][part]["ft_measured"])

        with h5py.File("../insitu_calib/filtered_calibrations.mat", "r") as file:
            C = np.array(file["calibrations"][part]["C"])
            o = np.array(file["calibrations"][part]["o"])

        prev_predict_old_insitu = (C.T @ dataset["all_ft_measured"].T - o.T).T

        # with h5py.File("../insitu_calib/calibrations.mat", "r") as file:
        #     C = np.array(file["calibrations"][part]["C"])
        #     o = np.array(file["calibrations"][part]["o"])

        # prev_predict_new_insitu = (C.T @ dataset["all_ft_measured"].T - o.T).T

        # TODO: define network input
        test_examples = np.concatenate(
                (
                    # dataset["ang_vel"],
                    # dataset["lin_acc"],
                    # dataset["orientation_quat"],
                    dataset["ft_measured"],
                    dataset["ft_temperature"],
                    # dataset["joints"],
                ),
                axis=1,
        )
        test_labels = dataset["ft_expected"]

        custom_objects = {"LinearTransformationMatrix": LinearTransformationMatrix}
        model = load_model("../models/" + model_name + ".h5", custom_objects)

        predicted = model.predict(test_examples)
        predicted = predicted.reshape(predicted.shape[0], predicted.shape[1])

        predict_dataset = (
                predicted * scaling["ft_expected"]["std"] + scaling["ft_expected"]["mean"]
        )

        titles = [
            "Force - x (N)",
            "Force - y (N)",
            "Force - z (N)",
            "Torque - x (Nm)",
            "Torque - y (Nm)",
            "Torque - z (Nm)",
        ]

        fig, ax = plt.subplots(6)
        for i in range(6):

            # # TODO: plot separately
            # offset = dataset["all_ft_measured"][50, i] - dataset["ft_expected"][50, i]
            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     dataset["all_ft_measured"][:, i] - offset,
            #     label="FT value with pure offset removal",
            # )

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                dataset["ft_expected"][:, i],
                label="Ground Truth",
            )

            # Compute RMSE for the neural network
            MSE_nn = np.square(np.subtract(predict_dataset[:, i], dataset["ft_expected"][:, i])).mean()
            RMSE_nn = math.sqrt(MSE_nn)

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                predict_dataset[:, i],
                label="Neural Network - RMSE: " + str(round(RMSE_nn,2)),
            )

            # Compute RMSE for the linear model
            MSE_lin_old_insitu = np.square(np.subtract(prev_predict_old_insitu[:, i], dataset["ft_expected"][:, i])).mean()
            RMSE_lin_old_insitu = math.sqrt(MSE_lin_old_insitu)

            # MSE_lin_new_insitu = np.square(np.subtract(prev_predict_new_insitu[:, i], dataset["ft_expected"][:, i])).mean()
            # RMSE_lin_new_insitu = math.sqrt(MSE_lin_new_insitu)

            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     prev_predict_old_insitu[:, i],
            #     label="Linear insitu - RMSE: " + str(round(RMSE_lin_old_insitu, 2)),
            # )

            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     prev_predict_new_insitu[:, i],
            #     label="Linear new insitu - RMSE: " + str(round(RMSE_lin_new_insitu, 2)),
            # )

            # Shrink current axis by 20% and put the legend to the right
            box = ax[i].get_position()
            ax[i].set_position([box.x0, box.y0, box.width * 0.8, box.height])
            ax[i].legend(loc='center left', bbox_to_anchor=(1, 0.5))

            ax[i].set_xlabel("time (s)")
            ax[i].set_ylabel(titles[i])
            ax[1].grid()

        fig.suptitle("Wrench estimation - " + test_dataset + " - " + part , fontsize=16)
        # manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle()
        plt.show()
        fig.savefig("../figures/fig1_"+test_dataset+"_"+model_specification+".png")

        # TODO: tune filters
        ft_filtered_NN = predict_dataset
        ft_filtered_old_insitu = prev_predict_old_insitu
        # ft_filtered_new_insitu = prev_predict_new_insitu
        # b, a = butter(2, 0.01, btype='low', analog=False)
        # b, a = butter(2, 0.9999, btype='low', analog=False)
        # ft_filtered_NN = np.zeros((len(predict_dataset[:, 0]), 3))
        # ft_filtered_NN[:, 0] = filtfilt(b, a, predict_dataset[:, 0])
        # ft_filtered_NN[:, 1] = filtfilt(b, a, predict_dataset[:, 1])
        # ft_filtered_NN[:, 2] = filtfilt(b, a, predict_dataset[:, 2])
        # ft_filtered_insitu = np.zeros((len(prev_predict_old_insitu[:, 0]), 3))
        # ft_filtered_insitu[:, 0] = filtfilt(b, a, prev_predict_old_insitu[:, 0])
        # ft_filtered_insitu[:, 1] = filtfilt(b, a, prev_predict_old_insitu[:, 1])
        # ft_filtered_insitu[:, 2] = filtfilt(b, a, prev_predict_old_insitu[:, 2])
        b, a = butter(2, 0.0003, btype='low', analog=False)
        # b, a = butter(2, 0.9999, btype='low', analog=False)
        fig = plt.figure()

        ground_truth = np.linalg.norm(dataset["ft_expected"][:, :3], axis=1) / 9.80665 # - 3.5 + 0.165
        plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
                 ground_truth,
                 label="Ground Truth")

        # Compute RMSE for the neural network
        filtered_nn_output = np.linalg.norm(ft_filtered_NN, axis=1) / 9.80665 # - 3.5 + 0.165
        # filtered_nn_output = filtfilt(b, a, np.linalg.norm(ft_filtered_NN, axis=1) / 9.80665 - 3.5 + 0.165)
        MSE_nn = np.square(np.subtract(filtered_nn_output, ground_truth)).mean()
        RMSE_nn = math.sqrt(MSE_nn)

        plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
                 filtered_nn_output,
                 label="Neural Network - RMSE: " + str(round(RMSE_nn, 2)))

        # Compute RMSE for the linear model
        filtered_linear = np.linalg.norm(ft_filtered_old_insitu, axis=1) / 9.80665
        # filtered_linear = filtfilt(b, a, np.linalg.norm(ft_filtered_insitu, axis=1) / 9.80665 - 3.5 + 0.165)
        MSE_lin = np.square(np.subtract(filtered_linear, ground_truth)).mean()
        RMSE_lin = math.sqrt(MSE_lin)

        # plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
        #          filtered_linear,
        #          label="Linear insitu - RMSE: " + str(round(RMSE_lin, 2)))

        # Compute RMSE for the linear model
        # filtered_linear = np.linalg.norm(ft_filtered_new_insitu, axis=1) / 9.80665
        # filtered_linear = filtfilt(b, a, np.linalg.norm(ft_filtered_insitu, axis=1) / 9.80665 - 3.5 + 0.165)
        MSE_lin = np.square(np.subtract(filtered_linear, ground_truth)).mean()
        RMSE_lin = math.sqrt(MSE_lin)

        # plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
        #          filtered_linear,
        #          label="Linear new insitu - RMSE: " + str(round(RMSE_lin, 2)))
        
        plt.xlabel("time (s)")
        plt.ylabel("norm(f) / g")
        fig.suptitle("Weight estimation - " + test_dataset + " - " + part , fontsize=16)
        plt.grid()
        plt.legend()
        # manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle()
        plt.show()
        fig.savefig("../figures/fig2_"+test_dataset+"_"+model_specification+".png")

        fig, ax = plt.subplots(6)
        for i in range(6):

            # offset = dataset["all_ft_measured"][0, i] - dataset["ft_expected"][0, i]
            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     np.abs(dataset["all_ft_measured"][:, i] - offset - dataset["ft_expected"][:, i]),
            #     label="FT value with pure offset removal",
            # )

            # Compute mean of the neural network error
            nn_abs_error = np.abs(predict_dataset[:, i] - dataset["ft_expected"][:, i])

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                nn_abs_error,
                label="Neural Network - mean: " + str(round(nn_abs_error.mean(),2)),
            )

            # Compute mean of the linear model error
            linear_abs_error = np.abs(prev_predict_old_insitu[:, i] - dataset["ft_expected"][:, i])

            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     linear_abs_error,
            #     label="Linear insitu - mean: " + str(round(linear_abs_error.mean(), 2)),
            # )

            # linear_abs_error = np.abs(prev_predict_new_insitu[:, i] - dataset["ft_expected"][:, i])

            # ax[i].plot(
            #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
            #     linear_abs_error,
            #     label="Linear new insitu - mean: " + str(round(linear_abs_error.mean(), 2)),
            # )

            # Shrink current axis by 20% and put the legend to the right
            box = ax[i].get_position()
            ax[i].set_position([box.x0, box.y0, box.width * 0.8, box.height])
            ax[i].legend(loc='center left', bbox_to_anchor=(1, 0.5))

            ax[i].set_xlabel("time (s)")
            ax[i].set_ylabel(titles[i])
            ax[1].grid()

        fig.suptitle("Wrench estimation error - " + test_dataset + " - " + part , fontsize=16)
        # manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle()
        plt.show()
        fig.savefig("../figures/fig3_"+test_dataset+"_"+model_specification+".png")


