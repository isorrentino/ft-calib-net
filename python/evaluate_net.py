import h5py
import math
import pickle
import numpy as np
from tensorflow import keras
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Fixed training params: val_size=0.33, elu, batch_size=32
model_specification = "20220928-091609_NN28x14_dp0.05_FT_temp_15ep"

test_datasets = ["calib_dataset", "2022_09_22_test_dataset", "2022_09_16_test_dataset", "2022_09_09_01_test_dataset", "2022_09_09_02_test_dataset"]

part = "r_arm"
dataset = dict()

model_name = "r_arm_net_" + model_specification
scaling_name = "r_arm_scaling_" + model_specification

if __name__ == "__main__":

    with open("../models/" + scaling_name + ".pickle", "rb") as handle:
        scaling = pickle.load(handle)

    for test_dataset in test_datasets:

        with h5py.File("../datasets/" + test_dataset + ".mat", "r") as file:
            dataset["ang_vel"] = (np.array(file["dataset"][part]["ang_vel"]) - scaling["ang_vel"]["mean"]) / (scaling["ang_vel"]["std"])
            dataset["lin_acc"] = (np.array(file["dataset"][part]["lin_acc"]) - scaling["lin_acc"]["mean"]) / (scaling["lin_acc"]["std"])

            # TODO: quaternion scaling
            dataset["orientation_quat"] = np.array(file["dataset"][part]["orientation_quat"])
            # dataset["orientation_quat"] = (np.array(file["dataset"][part]["orientation_quat"]) - scaling["orientation_quat"]["mean"]) / (scaling["orientation_quat"]["std"])

            dataset["ft_measured"] = (np.array(file["dataset"][part]["ft_measured"]) - scaling["ft_measured"]["mean"]) / (scaling["ft_measured"]["std"])
            dataset["ft_temperature"] = (np.array(file["dataset"][part]["ft_temperature"]) - scaling["ft_temperature"]["mean"]) / (scaling["ft_temperature"]["std"])
            dataset["joints"] = (np.array(file["dataset"]["joints"]) - scaling["joints"]["mean"]) / (scaling["joints"]["std"])

            dataset["ft_expected"] = np.array(file["dataset"][part]["ft_expected"])

            dataset["all_ft_measured"] = np.array(file["dataset"][part]["ft_measured"])

        with h5py.File("../insitu_calib/calibrations.mat", "r") as file:
            C = np.array(file["calibrations"][part]["C"])
            o = np.array(file["calibrations"][part]["o"])

        prev_predict = (C.T @ dataset["all_ft_measured"].T - o.T).T

        # TODO: define network input
        test_examples = np.expand_dims(
            np.concatenate(
                (
                    # dataset["ang_vel"],
                    # dataset["lin_acc"],
                    # dataset["orientation_quat"],
                    dataset["ft_measured"],
                    dataset["ft_temperature"],
                    # dataset["joints"],
                ),
                axis=1,
            ),
            axis=2,
        )
        test_labels = np.expand_dims(dataset["ft_expected"], axis=2)

        model = keras.models.load_model("../models/" + model_name + ".h5")

        predict_dataset = (
                model.predict(test_examples) * scaling["ft_expected"]["std"] + scaling["ft_expected"]["mean"]
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
            MSE_lin = np.square(np.subtract(prev_predict[:, i], dataset["ft_expected"][:, i])).mean()
            RMSE_lin = math.sqrt(MSE_lin)

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                prev_predict[:, i],
                label="Linear - RMSE: " + str(round(RMSE_lin, 2)),
            )

            # Shrink current axis by 20% and put the legend to the right
            box = ax[i].get_position()
            ax[i].set_position([box.x0, box.y0, box.width * 0.8, box.height])
            ax[i].legend(loc='center left', bbox_to_anchor=(1, 0.5))

            ax[i].set_xlabel("time (s)")
            ax[i].set_ylabel(titles[i])
            ax[1].grid()

        fig.suptitle("Wrench estimation - " + test_dataset + " - " + part , fontsize=16)
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
        plt.show()
        fig.savefig("../figures/fig1_"+test_dataset+"_"+model_specification+".png")

        # TODO: tune filters
        b, a = butter(2, 0.01, btype='low', analog=False)
        # b, a = butter(2, 0.9999, btype='low', analog=False)
        ft_filtered_NN = np.zeros((len(predict_dataset[:, 0]), 3))
        ft_filtered_NN[:, 0] = filtfilt(b, a, predict_dataset[:, 0])
        ft_filtered_NN[:, 1] = filtfilt(b, a, predict_dataset[:, 1])
        ft_filtered_NN[:, 2] = filtfilt(b, a, predict_dataset[:, 2])
        ft_filtered_insitu = np.zeros((len(prev_predict[:, 0]), 3))
        ft_filtered_insitu[:, 0] = filtfilt(b, a, prev_predict[:, 0])
        ft_filtered_insitu[:, 1] = filtfilt(b, a, prev_predict[:, 1])
        ft_filtered_insitu[:, 2] = filtfilt(b, a, prev_predict[:, 2])
        b, a = butter(2, 0.0003, btype='low', analog=False)
        # b, a = butter(2, 0.9999, btype='low', analog=False)
        fig = plt.figure()
        plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
                 np.linalg.norm(dataset["ft_expected"][:, :3], axis=1) / 9.80665 - 3.5 + 0.165,
                 label="Ground Truth")
        plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
                 filtfilt(b, a, np.linalg.norm(ft_filtered_NN, axis=1) / 9.80665 - 3.5 + 0.165),
                 label="Neural Network")
        plt.plot(np.array(range(len(predict_dataset[:, 0]))) * 0.01,
                 filtfilt(b, a, np.linalg.norm(ft_filtered_insitu, axis=1) / 9.80665 - 3.5 + 0.165),
                 label="Linear")
        plt.xlabel("time (s)")
        plt.ylabel("norm(f) / g")
        fig.suptitle("Weight estimation - " + test_dataset + " - " + part , fontsize=16)
        plt.grid()
        plt.legend()
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
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

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                np.abs(predict_dataset[:, i] - dataset["ft_expected"][:, i]),
                label="Neural Network",
            )

            ax[i].plot(
                np.array(range(len(predict_dataset[:, i]))) * 0.01,
                np.abs(prev_predict[:, i] - dataset["ft_expected"][:, i]),
                label="Linear",
            )

            if i == 0:
                ax[i].legend()

            ax[i].set_xlabel("time (s)")
            ax[i].set_ylabel(titles[i])
            ax[1].grid()

        fig.suptitle("Wrench estimation error - " + test_dataset + " - " + part , fontsize=16)
        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()
        plt.show()
        fig.savefig("../figures/fig3_"+test_dataset+"_"+model_specification+".png")


