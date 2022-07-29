import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow import keras
import h5py
import numpy as np
import pickle


part = "l_leg"
dataset = dict()

if __name__ == "__main__":


    with open("../autogenerated/scaling.pickle", "rb") as handle:
        scaling = pickle.load(handle)

    with h5py.File("../datasets/test_dataset.mat", "r") as file:
        dataset["ang_vel"] = (
                                     np.array(file["dataset"][part]["ang_vel"]) - scaling[part]["ang_vel"]["mean"]
                             ) / (scaling[part]["ang_vel"]["std"])
        dataset["lin_acc"] = (
                                     np.array(file["dataset"][part]["lin_acc"]) - scaling[part]["lin_acc"]["mean"]
                             ) / (scaling[part]["lin_acc"]["std"])
        dataset["orientation_quat"] = np.array(file["dataset"][part]["orientation_quat"])
        dataset["ft_expected"] = np.array(file["dataset"][part]["ft_expected"])
        dataset["ft_measured"] = (
                                         np.array(file["dataset"][part]["ft_measured"])
                                         - scaling[part]["ft_measured"]["mean"]
                                 ) / (scaling[part]["ft_measured"]["std"])
        dataset["all_ft_measured"] = np.array(file["dataset"][part]["ft_measured"])

    # with h5py.File("old_approach/calib_soa.mat", "r") as file:
    #     C = np.array(file["results"]["sol"]["l_arm_ft_sensor"]["C"])
    #     o = np.array(file["results"]["sol"]["l_arm_ft_sensor"]["o"])

    # prev_predict = (C.T @ dataset["all_ft_measured"].T - o.T).T

    train_examples = np.expand_dims(
        np.concatenate(
            (
                dataset["ang_vel"],
                dataset["lin_acc"],
                dataset["orientation_quat"],
                dataset["ft_measured"],
            ),
            axis=1,
        ),
        axis=2,
    )
    train_labels = np.expand_dims(dataset["ft_expected"], axis=2)

    train_dataset = tf.data.Dataset.from_tensor_slices((train_examples, train_labels))
    model = keras.models.load_model("../autogenerated/models_" + part + "/")
    predict_dataset = (
            model.predict(train_examples) * scaling[part]["ft_expected"]["std"]
            + scaling[part]["ft_expected"]["mean"]
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
        offset = dataset["all_ft_measured"][50, i] - dataset["ft_expected"][50, i]
        ax[i].plot(
            np.array(range(len(predict_dataset[:, i]))) * 0.01,
            dataset["all_ft_measured"][:, i] - offset,
            label="FT value with pure offset removal",
        )
        ax[i].plot(
            np.array(range(len(predict_dataset[:, i]))) * 0.01,
            predict_dataset[:, i],
            label="NN output",
        )
        ax[i].plot(
            np.array(range(len(predict_dataset[:, i]))) * 0.01,
            dataset["ft_expected"][:, i],
            label="Ground-truth",
        )
        # ax[i].plot(
        #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
        #     prev_predict[:, i],
        #     label="Linear in-situ calib",
        # )
        if i == 0:
            ax[i].legend()
        ax[i].set_xlabel("time (s)")
        ax[i].set_ylabel(titles[i])
        ax[1].grid()

    fig.suptitle("Test set - " + part, fontsize=16)
    plt.show()

    fig, ax = plt.subplots(6)
    for i in range(6):
        offset = dataset["all_ft_measured"][0, i] - dataset["ft_expected"][0, i]

        ax[i].plot(
            np.array(range(len(predict_dataset[:, i]))) * 0.01,
            dataset["all_ft_measured"][:, i] - offset - dataset["ft_expected"][:, i],
            label="FT value with pure offset removal",
        )

        ax[i].plot(
            np.array(range(len(predict_dataset[:, i]))) * 0.01,
            predict_dataset[:, i] - dataset["ft_expected"][:, i],
            label="NN output",
        )
        # ax[i].plot(
        #     np.array(range(len(predict_dataset[:, i]))) * 0.01,
        #     prev_predict[:, i] - dataset["ft_expected"][:, i],
        #     label="Linear in-situ calib",
        # )
        if i == 0:
            ax[i].legend()

        ax[i].set_xlabel("time (s)")
        ax[i].set_ylabel(titles[i])
        ax[1].grid()

    fig.suptitle("Error Test set - " + part, fontsize=16)

    plt.show()
