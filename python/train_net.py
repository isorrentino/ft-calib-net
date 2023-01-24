import h5py
import pickle
import numpy as np
import tensorflow as tf
from tensorflow import keras
from datetime import datetime
from sklearn.model_selection import train_test_split
import pickle


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


def my_loss_fn(y_true, y_pred):
    squared_difference = 0
    f_max = tf.reduce_max(y_true[:3])
    m_max = tf.reduce_max(y_true[3:])
    for i in range(3):
        squared_difference += tf.square(y_true[i] - y_pred[i])
    for i in range(3):
        squared_difference += f_max/m_max * tf.square(y_true[i+3] - y_pred[i+3])
    return squared_difference  # Note the `axis=-1`


def train_net(file, part):
    # Read the dataset
    dataset = dict()
    with h5py.File(file, "r") as f:
        # dataset["ang_vel"] = np.array(f["dataset"][part]["ang_vel"])
        # dataset["lin_acc"] = np.array(f["dataset"][part]["lin_acc"])
        # dataset["orientation_quat"] = np.array(f["dataset"][part]["orientation_quat"])
        dataset["ft_measured"] = np.array(f["dataset"][part]["ft_measured"])
        dataset["ft_temperature"] = np.array(f["dataset"][part]["ft_temperature"])
        dataset["joints"] = np.array(f["dataset"]["joints"])
        dataset["ft_expected"] = np.array(f["dataset"][part]["ft_expected"])

    # Scale the temperature by a factor of 10 since a bug in the calibration dataset multiplies it by 10
    dataset["ft_temperature"] = [elem / 10.0 for elem in dataset["ft_temperature"]]

    # Scale the dataset
    scaling = dict()
    for key, value in dataset.items():
        if key != "orientation_quat":  # TODO: quaternions are normalized but have no 0 mean and 1 std
            scaling[key] = dict()
            scaling[key]["mean"] = np.mean(value, axis=0)
            scaling[key]["std"] = np.std(value, axis=0)
            dataset[key] = (value - scaling[key]["mean"]) / scaling[key]["std"]

    # Define network input # TODO: ft_measured + temperature - NO joints + orientation_quat -> change also eval
    numpy_dataset = np.concatenate((dataset["ft_measured"],
                                    # dataset["ang_vel"],
                                    # dataset["lin_acc"],
                                    # dataset["orientation_quat"],
                                    dataset["ft_temperature"],
                                    # dataset["joints"],
                                    ), axis=1)
    numpy_expected = dataset["ft_expected"]

    # Split between training and validation
    train_examples = np.expand_dims(numpy_dataset, axis=2)
    train_labels = np.expand_dims(numpy_expected, axis=2)
    x_train, x_valid, y_train, y_valid = train_test_split(
        train_examples, train_labels, test_size=0.5, shuffle=False  # TODO: validation size?
    )

    # Define the network model # TODO: number of layers? number of units? activation? dropout rate? weights initialization?
    dropout_rate = 0.005

    # model = keras.Sequential(
    #     # [keras.layers.Dropout(input_shape=(7,), # TODO: not on small inputs?
    #     #                       rate=dropout_rate),
    #      [keras.layers.Dense(input_shape=(7,),
    #                         units=14,
    #                         # kernel_initializer="uniform",
    #                         # kernel_regularizer=keras.regularizers.L2(0.01),
    #                         # activity_regularizer=keras.regularizers.L2(0.01),
    #                         activation="elu"),
    #      keras.layers.Dropout(rate=dropout_rate),
    #      keras.layers.Dense(units=28,
    #                         activation="elu"),
    #      keras.layers.Dropout(rate=dropout_rate),
    #      keras.layers.Dense(units=6),
    #      ]
    # )


    # model = keras.Sequential(
    #     [LinearTransformationMatrix(6),]
    # )


    model = keras.Sequential(
        # [keras.layers.Dropout(input_shape=(7,), # TODO: not on small inputs?
        #                       rate=dropout_rate),
        [keras.layers.Dense(input_shape=(7,),
                            units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=42,
                            activation="relu"),
         # keras.layers.Dropout(rate=dropout_rate),
         keras.layers.Dense(units=6,
                            activation="tanh"),
         # keras.layers.Dropout(rate=dropout_rate),
         # keras.layers.Dense(units=6),
         # keras.layers.Dropout(rate=dropout_rate),
         LinearTransformationMatrix(units=6),
         keras.layers.Dense(units=6),
         ]
    )

    # Define loss, metrics and optimizer
    model.compile(
        # loss=tf.keras.losses.MeanSquaredError(reduction="auto", name="mean_squared_error"), # TODO: Change loss as in the paper
        loss=my_loss_fn,
        metrics=keras.metrics.MeanSquaredError(name="mean_squared_error", dtype=None),
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.01), # TODO: learning rate? variable? weight decay? variable?
    )

    # Define training parameters # TODO: batch size? epochs?
    model.fit(x=x_train, y=y_train, epochs=100, batch_size=7000, validation_data=(x_valid, y_valid))

    return model, scaling


if __name__ == "__main__":

    parts = ['r_arm']

    scaling = dict()
    models = dict()

    for part in parts:
        models[part], scaling[part] = train_net("../calib_dataset/r_arm_vel_acc_used_calib_dataset.mat", part)

    # for part in parts:
    #     models[part].save("../autogenerated/models_" + part + "/")
    #     models[part].save("../export_model/" + part + "_net.h5", include_optimizer=False)
    #
    # with open("../autogenerated/scaling.pickle", "wb") as handle:
    #     pickle.dump(scaling, handle)

    now = datetime.now().strftime("%Y%m%d-%H%M%S")
    for part in parts:
        models[part].save("../models/" + part + "_net_" + str(now) + ".h5", include_optimizer=False)
        with open("../models/" + part + "_scaling_" + str(now) + ".pickle", "wb") as handle:
            pickle.dump(scaling[part], handle)
