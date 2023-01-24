import numpy as np
import tensorflow as tf
from tensorflow import keras
import matplotlib.pyplot as plt

class CustomLayer(keras.layers.Layer):
    def __init__(self, units=6, **kwargs):
        super(CustomLayer, self).__init__(**kwargs)
        self.units = units

    def build(self, input_shape):
        w_init = tf.constant([[1.0, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7]])
        self.w = tf.Variable(
            initial_value=w_init,
            trainable=False,
            name="w"
        )
        # self.w = self.add_weight(
        #     shape=(7, self.units),
        #     initializer="random_normal",
        #     trainable=False,
        #     name="w"
        # )

    def call(self, inputs):
        return tf.matmul(inputs, tf.transpose(self.w))

    def get_config(self):
        config = super(CustomLayer, self).get_config()
        config.update({"units": self.units})
        return config


def custom_activation(x):
    return tf.nn.tanh(x) ** 2


# Make a model with the CustomLayer and custom_activation
inputs = keras.Input((7,))
x = CustomLayer(6)(inputs)
outputs = keras.layers.Activation('relu')(x)
model = keras.Model(inputs, outputs)

# Retrieve the config
config = model.get_config()

# At loading time, register the custom objects with a `custom_object_scope`:
custom_objects = {"CustomLayer": CustomLayer}
with keras.utils.custom_object_scope(custom_objects):
    new_model = keras.Model.from_config(config)

# Train the model.
new_model.compile(optimizer="adam", loss="mean_squared_error")
test_input = np.random.random((128, 7))
# test_target = np.random.random((128, 6))
matrix_cal = np.matrix([[1.0, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7],
             [1, 2, 3, 4, 5, 6, 7]])
test_target = test_input @ np.transpose(matrix_cal)

new_model.fit(test_input, test_target, epochs=2, batch_size=10)

# Calling `save('my_model.h5')` creates a h5 file `my_model.h5`.
new_model.save("../models/my_h5_model.h5")

# It can be used to reconstruct the model identically.
reconstructed_model = keras.models.load_model("../models/my_h5_model.h5", custom_objects)

predicted = reconstructed_model.predict(test_input)

fig, ax = plt.subplots(6)
for i in range(6):
    ax[i].plot(
        np.array(range(len(test_target[:, i]))),
        test_target[:, i],
        label="Ground Truth",
    )

    ax[i].plot(
        np.array(range(len(predicted[:, i]))),
        predicted[:, i],
        label="Neural Network",
    )
plt.show()


