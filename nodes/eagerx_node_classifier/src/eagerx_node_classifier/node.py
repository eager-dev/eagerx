from typing import Dict, List, Union
import matplotlib.pyplot as plt
import numpy as np
import h5py
from collections import deque

# IMPORT ROS
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

# IMPORT EAGERX
from eagerx_core.basenode import Node
from eagerx_core.utils.utils import substitute_xml_args


class Classifier(Node):
    def __init__(self, data_path, show, epochs, num_examples, window_length, cam_rate, **kwargs):
        assert window_length > 1, 'Window lenght must be large than 1 to infer temporal information.'
        self.model = self._train_model(data_path, show=show, epochs=epochs, num_examples=num_examples)
        self.window_length = window_length
        self.window = None
        self.cam_rate = cam_rate
        super().__init__(**kwargs)

    def _train_model(self, data_path, show=False, epochs=30, num_examples=12000):
        # LOAD DATASET
        data_path = substitute_xml_args(data_path)
        with h5py.File(data_path, 'r') as hf:
            observation = hf['observation.h5'][:]
            state = hf['state.h5'][:]
        print('Loaded observation data: %s' % str(observation.shape))
        print('Loaded state data: %s' % str(state.shape))

        # DATASET PARAMETERS
        # num_examples = 12000
        observation = observation[:num_examples]
        state = state[:num_examples]

        # DATA PRE-PROCESSING
        # Scale pixel values to a range of 0 to 1 before feeding them to the neural network model.
        observation = observation.astype(np.float32) / 255.

        # CREATE TEST DATASET
        test_split = 0.2
        if 0 < test_split < 1:
            split_at = int(len(observation) * (1 - test_split))
        else:
            raise ValueError('Must hold-out data as a test dataset. Set parameter 0 < test_split < 1.')
        test_obs = observation[split_at:, :, :, :]
        test_theta = state[split_at:, 0]
        test_trig = np.hstack([np.cos(test_theta)[:, None], np.sin(test_theta)[:, None]])

        # CREATE TRAINING DATASET
        train_obs = observation[:split_at, :, :, :]
        train_theta = state[:split_at, 0]
        train_trig = np.hstack([np.cos(train_theta)[:, None], np.sin(train_theta)[:, None]])

        # VERIFY TRAINING DATA
        # To verify that the data is in the correct format and that you're ready to build and train the network,
        # let's display the first 25 images from the dataset and display the corresponding theta value below each image.
        if show:
            plt.figure(figsize=(10, 10))
            for i in range(25):
                plt.subplot(5, 5, i + 1)
                plt.xticks([])
                plt.yticks([])
                plt.grid(False)
                plt.imshow(train_obs[i])
                plt.xlabel(str(round(train_theta[i] / np.pi, 2)) + '$\\pi$')

        # MODEL PARAMETERS
        # model = self._tensorflow((train_obs, train_theta, train_trig), (test_obs, test_theta, test_trig), model_type='model_cnn', show=show, epochs=epochs)
        model = self._torch((train_obs, train_theta, train_trig), (test_obs, test_theta, test_trig), show=show, epochs=epochs)
        return model

    def _tensorflow(self, train, test, model_type='model_cnn', validation_split=0.2, shuffle=True, batch_size=64, epochs=30, show=False):
        import os
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
        import tensorflow as tf
        train_obs = train[0]
        train_theta = train[1]
        train_trig = train[2]
        test_obs = test[0]
        test_theta = test[1]
        test_trig = test[2]

        # USE GPU IF AVAILABLE
        gpus = tf.config.experimental.list_physical_devices('GPU')
        if len(gpus) > 0:
            tf.config.experimental.set_memory_growth(gpus[0], True)

        # BUILD MODELS
        # Building the neural network requires configuring the layers of the model, then compiling the model.
        if model_type == 'model_theta':  # Predict theta directly
            model = tf.keras.Sequential([
                tf.keras.layers.Flatten(),
                tf.keras.layers.Dense(128, activation='relu'),
                tf.keras.layers.Dense(1, activation=None)],
                name='model_theta')
            str_model_type = '$M^\\theta$'
        elif model_type == 'model_trig':  # Predict trigonometric functions of theta
            model = tf.keras.Sequential([
                tf.keras.layers.Flatten(),
                tf.keras.layers.Dense(128, activation='relu'),
                tf.keras.layers.Dense(2, activation=None)],
                name='model_trig')
            str_model_type = '$M^{trig}$'
        elif model_type == 'model_cnn':  # Use a CNN
            model = tf.keras.Sequential([
                tf.keras.layers.Conv2D(32, (3, 3), activation='relu'),
                tf.keras.layers.MaxPooling2D((2, 2)),
                tf.keras.layers.Flatten(),
                tf.keras.layers.Dense(2, activation=None)],
                name='model_cnn')
            str_model_type = '$M^{cnn}$'
        else:
            raise ValueError('Unknown model type selected.')

        # VERIFY THAT A MODEL WAS DEFINED
        try:
            model
        except NameError:
            raise ValueError("Variable 'model' not defined! Make sure to name every keras model 'model'!")

        # COMPILE MODEL
        model.compile(optimizer='adam', loss=tf.keras.losses.MeanSquaredError())

        # FIT MODEL ON DATA
        if model_type != 'model_theta':
            history = model.fit(train_obs, train_trig, batch_size=batch_size, epochs=epochs,
                                validation_split=validation_split, shuffle=shuffle)
        else:
            history = model.fit(train_obs, train_theta, batch_size=batch_size, epochs=epochs,
                                validation_split=validation_split, shuffle=shuffle)

        # EVALUATE PERFORMANCE ON TEST DATASET
        if model_type != 'model_theta':
            test_scores = model.evaluate(test_obs, test_trig, verbose=2)
            output = model.predict(test_obs)
            pred_theta = np.arctan2(output[:, 1], output[:, 0])
        else:
            test_scores = model.evaluate(test_obs, test_theta, verbose=2)
            pred_theta = model.predict(test_obs)[:, 0]
        print("Test loss:", test_scores)

        self._validate_model(pred_theta, test_theta, str_model_type=str_model_type, show=show)

        # MODEL SUMMARY
        if show:
            plt.show()
        model.summary()
        return model

    def _torch(self, train, test, shuffle=True, batch_size=64, epochs=30, show=False):
        import torch
        import torch.nn as nn
        from torch.utils.data import TensorDataset, DataLoader
        import torch.optim as optim
        from eagerx_node_classifier.learner import Net
        import warnings
        warnings.filterwarnings("ignore")
        
        train_obs = torch.from_numpy(train[0]).permute([0, 3, 1, 2])
        train_theta = torch.from_numpy(train[1])
        train_trig = torch.from_numpy(train[2])
        test_obs = torch.from_numpy(test[0]).permute([0, 3, 1, 2])
        test_theta = torch.from_numpy(test[1])
        test_trig = torch.from_numpy(test[2])

        # PREPARE TORCH DATASET & DATALOADER
        trainset = TensorDataset(train_obs, train_trig)
        trainloader = DataLoader(trainset, batch_size=batch_size, shuffle=shuffle, num_workers=2)
        testset = TensorDataset(test_obs, test_trig)
        testloader = DataLoader(testset, batch_size=batch_size, shuffle=False, num_workers=2)

        # BUILD MODELS
        model = Net()

        # DEFINE LOSS AND OPTIMIZER
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=0.001)

        for epoch in range(epochs):  # loop over the dataset multiple times
            running_loss = 0.0
            for i, data in enumerate(trainloader, 0):
                # get the inputs; data is a list of [inputs, labels]
                inputs, labels = data

                # zero the parameter gradients
                optimizer.zero_grad()

                # forward + backward + optimize
                outputs = model(inputs)
                loss = criterion(outputs, labels)
                loss.backward()
                optimizer.step()

                # print statistics
                running_loss += loss.item()
                if i % 150 == 149:  # print every 2000 mini-batches
                    print('[%d, %5d] loss: %.5f' %
                          (epoch + 1, i + 1, running_loss / 2000))
                    running_loss = 0.0

        print('Finished Training')

        # VALIDATE MODEL
        output = model.predict(test_obs).detach().numpy()
        pred_theta = np.arctan2(output[:, 1], output[:, 0])
        self._validate_model(pred_theta, test_theta, str_model_type='$M^{cnn}$', show=show)

        # MODEL SUMMARY
        if show:
            plt.show()
        return model

    def _validate_model(self, pred_theta, test_theta, str_model_type='model_cnn', show=False):
        # EVALUATE MODEL ACCURACY
        # Calculate average error per bin over theta range [-pi, pi]
        test_error = np.abs(test_theta - pred_theta)
        test_error[test_error > np.pi] -= 2 * np.pi
        test_error = np.abs(test_error)
        bins = np.linspace(-np.pi, np.pi, 21)
        digitized = np.digitize(test_theta, bins)
        bin_means = np.array([test_error[digitized == i].mean() for i in range(1, len(bins))])
        if show:
            fig, ax = plt.subplots()
            ax.bar(bins[:-1], bin_means, width=np.diff(bins), edgecolor="black", align="edge")
            ax.set_xlabel('$\\theta$ (rad)')
            ax.set_ylabel('$|\\bar{\\theta} -\\theta|$ (rad)')
            ax.set_title('%s - Average prediction error %s' % (str_model_type, '{:.2e}'.format(test_error.mean())))

    def _change_dtype_np_image(self, image, dtype):
        if dtype and image.dtype != dtype:
            if image.dtype in ('float32', 'float64') and dtype == 'uint8':
                image = (image * 255).astype(dtype)
            elif image.dtype == 'uint8' and dtype in ('float32', 'float64'):
                image = image.astype(dtype) / 255
            else:
                message = 'Cannot convert observations from {} to {}.'
                raise NotImplementedError(message.format(image.dtype, dtype))
        return image

    def _show_ros_image(self, msg):
        import matplotlib.pyplot as plt
        rgb_back = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        plt.imshow(rgb_back)
        plt.show()

    def reset(self):
        self.window = deque(maxlen=self.window_length)
        pass

    def callback(self, node_tick: int, t_n: float,
                 pos_vel: Dict[str, Union[List[Float32MultiArray], float, int]] = None,
                 image: Dict[str, Union[List[Image], float, int]] = None) -> Dict[str, Float32MultiArray]:
        # Prepare image batch
        im_batch = []
        for ros_im in image['msg']:
            if ros_im.height == 0 or ros_im.width == 0:
                continue
            else:
                if isinstance(ros_im.data, bytes):
                    im = np.frombuffer(ros_im.data, dtype=np.uint8).reshape(ros_im.height, ros_im.width, -1)
                else:
                    im = np.array(ros_im.data, dtype=np.uint8).reshape(ros_im.height, ros_im.width, -1)
                im_float32 = self._change_dtype_np_image(im, 'float32')
                im_batch.append(im_float32)

        # Classify images
        if len(im_batch) > 0:
            estimated_pos_batch = self.model.predict(np.array(im_batch))

        # Add to deque
        for est in estimated_pos_batch:
            self.window.append(np.arctan2(est[1], est[0]))

        # Infer angular velocity
        np_window = np.array(self.window)
        diff = np.diff(np_window)
        diff_wrapped = np.arctan2(np.sin(diff), np.cos(diff))
        if len(diff_wrapped) > 0:
            thdot = (diff_wrapped.mean()) * self.cam_rate
        else:
            thdot = 0

        # Prepare ROS msg
        obs = np.array(pos_vel['msg'][-1].data)
        if len(self.window) == 0:
            rospy.logwarn_once('No image received to classify, so feeding through pos_vel.')
            output_msg = dict(estimated_pos_vel=pos_vel['msg'][-1])
        else:
            est_obs = np.array([np.cos(self.window[-1]), np.sin(self.window[-1]), thdot], dtype='float32')
            output_msg = dict(estimated_pos_vel=Float32MultiArray(data=est_obs))
        return output_msg
