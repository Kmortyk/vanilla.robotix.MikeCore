import os

import tensorflow.keras.backend as K
import tensorflow as tf


class SessionConfig:
    @staticmethod
    def configure(allow_growth=True):
        os.environ['CUDA_VISIBLE_DEVICES'] = "0"
        if allow_growth:
            SessionConfig.__gpu_allow_growth()

    @staticmethod
    def __gpu_allow_growth():
        config = tf.compat.v1.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.compat.v1.Session(config=config)
        K.set_session(sess)

