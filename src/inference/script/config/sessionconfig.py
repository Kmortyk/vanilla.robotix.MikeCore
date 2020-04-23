import os

from tensorflow.compat.v1.keras import backend as K
import tensorflow as tf


class SessionConfig:
    def configure(self, allow_growth=True):
        self.session = None
        # os.environ['CUDA_VISIBLE_DEVICES'] = "0"
        if allow_growth:
            self.__gpu_allow_growth()

    def __gpu_allow_growth(self):
        config = tf.compat.v1.ConfigProto()
        # config.gpu_options.allow_growth = True
        # config.gpu_options.per_process_gpu_memory_fraction = 0.6
        self.session = tf.compat.v1.Session(config=config)
        K.set_session(self.session)

