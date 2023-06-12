
import numpy as np
import tensorflow.compat.v1 as tf

class Actions():
    # Define 11 choices of actions:
    # [v_pref,        [-pi/6, -pi/12, 0, pi/12, pi/6]]
    # [0.5 * v_pref,  [-pi/6, 0, pi/6]]
    # [0,             [-pi/6, 0, pi/6]]
    def __init__(self):
        self.actions = np.mgrid[1.0:1.1:0.5, -np.pi/6:np.pi/6+0.01:np.pi/12].reshape(2, -1).T
        self.actions = np.vstack([self.actions,np.mgrid[0.5:0.6:0.5, -np.pi/6:np.pi/6+0.01:np.pi/6].reshape(2, -1).T])
        self.actions = np.vstack([self.actions,np.mgrid[0.0:0.1:0.5, -np.pi/6:np.pi/6+0.01:np.pi/6].reshape(2, -1).T])
        self.num_actions = len(self.actions)

class NetworkVPCore(object):
    def __init__(self, device):
        self.device = device

    def crop_x(self, x):
        if x.shape[-1] > self.x.shape[-1]:
            x_ = x[:,:self.x.shape[-1]]
        elif x.shape[-1] < self.x.shape[-1]:
            x_ = np.zeros((x.shape[0], self.x.shape[-1]))
            x_[:,:x.shape[1]] = x
        else:
            x_ = x
        return x_

    def predict_p(self, x):
        x = self.crop_x(x)
        return self.sess.run(self.softmax_p, feed_dict={self.x: x})

    def simple_load(self, filename=None):
        if filename is None:
            print("[network.py] Didn't define simple_load filename")
            raise NotImplementedError

        self.graph = tf.Graph()
        with self.graph.as_default() as g:
            with tf.device(self.device):

                self.sess = tf.Session(
                    graph=self.graph,
                    config=tf.ConfigProto(
                        allow_soft_placement=True,
                        log_device_placement=False,
                        gpu_options=tf.GPUOptions(allow_growth=True)))

                new_saver = tf.train.import_meta_graph(filename+".meta", clear_devices=True)
                self.sess.run(tf.global_variables_initializer())
                new_saver.restore(self.sess, filename)

                self.x = g.get_tensor_by_name("X:0")
                self.v = g.get_tensor_by_name("Squeeze:0")
                self.softmax_p = g.get_tensor_by_name("Softmax:0")

class NetworkVPRNN(NetworkVPCore):
    def __init__(self, device):
        super(self.__class__, self).__init__(device)
