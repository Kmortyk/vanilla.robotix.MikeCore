from tensorflow.keras.applications.mobilenet_v2 import MobileNetV2 as Net
from tensorflow.python.framework import graph_io
from tensorflow.keras.models import load_model
import tensorflow.contrib.tensorrt as trt
import tensorflow as tf
import os

model = Net(weights='imagenet')
os.makedirs('./model', exist_ok=True)

# Save the h5 file to path specified.
model.save("./model/model.h5")

# Clear any previous session.
tf.keras.backend.clear_session()

save_pb_dir = './model'
model_fname = './model/model.h5'
def freeze_graph(graph, session, output, save_pb_dir='.', save_pb_name='frozen_model.pb', save_pb_as_text=False):
    with graph.as_default():
        graphdef_inf = tf.graph_util.remove_training_nodes(graph.as_graph_def())
        graphdef_frozen = tf.graph_util.convert_variables_to_constants(session, graphdef_inf, output)
        graph_io.write_graph(graphdef_frozen, save_pb_dir, save_pb_name, as_text=save_pb_as_text)
        return graphdef_frozen

# This line must be executed before loading Keras model.
tf.keras.backend.set_learning_phase(0) 

model = load_model(model_fname)
session = tf.keras.backend.get_session()

input_names = [t.op.name for t in model.inputs]
output_names = [t.op.name for t in model.outputs]

# Prints input and output nodes names, take notes of them.
print(input_names, output_names)
print("create frozen graph")
frozen_graph = freeze_graph(session.graph, session, [out.op.name for out in model.outputs], save_pb_dir=save_pb_dir)

print("create trt graph")
trt_graph = trt.create_inference_graph(input_graph_def=frozen_graph, outputs=output_names, max_batch_size=1,
                                       max_workspace_size_bytes=1 << 25, precision_mode='FP16', minimum_segment_size=50)
print("write graph")
graph_io.write_graph(trt_graph, "./model/", "trt_graph.pb", as_text=False)