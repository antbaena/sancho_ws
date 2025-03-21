import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        print("Asignación de memoria dinámica activada")
    except RuntimeError as e:
        print(e)



from keras_facenet import FaceNet
import numpy as np
print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))


model = FaceNet()

def encode_face(face):
    '''Encondes a given cutted face using FaceNet.
    
    Args:
        face (Image): The cutted face.
    
    Returns:
        features (Array: float): Embeddings/features vector.
    '''

    shapeFace = np.expand_dims(face, axis=0)
    features = model.embeddings(shapeFace)[0]

    return features