import dlib
import cv2
if(dlib.DLIB_USE_CUDA):
    print("Usando cuda")
else:
    print("No usando cuda")
cnn_face_detector = dlib.cnn_face_detection_model_v1('install/py_human_face_recognition/share/py_human_face_recognition/models/mmod_human_face_detector.dat')

def get_faces(frame, verbose=False):
    """Detects faces on a given frame using DLIB CNN Face Detector

    Note:
        This function support multiple faces on the same frame

    Args:
        frame (Image): The frame
        verbose (bool): If true shows the faces detected
    
    Returns:
        face_positions (Array: Array: int): Array with x and y coordinates, height and width of the rectangle
            containing each face
        scores (Array: float): Array of scores of the detected faces. Greater score means more probability
            that the face detected is a face.
    """

    frame_copy = frame.copy()
    faces = cnn_face_detector(frame, 1)

    face_positions = []
    scores = []
    for face in faces:
        rect = face.rect
        x, y, w, h = (rect.left(), rect.top(), rect.width(), rect.height())
        face_positions.append((x, y, w, h))
        scores.append(face.confidence)

        if verbose:
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        cv2.imshow('Face Detection', frame_copy)

    return face_positions, scores, -1
