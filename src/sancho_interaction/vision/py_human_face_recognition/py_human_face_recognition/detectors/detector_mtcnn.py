import cv2
from mtcnn.mtcnn import MTCNN

faceDetector = MTCNN()

def get_faces(frame, verbose=False):
    """Detects faces on a given frame using MTCNN.

    Note:
        This function support multiple faces on the same frame.

    Args:
        frame (Image): The frame.
        verbose (bool): If true shows the faces detected.

    Returns:
        cuttedFaces (Array: Image): Array with the list of faces detected (can be empty).
        facePositions (Array: Array: int): Array with x and y coordinates, height and width of the rectangle
            containing each face.
        scores (Array: float): Cascade Classifier does not give scores, for compatibility, this array will
            always be full of the same number.
    """

    frameCopy = frame.copy()
    faces = faceDetector.detect_faces(frame)

    scores = []
    cuttedFaces = []
    facePositions = []
    for face in faces:
        x, y, w, h = face['box']
        cuttedFaces.append(frame[y:(y+h), x:(x+w)])
        facePositions.append((x, y, w, h))
        scores.append(10)

        if verbose:
            cv2.rectangle(frameCopy, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        cv2.imshow('Face Detection', frameCopy)

    return facePositions, scores