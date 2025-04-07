import cv2

face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def get_faces(frame, verbose=False):
    """Detects faces on a given frame using OpenCV Cascade Classifier.

    Note:
        This function support multiple faces on the same frame.

    Args:
        frame (Image): The frame.
        verbose (bool): If true shows the faces detected.
    
    Returns:
        face_positions (Array: Array: int): Array with x and y coordinates, height and width of the rectangle
            containing each face.
        scores (Array: float): Cascade Classifier does not give scores, for compatibility, this array will
            always be full of the same number.
    """

    frame_copy = frame.copy()
    faces = face_detector.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    scores = []
    face_positions = []
    for (x, y, w, h) in faces:
        face_positions.append((x, y, w, h))
        scores.append(10)

        if verbose:
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        cv2.imshow('Face Detection', frame_copy)

    return face_positions, scores