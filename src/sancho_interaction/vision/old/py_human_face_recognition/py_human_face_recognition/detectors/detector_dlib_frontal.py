import dlib
import cv2

face_detector = dlib.get_frontal_face_detector()

def get_faces(frame, verbose=False):
    """Detects faces on a given frame using DLIB Frontal Face Detector

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
    faces, scores, idx = face_detector.run(frame, 1, 0)

    face_positions = []
    for face in faces:
        x, y, w, h = (face.left(), face.top(), face.width(), face.height())
        face_positions.append((x, y, w, h))

        if verbose:
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        cv2.imshow('Face Detection', frame_copy)

    return face_positions, scores, idx