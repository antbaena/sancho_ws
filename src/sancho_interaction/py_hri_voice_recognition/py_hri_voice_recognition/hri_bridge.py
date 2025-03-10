from std_msgs.msg import String
from cv_bridge import CvBridge

from human_face_recognition_msgs.msg import FacePosition

class HRIBridge:

    def __init__(self):
        self.br = CvBridge()
    
    def imgmsg_to_cv2(self, img_msg, desired_encoding='passthrough'):
        return self.br.imgmsg_to_cv2(img_msg, desired_encoding)
    
    def cv2_to_imgmsg(self, cvim, encoding='passthrough'):
        return self.br.cv2_to_imgmsg(cvim, encoding)
    
    def detector_to_msg(self, positions, scores):
        positions_msg = []
        for (x,y,w,h) in positions:
            pos = FacePosition()
            pos.x = x
            pos.y = y
            pos.w = w
            pos.h = h
            positions_msg.append(pos)

        scores_msg = scores

        return positions_msg, scores_msg

    def msg_to_detector(self, positions_msg, scores_msg):
        positions = []
        for pos in positions_msg:
            positions.append([pos.x, pos.y, pos.w, pos.h])

        scores = scores_msg

        return positions, scores
    
    def recognizer_to_msg(self, face_aligned, features, classified, distance, pos):
        face_aligned_msg = self.cv2_to_imgmsg(face_aligned, "bgr8")
        features_msg = [float(feature) for feature in features]
        classified_msg = String(data=str(classified))
        distance_msg = float(distance)
        pos_msg = pos

        return face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg
    
    def msg_to_recognizer(self, face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg):
        face_aligned = self.imgmsg_to_cv2(face_aligned_msg, "bgr8")
        features = features_msg
        classified = classified_msg.data
        distance = distance_msg
        pos = pos_msg

        return face_aligned, features, classified, distance, pos