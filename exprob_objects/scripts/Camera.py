import rospy
from exprob_msgs.srv import Oracle, OracleRequest


class Camera:
    def __init__(self):
        rospy.loginfo("Searching for Hints")
        self.search_hint()

    def get_hint(self):
        if self.response.result == "ok":
            return self.response.hint_id
        if self.response.result == "no new hint":
            return -1

    def search_hint(self):
        req = OracleRequest()
        req.goal = "generate hint"
        self.response = self.call_service(
            req=req, srv_name="/oracle_srv", srv_type=Oracle()
        )
        self.decode_image(self.response)

    def call_service(self, req, srv_name, srv_type):
        rospy.wait_for_service(srv_name)
        try:
            response = rospy.ServiceProxy(srv_name, srv_type)
            return response(req)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def decode_image(self, image_data):
        pass  ##TODO: use computer vision to decode the QRCode
