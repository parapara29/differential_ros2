import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

from cam_services.srv import StartRecording, StopRecording

class RGBRecorderNode(Node):
    def __init__(self):
        super().__init__("rgb_recorder_node")

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/rgb",  # Replace this with the actual RGB topic name
            self.image_callback,
            10,
        )
        self.is_recording = False
        self.fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.output_file = '/home/paradox/Videos/isaac_sim_rec/teleop.mp4' # Replace this with the desired output file path
        self.video_writer = None

        self.start_recording_service = self.create_service(
            StartRecording,
            "start_recording",
            self.start_recording_callback,
        )

        self.stop_recording_service = self.create_service(
            StopRecording,
            "stop_recording",
            self.stop_recording_callback,
        )

    def image_callback(self, msg):
        if self.is_recording:
            if self.video_writer is None:
                # Add timestamp with colons to the filename
                timestamp = time.strftime("%Y_%m_%d_%H:%M:%S", time.localtime())
                # timestamp_with_colons = ":".join([timestamp[i:i+2] for i in range(0, len(timestamp), 2)])
                self.output_file_with_timestamp = self.output_file.replace(".mp4", f"_{timestamp}.mp4")

                height, width = msg.height, msg.width
                self.video_writer = cv2.VideoWriter(
                    self.output_file_with_timestamp, self.fourcc, 30, (width, height)
                )

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.video_writer.write(cv_image)

    def start_recording_callback(self, request, response):
        if not self.is_recording:
            self.is_recording = True
            response.success = True
        else:
            response.success = False
        return response

    def stop_recording_callback(self, request, response):
        if self.is_recording:
            self.is_recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            response.success = True
        else:
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RGBRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()