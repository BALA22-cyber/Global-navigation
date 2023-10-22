import rospy
# from social_navigation.social_navigation_planners import ObjectDetection, ObjectDetectionResult
from social_navigation.rosbag_publisher import publish_images_from_rosbag

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('global_navigation', anonymous=True)

    # Create instances of TaskManager and ObjectDetection
    # object_detection = ObjectDetection()

    # # Initialize object detection result publisher
    # detection_result_handler = ObjectDetectionResult()

    # Provide the path to your ROS bag and the relevant topic name
    rosbag_path = 'irib2.bag'
    topic_name = '/camera/color/image_raw/compressed'  # Change this to the appropriate topic name

    # Set the playback speed (adjust as needed)
    playback_speed = 36

    # Start publishing images from the ROS bag
    publish_images_from_rosbag(rosbag_path, topic_name, playback_speed)

    # Create an instance of TaskManager
    # navigation = TaskManager()

    # # Run the navigation system
    # navigation.run()
