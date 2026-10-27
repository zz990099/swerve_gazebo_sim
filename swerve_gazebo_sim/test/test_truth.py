"""Decode simulator truth independently of ROS and the wheel estimator."""

from gazebo_truth import decode_odometry


def test_protobuf_json_defaults_and_timestamp_fields():
    message = decode_odometry(
        {
            "header": {"stamp": {"sec": "123", "nsec": 450000000}},
            "pose": {"position": {"x": 2.5}, "orientation": {"w": 1}},
            "twist": {"linear": {"y": -0.3}, "angular": {"z": 0.2}},
        }
    )
    assert message.header.stamp.sec == 123
    assert message.header.stamp.nanosec == 450000000
    assert message.pose.pose.position.x == 2.5
    assert message.pose.pose.position.y == 0
    assert message.pose.pose.orientation.w == 1
    assert message.twist.twist.linear.y == -0.3
    assert message.twist.twist.angular.z == 0.2


def test_zero_timestamp_and_stationary_twist_omit_zero_fields():
    message = decode_odometry(
        {
            "header": {"stamp": {}},
            "pose": {"orientation": {"w": 1}},
            "twist": {},
        }
    )
    assert message.header.stamp.sec == message.header.stamp.nanosec == 0
    assert message.twist.twist.linear.x == message.twist.twist.angular.z == 0
