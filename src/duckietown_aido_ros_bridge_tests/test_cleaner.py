from duckietown_aido_ros_bridge import PondCleaner


def test_pondcleaner1():
    pc = PondCleaner()
    print(pc.clean())
