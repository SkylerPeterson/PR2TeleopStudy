import rosbag
bag = rosbag.Bag("/home/skyler/OculusData/Default_Joint_State_1.bag")
for topic, msg, t in bag.read_messages():
    print msg
bag.close()
