import rosbag
import rospy


input_bag = rosbag.Bag("/home/haoyang-22/.ros/123-9-24-21-12-31.bag", "r")
output_bag = rosbag.Bag("/home/haoyang-22/project/ESVO/bag/1024-render-100.bag", "w")


new_frequency = 200  
current_time = None
sequence_number = 0

for topic, msg, timestamp in input_bag.read_messages():
   
    if current_time is None:
        current_time = timestamp
    else:
        current_time += rospy.Duration(1.0 / new_frequency)
    msg.header.stamp = current_time

   
    msg.header.seq = sequence_number
    sequence_number += 1

   
    output_bag.write(topic, msg, current_time)


input_bag.close()
output_bag.close()
