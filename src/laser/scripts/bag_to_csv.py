import rosbag
import pandas as pd
from sensor_msgs.msg import LaserScan
# The bag file should be in the same directory as your terminal
bag = [rosbag.Bag('/home/dock/workspace/src/front_right.bag'),rosbag.Bag('/home/dock/workspace/src/front_laser.bag'),rosbag.Bag('/home/dock/workspace/src/left_laser.bag')]
names = ["front_laser.csv","right_laser.csv","left_laser.csv"]
topic = '/pepper_robot/laser'
column_names = ['Duration']+[str(i) for i in range(1,16)]
print(column_names)
df = pd.DataFrame(columns=column_names)
i = 0
st = 0
ed = 15
for bg in bag:
    print("start: %d, end:%d" % (st,ed))

    df = pd.DataFrame(columns=column_names)
    for topic, msg,t in bg.read_messages(topics=topic):
        x = msg.time_increment
        y = msg.ranges[st:ed]
        df.loc[len(df),column_names]=[x]+list(y)
    st = ed +8
    ed = st+15
    df.to_csv(names[i])
    i+=1