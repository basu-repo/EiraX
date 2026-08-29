from pathlib import Path
import csv,sys,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag,out=Path(sys.argv[1]),Path(sys.argv[2])
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:x.type for x in r.get_all_topics_and_types()}
count={k:0 for k in types};size={k:0 for k in types};first={};last={};pose=[]
pose_type=get_message(types['/mavros/local_position/pose'])
while r.has_next():
 topic,data,t=r.read_next();count[topic]+=1;size[topic]+=len(data);first.setdefault(topic,t);last[topic]=t
 if topic=='/mavros/local_position/pose':
  p=deserialize_message(data,pose_type).pose.position;pose.append((t,p.x,p.y,p.z))
with (out/'topic_measurements.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['topic','message_count','duration_sec','frequency_hz','total_serialized_bytes','average_serialized_message_bytes','bandwidth_kbps'])
 for topic in sorted(types):
  duration=max(1e-9,(last[topic]-first[topic])/1e9);n=count[topic]
  w.writerow([topic,n,duration,n/duration,size[topic],size[topic]/max(1,n),size[topic]*8/duration/1000])
assert pose
t0,x0,y0,z0=pose[0]
with (out/'ros_pose_trace.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['elapsed_sec','east_m','north_m','up_m'])
 for t,x,y,z in pose:w.writerow([(t-t0)/1e9,x-x0,y-y0,z-z0])
