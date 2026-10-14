#!/usr/bin/env python3
"""Read-only SQLite ROS bag summary. Never publishes ROS messages."""
import collections
import math
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

db = sqlite3.connect('file:' + sys.argv[1] + '?mode=ro', uri=True)
topics = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics')}
origin = db.execute('select min(timestamp) from messages').fetchone()[0]
for i, (name, typ) in topics.items():
    if not any(k in name for k in ('odom', 'cmd_vel', 'rosout', 'status', 'bspline', 'execution_frozen', 'initial_path')):
        continue
    cls = get_message(typ)
    rows = db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,))
    last = None
    gaps, jumps, moving, rotating = [], [], 0, 0
    logs = collections.Counter()
    count = 0
    for stamp, raw in rows:
        msg = deserialize_message(raw, cls)
        sec = (stamp - origin) / 1e9
        count += 1
        if last is not None:
            gaps.append((stamp - last[0]) / 1e9)
        if 'Odometry' in typ:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if last is not None:
                d = math.hypot(p.x-last[1][0], p.y-last[1][1])
                a = abs(math.atan2(math.sin(yaw-last[1][2]), math.cos(yaw-last[1][2])))
                if d > .2 or a > .3:
                    jumps.append((round(sec,2),round(d,3),round(a,3),round(gaps[-1],3)))
            last = (stamp, (p.x,p.y,yaw))
        else:
            last = (stamp, None)
        if 'Twist' in typ:
            moving += int(math.hypot(msg.linear.x,msg.linear.y)>.02)
            rotating += int(abs(msg.angular.z)>.1 and math.hypot(msg.linear.x,msg.linear.y)<.01)
        if name == '/rosout' and msg.level >= 30:
            logs[(msg.name,msg.msg)] += 1
        elif 'status' in name and hasattr(msg,'data'):
            logs[str(msg.data)] += 1
    print('\nTOPIC', name, 'count', count, 'max_gap', round(max(gaps,default=0),3),
          'gaps>1s',sum(g>1 for g in gaps), 'moving',moving,'in_place_rotation',rotating)
    if jumps: print('POSE_CHANGES seconds,metres,radians,gap:', jumps[:60])
    for item, n in logs.most_common(25): print('LOG', n, item)
