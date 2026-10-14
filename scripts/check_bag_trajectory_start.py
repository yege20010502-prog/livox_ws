#!/usr/bin/env python3
"""Read-only audit of actual spline starts versus latest body pose."""
import bisect
import math
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

db = sqlite3.connect('file:' + sys.argv[1] + '?mode=ro', uri=True)
origin = db.execute('select min(timestamp) from messages').fetchone()[0]
def read(name):
    topic, typ = db.execute('select id,type from topics where name=?', (name,)).fetchone()
    cls = get_message(typ)
    return [(t, deserialize_message(b, cls)) for t, b in db.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp', (topic,))]
poses = read('/LIO/odom_vehicle')
stamps = [t for t, _ in poses]
rejected = 0
for stamp, msg in read('/planning/bspline'):
    j = bisect.bisect_right(stamps, stamp) - 1
    if j < 0 or (stamp-stamps[j])/1e9 > 1:
        continue
    degree = msg.order
    knots = msg.knots
    u = knots[degree]
    span = degree
    while span + 1 < len(msg.pos_pts) and knots[span+1] < u:
        span += 1
    pts = [[p.x, p.y, p.z] for p in msg.pos_pts[span-degree:span+1]]
    for level in range(1, degree+1):
        for i in range(degree, level-1, -1):
            a = span-degree+i
            denominator = knots[a+degree-level+1] - knots[a]
            weight = (u-knots[a])/denominator if denominator else 0
            pts[i] = [(1-weight)*x+weight*y for x,y in zip(pts[i-1], pts[i])]
    body = poses[j][1].pose.pose.position
    error = math.hypot(pts[degree][0]-body.x, pts[degree][1]-body.y)
    if error > .35:
        rejected += 1
        print(f'{(stamp-origin)/1e9:.2f}s traj={msg.traj_id} start/body={error:.3f}m')
print('Fresh-pose trajectories exceeding 0.35m:', rejected)
