"""Real Nav2 planning/control against a simulated plant, never a motor topic."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Twist
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml
from match_mur_navigation.session import LATCHED


def test_real_nav2_saved_goal_motion_limits_cancel_and_sensor_loss(tmp_path):
    size, resolution = 220,.05
    pixels = bytearray([254])*(size*size)
    for row in range(size):
        for col in range(size):
            if row < 3 or col < 3 or row >= size-3 or col >= size-3:
                pixels[row*size+col] = 0
            wx,wy = -5.5+(col+.5)*resolution, 5.5-(row+.5)*resolution
            if 3.0 <= wx <= 3.5 and abs(wy) <= .4:
                pixels[row*size+col] = 0
    (tmp_path/'room.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode()+pixels)
    map_path = tmp_path/'room.yaml'
    map_path.write_text(yaml.safe_dump(dict(image='room.pgm',resolution=resolution,
        origin=[-5.5,-5.5,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
    map_path.with_suffix('.mapping.yaml').write_text(yaml.safe_dump(dict(frame='mocap')))
    rclpy.init(args=[])
    node = rclpy.create_node('synthetic_navigation_plant')
    robot,ns = 'mur620c','/mur620c/navigation'
    source = '/test_navigation_'+uuid.uuid4().hex[:8]
    output_topic = source+'/simulated_motor_commands'
    pose_pub = node.create_publisher(PoseStamped,'/qualisys/'+robot+'/pose',100)
    odom_pub = node.create_publisher(Odometry,source+'/odom_enc',qos_profile_sensor_data)
    scan_pubs = {k:node.create_publisher(LaserScan,source+'/'+k,qos_profile_sensor_data) for k in ('f_scan','b_scan')}
    static_pub = node.create_publisher(TFMessage,source+'/tf_static',LATCHED)
    native_pub = node.create_publisher(GoalStatusArray,source+'/move_base/status',10)
    statuses,commands,paths = [],[],[]
    channels = {name:[] for name in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel_guarded')}
    for name in channels:
        node.create_subscription(Twist,ns+'/'+name,lambda msg,k=name:channels[k].append(
            (msg.linear.x,msg.angular.z)),10)
    latest_command = [None,0.]
    def command(message):
        latest_command[:] = [message,time.monotonic()]
        commands.append((time.monotonic(),message))
    node.create_subscription(TwistStamped,output_topic,command,10)
    node.create_subscription(String,ns+'/status',lambda m:statuses.append(json.loads(m.data)),LATCHED)
    node.create_subscription(NavPath,ns+'/plan',paths.append,10)
    client = ActionClient(node,NavigateToPose,ns+'/navigate_to_pose')
    planner = ActionClient(node,ComputePathToPose,ns+'/compute_path_to_pose')
    edges = []
    for parent,child,x,y,angle in (('base_footprint','base_link',0.,0.,0.),
            ('base_link','front_laser_link',.45,.2,0.),('base_link','back_laser_link',-.45,-.2,math.pi)):
        t = TransformStamped(child_frame_id=robot+'/'+child)
        t.header.frame_id = robot+'/'+parent
        t.transform.translation.x,t.transform.translation.y = x,y
        t.transform.rotation.z,t.transform.rotation.w = math.sin(angle/2),math.cos(angle/2)
        edges.append(t)
    calibration = TransformStamped(child_frame_id='mocap')
    calibration.header.frame_id = 'map'
    calibration.transform.translation.x,calibration.transform.translation.y = 10.,20.
    calibration.transform.rotation.z,calibration.transform.rotation.w = math.sin(math.pi/4),math.cos(math.pi/4)
    edges.append(calibration)
    x = y = angle = 0.
    last = time.monotonic()
    with (tmp_path/'nav2.log').open('w') as log:
        environment = dict(os.environ)
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
            environment.pop(key,None)
        process = subprocess.Popen(['ros2','launch','match_mur_navigation','navigation.launch.py',
            'robot:='+robot,'map:='+str(map_path),'sensor_namespace:='+source,
            'output_topic:='+output_topic,'output_directory:='+str(tmp_path/'parameters')],
            stdout=log,stderr=subprocess.STDOUT,env=environment,start_new_session=True)

        def pump(seconds, mocap=True):
            nonlocal x,y,angle,last
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path/'nav2.log').read_text()[-9000:]
                now = time.monotonic()
                dt = min(now-last,.05)
                last = now
                if latest_command[0] and now-latest_command[1] < .2:
                    twist = latest_command[0].twist
                    x += twist.linear.x*math.cos(angle)*dt
                    y += twist.linear.x*math.sin(angle)*dt
                    angle += twist.angular.z*dt
                when = node.get_clock().now().to_msg()
                if mocap:
                    p = PoseStamped()
                    p.header.frame_id,p.header.stamp = 'mocap',when
                    p.pose.position.x,p.pose.position.y = x,y
                    p.pose.orientation.z,p.pose.orientation.w = math.sin(angle/2),math.cos(angle/2)
                    pose_pub.publish(p)
                odom = Odometry()
                odom.header.frame_id,odom.header.stamp,odom.child_frame_id = robot+'/odom',when,robot+'/base_footprint'
                odom.pose.pose.position.x,odom.pose.pose.position.y = x,y
                odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(angle/2),math.cos(angle/2)
                if latest_command[0]:
                    odom.twist.twist = latest_command[0].twist
                odom_pub.publish(odom)
                status = GoalStatusArray()
                status.header.stamp = when
                native_pub.publish(status)
                static_pub.publish(TFMessage(transforms=edges))
                for name,frame,dx,dy,rotation in (
                        ('f_scan','front_laser_link',.45,.2,0.),
                        ('b_scan','back_laser_link',-.45,-.2,math.pi)):
                    scan = LaserScan()
                    scan.header.frame_id,scan.header.stamp = robot+'/'+frame,when
                    scan.angle_min,scan.angle_max,scan.angle_increment = -2.35,2.35,.02
                    scan.range_min,scan.range_max,scan.time_increment = .05,15.,0.
                    sx = x+math.cos(angle)*dx-math.sin(angle)*dy
                    sy = y+math.sin(angle)*dx+math.cos(angle)*dy
                    ranges = []
                    for index in range(236):
                        a = angle+rotation+scan.angle_min+index*scan.angle_increment
                        vx,vy = math.cos(a),math.sin(a)
                        distances = []
                        if abs(vx)>1e-6:
                            distances.append(((5.35 if vx>0 else -5.35)-sx)/vx)
                        if abs(vy)>1e-6:
                            distances.append(((5.35 if vy>0 else -5.35)-sy)/vy)
                        ranges.append(min(distances))
                    scan.ranges = ranges
                    scan_pubs[name].publish(scan)
                rclpy.spin_once(node,timeout_sec=.005)
                time.sleep(.02)

        def until(predicate, timeout, mocap=True):
            deadline = time.monotonic()+timeout
            while not predicate() and time.monotonic() < deadline:
                pump(.1,mocap)
            assert predicate(), ((statuses[-1] if statuses else 'No status'),
                                  dict(x=x,y=y,angle=angle,cmd=repr(latest_command[0]),
                                       recent_velocities=[m.twist.linear.x for _,m in commands[-10:]]),
                                  (tmp_path/'nav2.log').read_text()[-11000:])

        def send(goal_x):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id,goal.pose.header.stamp = 'map',node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = 10.,20.+goal_x
            goal.pose.pose.orientation.z,goal.pose.pose.orientation.w = math.sin(math.pi/4),math.cos(math.pi/4)
            future = client.send_goal_async(goal)
            until(future.done,3.)
            assert future.result().accepted, statuses[-1]
            return future.result()

        try:
            until(lambda:bool(statuses) and statuses[-1]['ready'] and client.server_is_ready(),25.)
            assert not commands, 'Starting a stack must not claim the motors'
            # Exercise footprint/inflation around a static obstacle without issuing a drive goal.
            until(planner.server_is_ready,3.)
            plan_goal = ComputePathToPose.Goal()
            plan_goal.goal.header.frame_id,plan_goal.goal.header.stamp = 'mocap',node.get_clock().now().to_msg()
            plan_goal.goal.pose.position.x,plan_goal.goal.pose.orientation.w = 4.3,1.
            plan_goal.planner_id = 'GridBased'
            planning = planner.send_goal_async(plan_goal)
            until(planning.done,3.)
            assert planning.result().accepted
            planned = planning.result().get_result_async()
            until(planned.done,5.)
            assert planned.result().status == 4
            assert max(abs(p.pose.position.y) for p in planned.result().result.path.poses) > .8
            assert not commands
            first = send(.7)
            result = first.get_result_async()
            until(result.done,20.)
            assert result.result().status == 4, (result.result(),(tmp_path/'nav2.log').read_text()[-5000:])
            assert .5 < x < .9
            assert paths and paths[-1].header.frame_id == 'mocap'
            assert paths[-1].poses[-1].pose.position.x == pytest.approx(.7,abs=.06)
            nonzero = [m for _,m in commands if abs(m.twist.linear.x)>1e-4 or abs(m.twist.angular.z)>1e-4]
            assert nonzero
            assert max(abs(m.twist.linear.x) for m in nonzero) <= .25001
            assert max(abs(m.twist.angular.z) for m in nonzero) <= .40001
            # Explicit action cancellation must stop forwarding and return CANCELED.
            pump(.3)
            second = send(1.8)
            second_result = second.get_result_async()
            until(lambda:statuses[-1]['motion_enabled'] and latest_command[0].twist.linear.x>.02,5.)
            canceled = second.cancel_goal_async()
            until(canceled.done,3.)
            assert canceled.result().goals_canceling
            until(second_result.done,5.)
            assert second_result.result().status == 5
            pump(.5)
            assert abs(latest_command[0].twist.linear.x) < 1e-6
            # A new goal is aborted on Mocap loss; fresh data must not revive it.
            third = send(2.)
            third_result = third.get_result_async()
            until(lambda:statuses[-1]['motion_enabled'] and latest_command[0].twist.linear.x>.02,5.)
            until(third_result.done,7.,mocap=False)
            assert third_result.result().status == 6
            pump(.5)
            assert not statuses[-1]['busy'] and not statuses[-1]['motion_enabled']
            assert abs(latest_command[0].twist.linear.x) < 1e-6
            # Global hardware TF and motor topics never receive outputs from this test stack.
            for topic in ('/tf','/tf_static','/'+robot+'/cmd_vel_stamped'):
                assert not any(e.node_namespace == ns for e in node.get_publishers_info_by_topic(topic))
        finally:
            (tmp_path/'diagnostics.json').write_text(json.dumps(dict(x=x,y=y,angle=angle,
                status=statuses[-1] if statuses else None,
                hardware=[(m.twist.linear.x,m.twist.angular.z) for _,m in commands[-30:]],
                channels={name:values[-30:] for name,values in channels.items()}),indent=2))
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=7.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            client.destroy()
            planner.destroy()
            node.destroy_node()
            rclpy.shutdown()
