group = moveit_commander.MoveGroupCommander(group_name)
sequence_move_group_srv = actionlib.SimpleActionClient("/sequence_move_group", moveit_msgs.msg.MoveGroupSequenceAction)

motion_plan_requests = []
# just in case, set the first point as the current pose
group.set_pose_target(group.get_current_pose(end_effector_link))
# Build request items
msi = moveit_msgs.msg.MotionSequenceItem()
msi.req = group.construct_motion_plan_request()
msi.blend_radius = 0.0

motion_plan_requests.append(msi)

waypoints = [(pose1, 0.01), (pose2, 0.01), (pose3, 0.01)] # Pairs of pose, blend radius (just an example)
for wp, blend_radius in waypoints:
    group.clear_pose_targets() 
    group.set_pose_target(wp)
    msi = moveit_msgs.msg.MotionSequenceItem()
    msi.req = group.construct_motion_plan_request()
    msi.req.start_state = moveit_msgs.msg.RobotState() # only the first point can have a non-empty start state
    msi.blend_radius = blend_radius
    motion_plan_requests.append(msi)

# Force last point to be 0.0 to avoid raising an error in the planner
motion_plan_requests[-1].blend_radius = 0.0

# Make MotionSequence Request
goal = moveit_msgs.msg.MoveGroupSequenceGoal()
goal.request = moveit_msgs.msg.MotionSequenceRequest()
goal.request.items = motion_plan_requests

sequence_move_group_srv.send_goal_and_wait(goal)
response = sequence_move_group_srv.get_result()
    
group.clear_pose_targets()

if response.response.error_code.val == 1: 
    # The response is an array but I think there is always only one plan returned
    plan = response.response.planned_trajectories[0]
    group.execute(plan)
