uint16 PROJECTION = 1
uint16 ORIENTATION = 2
uint16 JOINT_SETPOINT = 3
uint16 JOINT_LIMIT_AVOIDANCE = 4

uint16       type
uint16       priority
bool         is_equality_task
string       frame #the frame name of the task
TaskDynamics dynamics
TaskLink[]   t_links



