uint16 PROJECTION = 1
uint16 ORIENTATION = 2
uint16 JOINT_SETPOINT = 3
uint16 JOINT_LIMIT_AVOIDANCE = 4

uint16       t_type
uint16       priority
bool         is_equality_task
string       task_frame
float64      ds #velocity damping safety zone
float64      di #velocity damping influence parameter
TaskDynamics dynamics
TaskLink[]   t_links



