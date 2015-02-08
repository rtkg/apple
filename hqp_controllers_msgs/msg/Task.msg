uint16 PROJECT_POINT_PLANE = 1    #keep a point on a plane or in the positive/negative halfspace defined by the plane
uint16 JOINT_SETPOINT = 2         #Servo to a joint setpoint
uint16 JOINT_VELOCITY_LIMITS = 3  #joint limits via velocity limits
uint16 PARALLEL_LINES = 4         #keep to lines parallel
uint16 ANGLE_LINES = 5            # keep one line in a cone described by the other line and an angle
uint16 PROJECT_POINT_CYLINDER = 6 # keep a point on/inside/outside a cylinder
uint16 COPLANAR_LINES = 7         # keep to lines coplanar
uint16 PROJECT_LINE_LINE = 8      # coplanar lines via projection on the normal distance


uint16       type
uint16       priority
string       sign
uint16[]     t_obj_ids
TaskDynamics dynamics


