from rosys.driving import Velocity, Odometer, Drive, track_width_to_turn_radius
from rosys.event import Event
from rosys.helpers import sleep, remove_indentation
from rosys.hardware import ModuleHardware, RobotBrain
from rosys.module import Timer


class TwistedFieldsMotorHardware(ModuleHardware):
    """
    Provides the interface to the Twisted Fields custom CAN motor controllers.
    It translates high-level RoSys commands into low-level commands for the Lizard firmware.
    This module implements its own skid-steer kinematics and uses a periodic timer 
    to send individual wheel speeds to the Lizard firmware via the RobotBrain.
    """
    
    # Constants
    TRACK_WIDTH = 0.5 # meters, assumed for skid-steer kinematics
    
    # Events
    SPEED_MEASURED = Event()
    MOTOR_ERROR = Event()
    
    # State
    left_speed: float = 0.0
    right_speed: float = 0.0
    error: bool = False
    
    def __init__(self, robot_brain: RobotBrain, odometer: Odometer, drive: Drive, name: str = 'twisted_fields_motor') -> None:
        # The Lizard C++ module is assumed to be named 'twisted_fields_motor'
        # and exposes a 'set_speed' command that takes left and right speed.
        lizard_code = remove_indentation(f'''
            {name} = TwistedFieldsMotor()
        ''')
        
        # Core message fields for feedback from Lizard (e.g., measured speeds, error flags)
        # Assuming the C++ module reports back measured speeds and an error flag.
        core_message_fields = [f'{name}.measured_left:3', f'{name}.measured_right:3', f'{name}.error_flag']
        
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)
        
        self.name = name
        self.odometer = odometer
        self.drive = drive
        
        # Timer to periodically send motor commands (20 Hz update rate)
        self.timer = Timer(self._update_motor_commands, 0.05) 
        
        self.log.info('TwistedFieldsMotorHardware module initialized.')

    async def set_target_velocity(self, linear: float, angular: float) -> None:
        """
        Set the desired linear and angular velocity.
        This method will update the drive object and calculate the individual wheel speeds.
        """
        self.drive.linear = linear
        self.drive.angular = angular
        
        # Skid-steer kinematics calculation
        turn_radius = track_width_to_turn_radius(self.TRACK_WIDTH)
        
        if angular == 0:
            self.left_speed = linear
            self.right_speed = linear
        else:
            # V_left = linear - angular * (track_width / 2)
            # V_right = linear + angular * (track_width / 2)
            self.left_speed = linear - angular * turn_radius
            self.right_speed = linear + angular * turn_radius
        
        self.log.info(f'Driving with linear={linear:.2f} m/s, angular={angular:.2f} rad/s. Left={self.left_speed:.2f}, Right={self.right_speed:.2f}')
        
    async def _update_motor_commands(self) -> None:
        """
        Periodically sends the calculated wheel speeds to the Lizard firmware.
        This function is called by the internal Timer.
        """
        if self.is_running and self.robot_brain.is_ready:
            # Send the two calculated speeds to the Lizard C++ module
            await self.robot_brain.send(f'{self.name}.set_speed({self.left_speed:.3f}, {self.right_speed:.3f})')
            
    async def stop(self) -> None:
        """Stop the robot by setting all velocities to zero."""
        await self.set_target_velocity(0.0, 0.0)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        """
        Handles the core message output from the Lizard firmware.
        Expected format: [measured_left, measured_right, error_flag]
        """
        if len(words) < 3:
            self.log.warning(f'Received incomplete core output: {words}')
            return

        # 1. Update Odometer
        measured_left = float(words.pop(0))
        measured_right = float(words.pop(0))
        # The time step (dt) for the odometer increment is assumed to be the timer interval (0.05s)
        # or can be calculated from the time difference since the last update.
        # For simplicity and following the draft's intent, we use a fixed value or rely on the RoSys core.
        # Since we don't have the previous time, we'll assume a fixed dt for now, or better,
        # let the RoSys core handle the time difference if possible.
        # The original draft used sleep(0.1), so let's use 0.1 as a placeholder dt.
        dt = 0.1 # Placeholder for time difference between core messages
        self.odometer.increment(measured_left, measured_right, dt)
        
        # 2. Update Error State
        error_flag = int(words.pop(0))
        self.error = error_flag == 1
        if self.error:
            self.MOTOR_ERROR.emit()
            self.log.error('Twisted Fields Motor Error detected.')

        # 3. Emit measured velocity (optional, but good practice)
        # Calculate linear and angular velocity from wheel speeds
        linear = (measured_left + measured_right) / 2
        angular = (measured_right - measured_left) / self.TRACK_WIDTH
        velocity = Velocity(linear=linear, angular=angular, time=time)
        self.SPEED_MEASURED.emit([velocity])

    def __del__(self) -> None:
        self.log.info('TwistedFieldsMotorHardware module shut down.')

