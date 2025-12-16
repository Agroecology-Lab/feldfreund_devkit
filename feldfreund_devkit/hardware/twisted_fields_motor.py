from rosys.driving import Velocity, Odometer, track_width_to_turn_radius
from rosys.event import Event
from rosys.helpers import remove_indentation
from rosys.hardware import ModuleHardware, RobotBrain, Wheels
from rosys.module import Timer
from typing import override


class TwistedFieldsMotorConfiguration:
    """Configuration for the Twisted Fields Motor Hardware."""
    def __init__(self, track_width: float = 0.5) -> None:
        self.track_width = track_width


class TwistedFieldsMotorHardware(Wheels, ModuleHardware):
    """
    Provides the interface to the Twisted Fields custom CAN motor controllers.
    
    This module acts as a high-level controller, translating RoSys linear/angular 
    velocity commands into two individual wheel speeds (left/right) using skid-steer 
    kinematics. It then sends these speeds periodically to the Lizard firmware 
    via the RobotBrain, which is assumed to handle the low-level CAN communication.
    It also processes feedback from the Lizard core to update the Odometer.
    """
    
    # Events
    # NOTE: The standard Wheels module emits VELOCITY_MEASURED. We keep SPEED_MEASURED 
    # for backward compatibility with the original draft's intent, but it's redundant.
    SPEED_MEASURED = Event()
    MOTOR_ERROR = Event()
    
    # State
    left_speed: float = 0.0
    right_speed: float = 0.0
    error: bool = False
    
    def __init__(self, 
                 config: TwistedFieldsMotorConfiguration,
                 robot_brain: RobotBrain, 
                 odometer: Odometer, 
                 name: str = 'twisted_fields_motor') -> None:
        
        # 1. Lizard Code Definition
        lizard_code = remove_indentation(f'''
            {name} = TwistedFieldsMotor()
        ''')
        
        # 2. Core Message Fields for Feedback
        # The C++ module is expected to report: measured_left, measured_right, error_flag
        core_message_fields = [f'{name}.measured_left:3', f'{name}.measured_right:3', f'{name}.error_flag']
        
        # Initialize ModuleHardware first
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)
        
        # Initialize Wheels properties
        self.width = config.track_width # Set the Wheels width property for consistency
        
        self.config = config
        self.name = name
        self.odometer = odometer
        
        # 3. Periodic Command Timer (20 Hz)
        self.timer = Timer(self._update_motor_commands, 0.05) 
        
        self.log.info('TwistedFieldsMotorHardware module initialized.')

    @override
    async def drive(self, linear: float, angular: float) -> None:
        """
        Set the desired linear and angular velocity.
        This method is the standard RoSys Wheels interface method.
        """
        # Skid-steer kinematics calculation
        # Use self.width (inherited from Wheels) which is set from config.track_width
        turn_radius = track_width_to_turn_radius(self.width)
        
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
        This function is called by the internal Timer (20 Hz).
        """
        if self.is_running and self.robot_brain.is_ready:
            # Send the two calculated speeds to the Lizard C++ module
            # Assumes the C++ module implements a 'set_speeds(left_speed, right_speed)' command.
            await self.robot_brain.send(f'{self.name}.set_speeds({self.left_speed:.3f}, {self.right_speed:.3f})')
            
    @override
    async def stop(self) -> None:
        """Stop the robot by setting all velocities to zero."""
        await self.drive(0.0, 0.0)

    def handle_core_output(self, time: float, words: list[str]) -> None:
        """
        Handles the core message output from the Lizard firmware.
        Expected format: [measured_left, measured_right, error_flag]
        """
        if len(words) < 3:
            self.log.warning(f'Received incomplete core output: {words}')
            return

        try:
            # 1. Parse Feedback
            measured_left = float(words.pop(0))
            measured_right = float(words.pop(0))
            error_flag = int(words.pop(0))
            
            # 2. Update Odometer
            # Use a placeholder dt=0.1, assuming this is the core message frequency.
            dt = 0.1 
            self.odometer.increment(measured_left, measured_right, dt)
            
            # 3. Update Error State
            new_error_state = error_flag == 1
            if new_error_state and not self.error:
                self.MOTOR_ERROR.emit()
                self.log.error('Twisted Fields Motor Error detected.')
            self.error = new_error_state

            # 4. Emit measured velocity
            # Calculate linear and angular velocity from wheel speeds
            linear = (measured_left + measured_right) / 2
            angular = (measured_right - measured_left) / self.width # Use self.width
            velocity = Velocity(linear=linear, angular=angular, time=time)
            self.SPEED_MEASURED.emit([velocity])
            self.VELOCITY_MEASURED.emit([velocity]) # Emit standard Wheels event
            
        except ValueError as e:
            self.log.error(f'Failed to parse core output values: {e} with words: {words}')

    def __del__(self) -> None:
        self.log.info('TwistedFieldsMotorHardware module shut down.')

