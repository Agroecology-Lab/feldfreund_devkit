from rosys.module import Module, Timer
from rosys.driving import Velocity, Odometer, Drive, track_width_to_turn_radius
from rosys.event import Event
from rosys.helpers import sleep

class TwistedFieldsMotor(Module):
    # Constants
    TRACK_WIDTH = 0.5 # meters, assumed for skid-steer kinematics
    
    # Events
    SPEED_MEASURED = Event()
    MOTOR_ERROR = Event()
    
    # State
    velocity: Velocity = Velocity(linear=0.0, angular=0.0)
    left_speed: float = 0.0
    right_speed: float = 0.0
    error: bool = False
    
    def __init__(self, odometer: Odometer, drive: Drive) -> None:
        super().__init__()
        self.odometer = odometer
        self.drive = drive
        
        # Timer to periodically send motor commands
        self.timer = Timer(self._update_motor_commands, 0.05) # 20 Hz update rate
        
        self.log.info('TwistedFieldsMotor module initialized.')

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
            # turn_radius is the distance from the center of rotation to the center of the robot.
            # The wheel speed is proportional to the distance from the center of rotation.
            # V_left = linear - angular * (track_width / 2)
            # V_right = linear + angular * (track_width / 2)
            # Since track_width_to_turn_radius returns track_width / 2, the formula is correct.
            self.left_speed = linear - angular * turn_radius
            self.right_speed = linear + angular * turn_radius
        
        self.log.info(f'Driving with linear={linear:.2f} m/s, angular={angular:.2f} rad/s. Left={self.left_speed:.2f}, Right={self.right_speed:.2f}')
        
    async def _update_motor_commands(self) -> None:
        """
        Periodically sends the calculated wheel speeds to the Lizard firmware.
        This function is called by the internal Timer.
        
        NOTE: In a real system, this method would communicate the self.left_speed and 
        self.right_speed to the C++ Lizard module, which would then send the CAN messages.
        Since the communication mechanism is not provided, this remains a placeholder 
        but is now correctly named and structured.
        """
        if self.is_running:
            # Placeholder for actual communication logic to the C++ layer
            # Example: self.lizard_motor_module.set_speed(self.left_speed, self.right_speed)
            pass

    async def stop(self) -> None:
        """Stop the robot by setting all velocities to zero."""
        await self.set_target_velocity(0.0, 0.0)

    async def _update(self) -> None:
        """
        Main update loop for the module.
        In a real system, this would periodically read motor feedback from the Lizard firmware.
        """
        # The periodic command sending is handled by the Timer.
        # This loop can be used for reading feedback or other long-running tasks.
        while True:
            # Placeholder for reading motor feedback and updating odometer
            # Example: measured_left, measured_right = self.lizard_motor_module.get_measured_speeds()
            # self.odometer.increment(measured_left, measured_right, 0.1)
            await sleep(0.1)

    def __del__(self) -> None:
        self.log.info('TwistedFieldsMotor module shut down.')

"""
This module provides the interface to the Twisted Fields custom CAN motor controllers.
It translates high-level RoSys commands into low-level commands for the Lizard firmware.
"""
