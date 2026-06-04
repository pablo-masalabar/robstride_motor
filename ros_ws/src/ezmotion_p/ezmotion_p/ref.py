from .steppable_system import SteppableSystem
from .motors.motors_manager import MotorsManager
from .motors.motor import Motor
from feathersdk.utils.feathertypes import Optional
from enum import Enum
import threading
import canopen
import os
import time
from feathersdk.utils import constants
from feathersdk.utils.common import currtime, timediff, dont_allow_on_fork, FeatherThread
from feathersdk.robot.battery_system import PowerEvent, BatterySystem
from feathersdk.utils.logger import info, error, warning
from feathersdk.robot.instance_lock import check_still_own_lock
from feathersdk.utils.feathertypes import *
import logging

MAX_POSITION_JUMP_THRESHOLD = 10_000
STEPS_PER_ROTATION = 65536
DEFAULT_RPS = 100  # RPS = rotations per second
DEFAULT_ACCELERATION_RPS2 = 50
DEFAULT_DECELERATION_RPS2 = 12

ROTATIONS_PER_CM_TRAVELED = 2.54  # 1 cm traveled per 2.54 rotation
MM_IN_CM = 10

ZERO_VELOCITY_THRESHOLD = 1000
ZERO_VELOCITY_MIN_SEQUENCE = 10

def overwrite_file(filepath: str, data: list):
    """Overwrite file with comma-separated data.
    
    Args:
        filepath (str): Path to the file to overwrite
        data (list): List of strings to write as comma-separated values
    """
    with open(filepath, "w") as file:
        file.write(",".join(data))


def read_array_from_file(filepath: str):
    """Read comma-separated values from file.
    
    Args:
        filepath (str): Path to the file to read
    
    Returns:
        list: List of strings read from the file
    """
    with open(filepath, "r") as file:
        return file.read().split(",")

class EZMotionControlMode(Enum):
    """EZMotion motor control modes."""
    POSITION = 1
    VELOCITY = 3
    HOMING = 6

class EZMotionCommands(Enum):
    """EZMotion CANopen command addresses."""
    SET_MAX_HOMING_TORQUE = 0x2070
    SET_OPERATION_STATE = 0x6040
    SET_CONTROL_MODE = 0x6060
    SET_POSITION = 0x607A
    SET_VELOCITY = 0x60FF
    SET_MAX_VELOCITY = 0x6081
    SET_MAX_ACCELERATION = 0x6083
    SET_MAX_DECELERATION = 0x6084
    GET_STATUS_WORD = 0x6041
    GET_CURRENT_POSITION = 0x6064
    GET_CURRENT_TORQUE = 0x6077
    GET_CURRENT_VELOCITY = 0x606C
    GET_CURRENT_CURRENT = 0x6078
    GET_TEMPERATURE = 0x2040
    GET_ERROR_REGISTER = 0x200B
    MAX_TORQUE = 0x6072
    HOMING_METHOD = 0x6098
    HOMING_SPEED = 0x6099
    HOMING_OFFSET = 0x607C
    HOMING_ACCELERATION = 0x609A

class EZMotionOperationState(Enum):
    """EZMotion motor operation states."""
    SHUTDOWN = 0x0006
    ENABLED = 0x000F
    TRANSITION_TO_NEW_POSITION = 0x001F

class EZMotionHomingMethod(Enum):
    """EZMotion homing methods."""
    HOME_USING_MAX_TORQUE_UP = -0x03
    HOME_USING_MAX_TORQUE_DOWN = -0x02


class EZMotionMotor(Motor):
    """EZMotion motor representation."""
    def __init__(self, motor_id: int, can_interface: str, motor_name: str, config: dict):
        """Initialize EZMotion motor.
        
        Args:
            motor_id (int): CAN node ID of the motor
            can_interface (str): CAN interface name
            motor_name (str): Name identifier for the motor
            config (dict): Motor configuration with optional keys:
                - default_target_velocity_rps: Default target velocity in rotations per second
                - default_target_acceleration_rps2: Default acceleration in rotations per second squared
                - default_target_deceleration_rps2: Default deceleration in rotations per second squared
        """
        self.motor_id = motor_id
        self.motor_name = motor_name
        self.target_velocity = config.get("default_target_velocity_rps", DEFAULT_RPS)
        self.target_acceleration = config.get("default_target_acceleration_rps2", DEFAULT_ACCELERATION_RPS2)
        self.target_deceleration = config.get("default_target_deceleration_rps2", DEFAULT_DECELERATION_RPS2)
        self.iface = can_interface

class CanOpenConditionalCommand:

    def __init__(self, parent, command: int, value: int, subindex: Optional[int] = None, skip_condition: callable = None, on_success: callable = None):
        self.parent = parent
        self.command = command
        self.value = value
        self.subindex = subindex
        self.success = False
        self.skip_condition = skip_condition
        self.on_success = on_success
        self.start_time = None
        self.end_time = None
    
    def mark_success(self):
        self.success = True
        self.end_time = currtime()
        if self.on_success is not None:
            self.on_success()

    def should_skip(self):
        if self.skip_condition is not None and self.skip_condition():
            self.mark_success()
            return True
        return False

    def execute(self):
        if self.start_time is None:
            self.start_time = currtime()
        if self.should_skip():
            return True
        try:
            if self.subindex is None:
                self.parent.node.sdo[self.command].raw = self.value
            else:
                self.parent.node.sdo[self.command][self.subindex].raw = self.value
            self.mark_success()
            info("command succeeded", self.command, self.value, self.subindex, self.end_time - self.start_time)
            return True
        except Exception as e:
            if self.parent.print_packet_loss:
                print(
                    "_safe_command() exception - ",
                    type(e).__name__ + ": " + str(e) + ".",
                    "Command: ",
                    self.command,
                    "Value: ",
                    self.value,
                )
            return False

class CanOpenTransaction:

    def __init__(self, conditional_commands: List[CanOpenConditionalCommand]):
        self.conditional_commands = conditional_commands
        self.successfully_completed = False
        self.start_time = None
        self.end_time = None

    def execute_next(self):
        # peek at the next command, execute it, and if it succeeds, pop it off the list and return True
        # if it fails, return False
        if self.start_time is None:
            self.start_time = currtime()
        if len(self.conditional_commands) == 0:
            return True
        command = self.conditional_commands[0]
        if command.execute():
            self.conditional_commands.pop(0)
            if len(self.conditional_commands) == 0:
                self.end_time = currtime()
                info("Transaction execution time: ", self.end_time - self.start_time)
                self.successfully_completed = True
            return True
        return False

    def execute(self):
        while self.execute_next():
            pass
        return self.successfully_completed

    def execute_next_n(self, n: int):
        for _ in range(n):
            self.execute_next()
        return self.successfully_completed


class TorsoPlatform(SteppableSystem):
    """Base class for torso platform control."""

    def __init__(self):
        """Initialize torso platform."""
        super().__init__()
        self.operating_frequency = 30

    def set_movement_profile(self, target_velocity, target_acceleration, target_jerk):
        pass

    def go_to(self, height_from_bottom: float):
        pass

    def recalibrate(self):
        pass

    async def get_position(self):
        pass

    async def get_state(self):
        pass

    async def health_check(self):
        pass

class EZMotionTorsoPlatform(TorsoPlatform):
    """EZMotion-based torso platform implementation."""

    def __init__(self, motors_manager: MotorsManager, power: BatterySystem, can_interface: Optional[str], config: dict):
        """Initialize EZMotion torso platform.
        
        Args:
            motors_manager (MotorsManager): Manager for motor communication
            power (BatterySystem): Battery system for power event handling
            can_interface (Optional[str]): CAN interface name, or None to skip CAN initialization
            config (dict): Configuration dictionary with required "motors" key and optional keys:
                - top_height_mm: Maximum height in millimeters (default: 600)
                - bottom_height_mm: Minimum height in millimeters (default: 50)
                - homing_offset_from_top_num_rotations: Homing offset from top in rotations (default: 1)
                - homing_max_torque_percent: Maximum torque for homing in percent (default: 1000)
                - homing_acceleration: Homing acceleration in rotations per second squared (default: 50)
                - print_packet_loss: If True, print packet loss warnings (default: False)
                - show_all_sdo_errors: If True, show all SDO errors (default: True)
        
        Raises:
            Exception: If power cycle detected and system must be recalibrated
        """
        self.cfg = config
        self.is_forked = False
        self.motors_map = {}
        self.power = power
        self.top_height_mm = self.cfg.get("top_height_mm", 600)
        self.bottom_height_mm = self.cfg.get("bottom_height_mm", 50)
        self.homing_offset_from_top_num_rotations = self.cfg.get("homing_offset_from_top_num_rotations", 1)
        self.homing_max_torque_percent = self.cfg.get("homing_max_torque_percent", 1000)
        self.homing_acceleration_rps2 = self.cfg.get("homing_acceleration", 50)
        self.print_packet_loss = self.cfg.get("print_packet_loss", False)
        self.last_sent_velocity = 0
        self.last_sent_acceleration = 0
        self.last_sent_deceleration = 0
        self.last_sent_position = 0

        for motor_name, motor_dict in self.cfg["motors"].items():
            self.main_motor_id = int(motor_dict["id"])
            self.main_motor_model = motor_dict["model"]
            self.motors_map[motor_name] = EZMotionMotor(self.main_motor_id, can_interface, motor_name, motor_dict["motor_config"])
        self.calibration_file_path = constants.MOTORS_CALIBRATION_PATH + "/_ezmotion_TvC.json"
        self.motors_manager = motors_manager
        self.motors_manager.add_motors(self.motors_map, family_name="torso")
        self.health_check_future = None
        self.zero_velocity_count = 0
        self.reference_position = None
        # self.comms = CommsManager()
        self.network = canopen.Network()

        if can_interface is not None:
            self.network.connect(channel=can_interface, bustype="socketcan", bitrate=1000000)
            eds_file = os.path.join(os.path.dirname(__file__), "motors", self.main_motor_model + ".eds")
            self.node = self.network.add_node(self.main_motor_id, eds_file)
        else:
            self.node = None
        
        self.healthy_state = False
        # assigned_velocity was the last velocity the user requested.
        self.assigned_velocity = 0
        # last_registered_velocity is the velocity that was confirmed received by the motor.
        self.last_registered_velocity = (0, -1)
        self.last_step_time = currtime()

        self.current_position = None
        try:
            last_state = read_array_from_file(self.calibration_file_path)
            if len(last_state) != 3:
                raise Exception("Calibration file corrupted: expected 3 fields, got %d" % len(last_state))
            
            command, value, timestamp = last_state
            try:
                value, timestamp = float(value), float(timestamp)
            except Exception as e:
                raise Exception(f"Calibration file corrupted: value or timestamp is not a float: {value}, {timestamp}")

            if self.power is None:
                raise Exception("Error: Power system not initialized.")
            elif self.power.last_powered_up_time() > timestamp:
                raise Exception("Error: Power cycle detected. System must be recalibrated.")
            
            if command == "recalibrated":
                self.current_position = self.get_current_position_with_retry()
                self.curr_height = value
                self._set_reference_position(self.current_position, self.curr_height)
                #self.healthy_state = True
            if command == "finished_moving":
                self.current_position = self.get_current_position_with_retry()
                self.curr_height = value
                self._set_reference_position(self.current_position, self.curr_height)
                #self.healthy_state = True
        except Exception as e:
            error(f"Torso must be recalibrated: {str(e)}")

        self.operation_state = EZMotionOperationState.SHUTDOWN
        self.last_command = None
        self.init_ezmotion()
        self.moving = False
        self.control_mode = EZMotionControlMode.POSITION
        self.recalibrating = False
        self.aborted = False

        # Suppress all logging from canopen. TODO: Filter the messages to keep the ones we want to worry about.
        # No CRITICAL logging messages in canopen 2.4.1, so this should disable all sdo.client/server messages.
        sdo_level = logging.INFO if self.cfg.get("show_all_sdo_errors", True) else logging.CRITICAL
        logging.getLogger("canopen.sdo.client").setLevel(sdo_level)
        logging.getLogger("canopen.sdo.server").setLevel(sdo_level)

        self.height = (None, 0, 0, 0) # (height, timestamp, num_updates)
        self.velocity = (None, 0, 0, 0)
        self.torque = (None, 0, 0, 0)
        self.temperature = (None, 0, 0, 0)
        self.position = (None, 0, 0, 0)

        self.transaction_lock = threading.Lock()
        self.transaction = None
        self._canopen_stop = False
        self.canopen_thread = None

        super().__init__()

        if self.node is not None:
            self.canopen_thread = FeatherThread(target=self.canopen_thread_main, args=(self.node,))
            self.canopen_thread.start()


    @dont_allow_on_fork
    def canopen_thread_main(self, node: canopen.Node):
        try:
            next_clock_time = 0
            cnt = 0
            printed_at = -1
            while not self._canopen_stop:
                if cnt % 40 == 0 and printed_at != cnt:
                    printed_at = cnt
                if cnt % (self.operating_frequency / 20) == 0:
                    if not check_still_own_lock():
                        error("Torso: Lock not owned by this process. Skipping CANopen thread.")
                        break
                now = currtime()
                if now > next_clock_time:
                    try:
                        # For some reason, this has to be here to detect homing was successful, and we need to
                        # figure out why TODO:
                        status_word = self.get_status_word()
                        if self.transaction is not None:
                            self.transaction.execute_next_n(6)
                        position = self.get_current_position()
                        if position is not False:
                            self.position = (position, currtime(), self.position[2] + 1)
                            #if self.current_position is not None and \
                            #    abs(self.position[0] - self.current_position) > MAX_POSITION_JUMP_THRESHOLD and self.healthy_state:
                            #    error("Position jump detected, needs recalibration", self.position[0], self.current_position)
                            #     self.healthy_state = False
                            self.current_position = self.position[0]
                        if self.reference_position is not None:
                            height = self._compute_current_height(self.position[0])
                            if height is not False:
                                # Height's timestamp is the same as the position's timestamp.
                                self.height = (height, self.position[1], self.position[2])

                        velocity = self._safe_read(EZMotionCommands.GET_CURRENT_VELOCITY.value)
                        velocity_mm_per_second = velocity / 65536 / 2.54 * -10
                        # We ignore velocities greater than 200 mm/s to avoid extreme sensor noise coming from the motor.
                        if velocity is not False and abs(velocity_mm_per_second) < 200:
                            self.velocity = (velocity_mm_per_second, currtime(), self.velocity[2] + 1)
                        temp = self._safe_read(EZMotionCommands.GET_TEMPERATURE.value)
                        if temp is not False:
                            self.temperature = (temp, currtime(), self.temperature[2] + 1)
                        torque = self.get_current_torque()
                        if torque is not False:
                            self.torque = (torque, currtime(), self.torque[2] + 1)

                    except Exception as e:
                        error(f"Error in canopen thread: {e}")
                    next_clock_time = now + 1.0 / self.operating_frequency
                    cnt += 1
                time.sleep(0.001)
        except Exception as e:
            error(f"super-Error in canopen thread: {e}")
        finally:
            info("Torso: Exiting canopen_thread_main")
    
    @dont_allow_on_fork
    def _get_node(self):
        """Get CANopen node.
        
        Returns:
            canopen.Node: The CANopen node instance
        
        Raises:
            ValueError: If node is not initialized
        """
        if self.node is None:
            raise ValueError("Node is not initialized")
        return self.node

    @dont_allow_on_fork
    def init_ezmotion(self):
        """Initialize EZMotion motor to position control mode."""
        if self.node is None:
            error("Node is not initialized, skipping EZMotion initialization")
            return
        self.shutdown()
        self.set_control_mode(EZMotionControlMode.POSITION)
        # self.enable()
        # self.set_movement_profile("TvC", DEFAULT_RPS, DEFAULT_ACCELERATION_RPS2, DEFAULT_DECELERATION_RPS2)

    @dont_allow_on_fork
    def send_movement_profile(self, target_velocity=None, target_acceleration=None, target_deceleration=None):
        """Send movement profile parameters to motor (only if changed).
        
        Args:
            target_velocity (float, optional): Target velocity in rotations per second
            target_acceleration (float, optional): Target acceleration in rotations per second squared
            target_deceleration (float, optional): Target deceleration in rotations per second squared
        """
        if target_velocity is not None:
            if target_velocity != self.last_sent_velocity:
                if self._ensure_command(EZMotionCommands.SET_MAX_VELOCITY.value, target_velocity * STEPS_PER_ROTATION): 
                    self.last_sent_velocity = target_velocity
        if target_acceleration is not None:
            if target_acceleration != self.last_sent_acceleration:
                if self._ensure_command(EZMotionCommands.SET_MAX_ACCELERATION.value, target_acceleration * STEPS_PER_ROTATION):
                    self.last_sent_acceleration = target_acceleration
        if target_deceleration is not None:
            if target_deceleration != self.last_sent_deceleration:
                if self._ensure_command(EZMotionCommands.SET_MAX_DECELERATION.value, target_deceleration * STEPS_PER_ROTATION):
                    self.last_sent_deceleration = target_deceleration

    @dont_allow_on_fork
    def set_movement_profile(self, target_velocity, target_acceleration, target_deceleration):
        """Set movement profile parameters for the motor.
        
        Args:
            target_velocity (float, optional): Target velocity in rotations per second
            target_acceleration (float, optional): Target acceleration in rotations per second squared
            target_deceleration (float, optional): Target deceleration in rotations per second squared
        """
        motor = self.motors_map["TvC"]
        if target_velocity is not None:
            motor.target_velocity = target_velocity
        if target_acceleration is not None:
            motor.target_acceleration = target_acceleration
        if target_deceleration is not None:
            motor.target_deceleration = target_deceleration

    @dont_allow_on_fork
    def _step(self):
        """Private method to monitor movement and handle target reached events."""
        try:
            self.last_step_time = currtime()
            if self.moving:
                status_word = self.get_status_word()
                if status_word & (1 << 10):
                    info("target reached")
                    if self.recalibrating:
                        self.shutdown()
                        self._on_recalibrated()
                    elif self.control_mode == EZMotionControlMode.POSITION:
                        self._on_target_reached()
                else:
                    # 3 reads
                    current_torque = self.torque[0] # if self.torque is not None else self.get_current_torque()
                    if current_torque > 1000 or current_torque < -1000:
                        info(f"calibration triggered {current_torque}")
                        self.shutdown()
                        if self.recalibrating:
                            self._on_recalibrated()
                    current_velocity = self.velocity[0] # if self.velocity is not None else self.get_current_velocity()
                    self.current_position = self.position[0] #if self.position is not None else self.get_current_position()
                    if current_velocity is not None and abs(current_velocity) < ZERO_VELOCITY_THRESHOLD:
                        self.zero_velocity_count += 1
                    else:
                        self.zero_velocity_count = 0
                    if self.zero_velocity_count > ZERO_VELOCITY_MIN_SEQUENCE and (self.last_command is not None and self.last_command[0] != "recalibrate"):
                        self.moving = False
                        self._on_target_reached()
        except Exception as e:
            import traceback
            error("Error in _step: ", traceback.format_exc())

    @dont_allow_on_fork
    def _ensure_command(self, command: int, value: int, subindex: Optional[int] = None, retries: int = 5):
        """Send command with retry logic.
        
        Args:
            command (int): CANopen command address
            value (int): Value to write
            subindex (Optional[int]): Subindex for the command
            retries (int): Number of retry attempts (default: 5)
        
        Returns:
            bool: True if command succeeded, False otherwise
        """
        for i in range(retries):
            if self._safe_command(command, value, subindex):
                return True
        return False

    @dont_allow_on_fork
    def _safe_command(self, command: int, value: int, subindex: Optional[int] = None):
        """Send command to motor with error handling.
        
        Args:
            command (int): CANopen command address
            value (int): Value to write
            subindex (Optional[int]): Subindex for the command
        
        Returns:
            bool: True if command succeeded, False otherwise
        """
        try:
            if subindex is None:
                self.node.sdo[command].raw = value
            else:
                self.node.sdo[command][subindex].raw = value
            return True
        except Exception as e:
            if self.print_packet_loss:
                print(
                    "_safe_command() exception - ",
                    type(e).__name__ + ": " + str(e) + ".",
                    "Command: ",
                    command,
                    "Value: ",
                    value,
                )
            return False

    @dont_allow_on_fork
    def _safe_read(self, command: int, subindex: Optional[int] = None):
        """Read value from motor with error handling.
        
        Args:
            command (int): CANopen command address to read from
        
        Returns:
            int | bool: Read value, or False if read failed
        """
        try:
            if subindex is None:
                return self._get_node().sdo[command].raw
            else:
                return self._get_node().sdo[command][subindex].raw
        except Exception as e:
            return False
    
    @dont_allow_on_fork
    def has_error(self):
        """Check if motor has error."""
        return self._safe_read(EZMotionCommands.GET_ERROR_REGISTER.value, 0x0B) != 0
    
    @dont_allow_on_fork
    def clear_errors(self):
        """Clear motor errors."""
        self._safe_command(EZMotionCommands.SET_OPERATION_STATE.value, 128)

    @dont_allow_on_fork
    def _is_position_control_mode(self):
        return self.control_mode == EZMotionControlMode.POSITION
    
    @dont_allow_on_fork
    def _is_enabled(self):
        return self.operation_state == EZMotionOperationState.ENABLED

    @dont_allow_on_fork
    def _record_control_mode(self, control_mode: EZMotionControlMode):
        self.control_mode = control_mode
    
    @dont_allow_on_fork
    def _record_operation_state(self, operation_state: EZMotionOperationState):
        self.operation_state = operation_state

    @dont_allow_on_fork
    def _record_velocity(self, velocity: float):
        self.last_sent_velocity = velocity

    @dont_allow_on_fork
    def _record_acceleration(self, acceleration: float):
        self.last_sent_acceleration = acceleration

    @dont_allow_on_fork
    def _record_deceleration(self, deceleration: float):
        self.last_sent_deceleration = deceleration

    @dont_allow_on_fork
    def _record_last_sent_position(self, position: int):
        self.last_sent_position = position
    
    @dont_allow_on_fork
    def _record_recalibrating(self, recalibrating: bool):
        self.recalibrating = recalibrating
        info(f"started record recalibrating: {recalibrating}")
        overwrite_file(self.calibration_file_path, ["recalibrating", str(0), str(time.time())])

    @dont_allow_on_fork
    def _record_moving(self, moving: bool):
        self.moving = moving

    @dont_allow_on_fork
    def _go_to_raw_position(self, position: int, velocity=None):
        """Move to raw position in motor steps.
        
        Args:
            position (int): Target position in motor steps
            velocity (float, optional): Velocity in rotations per second. If None, uses motor's default velocity.
        """
        self.commands = [
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_CONTROL_MODE.value, 
                EZMotionControlMode.POSITION.value, 
                skip_condition=self._is_position_control_mode,
                on_success=lambda: self._record_control_mode(EZMotionControlMode.POSITION)),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_OPERATION_STATE.value, 
                EZMotionOperationState.ENABLED.value, 
                skip_condition=self._is_enabled,
                on_success=lambda: self._record_operation_state(EZMotionOperationState.ENABLED)),
        ]
        if velocity is not None:
            self.commands.append(CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_MAX_VELOCITY.value, 
                velocity * STEPS_PER_ROTATION,
                skip_condition=lambda: velocity == self.last_sent_velocity,
                on_success=lambda: self._record_velocity(velocity)))
        else:
            motor = self.motors_map["TvC"]
            self.commands.append(CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_MAX_VELOCITY.value, 
                motor.target_velocity * STEPS_PER_ROTATION,
                skip_condition=lambda: motor.target_velocity == self.last_sent_velocity,
                on_success=lambda: self._record_velocity(motor.target_velocity)))
            self.commands.append(CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_MAX_ACCELERATION.value, 
                motor.target_acceleration * STEPS_PER_ROTATION,
                skip_condition=lambda: motor.target_acceleration == self.last_sent_acceleration,
                on_success=lambda: self._record_acceleration(motor.target_acceleration)))
            self.commands.append(CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_MAX_DECELERATION.value, 
                motor.target_deceleration * STEPS_PER_ROTATION,
                skip_condition=lambda: motor.target_deceleration == self.last_sent_deceleration,
                on_success=lambda: self._record_deceleration(motor.target_deceleration)))
       
        self.commands.append(CanOpenConditionalCommand(
            self,
            EZMotionCommands.SET_POSITION.value, 
            position,
            skip_condition=lambda: self.last_sent_position == position,
            on_success=lambda: self._record_last_sent_position(position)))

        self.commands.append(CanOpenConditionalCommand(
            self,
            EZMotionCommands.SET_OPERATION_STATE.value, 
            EZMotionOperationState.ENABLED.value,
            on_success=lambda: self._record_operation_state(EZMotionOperationState.ENABLED)))
        self.commands.append(CanOpenConditionalCommand(
            self,
            EZMotionCommands.SET_OPERATION_STATE.value, 
            EZMotionOperationState.TRANSITION_TO_NEW_POSITION.value,
            on_success=lambda: (self._record_operation_state(EZMotionOperationState.TRANSITION_TO_NEW_POSITION), self._record_moving(True))))
        self.transaction = CanOpenTransaction(self.commands)

    @dont_allow_on_fork
    def _compute_current_height(self, current_position: int = None):
        """Compute current height from reference position.
        
        Returns:
            float: Current height in millimeters
        """
        if current_position is None:
            self.current_position = self.get_current_position()
        num_rotations_offset = (
            self.current_position - self.reference_position["associated_position"]
        ) / STEPS_PER_ROTATION
        return (
            self.reference_position["associated_height"] - num_rotations_offset / ROTATIONS_PER_CM_TRAVELED * MM_IN_CM
        )

    @dont_allow_on_fork
    def _on_target_reached(self):
        """Handle target reached event and save state."""
        self.current_position = self.get_current_position_with_retry()
        num_rotations_offset = (
            self.current_position - self.reference_position["associated_position"]
        ) / STEPS_PER_ROTATION
        self.curr_height = (
            self.reference_position["associated_height"] - num_rotations_offset / ROTATIONS_PER_CM_TRAVELED * MM_IN_CM
        )
        overwrite_file(self.calibration_file_path, ["finished_moving", str(self.curr_height), str(time.time())])
        self.shutdown()

    @dont_allow_on_fork
    def _set_reference_position(self, current_position: int, current_height: float):
        """Set reference position for height calculations.
        
        Args:
            current_position (int): Current motor position in steps
            current_height (float): Current height in millimeters
        """
        self.reference_position = {"associated_position": current_position, "associated_height": current_height}
        self.max_height_target_position = self.calc_max_height_target_position(current_height, current_position)
        self.min_height_target_position = self.calc_min_height_target_position(current_height, current_position)
        info("max min target positions", self.max_height_target_position, self.min_height_target_position)

    @dont_allow_on_fork
    def _on_recalibrated(self):
        """Handle recalibration completion and save state."""
        info("on_recalibrated torso")
        self.current_position = self.get_current_position_with_retry()
        # self.min_position = current_position + (NUM_ROTATIONS_OFFSET_FROM_TOP * STEPS_PER_ROTATION)
        self.curr_height = self.top_height_mm
        self._set_reference_position(self.current_position, self.curr_height)
        overwrite_file(self.calibration_file_path, ["recalibrated", str(self.curr_height), str(time.time())])
        # Motor's target position should now be the current position.
        self.healthy_state = True
        self.recalibrating = False
        info("Torso calibration successful")

    @dont_allow_on_fork
    def wait_for_recalibration(self):
        """Wait for recalibration to complete."""
        while self.recalibrating and not self.aborted and not self.is_forked:
            time.sleep(0.1)

    @dont_allow_on_fork
    def recalibrate(self):
        """Recalibrate torso by homing to top position."""
        if self.node is None:
            raise ValueError("Node is not initialized; cannot recalibrate torso.")
        self.last_command = ["recalibrate", currtime()]
        # Mark recalibration as in-progress immediately so callers that invoke
        # wait_for_recalibration() right after this will actually block.
        self._record_recalibrating(True)
        commands = [
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_OPERATION_STATE.value,
                EZMotionOperationState.ENABLED.value,
                skip_condition=lambda: self.operation_state == EZMotionOperationState.ENABLED,
                on_success=lambda: self._record_operation_state(EZMotionOperationState.ENABLED)),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_OPERATION_STATE.value,
                EZMotionOperationState.SHUTDOWN.value,
                skip_condition=lambda: self.operation_state == EZMotionOperationState.SHUTDOWN,
                on_success=lambda: self._record_operation_state(EZMotionOperationState.SHUTDOWN)),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_CONTROL_MODE.value,
                EZMotionControlMode.HOMING.value,
                skip_condition=lambda: self.control_mode == EZMotionControlMode.HOMING,
                on_success=lambda: self._record_control_mode(EZMotionControlMode.HOMING)),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_OPERATION_STATE.value,
                EZMotionOperationState.ENABLED.value,
                skip_condition=lambda: self.operation_state == EZMotionOperationState.ENABLED,
                on_success=lambda: self._record_operation_state(EZMotionOperationState.ENABLED)),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.HOMING_METHOD.value,
                EZMotionHomingMethod.HOME_USING_MAX_TORQUE_UP.value),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_MAX_HOMING_TORQUE.value,
                self.homing_max_torque_percent,
                subindex=0x01),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.HOMING_SPEED.value,
                STEPS_PER_ROTATION,
                subindex=0x01),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.HOMING_SPEED.value,
                STEPS_PER_ROTATION,
                subindex=0x02),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.HOMING_ACCELERATION.value,
                self.homing_acceleration_rps2 * STEPS_PER_ROTATION),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.HOMING_OFFSET.value,
                self.homing_offset_from_top_num_rotations * STEPS_PER_ROTATION),
            CanOpenConditionalCommand(
                self,
                EZMotionCommands.SET_OPERATION_STATE.value,
                EZMotionOperationState.TRANSITION_TO_NEW_POSITION.value,
                on_success=lambda: self._record_moving(True)),
        ]
        self.transaction = CanOpenTransaction(commands)
        info("recalibrating torso")

    @dont_allow_on_fork
    def _go_to_raw_position_safely(self, position: int):
        """Move to raw position using safe low-speed profile.
        
        Args:
            position (int): Target position in motor steps
        """
        if self.control_mode != EZMotionControlMode.POSITION:
            self.set_control_mode(EZMotionControlMode.POSITION)
        self.enable()
        self._safe_command(EZMotionCommands.SET_POSITION.value, position)
        motor = self.motors_map["TvC"]
        self.send_movement_profile(5, 5, 5)
        self.transition_to_new_position()

    @dont_allow_on_fork
    def enable_velocity_mode(self):
        """Enable velocity control mode and set velocity to zero."""
        self.wait_for_recalibration()
        self.enable()
        if self._safe_command(EZMotionCommands.SET_VELOCITY.value, 0 * STEPS_PER_ROTATION):
            self.last_registered_velocity = (0, currtime())
        motor = self.motors_map["TvC"]
        self.send_movement_profile(motor.target_velocity, motor.target_acceleration, motor.target_deceleration)
        self.transition_to_new_position()

    @dont_allow_on_fork
    def _get_max_safe_height(self):
        """Get maximum safe height position accounting for spring compression.
        
        Returns:
            int: Maximum safe position in motor steps
        """
        return self.max_height_target_position + (STEPS_PER_ROTATION * 2)

    @dont_allow_on_fork
    def _get_min_safe_height(self):
        """Get minimum safe height position accounting for spring expansion.
        
        Returns:
            int: Minimum safe position in motor steps
        """
        return self.min_height_target_position - (STEPS_PER_ROTATION * 4)

    @dont_allow_on_fork
    def _is_at_max_height(self):
        """Check if at maximum safe height.
        
        Returns:
            bool: True if at or above maximum safe height
        """
        return self.current_position <= self._get_max_safe_height()

    @dont_allow_on_fork
    def _is_at_min_height(self):
        """Check if at minimum safe height.
        
        Returns:
            bool: True if at or below minimum safe height
        """
        return self.current_position >= self._get_min_safe_height()

    @dont_allow_on_fork
    def update_velocity(self, mm_per_second: float, skip_overwrite: bool = False):
        """Update torso velocity in velocity control mode.
        
        Args:
            mm_per_second (float): Velocity in millimeters per second, range -200 to 200
            skip_overwrite (bool): If True, skip overwriting calibration file
        
        Raises:
            Exception: If system not healthy or velocity exceeds maximum
        """
        if not self.healthy_state:
            raise Exception("Error: System must be recalibrated.")
        if mm_per_second > 200 or mm_per_second < -200:
            raise Exception("Error: Velocity is too high. Maximum velocity is 200 mm/s.")
        self.assigned_velocity = mm_per_second
        if self.current_position is not False:
            if (self._is_at_max_height() and mm_per_second > 0) or (self._is_at_min_height() and mm_per_second < 0):
                return
        rotations_per_second = mm_per_second / MM_IN_CM * ROTATIONS_PER_CM_TRAVELED
        # TODO: Currently, if we overflow the CAN line with too many commands, reading the position feedback will be delayed for the clips.
        target_velocity = -1 * rotations_per_second * STEPS_PER_ROTATION
        if self.last_command is None or self.last_command[0] != "update_velocity" or self.last_command[1] != target_velocity:
            self.last_command = ["update_velocity", target_velocity]
            if not skip_overwrite:
                overwrite_file(self.calibration_file_path, ["update_velocity", str(target_velocity), str(time.time())])
            if mm_per_second > 0:
                self._go_to_raw_position(
                    self.max_height_target_position, rotations_per_second
                )
            elif mm_per_second < 0:
                self._go_to_raw_position(
                    self.min_height_target_position, -1 * rotations_per_second
                )
            else:
                self._go_to_raw_position(self.current_position, 0)
    
    @dont_allow_on_fork
    def update_relative(self, delta_mm: float, velocity: Optional[float] = None):
        """Update position by delta_mm relative to current position. Will clamp final position to be within valid range.
        
        Args:
            delta_mm (float): Delta height in millimeters from current position
            velocity (float): Velocity in millimeters per second [0, 200] mm/s, or None to use motor default
        """
        if not self.healthy_state or self.height is None or self.height[0] is None:
            raise Exception("Error: System must be recalibrated.")

        if velocity is not None:
            if velocity < 0 or velocity > 200:
                raise Exception("Error: Velocity is out of range. Must be between 0 and 200 mm/s.")
            self.target_velocity = velocity / MM_IN_CM * ROTATIONS_PER_CM_TRAVELED * (-1 if delta_mm < 0 else 1)
        
        height = max(self.bottom_height_mm, min(self.height[0] + delta_mm, self.top_height_mm))
        #info(f"Updating relative position delta_mm: {delta_mm}, velocity: {velocity}, current height: {self.height[0]}, target height: {height}, target velocity: {self.target_velocity}")
        self.go_to(height)

    @dont_allow_on_fork
    def go_to(self, height_from_bottom: float):
        """Move to target height from bottom.
        
        Args:
            height_from_bottom (float): Target height in millimeters from bottom
        
        Raises:
            Exception: If system not healthy or height is outside valid range
        """
        if not self.healthy_state:
            raise Exception("Error: System must be recalibrated.")
        if height_from_bottom > self.top_height_mm:
            raise Exception(f"Error: Height from bottom {height_from_bottom}mm is greater than maximum supported height {self.top_height_mm}mm.")
        if height_from_bottom < self.bottom_height_mm:
            raise Exception(f"Error: Height from bottom {height_from_bottom}mm is less than minimum supported height {self.bottom_height_mm}mm.")

        height_offset = height_from_bottom - self.reference_position["associated_height"]
        change_in_position = height_offset / MM_IN_CM * ROTATIONS_PER_CM_TRAVELED * STEPS_PER_ROTATION
        target_position = self.reference_position["associated_position"] - change_in_position
        if self.last_command is None or self.last_command[0] != "go_to" or self.last_command[1] != target_position:
            self.last_command = ["go_to", target_position]
            overwrite_file(self.calibration_file_path, ["go_to", str(target_position), str(time.time())])
            self._go_to_raw_position(target_position)

    @dont_allow_on_fork
    def calc_max_height_target_position(self, associated_height: float, associated_position: int):
        """Calculate maximum height target position.
        
        Args:
            associated_height (float): Reference height in millimeters
            associated_position (int): Reference position in motor steps
        
        Returns:
            int: Maximum height target position in motor steps
        """
        height_offset = self.top_height_mm - associated_height
        change_in_position = height_offset / MM_IN_CM * ROTATIONS_PER_CM_TRAVELED * STEPS_PER_ROTATION
        return associated_position - change_in_position

    @dont_allow_on_fork
    def calc_min_height_target_position(self, associated_height: float, associated_position: int):
        """Calculate minimum height target position.
        
        Args:
            associated_height (float): Reference height in millimeters
            associated_position (int): Reference position in motor steps
        
        Returns:
            int: Minimum height target position in motor steps
        """
        height_offset = self.bottom_height_mm - associated_height
        change_in_position = height_offset / MM_IN_CM * ROTATIONS_PER_CM_TRAVELED * STEPS_PER_ROTATION
        return associated_position - change_in_position

    @dont_allow_on_fork
    def shutdown(self):
        """Shutdown motor and stop movement."""
        self.moving = False
        if self._is_enabled():
            if self._ensure_command(
                EZMotionCommands.SET_OPERATION_STATE.value, EZMotionOperationState.SHUTDOWN.value
            ):
                self._record_operation_state(EZMotionOperationState.SHUTDOWN)

    def enable(self):
        """Enable motor operation."""
        if not self._is_enabled():
            if self._ensure_command(
                EZMotionCommands.SET_OPERATION_STATE.value, EZMotionOperationState.ENABLED.value
            ):
                self._record_operation_state(EZMotionOperationState.ENABLED)

    @dont_allow_on_fork
    def transition_to_new_position(self):
        """Transition motor to new position state."""
        set_operation_state = self._ensure_command(EZMotionCommands.SET_OPERATION_STATE.value, EZMotionOperationState.ENABLED.value)
        if not set_operation_state:
            return False
        self.moving = self._ensure_command(
            EZMotionCommands.SET_OPERATION_STATE.value, EZMotionOperationState.TRANSITION_TO_NEW_POSITION.value
        )
        return self.moving

    @dont_allow_on_fork
    def set_control_mode(self, control_mode: EZMotionControlMode):
        """Set motor control mode.
        
        Args:
            control_mode (EZMotionControlMode): Control mode to set
        """
        if self._ensure_command(EZMotionCommands.SET_CONTROL_MODE.value, control_mode.value):
            self.control_mode = control_mode
            return True
        return False

    @dont_allow_on_fork
    def get_status_word(self):
        """Get motor status word.
        
        Returns:
            int | bool: Status word value, or False if read failed
        """
        return self._safe_read(EZMotionCommands.GET_STATUS_WORD.value)

    @dont_allow_on_fork
    def get_current_position(self):
        """Get current motor position.
        
        Returns:
            int | bool: Current position in motor steps, or False if read failed
        """
        return self._safe_read(EZMotionCommands.GET_CURRENT_POSITION.value)

    @dont_allow_on_fork
    def get_current_velocity(self):
        """Get current motor velocity.
        
        Returns:
            int | bool: Current velocity in steps per second, or False if read failed
        """
        return self._safe_read(EZMotionCommands.GET_CURRENT_VELOCITY.value)

    @dont_allow_on_fork
    def health_check(self):
        """Perform health check on motor.
        
        Returns:
            dict: Dictionary with keys: temperature, current, position
        """
        return {
            "temperature": self._safe_read(EZMotionCommands.GET_TEMPERATURE.value),
            "current": self._safe_read(EZMotionCommands.GET_CURRENT_CURRENT.value),
            "position": self.get_current_position(),
        }

    @dont_allow_on_fork
    def get_state(self):
        """Get current motor state.
        
        Returns:
            dict: Dictionary with keys: temperature, velocity, torque, position, height
        """
        return {
            "temperature": self.temperature[0],
            "velocity": self.velocity[0],
            "torque": self.torque[0],
            "position": self.position[0],
            "height": self.height[0]
        }

    @dont_allow_on_fork
    def get_current_position_with_retry(self):
        """Get current position with retry logic.
        
        Returns:
            int: Current position in motor steps
        
        Raises:
            Exception: If position cannot be read after retries
        """
        for i in range(25):
            current_position = self.get_current_position()  # Ensure the position is read
            if current_position is not False:
                return current_position
            time.sleep(0.01)
        raise Exception("Error: Failed to get essential current position.")

    @dont_allow_on_fork
    def get_current_torque(self):
        """Get current motor torque.
        
        Returns:
            int | bool: Current torque value, or False if read failed
        """
        return self._safe_read(EZMotionCommands.GET_CURRENT_TORQUE.value)

    @dont_allow_on_fork
    def on_abort(self):
        warning("TorsoPlatform on_abort")
        """Handle system abort event."""
        if self.aborted:
            return
        self.aborted = True
        if not hasattr(self, "current_position"):
            return
        try:
            self.update_velocity(0, skip_overwrite=True)
        except Exception as e:
            warning(f"Skipping update_velocity: {e}")
        self.shutdown()
        self.network.disconnect()
        self._canopen_stop = True
        if self.canopen_thread is not None:
            self.canopen_thread.join(timeout=1.0)

    @dont_allow_on_fork
    def on_power_event(self, event: PowerEvent):
        """Handle power system events.
        
        Args:
            event (PowerEvent): The power event type (POWER_OFF or POWER_RESTORED)
        """
        if event == PowerEvent.POWER_OFF:
            self.healthy_state = False
        elif event == PowerEvent.POWER_RESTORED:
            self.healthy_state = False
    
    def on_fork(self):
        self.is_forked = True
        self.aborted = True
        self.cfg = {}
        self.motors_map = {}
        self.power = None
        self.print_packet_loss = False
        self.last_sent_velocity = 0
        self.last_sent_acceleration = 0
        self.last_sent_deceleration = 0
        self.last_sent_position = 0
        self.main_motor_id = None
        self.main_motor_model = None
        self.motors_map = {}
        self.calibration_file_path = None
        self.motors_manager = None
        self.health_check_future = None
        self.zero_velocity_count = 0
        self.reference_position = None
        self.network = None
        self.node = None
        self.current_position = None
        self.healthy_state = False
        self.assigned_velocity = 0
        self.last_registered_velocity = (0, -1)
        self.last_step_time = currtime()
        self.operation_state = EZMotionOperationState.SHUTDOWN
        self.last_command = None
        self.moving = False
        self.control_mode = EZMotionControlMode.POSITION
        self.recalibrating = False
        self.height = (None, 0, 0, 0) # (height, timestamp, num_updates)
        self.velocity = (None, 0, 0, 0)
        self.torque = (None, 0, 0, 0)
        self.temperature = (None, 0, 0, 0)
        self.position = (None, 0, 0, 0)
