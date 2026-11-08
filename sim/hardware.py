"""True physical parameters of the robot parts (SI units unless noted).

These describe the real world in the simulator. The robot program has its own
belief about them in firmware/include/robot_config.h.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class MotorSpec:
    """DC motor 1:48, 3-6 V, 200 rpm, double shaft (Botland 16016)."""
    rated_voltage: float = 6.0
    rated_rpm: float = 200.0
    supply_voltage: float = 6.0      # 4x AA
    driver_drop_v: float = 0.3       # TB6612FNG MOSFET drop at ~0.25 A
    deadband_pwm: int = 0            # PWM below which the motor stalls (0 = ideal)
    pwm_max: int = 255


@dataclass(frozen=True)
class ChassisSpec:
    wheel_diameter: float = 0.065
    wheel_width: float = 0.026
    track: float = 0.13              # wheel center to wheel center
    length: float = 0.15
    width: float = 0.12
    axle_from_front: float = 0.10    # rear-axle chassis with a front caster


@dataclass(frozen=True)
class ServoSpec:
    """SG90 micro servo."""
    speed_deg_s: float = 600.0       # 0.1 s / 60 deg @ 4.8 V
    min_deg: float = 0.0
    max_deg: float = 180.0
    initial_deg: float = 90.0
    axis_from_front: float = 0.02    # servo shaft behind the front edge
    sensor_offset: float = 0.015     # HC-SR04 transducers in front of the shaft


@dataclass(frozen=True)
class SonarSpec:
    """HC-SR04 (Botland 5686)."""
    min_range: float = 0.02
    max_range: float = 2.0           # range given by the seller
    beam_deg: float = 15.0           # full measuring angle from the datasheet
    rays: int = 9
    max_incidence_deg: float = 40.0  # steeper hits reflect away: no echo
    noise_abs: float = 0.002         # gaussian sigma [m]
    noise_rel: float = 0.003         # extra sigma per meter of distance
    temperature_c: float = 20.0
    trigger_overhead_us: int = 500   # trigger pulse + 8x40 kHz burst before echo goes high
    timeout_us: int = 38000          # echo line stays high this long without an echo
    min_cycle_ms: int = 60

    @property
    def speed_of_sound(self) -> float:
        return 331.3 + 0.606 * self.temperature_c


@dataclass(frozen=True)
class McuSpec:
    """Simulated time spent in every HAL call that does not wait by itself."""
    call_cost_us: int = 20


@dataclass(frozen=True)
class Hardware:
    motor: MotorSpec = field(default_factory=MotorSpec)
    chassis: ChassisSpec = field(default_factory=ChassisSpec)
    servo: ServoSpec = field(default_factory=ServoSpec)
    sonar: SonarSpec = field(default_factory=SonarSpec)
    mcu: McuSpec = field(default_factory=McuSpec)


PHYSICS_STEP_US = 1000
