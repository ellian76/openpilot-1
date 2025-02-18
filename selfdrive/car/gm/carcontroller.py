from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.start_time = 0.
    self.apply_steer_last = 0
    self.apply_gas = 0
    self.apply_brake = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False

    self.params = CarControllerParams()

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, c, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params
    
    if enabled:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # Send CAN commands.
    can_sends = []

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = c.active and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # Pedal/Regen
    comma_pedal =0  #for supress linter error.
#    accelMultiplier = 0.475 #default initializer.
#    if CS.out.vEgo * CV.MS_TO_KPH < 10 :
#      accelMultiplier = 0.400
#    elif CS.out.vEgo * CV.MS_TO_KPH < 40 :
#      accelMultiplier = 0.475
#    else : # above 40 km/h
#      accelMultiplier = 0.425

    if not enabled or not CS.adaptive_Cruise or not CS.CP.enableGasInterceptor:
      comma_pedal = 0
    elif CS.adaptive_Cruise:
      gas_mult = interp(CS.out.vEgo, [0., 5.], [0.65, 1.0])
      comma_pedal = clip(gas_mult * (gas - brake), 0., 1.)
    
#      minimumPedalOutputBySpeed = interp(CS.out.vEgo, VEL, MIN_PEDAL)
#      pedal_accel = actuators.accel * 0.45
#      comma_pedal = clip(pedal_accel, minimumPedalOutputBySpeed, 1.)
#      comma_pedal, self.accel_steady = accel_hysteresis(comma_pedal, self.accel_steady)
            
      if brake > 0.1:
        can_sends.append(gmcan.create_regen_paddle_command(self.packer_pt, CanBus.POWERTRAIN))

    if (frame % 4) == 0:
      idx = (frame // 4) % 4
      can_sends.append(create_gas_interceptor_command(self.packer_pt, comma_pedal, idx))
      



    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake

    return new_actuators, can_sends
