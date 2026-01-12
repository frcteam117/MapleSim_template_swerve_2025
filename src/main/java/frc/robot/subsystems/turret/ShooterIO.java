package frc.robot.subsystems.turret;

import frc.robot.util.logging.LogUtil.MotorState;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public MotorState flywheel;
    public MotorState hood;
    public MotorState turret;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setFlywheelVoltage(double V) {}

  public default void setHoodVoltage(double V) {}

  public default void setTurretVoltage(double V) {}

  public default void setNextFlywheelState(double nextV_radPs) {}

  public default void setNextHoodState(double nextP_rad, double nextV_radPs) {}

  public default void setNextTurretState(double nextP_rad, double nextV_radPs) {}
}
