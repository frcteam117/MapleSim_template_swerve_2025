package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double mechanismPosition_rad = 0;
    public double mechanismVelocity_radPs = 0;
    public double motorVoltage_V = 0;
    public double statorCurrent_A = 0;
    public double supplyCurrent_A = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Sets the voltage applied to the turret's motor without feedback.
   *
   * @param voltage_V the voltage applied to the motor
   */
  public default void setVoltage(double voltage_V) {}

  /**
   * Sets the voltage based on a feedforard value added to a velocity pid.
   *
   * @param nextVelocity_radPs the target velocity of the elevator in the next timestep
   */
  public default void setNextVelocity(double nextVelocity_radPs) {}
}
