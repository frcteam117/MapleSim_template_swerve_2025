package frc.robot.util.control.profiling;

import frc.robot.util.control.Setpoints.Setpoint;

public interface Profile<Input extends Setpoint, Output extends Setpoint> {
  public Output calculate(Input goalSetpoint);

  public void updateState(Input startingSetpoint);
}
