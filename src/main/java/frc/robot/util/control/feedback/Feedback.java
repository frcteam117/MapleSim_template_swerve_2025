package frc.robot.util.control.feedback;

import frc.robot.util.control.MechanismStates.MechanismState;
import frc.robot.util.control.Setpoints.Setpoint;

public interface Feedback<State extends MechanismState, Next extends Setpoint> {
  public double calculateVolts(State currentState, Next nextSetpoint);

  public void updateState(Next startingSetpoint);
}
