package frc.robot.util.control.feedback;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.control.MechanismStates.AngularMechanismState;
import frc.robot.util.control.Setpoints.AngularSetpoint;

public class SimpleAngularPIDF implements Feedback<AngularMechanismState, AngularSetpoint> {
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private AngularSetpoint lastNextSetpoint;

  public SimpleAngularPIDF(
      PIDController pid, SimpleMotorFeedforward ff, AngularSetpoint startingState) {
    pid.enableContinuousInput(-Math.PI, Math.PI);
    this.pid = pid;
    this.ff = ff;
    lastNextSetpoint = startingState;
  }

  @Override
  public double calculateVolts(AngularMechanismState currentState, AngularSetpoint nextSetpoint) {
    double V = ff.calculateWithVelocities(lastNextSetpoint.radPs(), nextSetpoint.radPs())
        + pid.calculate(currentState.rad(), lastNextSetpoint.rad());
    lastNextSetpoint = nextSetpoint;
    return V;
  }

  @Override
  public void updateState(AngularSetpoint lastNextSetpoint) {
    this.lastNextSetpoint = lastNextSetpoint;
  }
}
