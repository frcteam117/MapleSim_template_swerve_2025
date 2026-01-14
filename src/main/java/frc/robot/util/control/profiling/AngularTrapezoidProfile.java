package frc.robot.util.control.profiling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.Robot;
import frc.robot.util.control.Setpoints.AngularSetpoint;

public class AngularTrapezoidProfile implements Profile<AngularSetpoint, AngularSetpoint> {
  private final TrapezoidProfile trapezoidProfile;
  private final double period_s = Robot.codePeriod_s;
  private TrapezoidProfile.State prevState;

  public AngularTrapezoidProfile(TrapezoidProfile trapezoidProfile, AngularSetpoint currentState) {
    this.trapezoidProfile = trapezoidProfile;
    prevState = new TrapezoidProfile.State(currentState.rad(), currentState.radPs());
  }

  @Override
  public AngularSetpoint calculate(AngularSetpoint goalSetpoint) {
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(
        prevState.position
            + MathUtil.inputModulus(goalSetpoint.rad() - prevState.position, -Math.PI, Math.PI),
        goalSetpoint.radPs());
    prevState = trapezoidProfile.calculate(period_s, prevState, goalState);
    return new AngularSetpoint(prevState.position, prevState.velocity);
  }

  @Override
  public void updateState(AngularSetpoint startingState) {
    prevState = new TrapezoidProfile.State(startingState.rad(), startingState.radPs());
  }
}
