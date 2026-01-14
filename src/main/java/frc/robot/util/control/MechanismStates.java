package frc.robot.util.control;

public class MechanismStates {
  public interface MechanismState {}

  public record AngularMechanismState(
      double rad, double radPs, double V, double motor_A, double supply_A)
      implements MechanismState {}

  public record LinearMechanismState(
      double mechanism_m, double mechanism_mPs, double motor_V, double motor_A, double supply_A)
      implements MechanismState {}
}
