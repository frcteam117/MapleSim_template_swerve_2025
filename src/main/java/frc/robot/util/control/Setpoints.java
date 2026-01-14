package frc.robot.util.control;

public class Setpoints {
  public interface Setpoint {}

  public record VoltageSetpoint(double V) implements Setpoint {}

  public record LinearSetpoint(double m, double mPs) implements Setpoint {}

  public record AngularSetpoint(double rad, double radPs) implements Setpoint {}
}
