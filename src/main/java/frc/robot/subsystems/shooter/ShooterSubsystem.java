package frc.robot.subsystems.shooter;

import static frc.robot.Constants.robotPeriod_s;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  /** Interface to control the funnel's hardware. */
  private final ShooterIO io;

  /** The code's inputs from the funnel's hardware. */
  private final ShooterIOInputs ioInputs = new ShooterIOInputs();

  /** Constructor for the IntakeSubsystem. */
  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(ioInputs);
    // Logger.processInputs(name + "/ioInputs", ioInputs);
  }

  public void setVoltage(double voltage_V) {
    io.setVoltage(voltage_V);
  }

  public Command runVoltageCommandFactory(DoubleSupplier voltageSupplier_V) {
    return this.run(() -> {
      setVoltage(voltageSupplier_V.getAsDouble());
    });
  }

  public void setGoalVelocity(double velocity_radPs) {
    TrapezoidProfile.State nextState = profile.calculate(
        robotPeriod_s,
        new TrapezoidProfile.State(ioInputs.mechanismVelocity_radPs, 0.0),
        new TrapezoidProfile.State(velocity_radPs, 0.0));
    io.setNextVelocity(nextState.position);
  }

  public Command runGoalVelocityCommandFactory(DoubleSupplier velocitySupplier_radPs) {
    return this.run(() -> {
      setGoalVelocity(velocitySupplier_radPs.getAsDouble());
    });
  }

  public ShooterIOInputs getInputs() {
    return ioInputs;
  }
}
