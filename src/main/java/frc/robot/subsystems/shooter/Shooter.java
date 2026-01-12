package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.robotPeriod_s;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;
import frc.robot.subsystems.shooter.ShooterConstants.Hood;
import frc.robot.subsystems.shooter.ShooterConstants.Turret;
import frc.robot.util.SysIdUtil;
import frc.robot.util.SysIdUtil.SysIdType;
import frc.robot.util.logging.LogUtil.AngularMechanismState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Interface to control the shooter's hardware. */
  private final ShooterIO io;

  /** The code's inputs from the shooter's hardware. */
  private final ShooterIOInputsAutoLogged ioInputs = new ShooterIOInputsAutoLogged();

  // motion profiling
  private final TrapezoidProfile
      flywheelProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(Flywheel.max_radPs2, Flywheel.max_radPs3)),
      hoodProfile =
          new TrapezoidProfile(new TrapezoidProfile.Constraints(Hood.max_radPs, Hood.max_radPs2)),
      turretProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(Turret.max_radPs, Turret.max_radPs2));

  private double flywheelLast_radPs = 0;

  // Sys Id
  private final SysIdRoutine
      flywheelSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput(Flywheel.name + "/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> setFlywheelVoltage(voltage.in(Volts)), null, this)),
      hoodSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput(Hood.name + "/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> setHoodVoltage(voltage.in(Volts)), null, this)),
      turretSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput(Turret.name + "/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> setTurretVoltage(voltage.in(Volts)), null, this));

  /** Constructor for the ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(ioInputs);
    Logger.processInputs(name + "/ioInputs", ioInputs);
    Logger.recordOutput(name + "/Pose3d", new Pose3d[] {
      new Pose3d(0, 0, -1, Rotation3d.kZero),
      new Pose3d(0, 0, -1, Rotation3d.kZero),
      new Pose3d(
          0.089575,
          0.029992,
          .5 + ioInputs.flywheel.mechanism_radPs() / 1000,
          new Rotation3d(0, -ioInputs.hood.mechanism_rad(), ioInputs.turret.mechanism_rad()))
    });
  }

  public void stop() {
    setVoltages(0, 0, 0);
  }

  public void setVoltages(double flywheel_V, double hood_V, double turret_V) {
    setFlywheelVoltage(flywheel_V);
    setHoodVoltage(hood_V);
    setTurretVoltage(turret_V);
  }

  public void setFlywheelVoltage(double V) {
    io.setFlywheelVoltage(V);
    flywheelLast_radPs = ioInputs.flywheel.mechanism_radPs();
  }

  public void setHoodVoltage(double V) {
    io.setHoodVoltage(V);
  }

  public void setTurretVoltage(double V) {
    io.setTurretVoltage(V);
  }

  public void setGoals(double flywheel_radPs, double hood_rad, double turret_rad) {
    setFlywheelGoalVelocity(flywheel_radPs);
    setHoodGoalPosition(hood_rad);
    setTurretGoalPosition(turret_rad);
  }

  public void setFlywheelGoalVelocity(double radPs) {
    io.setFlywheelNextState(flywheelProfile.calculate(
            robotPeriod_s,
            new TrapezoidProfile.State(
                ioInputs.flywheel.mechanism_radPs(),
                (ioInputs.flywheel.mechanism_radPs() - flywheelLast_radPs) / robotPeriod_s),
            new TrapezoidProfile.State(radPs, 0.0))
        .position);
    flywheelLast_radPs = ioInputs.flywheel.mechanism_radPs();
  }

  public void setHoodGoalState(double rad, double radPs) {
    TrapezoidProfile.State nextState = hoodProfile.calculate(
        robotPeriod_s,
        new TrapezoidProfile.State(ioInputs.hood.mechanism_rad(), ioInputs.hood.mechanism_radPs()),
        new TrapezoidProfile.State(rad, radPs));
    io.setHoodNextState(nextState.position, nextState.velocity);
  }

  public void setHoodGoalPosition(double rad) {
    setHoodGoalState(rad, 0);
  }

  public void setTurretGoalState(double rad, double radPs) {
    TrapezoidProfile.State nextState = turretProfile.calculate(
        robotPeriod_s,
        new TrapezoidProfile.State(
            ioInputs.turret.mechanism_rad(), ioInputs.turret.mechanism_radPs()),
        new TrapezoidProfile.State(rad, radPs));
    io.setTurretNextState(nextState.position, nextState.velocity);
  }

  public void setTurretGoalPosition(double rad) {
    setTurretGoalState(rad, 0);
  }

  public Command getFlywheelSysId(SysIdType type) {
    return SysIdUtil.getSysIdCommand(flywheelSysId, type);
  }

  public Command getHoodSysId(SysIdType type) {
    return SysIdUtil.getSysIdCommand(hoodSysId, type);
  }

  public Command getTurretSysId(SysIdType type) {
    return SysIdUtil.getSysIdCommand(turretSysId, type);
  }

  public AngularMechanismState getFlywheelState() {
    return ioInputs.flywheel;
  }

  public AngularMechanismState getHoodState() {
    return ioInputs.hood;
  }

  public AngularMechanismState getTurretState() {
    return ioInputs.turret;
  }
}
