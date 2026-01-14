package frc.robot.util.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.util.control.MechanismStates.AngularMechanismState;
import frc.robot.util.control.Setpoints.AngularSetpoint;
import frc.robot.util.control.feedback.Feedback;
import frc.robot.util.control.feedback.SimpleAngularPIDF;
import frc.robot.util.control.profiling.AngularTrapezoidProfile;
import frc.robot.util.control.profiling.Profile;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private final DCMotorSim simulator;

  private AngularMechanismState currentState;

  private final Feedback<AngularMechanismState, AngularSetpoint> feedback;
  private final Profile<AngularSetpoint, AngularSetpoint> profile;

  private final String logName;

  private final SparkMax spark;
  private final RelativeEncoder encoder;

  @SuppressWarnings("static-access")
  public Turret(TurretConfig config) {
    logName = config.name;
    if (RobotBase.isReal()) {
      spark = new SparkMax(config.canId, MotorType.kBrushless);
      spark.configure(
          config.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      encoder = spark.getEncoder();
      simulator = null;
      profile = new AngularTrapezoidProfile(
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(config.max_radPs, config.max_radPs2)),
          new AngularSetpoint(config.start_rad, config.start_radPs));
      feedback = new SimpleAngularPIDF(
          config.realPID, config.realFF, new AngularSetpoint(config.start_rad, config.start_radPs));
    } else {
      spark = null;
      encoder = null;
      simulator = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(config.gearbox, config.moi_kgm2, config.reduction),
          config.gearbox);
      simulator.setState(config.start_rad, config.start_radPs);
      profile = new AngularTrapezoidProfile(
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(config.max_radPs, config.max_radPs2)),
          new AngularSetpoint(config.start_rad, config.start_radPs));
      feedback = new SimpleAngularPIDF(
          config.simPID, config.simFF, new AngularSetpoint(config.start_rad, config.start_radPs));
    }
    update();
  }

  public AngularMechanismState update() {
    if (RobotBase.isReal()) {
      currentState = new AngularMechanismState(
          encoder.getPosition(),
          encoder.getVelocity(),
          spark.getBusVoltage() * spark.getAppliedOutput(),
          spark.getOutputCurrent(),
          spark.getOutputCurrent() * spark.getAppliedOutput());
    } else {
      currentState = new AngularMechanismState(
          simulator.getAngularPositionRad(),
          simulator.getAngularVelocityRadPerSec(),
          simulator.getInputVoltage(),
          simulator.getCurrentDrawAmps(),
          simulator.getCurrentDrawAmps()
              * simulator.getInputVoltage()
              / RoboRioSim.getVInVoltage());
    }
    return currentState;
  }

  public void setVoltage(double V) {
    profile.updateState(new AngularSetpoint(currentState.rad(), currentState.radPs()));
    feedback.updateState(new AngularSetpoint(currentState.rad(), currentState.radPs()));

    Logger.recordOutput(logName + "/Setpoint/Goal", new AngularSetpoint(Double.NaN, Double.NaN));
    Logger.recordOutput(logName + "/Setpoint/Next", new AngularSetpoint(Double.NaN, Double.NaN));
    Logger.recordOutput(logName + "/Setpoint/V", V);

    if (RobotBase.isReal()) {
      spark.setVoltage(V);
    } else {
      simulator.setInputVoltage(V);
    }
  }

  public void setGoalState(AngularSetpoint goalSetpoint) {
    AngularSetpoint nextSetpoint = profile.calculate(goalSetpoint);
    double V = feedback.calculateVolts(currentState, nextSetpoint);

    Logger.recordOutput(logName + "/Setpoint/Goal", goalSetpoint);
    Logger.recordOutput(logName + "/Setpoint/Next", nextSetpoint);
    Logger.recordOutput(logName + "/Setpoint/V", V);

    if (RobotBase.isReal()) {
      spark.setVoltage(V);
    } else {
      simulator.setInputVoltage(V);
    }
  }

  public abstract static class TurretConfig {
    public static final String name = null;
    // physical constants
    public static final double moi_kgm2 = Double.NaN;
    public static final double mass_kg = Double.NaN;
    public static final int canId = Integer.MIN_VALUE;
    public static final double reduction = Double.NaN;
    public static final DCMotor gearbox = null;

    // software limits
    public static final double start_rad = 0;
    public static final double start_radPs = 0;
    public static final double min_rad = Double.NaN;
    public static final double max_rad = Double.NaN;
    public static final double max_radPs = Double.NaN;
    public static final double max_radPs2 = Double.NaN;
    public static final int maxStator_A = Integer.MIN_VALUE;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public static final SimpleMotorFeedforward realFF = null, simFF = null;
    public static final PIDController realPID = null, simPID = null;
  }
}
