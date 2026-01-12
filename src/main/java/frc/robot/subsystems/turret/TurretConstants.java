package frc.robot.subsystems.turret;

import static frc.robot.Constants.nominalVoltage_V;
import static frc.robot.Constants.robotPeriod_s;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TurretConstants {
  public static final String name = "Turret";
  public static final int canId = 9;
  public static final double reduction = 5;
  public static final DCMotor gearbox = DCMotor.getNEO(1);
  public static final SparkMaxConfig motorConfig = new SparkMaxConfig();
  public static final TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(10, 20));

  public static final SimpleMotorFeedforward
      realFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s),
      simFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s);
  public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
      simPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);

  static {
    motorConfig
        .voltageCompensation(nominalVoltage_V)
        .smartCurrentLimit(20, 30)
        .encoder
        .positionConversionFactor(2 * Math.PI / reduction)
        .velocityConversionFactor(Math.PI / (30 * reduction));
  }
}
