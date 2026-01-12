package frc.robot.subsystems.turret;

import static frc.robot.Constants.nominalVoltage_V;
import static frc.robot.Constants.robotPeriod_s;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.UnitUtil;

public class ShooterConstants {
  public static final String name = "Shooter";

  public static class Flywheel {
    public static final String name = ShooterConstants.name + "Flywheel";
    // physical constants
    public static final double moi_kgm2 = 0.1;
    public static final int canId = 9;
    public static final double reduction = 1;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double maxA_radPs2 = 10000;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    // motion profiling
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);
    }
  }

  public static class Hood {
    public static final String name = ShooterConstants.name + "Hood";
    // physical constants
    public static final double moi_kgm2 = 0.1;
    public static final double mass_kg = 0.5;
    public static final double cmRadius_m = 0.1;
    public static final double cmAngle_rad = -1.0;
    public static final int canId = 9;
    public static final double reduction = 3;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double minP_rad = 0;
    public static final double maxP_rad = 2;
    public static final double maxV_radPs = 4;
    public static final double maxA_radPs2 = 6;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public static final ArmFeedforward
        realFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);
    }
  }

  public static class Turret {
    public static final String name = ShooterConstants.name + "Turret";
    // physical constants
    public static final double moi_kgm2 = 0.1;
    public static final double mass_kg = 0.5;
    public static final int canId = 9;
    public static final double reduction = 20;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // software limits
    public static final double minP_rad = 0;
    public static final double maxP_rad = 2;
    public static final double maxV_radPs = 4;
    public static final double maxA_radPs2 = 6;
    public static final int maxStator_A = 30;
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public static final ArmFeedforward
        realFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, robotPeriod_s),
        simFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);

    static {
      motorConfig
          .voltageCompensation(nominalVoltage_V)
          .smartCurrentLimit(maxStator_A)
          .encoder
          .positionConversionFactor(UnitUtil.rotTorad(1) / reduction)
          .velocityConversionFactor(UnitUtil.RPMToradPs(1) / reduction);
    }
  }
}
