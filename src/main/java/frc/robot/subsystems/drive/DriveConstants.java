// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.nominalVoltage_V;
import static frc.robot.Constants.robotPeriod_s;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.nova.NovaConfig;
import frc.robot.util.nova.NovaConfig.BrakeMode;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DriveConstants {
  public static final String name = "Drive";
  public static final LoggedNetworkBoolean tunable =
      new LoggedNetworkBoolean("Tunable/" + name + "/Tunable", false);
  public static final double wheelRadius_m = Units.inchesToMeters((3.875 - .12) / 2);
  public static final double maxSpeed_mPs = Units.feetToMeters(15);
  public static final double odometryFrequency_Hz = 100.0;
  public static final double trackWidth_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double wheelBase_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double driveBaseRadius_m = Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth_m / 2.0, wheelBase_m / 2.0),
        new Translation2d(trackWidth_m / 2.0, -wheelBase_m / 2.0),
        new Translation2d(-trackWidth_m / 2.0, wheelBase_m / 2.0),
        new Translation2d(-trackWidth_m / 2.0, -wheelBase_m / 2.0)
      };

  public static class DriveMotor {
    // Drive motor configuration
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {3, 5, 1, 7};

    public static final double reduction = 6.25666667;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    public static final NovaConfig config = new NovaConfig();

    static {
      config.setBrakeMode(BrakeMode.BRAKE).setVoltageCompensation(nominalVoltage_V);
      config.limits.setMaxStatorCurrent(60).setMaxSupplyCurrent(60);
      config
          .canFreq
          .setSensorPeriod(1 / odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);
    }

    // Drive PID configuration
    public static final SimpleMotorFeedforward realFF =
        new SimpleMotorFeedforward(0.0, 0.1, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);
    public static final SimpleMotorFeedforward simFF =
        new SimpleMotorFeedforward(0.0, 0.16, 0.008, robotPeriod_s);
    public static final PIDController simPID = new PIDController(0.2, 0.0, 0.0, robotPeriod_s);

    public static TunableDouble realS =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realS", realFF.getKs(), realFF::setKs);
    public static TunableDouble realV =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realV", realFF.getKv(), realFF::setKv);
    public static TunableDouble realA =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realA", realFF.getKa(), realFF::setKa);
    public static TunableDouble realP =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realP", realPID.getP(), realPID::setP);
    public static TunableDouble realI =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realI", realPID.getI(), realPID::setI);
    public static TunableDouble realD =
        new TunableDouble("Tunable/" + name + "/DriveMotor/realD", realPID.getD(), realPID::setD);

    public static TunableDouble simS =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simS", simFF.getKs(), simFF::setKs);
    public static TunableDouble simV =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simV", simFF.getKv(), simFF::setKv);
    public static TunableDouble simA =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simA", simFF.getKa(), simFF::setKa);
    public static TunableDouble simP =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simP", simPID.getP(), simPID::setP);
    public static TunableDouble simI =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simI", simPID.getI(), simPID::setI);
    public static TunableDouble simD =
        new TunableDouble("Tunable/" + name + "/DriveMotor/simD", simPID.getD(), simPID::setD);

    public static void updateTunable() {
      realS.update(tunable.get());
      realV.update(tunable.get());
      realA.update(tunable.get());
      realP.update(tunable.get());
      realI.update(tunable.get());
      realD.update(tunable.get());
      simS.update(tunable.get());
      simV.update(tunable.get());
      simA.update(tunable.get());
      simP.update(tunable.get());
      simI.update(tunable.get());
      simD.update(tunable.get());
    }
  }

  public static class TurnMotor {
    // Turn motor configuration
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {4, 6, 2, 8};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    public static final NovaConfig config = new NovaConfig();

    static {
      config.setBrakeMode(BrakeMode.BRAKE).setVoltageCompensation(nominalVoltage_V);
      config.limits.setMaxStatorCurrent(50).setMaxSupplyCurrent(20);
      config
          .canFreq
          .setSensorPeriod(1 / odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);
    }

    // Turn PID configuration
    public static final PIDController realPID = new PIDController(2.0, 0.0, 0.0, robotPeriod_s);
    public static final PIDController simPID = new PIDController(16.0, 0.0, 0.3, robotPeriod_s);

    static {
      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);
    }

    public static TunableDouble realP =
        new TunableDouble("Tunable/" + name + "/TurnMotor/realP", realPID.getP(), realPID::setP);
    public static TunableDouble realI =
        new TunableDouble("Tunable/" + name + "/TurnMotor/realI", realPID.getI(), realPID::setI);
    public static TunableDouble realD =
        new TunableDouble("Tunable/" + name + "/TurnMotor/realD", realPID.getD(), realPID::setD);

    public static TunableDouble simP =
        new TunableDouble("Tunable/" + name + "/TurnMotor/simP", simPID.getP(), simPID::setP);
    public static TunableDouble simI =
        new TunableDouble("Tunable/" + name + "/TurnMotor/simI", simPID.getI(), simPID::setI);
    public static TunableDouble simD =
        new TunableDouble("Tunable/" + name + "/TurnMotor/simD", simPID.getD(), simPID::setD);

    public static void updateTunable() {
      realP.update(tunable.get());
      realI.update(tunable.get());
      realD.update(tunable.get());
      simP.update(tunable.get());
      simI.update(tunable.get());
      simD.update(tunable.get());
    }
  }

  public static class AbsEncoder {
    /** FL, FR, BL, BR */
    public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    // Zeroed rotation values for each module
    /** FL, FR, BL, BR */
    public static final double[] zeroRotations_rad = new double[] {0.0, 0.0, 0.0, 0.0};
  }

  // PathPlanner configuration
  public static final double robotMass_kg = 18.35;
  public static final double robotMOI_kgm2 = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass_kg,
          robotMOI_kgm2,
          new ModuleConfig(
              wheelRadius_m,
              maxSpeed_mPs,
              wheelCOF,
              DriveMotor.gearbox.withReduction(DriveMotor.reduction),
              DriveMotor.config.limits.getMaxStatorCurrent(),
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(Inches.of(30.625), Inches.of(30.625))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMass_kg))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DriveMotor.gearbox,
                  TurnMotor.gearbox,
                  DriveMotor.reduction,
                  TurnMotor.reduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadius_m),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));

  public static void updateTunable() {
    DriveMotor.updateTunable();
    TurnMotor.updateTunable();
  }
}
