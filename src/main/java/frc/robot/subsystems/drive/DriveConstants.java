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
import static frc.robot.Constants.robotPeriod_s;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DriveConstants {
  public static final String name = "Drive";
  public static final LoggedNetworkBoolean tunable =
      new LoggedNetworkBoolean("Tunable/" + name + "/Tunable", false);
  public static final double wheelRadius_m = Units.inchesToMeters((3.875 - .12) / 2);
  public static final double maxSpeed_mPs = Units.feetToMeters(15);
  public static final double odometryFrequency_Hz = 100.0;
  public static final double trackWidth_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double wheelBase_m = Units.inchesToMeters(21.625 - (2 * 1.6875));
  public static final double driveBaseRadius = Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth_m / 2.0, wheelBase_m / 2.0),
        new Translation2d(trackWidth_m / 2.0, -wheelBase_m / 2.0),
        new Translation2d(-trackWidth_m / 2.0, wheelBase_m / 2.0),
        new Translation2d(-trackWidth_m / 2.0, -wheelBase_m / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 3;
  public static final int backLeftDriveCanId = 1;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 4;
  public static final int backLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftEncoderPort = 0;
  public static final int backLeftEncoderPort = 2;
  public static final int frontRightEncoderPort = 1;
  public static final int backRightEncoderPort = 3;

  public static class DriveMotor {
    private static boolean tunable = false;
    // Drive motor configuration
    public static final int stallLimit_A = 60;
    public static final double reduction = 6.25666667;
    // (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // Drive encoder configuration
    public static final double encoderPositionFactor =
        2 * Math.PI / reduction; // Rotor Rotations -> Wheel Radians
    public static final double encoderVelocityFactor =
        ((2 * Math.PI) / 60.0) / reduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final SimpleMotorFeedforward realFF =
        new SimpleMotorFeedforward(0.0, 0.1, 0.0, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s);
    public static final SimpleMotorFeedforward simFF =
        new SimpleMotorFeedforward(0.0, 0.16, 0.008, robotPeriod_s);
    public static final PIDController simPID = new PIDController(0.2, 0.0, 0.0, robotPeriod_s);
    public static LoggedNetworkNumber realS = null;
    public static LoggedNetworkNumber realV = null;
    public static LoggedNetworkNumber realA = null;
    public static LoggedNetworkNumber realP = null;
    public static LoggedNetworkNumber realI = null;
    public static LoggedNetworkNumber realD = null;
    public static LoggedNetworkNumber simS = null;
    public static LoggedNetworkNumber simV = null;
    public static LoggedNetworkNumber simA = null;
    public static LoggedNetworkNumber simP = null;
    public static LoggedNetworkNumber simI = null;
    public static LoggedNetworkNumber simD = null;

    public static void updateTunable() {
      if (DriveConstants.tunable.get()) {
        if (!tunable) {
          tunable = true;
          realP = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realP", realPID.getP());
          realI = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realI", realPID.getI());
          realD = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realD", realPID.getD());
          realS = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realS", realFF.getKs());
          realV = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realV", realFF.getKv());
          realA = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/realA", realFF.getKa());
          simP = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simP", simPID.getP());
          simI = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simI", simPID.getI());
          simD = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simD", simPID.getD());
          simS = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simS", simFF.getKs());
          simV = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simV", simFF.getKv());
          simA = new LoggedNetworkNumber("Tunable/" + name + "/DriveMotor/simA", simFF.getKa());
        } else {
          realPID.setP(realP.get());
          realPID.setI(realI.get());
          realPID.setD(realD.get());
          realFF.setKs(realS.get());
          realFF.setKv(realV.get());
          realFF.setKa(realA.get());
          simPID.setP(simP.get());
          simPID.setI(simI.get());
          simPID.setD(simD.get());
          simFF.setKs(simS.get());
          simFF.setKv(simV.get());
          simFF.setKa(simA.get());
        }
      } else {
        if (tunable) {
          tunable = false;
          realP = null;
          realI = null;
          realD = null;
          realS = null;
          realV = null;
          realA = null;
          simP = null;
          simI = null;
          simD = null;
          simS = null;
          simV = null;
          simA = null;
        }
      }
    }
  }

  public static class TurnMotor {
    // Turn motor configuration
    private static boolean tunable = false;
    public static final boolean inverted = false;
    public static final int currentLimit_A = 20;
    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnAbsEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnAbsEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
    public static final double turnEncoderPositionFactor = 2 * Math.PI / 25; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor =
        (2 * Math.PI / 25) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
    public static final PIDController realPID = new PIDController(2.0, 0.0, 0.0, robotPeriod_s);
    public static final PIDController simPID = new PIDController(16.0, 0.0, 0.3, robotPeriod_s);

    static {
      realPID.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);
      simPID.enableContinuousInput(turnPIDMinInput, turnPIDMaxInput);
    }

    public static LoggedNetworkNumber realP = null;
    public static LoggedNetworkNumber realI = null;
    public static LoggedNetworkNumber realD = null;
    public static LoggedNetworkNumber simP = null;
    public static LoggedNetworkNumber simI = null;
    public static LoggedNetworkNumber simD = null;

    public static void updateTunable() {
      if (DriveConstants.tunable.get()) {
        if (!tunable) {
          tunable = true;
          realP = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/realP", realPID.getP());
          realI = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/realI", realPID.getI());
          realD = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/realD", realPID.getD());
          simP = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/simP", simPID.getP());
          simI = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/simI", simPID.getI());
          simD = new LoggedNetworkNumber("Tunable/" + name + "/TurnMotor/simD", simPID.getD());
        } else {
          realPID.setP(realP.get());
          realPID.setI(realI.get());
          realPID.setD(realD.get());
          simPID.setP(simP.get());
          simPID.setI(simI.get());
          simPID.setD(simD.get());
        }
      } else {
        if (tunable) {
          tunable = false;
          realP = null;
          realI = null;
          realD = null;
          simP = null;
          simI = null;
          simD = null;
        }
      }
    }
  }

  // PathPlanner configuration
  public static final double robotMassKg = 18.35;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadius_m,
              maxSpeed_mPs,
              wheelCOF,
              DriveMotor.gearbox.withReduction(DriveMotor.reduction),
              DriveMotor.stallLimit_A,
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(Inches.of(30.625), Inches.of(30.625))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
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
