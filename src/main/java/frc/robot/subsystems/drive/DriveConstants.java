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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.util.logging.TunableBoolean;
import frc.robot.util.logging.TunableDouble;
import frc.robot.util.nova.NovaConfig;
import frc.robot.util.nova.NovaConfig.BrakeMode;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final String name = "Drive";
  public static final TunableBoolean tunable =
      new TunableBoolean("Tunable/" + name + "/.Tunable", false, () -> true);
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

  // PathPlanner configuration
  public static boolean ppAutoBuilderConfigChanged = false;
  public static boolean ppConfigChanged = false;
  public static boolean mapleSimConfigChanged = false;
  public static final PIDConstants linearPPpid = new PIDConstants(20.0, 0.0, 0.0);
  public static final PIDConstants angularPPpid = new PIDConstants(20.0, 0.0, 0.0);

  public static final RobotConfig ppConfig =
      new RobotConfig(
          18.35 + 40,
          6.883,
          new ModuleConfig(
              Units.inchesToMeters((3.875 - .12) / 2),
              Units.feetToMeters(10),
              1,
              DriveMotor.gearbox.withReduction(DriveMotor.reduction),
              DriveMotor.config.limits.getMaxStatorCurrent(),
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(Inches.of(30.625), Inches.of(30.625))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(ppConfig.massKG))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DriveMotor.gearbox,
                  TurnMotor.gearbox,
                  DriveMotor.reduction,
                  TurnMotor.reduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(ppConfig.moduleConfig.wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  ppConfig.moduleConfig.wheelCOF));

  public static final TunableDouble
      linearP =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/linearP",
              linearPPpid.kP,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      linearI =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/linearI",
              linearPPpid.kI,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      linearD =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/linearD",
              linearPPpid.kP,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      linearIZone =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/linearIZone",
              linearPPpid.iZone,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      angularP =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/angularP",
              angularPPpid.kP,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      angularI =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/angularI",
              angularPPpid.kI,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      angularD =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/angularD",
              angularPPpid.kP,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      angularIZone =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/angularIZone",
              angularPPpid.iZone,
              tunable,
              (value) -> ppAutoBuilderConfigChanged = true),
      robotMass_kg =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/robotMass_kg",
              ppConfig.massKG,
              tunable,
              (value) -> {
                ppAutoBuilderConfigChanged = true;
                ppConfigChanged = true;
                mapleSimConfigChanged = true;
                mapleSimConfig.withRobotMass(Kilograms.of(value));
              }),
      robotMOI_kgm2 =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/robotMOI_kgm2",
              ppConfig.MOI,
              tunable,
              (value) -> {
                ppAutoBuilderConfigChanged = true;
                ppConfigChanged = true;
              }),
      wheelCOF =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/wheelCOF",
              ppConfig.moduleConfig.wheelCOF,
              tunable,
              (value) -> {
                ppAutoBuilderConfigChanged = true;
                ppConfigChanged = true;
                mapleSimConfigChanged = true;
                mapleSimConfig.withSwerveModule(
                    new SwerveModuleSimulationConfig(
                        DriveMotor.gearbox,
                        TurnMotor.gearbox,
                        DriveMotor.reduction,
                        TurnMotor.reduction,
                        Volts.of(0.1),
                        Volts.of(0.1),
                        Meters.of(ppConfig.moduleConfig.wheelRadiusMeters),
                        KilogramSquareMeters.of(0.02),
                        value));
              }),
      wheelRadius_m =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/wheelRadius_m",
              ppConfig.moduleConfig.wheelRadiusMeters,
              tunable,
              (value) -> {
                ppAutoBuilderConfigChanged = true;
                ppConfigChanged = true;
                mapleSimConfigChanged = true;
                mapleSimConfig.withSwerveModule(
                    new SwerveModuleSimulationConfig(
                        DriveMotor.gearbox,
                        TurnMotor.gearbox,
                        DriveMotor.reduction,
                        TurnMotor.reduction,
                        Volts.of(0.1),
                        Volts.of(0.1),
                        Meters.of(value),
                        KilogramSquareMeters.of(0.02),
                        ppConfig.moduleConfig.wheelCOF));
              }),
      maxSpeed_mPs =
          new TunableDouble(
              "Tunable/" + name + "/ppConfig/maxSpeed_mPs",
              ppConfig.moduleConfig.maxDriveVelocityMPS,
              tunable,
              (value) -> {
                ppAutoBuilderConfigChanged = true;
                ppConfigChanged = true;
              });

  /** The maximum rotation velocity of a swerve module, in radians per second */
  public static final TunableDouble maxSteerVelocity_radPs =
      new TunableDouble(
          "Tunable/" + name + "/ppConfig/maxSteerVelocity_radPs",
          4,
          tunable,
          (value) -> {
            ppAutoBuilderConfigChanged = true;
            ppConfigChanged = true;
          });

  public static class AbsEncoder {
    /** FL, FR, BL, BR */
    public static final int[] analogPorts = new int[] {0, 1, 2, 3};

    // Zeroed rotation values for each module
    /** FL, FR, BL, BR */
    public static final double[] zeroRotations_rad = new double[] {0.0, 0.0, 0.0, 0.0};
  }

  public static class TurnMotor {
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {4, 6, 2, 8};

    public static final double reduction = 25;
    public static final DCMotor gearbox = DCMotor.getNEO(1);
    public static final NovaConfig config = new NovaConfig();
    public static final Constraints profileConstraints = new Constraints(5 * Math.PI, 50 * Math.PI);

    /** PID controllers for the turn (azimuth) motor in each swerve module. */
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.0, 0.1, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.004, 0.4960674, 0.006, robotPeriod_s);

    public static final PIDController realPID = new PIDController(2.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(4, 0.0, 0.05, robotPeriod_s);

    public static final SendableBuilderImpl sendableBuilder = new SendableBuilderImpl();

    static {
      config
          .setInversion(false)
          .setBrakeMode(BrakeMode.BRAKE)
          .setVoltageCompensation(nominalVoltage_V);
      config.limits.setMaxStatorCurrent(50).setMaxSupplyCurrent(20);
      config
          .canFreq
          .setSensorPeriod(1 / odometryFrequency_Hz)
          .setControlPeriod(0.02)
          .setCurrentPeriod(0.02)
          .setFaultPeriod(0.02);

      realPID.enableContinuousInput(0, 2 * Math.PI);
      simPID.enableContinuousInput(0, 2 * Math.PI);

      //   sendableBuilder.setTable(
      //       NetworkTableInstance.getDefault().getTable("Tunable/Drive/TurnMotor"));
      //   realPID.initSendable(sendableBuilder);
    }

    @SuppressWarnings("unused")
    private static final TunableDouble
        realS =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realS", realFF.getKs(), tunable, realFF::setKs),
        realV =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realV", realFF.getKv(), tunable, realFF::setKv),
        realA =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realA", realFF.getKa(), tunable, realFF::setKa),
        realP =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realP", realPID.getP(), tunable, realPID::setP),
        realI =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realI", realPID.getI(), tunable, realPID::setI),
        realD =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/realD", realPID.getD(), tunable, realPID::setD),
        simS =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simS", simFF.getKs(), tunable, simFF::setKs),
        simV =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simV", simFF.getKv(), tunable, simFF::setKv),
        simA =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simA", simFF.getKa(), tunable, simFF::setKa),
        simP =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simP", simPID.getP(), tunable, simPID::setP),
        simI =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simI", simPID.getI(), tunable, simPID::setI),
        simD =
            new TunableDouble(
                "Tunable/" + name + "/TurnMotor/simD", simPID.getD(), tunable, simPID::setD);
  }

  public static class DriveMotor {
    /** FL, FR, BL, BR */
    public static final int[] canIds = new int[] {3, 5, 1, 7};

    public static final double reduction = 6.25666667;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
    public static final NovaConfig config = new NovaConfig();
    public static final SimpleMotorFeedforward
        realFF = new SimpleMotorFeedforward(0.0, 0.1, 0.0, robotPeriod_s),
        simFF = new SimpleMotorFeedforward(0.036968, 0.15869, 0.034, robotPeriod_s);
    public static final PIDController realPID = new PIDController(0.0, 0.0, 0.0, robotPeriod_s),
        simPID = new PIDController(0.23931, 0.0, 0.0, robotPeriod_s);

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

    @SuppressWarnings("unused")
    private static final TunableDouble
        realS =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realS", realFF.getKs(), tunable, realFF::setKs),
        realV =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realV", realFF.getKv(), tunable, realFF::setKv),
        realA =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realA", realFF.getKa(), tunable, realFF::setKa),
        realP =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realP", realPID.getP(), tunable, realPID::setP),
        realI =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realI", realPID.getI(), tunable, realPID::setI),
        realD =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/realD", realPID.getD(), tunable, realPID::setD),
        simS =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simS", simFF.getKs(), tunable, simFF::setKs),
        simV =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simV", simFF.getKv(), tunable, simFF::setKv),
        simA =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simA", simFF.getKa(), tunable, simFF::setKa),
        simP =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simP", simPID.getP(), tunable, simPID::setP),
        simI =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simI", simPID.getI(), tunable, simPID::setI),
        simD =
            new TunableDouble(
                "Tunable/" + name + "/DriveMotor/simD", simPID.getD(), tunable, simPID::setD);
  }
}
