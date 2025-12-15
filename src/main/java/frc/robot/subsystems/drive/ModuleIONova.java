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

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.MotorType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.util.nova.NovaConfig;
import frc.robot.util.nova.NovaConfig.BrakeMode;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova turn motor
 * controller, and Thrifty absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Motion profiling
  private double driveVelocity_radPs = 0;
  private double lastNextVelocity_radPs = 0;

  private Rotation2d turnPosition = new Rotation2d();

  // Editable pids
  // private final PidProperty drivePIDProperty;
  // private final PidProperty turnPIDProperty;

  // Hardware objects
  private final ThriftyNova driveNova;
  private final ThriftyNova turnNova;
  private final AnalogEncoder turnEncoder;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIONova(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveNova =
        new ThriftyNova(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            MotorType.NEO);
    turnNova =
        new ThriftyNova(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.NEO);
    turnEncoder =
        new AnalogEncoder(
            switch (module) {
              case 0 -> frontLeftEncoderPort;
              case 1 -> frontRightEncoderPort;
              case 2 -> backLeftEncoderPort;
              case 3 -> backRightEncoderPort;
              default -> 0;
            });

    // Configure drive motor
    System.out.println(
        "Configuring drive motor. Module: "
            + module
            + "  CAN Id: "
            + switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });
    NovaConfig driveConfig = new NovaConfig();
    driveConfig
        .setBrakeMode(BrakeMode.COAST)
        .setVoltageCompensation(12.0)
        .setUsedPIDSlot(PIDSlot.SLOT0);
    driveConfig
        .limits
        .setMaxStatorCurrent(DriveMotor.stallLimit_A)
        .setMaxSupplyCurrent(DriveMotor.stallLimit_A);
    driveConfig.encoder.setUsedEncoder(EncoderType.INTERNAL);
    driveConfig.pid0.setPIDF(DriveMotor.realPID.getP(), 0.0, DriveMotor.realPID.getD(), 0.0);
    driveConfig
        .canFreq
        .setSensorPeriod(1 / odometryFrequency_Hz)
        .setControlPeriod(0.02)
        .setCurrentPeriod(0.02)
        .setFaultPeriod(0.02);
    driveConfig.configure(driveNova);
    System.out.println(
        "Finished configuring drive motor. Module: "
            + module
            + "  CAN Id: "
            + switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });

    // driveNova.factoryReset();
    // driveNova
    //         .setBrakeMode(true)
    //         .setVoltageCompensation(12.0)
    //         .usePIDSlot(PIDSlot.SLOT0)
    //         .setMaxCurrent(CurrentType.STATOR, driveMotorCurrentLimit)
    //         .setMaxCurrent(CurrentType.SUPPLY, driveMotorCurrentLimit)
    //         .useEncoderType(EncoderType.INTERNAL)
    //         .pid0
    //         .setP(driveKp)
    //         .setI(0.0)
    //         .setD(driveKd)
    //         .setFF(0.0);
    // driveNova
    //         .canFreq
    //         .setSensor(1 / odometryFrequency)
    //         .setControl(0.02)
    //         .setCurrent(0.02)
    //         .setFault(0.02);

    // Configure turn motor
    System.out.println(
        "Configuring Turn motor. Module: "
            + module
            + "  CAN Id: "
            + switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            });
    NovaConfig turnConfig = new NovaConfig();
    turnConfig
        .setInversion(TurnMotor.inverted)
        .setBrakeMode(BrakeMode.BRAKE)
        .setVoltageCompensation(12.0)
        .setUsedPIDSlot(PIDSlot.SLOT0);
    turnConfig
        .limits
        .setMaxStatorCurrent(TurnMotor.currentLimit_A)
        .setMaxSupplyCurrent(TurnMotor.currentLimit_A);
    turnConfig.encoder.setUsedEncoder(EncoderType.INTERNAL); // .setAbsoluteWrapping(true)
    turnConfig.pid0.setPIDF(TurnMotor.realPID.getP(), 0.0, TurnMotor.realPID.getD(), 0.0);
    turnConfig
        .canFreq
        .setSensorPeriod(1 / odometryFrequency_Hz)
        .setControlPeriod(0.02)
        .setCurrentPeriod(0.02)
        .setFaultPeriod(0.02);
    turnConfig.configure(turnNova);
    System.out.println(
        "Finished configuring Turn motor. Module: "
            + module
            + "  CAN Id: "
            + switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            });

    // turnNova.factoryReset();
    // turnNova.setInversion(turnInverted)
    //         .setBrakeMode(true)
    //         .setVoltageCompensation(12.0)
    //         .usePIDSlot(PIDSlot.SLOT0)
    //         .setMaxCurrent(CurrentType.STATOR, turnMotorCurrentLimit)
    //         .setMaxCurrent(CurrentType.SUPPLY, turnMotorCurrentLimit)
    //         .useEncoderType(EncoderType.INTERNAL)
    //         .pid0
    //         .setP(turnKp)
    //         .setI(0.0)
    //         .setD(turnKd)
    //         .setFF(0.0);
    // turnNova.canFreq
    //         .setSensor(1 / odometryFrequency)
    //         .setControl(0.02)
    //         .setCurrent(0.02)
    //         .setFault(0.02);

    // Create odometry queues
    timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        NovaOdometryThread.getInstance().registerSignal(driveNova::getPositionInternal);
    turnPositionQueue = NovaOdometryThread.getInstance().registerSignal(turnEncoder::get);

    // Create editable pid values
    // List<HeavyDoubleProperty> turnPIDProperties = new ArrayList<>();
    // turnPIDProperties.add(new HeavyDoubleProperty(
    //         (p) -> turnNova.pid0.setP(p), new GosDoubleProperty(false, "turnPID/kp", 0.07)));
    // turnPIDProperties.add(
    //         new HeavyDoubleProperty((i) -> turnNova.pid0.setI(i), new GosDoubleProperty(false,
    // "turnPID/ki",
    // 0.0)));
    // turnPIDProperties.add(
    //         new HeavyDoubleProperty((d) -> turnNova.pid0.setD(d), new GosDoubleProperty(false,
    // "turnPID/kd",
    // 0.0)));
    // turnPIDProperties.add(new HeavyDoubleProperty(
    //         (ff) -> turnNova.pid0.setFF(ff), new GosDoubleProperty(false, "turnPID/kff", 0.0)));
    // turnPIDProperty = new PidProperty(turnPIDProperties);

    // List<HeavyDoubleProperty> drivePIDProperties = new ArrayList<>();
    // drivePIDProperties.add(new HeavyDoubleProperty(
    //         (p) -> driveNova.pid0.setP(p), new GosDoubleProperty(false, "drivePID/kp", 0.07)));
    // drivePIDProperties.add(new HeavyDoubleProperty(
    //         (i) -> driveNova.pid0.setI(i), new GosDoubleProperty(false, "drivePID/ki", 0.0)));
    // drivePIDProperties.add(new HeavyDoubleProperty(
    //         (d) -> driveNova.pid0.setD(d), new GosDoubleProperty(false, "drivePID/kd", 0.0)));
    // drivePIDProperties.add(new HeavyDoubleProperty(
    //         (ff) -> driveNova.pid0.setFF(ff),
    //         new GosDoubleProperty(false, "drivePID/kff", 1.0 / driveEncoderPositionFactor * 4)));
    // drivePIDProperty = new PidProperty(drivePIDProperties);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.drivePosition_rad = driveNova.getPositionInternal() * DriveMotor.encoderPositionFactor;
    inputs.driveVelocity_radps = driveNova.getVelocityInternal() * DriveMotor.encoderVelocityFactor;
    driveVelocity_radPs = inputs.driveVelocity_radps;
    inputs.driveVoltage_V = driveNova.getVoltage();
    inputs.driveStatorCurrent_A = driveNova.getStatorCurrent();
    inputs.driveSupplyCurrent_A = driveNova.getSupplyCurrent();
    // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    // sparkStickyFault = false;
    inputs.turnAbsolutePosition =
        Rotation2d.fromRadians(turnEncoder.get() * TurnMotor.turnAbsEncoderPositionFactor)
            .minus(zeroRotation);
    turnPosition = inputs.turnAbsolutePosition;
    inputs.turnPosition =
        Rotation2d.fromRadians(turnNova.getPositionInternal() * TurnMotor.turnEncoderPositionFactor)
            .minus(zeroRotation);
    // deltaAngle = inputs.turnPosition.minus(inputs.turnAbsolutePosition);
    inputs.turnVelocity_radps =
        turnNova.getVelocityInternal() * TurnMotor.turnEncoderVelocityFactor;
    inputs.turnVoltage_V = turnNova.getVoltage();
    inputs.turnStatorCurrent_A = turnNova.getStatorCurrent();
    inputs.turnSupplyCurrent_A = turnNova.getSupplyCurrent();
    // inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions_rad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value * DriveMotor.encoderPositionFactor)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) ->
                    new Rotation2d(value * TurnMotor.turnAbsEncoderPositionFactor)
                        .minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    // Update pids
    // drivePIDProperty.updateIfChanged();
    // turnPIDProperty.updateIfChanged();
  }

  @Override
  public void setDriveVoltage(double voltage_V) {
    driveNova.setVoltage(voltage_V);
  }

  @Override
  public void setTurnVoltage(double voltage_V) {
    turnNova.setVoltage(voltage_V);
  }

  @Override
  public void setNextDriveVelocity(double nextVelocity_radPs) {
    driveNova.setVoltage(
        DriveMotor.realFF.calculateWithVelocities(driveVelocity_radPs, nextVelocity_radPs)
            + DriveMotor.realPID.calculate(driveVelocity_radPs, lastNextVelocity_radPs));
    lastNextVelocity_radPs = nextVelocity_radPs;
    // double ffVolts = Math.copySign(driveKs, nextVelocity_radps) + driveKv * nextVelocity_radps;
    // driveNova.setVelocityInternal(nextVelocity_radps, ffVolts);
  }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    driveNova.setVoltage(
        DriveMotor.realFF.calculateWithVelocities(
                nextVelocity_radPs - 0.02 * nextAcceleration_radPs2, nextVelocity_radPs)
            + DriveMotor.realPID.calculate(driveVelocity_radPs, lastNextVelocity_radPs));
    lastNextVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextTurnPosition(Rotation2d rotation) {
    turnNova.setVoltage(
        TurnMotor.realPID.calculate(turnPosition.getRadians(), rotation.getRadians()));
  }
}
