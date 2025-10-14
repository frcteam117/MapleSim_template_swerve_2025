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

import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.HeavyDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.MotorType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/**
 * Module IO implementation for Thrifty Nova drive motor controller, Thrifty Nova turn motor controller, and Thrifty
 * absolute encoder.
 */
public class ModuleIONova implements ModuleIO {
    private final Rotation2d zeroRotation;
    private Rotation2d deltaAngle;

    // Editable pids
    private final PidProperty drivePIDProperty;
    private final PidProperty turnPIDProperty;

    // Hardware objects
    private final ThriftyNova driveNova;
    private final ThriftyNova turnNova;
    private final AnalogEncoder turnEncoder;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIONova(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();};
        driveNova = new ThriftyNova(
                switch (module) {
                    case 0 -> frontLeftDriveCanId;
                    case 1 -> frontRightDriveCanId;
                    case 2 -> backLeftDriveCanId;
                    case 3 -> backRightDriveCanId;
                    default -> 0;
                },
                MotorType.NEO);
        turnNova = new ThriftyNova(
                switch (module) {
                    case 0 -> frontLeftTurnCanId;
                    case 1 -> frontRightTurnCanId;
                    case 2 -> backLeftTurnCanId;
                    case 3 -> backRightTurnCanId;
                    default -> 0;
                },
                MotorType.NEO);
        turnEncoder = new AnalogEncoder(
                switch (module) {
                    case 0 -> frontLeftEncoderPort;
                    case 1 -> frontRightEncoderPort;
                    case 2 -> backLeftEncoderPort;
                    case 3 -> backRightEncoderPort;
                    default -> 0;
                });

        // Configure drive motor
        // NovaConfig driveConfig = new NovaConfig();
        // driveConfig.setBrakeMode(BrakeMode.BRAKE).setVoltageCompensation(12.0).setUsedPIDSlot(PIDSlot.SLOT0);
        // driveConfig.limits.setMaxStatorCurrent(driveMotorCurrentLimit).setMaxSupplyCurrent(driveMotorCurrentLimit);
        // driveConfig.encoder.setUsedEncoder(EncoderType.INTERNAL);
        // driveConfig.pid0.setPIDF(driveKp, 0.0, driveKd, 0.0);
        // driveConfig
        //         .canFreq
        //         .setSensorPeriod(1 / odometryFrequency)
        //         .setControlPeriod(0.02)
        //         .setCurrentPeriod(0.02)
        //         .setFaultPeriod(0.02);
        // driveConfig.configure(driveNova);

        driveNova.factoryReset();
        driveNova
                .setBrakeMode(true)
                .setVoltageCompensation(12.0)
                .usePIDSlot(PIDSlot.SLOT0)
                .setMaxCurrent(CurrentType.STATOR, driveMotorCurrentLimit)
                .setMaxCurrent(CurrentType.SUPPLY, driveMotorCurrentLimit)
                .useEncoderType(EncoderType.INTERNAL)
                .pid0
                .setP(driveKp)
                .setI(0.0)
                .setD(driveKd)
                .setFF(0.0);
        driveNova
                .canFreq
                .setSensor(1 / odometryFrequency)
                .setControl(0.02)
                .setCurrent(0.02)
                .setFault(0.02);
        
        // Configure turn motor
        // NovaConfig turnConfig = new NovaConfig();
        // turnConfig
        //         .setInversion(turnInverted)
        //         .setBrakeMode(BrakeMode.BRAKE)
        //         .setVoltageCompensation(12.0)
        //         .setUsedPIDSlot(PIDSlot.SLOT0);
        // turnConfig.limits.setMaxStatorCurrent(turnMotorCurrentLimit).setMaxSupplyCurrent(turnMotorCurrentLimit);
        // turnConfig.encoder.setUsedEncoder(EncoderType.INTERNAL);//.setAbsoluteWrapping(true)
        // turnConfig.pid0.setPIDF(turnKp, 0.0, turnKd, 0.0);
        // turnConfig
        //         .canFreq
        //         .setSensorPeriod(1 / odometryFrequency)
        //         .setControlPeriod(0.02)
        //         .setCurrentPeriod(0.02)
        //         .setFaultPeriod(0.02);
        // turnConfig.configure(turnNova);

        turnNova.factoryReset();
        turnNova.setInversion(turnInverted)
                .setBrakeMode(true)
                .setVoltageCompensation(12.0)
                .usePIDSlot(PIDSlot.SLOT0)
                .setMaxCurrent(CurrentType.STATOR, turnMotorCurrentLimit)
                .setMaxCurrent(CurrentType.SUPPLY, turnMotorCurrentLimit)
                .useEncoderType(EncoderType.INTERNAL)
                .pid0
                .setP(turnKp)
                .setI(0.0)
                .setD(turnKd)
                .setFF(0.0);
        turnNova.canFreq
                .setSensor(1 / odometryFrequency)
                .setControl(0.02)
                .setCurrent(0.02)
                .setFault(0.02);

        // Create odometry queues
        timestampQueue = NovaOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = NovaOdometryThread.getInstance().registerSignal(driveNova::getPositionInternal);
        turnPositionQueue = NovaOdometryThread.getInstance().registerSignal(turnEncoder::get);

        // Create editable pid values
        List<HeavyDoubleProperty> turnPIDProperties = new ArrayList<>();
        turnPIDProperties.add(new HeavyDoubleProperty(
                (p) -> turnNova.pid0.setP(p), new GosDoubleProperty(false, "turnPID/kp", 0.07)));
        turnPIDProperties.add(
                new HeavyDoubleProperty((i) -> turnNova.pid0.setI(i), new GosDoubleProperty(false, "turnPID/ki", 0.0)));
        turnPIDProperties.add(
                new HeavyDoubleProperty((d) -> turnNova.pid0.setD(d), new GosDoubleProperty(false, "turnPID/kd", 0.0)));
        turnPIDProperties.add(new HeavyDoubleProperty(
                (ff) -> turnNova.pid0.setFF(ff), new GosDoubleProperty(false, "turnPID/kff", 0.0)));
        turnPIDProperty = new PidProperty(turnPIDProperties);

        List<HeavyDoubleProperty> drivePIDProperties = new ArrayList<>();
        drivePIDProperties.add(new HeavyDoubleProperty(
                (p) -> driveNova.pid0.setP(p), new GosDoubleProperty(false, "drivePID/kp", 0.07)));
        drivePIDProperties.add(new HeavyDoubleProperty(
                (i) -> driveNova.pid0.setI(i), new GosDoubleProperty(false, "drivePID/ki", 0.0)));
        drivePIDProperties.add(new HeavyDoubleProperty(
                (d) -> driveNova.pid0.setD(d), new GosDoubleProperty(false, "drivePID/kd", 0.0)));
        drivePIDProperties.add(new HeavyDoubleProperty(
                (ff) -> driveNova.pid0.setFF(ff),
                new GosDoubleProperty(false, "drivePID/kff", 1.0 / driveEncoderPositionFactor * 4)));
        drivePIDProperty = new PidProperty(drivePIDProperties);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.drivePositionRad = driveNova.getPositionInternal() * driveEncoderPositionFactor;
        inputs.driveVelocityRadPerSec = driveNova.getVelocityInternal() * driveEncoderVelocityFactor;
        inputs.driveAppliedVolts = driveNova.getVoltage();
        inputs.driveStatorCurrentAmps = driveNova.getStatorCurrent();
        inputs.driveSupplyCurrentAmps = driveNova.getSupplyCurrent();
        // inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        // sparkStickyFault = false;
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnEncoder.get() * turnAbsEncoderPositionFactor)
                .minus(zeroRotation);
        inputs.turnPosition = Rotation2d.fromRadians(turnNova.getPositionInternal() * turnEncoderPositionFactor)
                .minus(zeroRotation);
        deltaAngle = inputs.turnPosition.minus(inputs.turnAbsolutePosition);
        inputs.turnVelocityRadPerSec = turnNova.getVelocityInternal() * turnEncoderVelocityFactor;
        inputs.turnAppliedVolts = turnNova.getVoltage();
        inputs.turnStatorCurrentAmps = turnNova.getStatorCurrent();
        inputs.turnSupplyCurrentAmps = turnNova.getSupplyCurrent();
        // inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble((Double value) -> value * driveEncoderPositionFactor)
                .toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value * turnAbsEncoderPositionFactor).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        // Update pids
        drivePIDProperty.updateIfChanged();
        turnPIDProperty.updateIfChanged();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveNova.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnNova.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = Math.copySign(driveKs, velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveNova.setVelocityInternal(velocityRadPerSec, ffVolts);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                (MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput)
                                + deltaAngle.getRadians())
                        / turnEncoderPositionFactor;
        turnNova.setPositionInternal(setpoint);
    }
}
