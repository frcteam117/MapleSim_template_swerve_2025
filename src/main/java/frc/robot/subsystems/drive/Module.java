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

import static frc.robot.Constants.robotPeriod_s;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.DriveConstants.TurnMotor;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private TrapezoidProfile turnProfile = new TrapezoidProfile(TurnMotor.profileConstraints);
  private double lastTurnAngle_rad = 0.0;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositions_rad[i] * wheelRadius_m.getAsDouble();
      double angle_rad = inputs.odometryTurnPositions_rad[i];
      odometryPositions[i] =
          new SwerveModulePosition(positionMeters, Rotation2d.fromRadians(angle_rad));
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(Rotation2d.fromRadians(getAngle()));
    state.cosineScale(Rotation2d.fromRadians(inputs.turnPosition_rad));

    // Apply setpoints
    io.setNextDriveVelocity(state.speedMetersPerSecond / wheelRadius_m.getAsDouble());
    TrapezoidProfile.State turnState =
        turnProfile.calculate(
            robotPeriod_s,
            new TrapezoidProfile.State(inputs.turnPosition_rad, inputs.turnVelocity_radPs),
            new TrapezoidProfile.State(state.angle.getRadians(), 0.0));
    io.setNextTurnState(turnState.position, turnState.velocity);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void setNextState(SwerveModuleState state, double acceleration_mPs2) {
    // Optimize velocity setpoint
    double acceleration_radPs2 =
        Math.cos(state.angle.getRadians() - inputs.turnPosition_rad)
            * acceleration_mPs2
            / wheelRadius_m.getAsDouble();
    state.optimize(Rotation2d.fromRadians(inputs.turnPosition_rad));
    state.cosineScale(Rotation2d.fromRadians(inputs.turnPosition_rad));

    // Apply setpoints
    io.setNextDriveState(
        state.speedMetersPerSecond / wheelRadius_m.getAsDouble(), acceleration_radPs2);

    // TrapezoidProfile.State turnState =
    // turnProfile.calculate(
    //     robotPeriod_s,
    //     new TrapezoidProfile.State(inputs.turnPosition_rad, inputs.turnVelocity_radPs),
    //     new TrapezoidProfile.State(state.angle.getRadians(), 0.0));
    io.setNextTurnState(
        state.angle.getRadians(), (state.angle.getRadians() - lastTurnAngle_rad) / robotPeriod_s);
    lastTurnAngle_rad = state.angle.getRadians();
    // lastTurnVelocity_radPs = (state.angle.getRadians() - lastTurnAngle_rad) / robotPeriod_s
    // io.setDriveVoltage(.0);
    // io.setTurnVoltage(.1);
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double voltage_V) {
    // io.setDriveVoltage(voltage_V);
    // io.setNextTurnPosition(0.0);
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(voltage_V);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }

  /** Returns the current turn angle of the module in radians. */
  public double getAngle() {
    return inputs.turnAbsolutePosition_rad;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePosition_rad * wheelRadius_m.getAsDouble();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocity_radPs * wheelRadius_m.getAsDouble();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRadians(getAngle()));
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRadians(getAngle()));
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition_rad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocity_radPs;
  }

  public ModuleIOInputsAutoLogged getInputs() {
    return inputs;
  }

  public void setNextTurnState(double turnPosition_rad, double turnVelocity_radPs) {
    io.setNextTurnState(turnPosition_rad, turnVelocity_radPs);
    Logger.recordOutput("TurnSetpoints/module_" + index + "/potition_rad", turnPosition_rad);
    Logger.recordOutput("TurnSetpoints/module_" + index + "/velocity_radPs", turnVelocity_radPs);
  }
}
