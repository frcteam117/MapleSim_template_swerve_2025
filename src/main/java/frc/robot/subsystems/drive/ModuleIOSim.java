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

import frc.robot.subsystems.drive.DriveConstants.DriveMotor;
import frc.robot.subsystems.drive.DriveConstants.TurnMotor;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double nextVelocity_radPs = 0.0;
  private double rotationSetpoint_rad = 0.0;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(DriveMotor.config.limits.getMaxStatorCurrent()));
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(TurnMotor.config.limits.getMaxSupplyCurrent()));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + DriveMotor.simPID.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
                  nextVelocity_radPs);
    } else {
      DriveMotor.simPID.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          TurnMotor.simPID.calculate(
              moduleSimulation.getSteerAbsoluteFacing().getRadians(), rotationSetpoint_rad);
    } else {
      TurnMotor.simPID.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition_rad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocity_radps = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveVoltage_V = driveAppliedVolts;
    inputs.driveStatorCurrent_A = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    inputs.driveSupplyCurrent_A = Math.abs(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition_rad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
    inputs.turnVelocity_radPs =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnVoltage_V = turnAppliedVolts;
    inputs.turnStatorCurrent_A = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    inputs.turnSupplyCurrent_A = Math.abs(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositions_rad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions_rad =
        Arrays.stream(moduleSimulation.getCachedSteerAbsolutePositions())
            .mapToDouble((rotation) -> rotation.getRadians())
            .toArray();
  }

  @Override
  public void setDriveVoltage(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnVoltage(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setNextDriveVelocity(double nextVelocity_radPs) {
    driveClosedLoop = true;
    driveFFVolts = DriveMotor.simFF.calculate(nextVelocity_radPs);
    this.nextVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextDriveState(double nextVelocity_radPs, double nextAcceleration_radPs2) {
    driveClosedLoop = true;
    driveFFVolts = DriveMotor.simFF.calculate(nextVelocity_radPs, nextAcceleration_radPs2);
    this.nextVelocity_radPs = nextVelocity_radPs;
  }

  @Override
  public void setNextTurnPosition(double rotation_rad) {
    turnClosedLoop = true;
    rotationSetpoint_rad = rotation_rad;
  }
}
