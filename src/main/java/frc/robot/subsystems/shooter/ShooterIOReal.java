package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ShooterIOReal implements ShooterIO {
  // Sparkmax objects
  private final SparkMax flywheel = new SparkMax(Flywheel.canId, MotorType.kBrushless);
  private final RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  private final SparkMax hood = new SparkMax(Hood.canId, MotorType.kBrushless);
  private final RelativeEncoder hoodEncoder = hood.getEncoder();
  private final SparkMax turret = new SparkMax(Turret.canId, MotorType.kBrushless);
  private final RelativeEncoder turretEncoder = turret.getEncoder();

  // Motion profiling
  private double lastNextVelocity_radPs = 0.0;
  private double currentVelocity_radPs;

  public ShooterIOReal() {
    flywheel.configure(
        Flywheel.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood.configure(
        Hood.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turret.configure(
        Turret.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs ioInputs) {
    ioInputs.mechanismPosition_rad = flywheelEncoder.getPosition();
    ioInputs.mechanismVelocity_radPs = flywheelEncoder.getVelocity();
    currentVelocity_radPs = ioInputs.mechanismVelocity_radPs;

    ioInputs.motorVoltage_V = spark.getBusVoltage() * spark.getAppliedOutput();
    ioInputs.statorCurrent_A = spark.getOutputCurrent();
    ioInputs.supplyCurrent_A = ioInputs.statorCurrent_A * ioInputs.motorVoltage_V / 12.0;
  }

  @Override
  public void setVoltage(double voltage_V) {
    spark.setVoltage(voltage_V);
    lastNextVelocity_radPs = currentVelocity_radPs;
  }

  @Override
  public void setNextVelocity(double nextVelocity_radPs) {
    spark.setVoltage(realFF.calculateWithVelocities(currentVelocity_radPs, nextVelocity_radPs)
        + realPID.calculate(currentVelocity_radPs, lastNextVelocity_radPs));
    lastNextVelocity_radPs = nextVelocity_radPs;
  }
}
