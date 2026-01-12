package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class TurretIOReal implements TurretIO {
  // Sparkmax objects
  private SparkMax spark = new SparkMax(canId, MotorType.kBrushless);
  private RelativeEncoder encoder = spark.getEncoder();

  // Motion profiling
  private double lastNextVelocity_radPs = 0.0;
  private double currentVelocity_radPs;

  public TurretIOReal() {
    spark.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs ioInputs) {
    ioInputs.mechanismPosition_rad = encoder.getPosition();
    ioInputs.mechanismVelocity_radPs = encoder.getVelocity();
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
    spark.setVoltage(
        realFF.calculateWithVelocities(currentVelocity_radPs, nextVelocity_radPs)
            + realPID.calculate(currentVelocity_radPs, lastNextVelocity_radPs));
    lastNextVelocity_radPs = nextVelocity_radPs;
  }
}
