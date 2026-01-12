package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class TurretIOSim implements TurretIO {
  // Simulator
  private FlywheelSim sim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, .4, reduction), gearbox);
  // new FlywheelSim(
  //     LinearSystemId.identifyVelocitySystem(realFF.getKv(), realFF.getKa()), gearbox);

  // Motion profiling
  private double lastNextVelocity_radPs = 0.0;
  private double currentVelocity_radPs;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs ioInputs) {
    ioInputs.mechanismPosition_rad = 0;
    ioInputs.mechanismVelocity_radPs = sim.getAngularVelocityRadPerSec();
    currentVelocity_radPs = ioInputs.mechanismVelocity_radPs;

    ioInputs.motorVoltage_V = sim.getInputVoltage();
    ioInputs.statorCurrent_A = sim.getCurrentDrawAmps();
    ioInputs.supplyCurrent_A =
        ioInputs.statorCurrent_A * ioInputs.motorVoltage_V / RoboRioSim.getVInVoltage();
  }

  @Override
  public void setVoltage(double voltage_V) {
    sim.setInputVoltage(voltage_V);
    lastNextVelocity_radPs = currentVelocity_radPs;
  }

  @Override
  public void setNextVelocity(double nextVelocity_radPs) {
    sim.setInputVoltage(
        realFF.calculateWithVelocities(currentVelocity_radPs, nextVelocity_radPs)
            + realPID.calculate(currentVelocity_radPs, lastNextVelocity_radPs));
    lastNextVelocity_radPs = nextVelocity_radPs;
  }
}
