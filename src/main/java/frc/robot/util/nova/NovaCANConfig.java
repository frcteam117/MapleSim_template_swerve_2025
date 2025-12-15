package frc.robot.util.nova;

public class NovaCANConfig extends NovaBaseConfig {

  public NovaCANConfig setFaultPeriod(double period_S) {
    putParameter(NovaParameter.FREQ_FAULT.value, period_S);
    return this;
  }

  public NovaCANConfig setSensorPeriod(double period_S) {
    putParameter(NovaParameter.FREQ_SENSOR.value, period_S);
    return this;
  }

  public NovaCANConfig setQuadSensorPeriod(double period_S) {
    putParameter(NovaParameter.FREQ_QUAD_SENSOR.value, period_S);
    return this;
  }

  public NovaCANConfig setControlPeriod(double period_S) {
    putParameter(NovaParameter.FREQ_CONTROL.value, period_S);
    return this;
  }

  public NovaCANConfig setCurrentPeriod(double period_S) {
    putParameter(NovaParameter.FREQ_CURRENT.value, period_S);
    return this;
  }

  public NovaCANConfig apply(NovaCANConfig config) {
    this.apply(config);
    return this;
  }
}
