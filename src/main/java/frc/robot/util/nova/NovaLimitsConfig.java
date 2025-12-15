package frc.robot.util.nova;

public class NovaLimitsConfig extends NovaBaseConfig {

  public NovaLimitsConfig setMaxStatorCurrent(double maxCurrent_A) {
    putParameter(NovaParameter.MAX_STATOR_CURRENT.value, maxCurrent_A);
    return this;
  }

  public NovaLimitsConfig setMaxSupplyCurrent(double maxCurrent_A) {
    putParameter(NovaParameter.MAX_SUPPLY_CURRENT.value, maxCurrent_A);
    return this;
  }

  /**
   * Set the maximum forward and reverse percent output.
   *
   * @param maxFwd Max forward output [0, 1].
   * @param maxRev Max reverse output [0, 1].
   */
  public NovaLimitsConfig setMaxOutput(double maxFwd, double maxRev) {
    putParameter(NovaParameter.MAX_FORWARD.value, maxFwd);
    putParameter(NovaParameter.MAX_REVERSE.value, maxRev);
    return this;
  }

  /**
   * @param rampUp_S the time to ramp up to 100% output in seconds
   */
  public NovaLimitsConfig setRampUp(double rampUp_S) {
    putParameter(NovaParameter.RAMP_UP.value, rampUp_S);
    return this;
  }

  /**
   * @param rampUp_S the time to ramp down to 100% output in seconds
   */
  public NovaLimitsConfig setRampDown(double rampDown_S) {
    putParameter(NovaParameter.RAMP_DOWN.value, rampDown_S);
    return this;
  }

  public NovaLimitsConfig setSoftLimits(double revLimit, double fwdLimit) {
    putParameter(NovaParameter.SOFT_LIMIT_FWD.value, fwdLimit);
    putParameter(NovaParameter.SOFT_LIMIT_REV.value, revLimit);
    return this;
  }

  public NovaLimitsConfig enableSoftLimits(boolean isEnabled) {
    putParameter(NovaParameter.ENABLE_SOFT_LIMIT.value, isEnabled);
    return this;
  }

  public NovaLimitsConfig enableHardLimits(boolean isEnabled) {
    putParameter(NovaParameter.ENABLE_HARD_LIMIT.value, isEnabled);
    return this;
  }

  public NovaLimitsConfig apply(NovaLimitsConfig config) {
    this.apply(config);
    return this;
  }
}
