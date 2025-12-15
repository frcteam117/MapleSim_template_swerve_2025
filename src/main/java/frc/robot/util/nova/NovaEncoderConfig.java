package frc.robot.util.nova;

import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.ExternalEncoder;

public class NovaEncoderConfig extends NovaBaseConfig {
  /**
   * @param ticks the desired absolute encoder offset in encoder ticks
   */
  public NovaEncoderConfig setAbsoluteOffset(int ticks) {
    putParameter(NovaParameter.ABS_OFFSET.value, ticks);
    return this;
  }

  public NovaEncoderConfig setAbsoluteWrapping(boolean isEnabled) {
    putParameter(NovaParameter.ABSOLUTE_WRAPPING.value, isEnabled);
    return this;
  }

  public NovaEncoderConfig setExternalEncoder(ExternalEncoder externalEncoder) {
    putParameter(NovaParameter.EXTERNAL_ENCODER.value, externalEncoder);
    return this;
  }

  public NovaEncoderConfig setUsedEncoder(EncoderType encoderType) {
    putParameter(NovaParameter.ENCODER_SELECTED.value, encoderType);
    return this;
  }

  public NovaEncoderConfig setEncoderPosition(double position) {
    putParameter(NovaParameter.ENCODER_POSITION.value, position);
    return this;
  }

  public NovaEncoderConfig apply(NovaEncoderConfig config) {
    this.apply(config);
    return this;
  }
}
