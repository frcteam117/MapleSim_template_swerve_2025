package frc.robot.util.logging;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableDouble implements DoubleSupplier {
  private final String key;
  private final DoubleConsumer onChange;
  private double value;
  private LoggedNetworkNumber networkNumber = null;

  public TunableDouble(String key, double defaultValue, DoubleConsumer onChange) {
    this.key = key;
    this.onChange = onChange;
    value = defaultValue;
  }

  public TunableDouble(String key, double defaultValue) {
    this(key, defaultValue, (value) -> {});
  }

  public void update(boolean publish) {
    if (publish) {
      if (networkNumber == null) {
        networkNumber = new LoggedNetworkNumber(key, value);
      }
      if (value != networkNumber.get()) {
        value = networkNumber.get();
        onChange.accept(value);
      }
    } else {
      if (networkNumber != null) {
        networkNumber = null;
      }
    }
  }

  @Override
  public double getAsDouble() {
    return value;
  }
}
