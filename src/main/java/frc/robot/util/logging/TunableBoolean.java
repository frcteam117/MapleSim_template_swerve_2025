package frc.robot.util.logging;

import edu.wpi.first.util.function.BooleanConsumer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class TunableBoolean implements BooleanSupplier {
  private final String key;
  private final BooleanConsumer onChange;
  private boolean value;
  private LoggedNetworkBoolean networkBoolean = null;

  public TunableBoolean(String key, boolean defaultValue, BooleanConsumer onChange) {
    this.key = key;
    this.onChange = onChange;
    value = defaultValue;
  }

  public TunableBoolean(String key, boolean defaultValue) {
    this(key, defaultValue, (value) -> {});
  }

  public void update(boolean publish) {
    if (publish) {
      if (networkBoolean == null) {
        networkBoolean = new LoggedNetworkBoolean(key, value);
      }
      if (value != networkBoolean.get()) {
        value = networkBoolean.get();
        onChange.accept(value);
      }
    } else {
      if (networkBoolean != null) {
        networkBoolean = null;
      }
    }
  }

  @Override
  public boolean getAsBoolean() {
    return value;
  }
}
