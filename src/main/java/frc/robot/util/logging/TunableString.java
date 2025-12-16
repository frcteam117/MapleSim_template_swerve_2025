package frc.robot.util.logging;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class TunableString implements Supplier<String> {
  private final String key;
  private final Consumer<String> onChange;
  private String value;
  private LoggedNetworkString networkString = null;

  public TunableString(String key, String defaultValue, Consumer<String> onChange) {
    this.key = key;
    this.onChange = onChange;
    value = defaultValue;
  }

  public TunableString(String key, String defaultValue) {
    this(key, defaultValue, (value) -> {});
  }

  public void update(boolean publish) {
    if (publish) {
      if (networkString == null) {
        networkString = new LoggedNetworkString(key, value);
      }
      if (value != networkString.get()) {
        value = networkString.get();
        onChange.accept(value);
      }
    } else {
      if (networkString != null) {
        networkString = null;
      }
    }
  }

  @Override
  public String get() {
    return value;
  }
}
