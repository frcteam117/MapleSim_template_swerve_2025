package frc.robot.util.logging;

import java.util.ArrayList;
import java.util.List;

public class LogUtil {
  private static LogUtil instance = null;
  private final List<Runnable> updateMethods = new ArrayList<>();

  public static LogUtil getInstance() {
    if (instance == null) {
      instance = new LogUtil();
    }
    return instance;
  }

  public void registerUpdateMethod(Runnable updateMethod) {
    updateMethods.add(updateMethod);
  }

  public void runUpdateMethods() {
    for (Runnable runnable : updateMethods) {
      runnable.run();
    }
  }

  public static String toSuffix(String symbol) {
    return "_"
        + symbol
            .replace("u", "µ")
            .replace("*", "·")
            .replace('K', 'k')
            .replace("/", " ̸ ")
            .replace("<?>", "value");
  }
}
