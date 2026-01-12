package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIdUtil {
  public static enum SysIdType {
    QuasistaticForward,
    QuasistaticReverse,
    DynamicForward,
    DynamicReverse
  }

  public static Command getSysIdCommand(SysIdRoutine routine, SysIdType type) {
    switch (type) {
      case QuasistaticForward -> {
        return routine.quasistatic(Direction.kForward);
      }
      case QuasistaticReverse -> {
        return routine.quasistatic(Direction.kReverse);
      }
      case DynamicForward -> {
        return routine.dynamic(Direction.kForward);
      }
      case DynamicReverse -> {
        return routine.dynamic(Direction.kReverse);
      }
      default -> {
        return Commands.none();
      }
    }
  }
}
