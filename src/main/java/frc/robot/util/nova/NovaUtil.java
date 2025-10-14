package frc.robot.util.nova;

import static edu.wpi.first.units.Units.Seconds;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.Error;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;

public class NovaUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(ThriftyNova nova, int maxAttempts, Runnable command, Error... errors) {
        for (int i = 0; i < maxAttempts; i++) {
            boolean containedError = false;
            command.run();
            if (nova.errors.isEmpty()) {
                break;
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                for (Error error : errors) {
                    if (nova.errors.contains(error)) {
                        containedError = true;
                    }
                    nova.errors.remove(error);
                }

                if (containedError == false) {
                    break;
                }
            }
        }
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
