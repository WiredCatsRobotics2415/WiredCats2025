package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import lombok.Getter;

public class AllianceDependent<T> {
    private T blueAlliance;
    private T redAlliance;

    @Getter private static boolean isCurrentlyBlue = true;

    public AllianceDependent(T blueAlliance, T redAlliance) {
        this.blueAlliance = blueAlliance;
        this.redAlliance = redAlliance;
    }

    public T get() {
        return isCurrentlyBlue ? blueAlliance : redAlliance;
    }

    public static void updateCurrentAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isCurrentlyBlue = false;
            }
            if (ally.get() == Alliance.Blue) {
                isCurrentlyBlue = true;
            }
        } else {
            isCurrentlyBlue = true;
        }
    }
}
