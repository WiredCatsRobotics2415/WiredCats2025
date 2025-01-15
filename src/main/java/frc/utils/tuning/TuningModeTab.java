package frc.utils.tuning;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

public class TuningModeTab {
    private static TuningModeTab instance;
    private ShuffleboardTab thisTab;

    private int ELASTIC_SIZE_WIDTH = 10;
    private int ELASTIC_SIZE_HEIGHT = 6;

    private TuningModeTab() {
        thisTab = Shuffleboard.getTab("Tuning");
        int currentWidth = 0;
        int currentHeight = 0;

        for (String subsystemName : Characterizer.getCharacterizers().keySet()) {
            Characterizer characterizer = Characterizer.getCharacterizers().get(subsystemName);
            ShuffleboardLayout layout = thisTab.getLayout(subsystemName, BuiltInLayouts.kList)
                .withSize(currentWidth, ELASTIC_SIZE_HEIGHT / 2).withPosition(currentWidth, currentHeight)
                .withProperties(Map.of("Label position", "HIDDEN"));

            for (Command command : characterizer.commands) {
                layout.add(command);
            }

            currentWidth += 2;
            if (currentWidth > ELASTIC_SIZE_WIDTH) {
                currentHeight = 3;
                currentWidth = 0;
            }
        }
    }

    public static void enableTuningMode() {
        if (instance == null) {
            instance = new TuningModeTab();
        }
    }
}
