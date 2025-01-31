package frc.utils.tuning;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.RuntimeConstants;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class TuningModeTab {
    private static TuningModeTab instance;
    private ShuffleboardTab thisTab;

    private final int ElasticSizeWidth = 10;
    private final int ElasticSizeHeight = 6;

    private int currentWidth = 1;
    private int currentHeight = 1;

    private TuningModeTab() {
        thisTab = Shuffleboard.getTab("Tuning");

        for (String subsystemName : Characterizer.getCharacterizers().keySet()) {
            Characterizer characterizer = Characterizer.getCharacterizers().get(subsystemName);
            ShuffleboardLayout layout = thisTab.getLayout(subsystemName, BuiltInLayouts.kList)
                .withSize(2, ElasticSizeHeight / 2).withPosition(currentWidth, currentHeight)
                .withProperties(Map.of("Label position", "HIDDEN"));

            for (Command command : characterizer.commands) {
                layout.add(command);
            }

            currentWidth += 2;
            if (currentWidth > ElasticSizeWidth) {
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

    public static TuningModeTab getInstance() {
        if (instance == null && RuntimeConstants.TuningMode) instance = new TuningModeTab();
        return instance;
    }

    public void addBoolSupplier(String title, BooleanSupplier supplier) {
        if (currentWidth == ElasticSizeWidth) {
            currentHeight += 1;
            currentWidth = 0;
        }
        currentWidth = currentWidth + 1;
        thisTab.addBoolean(title, supplier).withWidget("Boolean Box").withPosition(currentWidth, currentHeight);
    }

    public void addCommand(String title, Command command) {
        if (currentWidth == ElasticSizeWidth) {
            currentHeight += 1;
            currentWidth = 0;
        }
        currentWidth = currentWidth + 1;
        thisTab.add(title, command).withWidget("Command");
        System.out.println("added command at " + currentWidth + " " + currentHeight);
    }
}
