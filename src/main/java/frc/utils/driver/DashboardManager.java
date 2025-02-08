package frc.utils.driver;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.driver.DashboardManager.LayoutConstants.LayoutInfo;
import java.util.function.BooleanSupplier;

public class DashboardManager {
    public static class LayoutConstants {
        public static final record LayoutInfo(int x, int y, int width, int height) {};

        public static final LayoutInfo MatchTime = new LayoutInfo(0, 0, 3, 2);
        public static final LayoutInfo IntakeStatus = new LayoutInfo(0, 2, 3, 2);

        public static final LayoutInfo Alerts = new LayoutInfo(7, 0, 3, 6);

        public static final LayoutInfo AutoSelector = new LayoutInfo(0, 0, 2, 1);

        //Added for arm
        public static final LayoutInfo CoastCommand = new LayoutInfo(3, 4, 2, 2);
    }

    private ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    private static DashboardManager instance;

    private DashboardManager() {
        teleopTab.addNumber("Match Time", () -> DriverStation.getMatchTime()).withWidget("Match Time")
            .withPosition(LayoutConstants.MatchTime.x, LayoutConstants.MatchTime.y)
            .withSize(LayoutConstants.MatchTime.width, LayoutConstants.MatchTime.height);
    }

    public static DashboardManager getInstance() {
        if (instance == null) instance = new DashboardManager();
        return instance;
    }

    public void addBoolSupplier(boolean onTeleop, String title, BooleanSupplier supplier, LayoutInfo layoutInfo) {
        ShuffleboardTab tab = onTeleop ? teleopTab : autoTab;
        tab.addBoolean(title, supplier).withWidget("Boolean Box").withPosition(layoutInfo.x, layoutInfo.y)
            .withSize(layoutInfo.width, layoutInfo.height);
    }

    public void addCommand(boolean onTeleop, String title, Command command, LayoutInfo layoutInfo) {
        ShuffleboardTab tab = onTeleop ? teleopTab : autoTab;
        tab.add(title, command).withWidget("Command").withPosition(layoutInfo.x, layoutInfo.y)
            .withSize(layoutInfo.width, layoutInfo.height);
    }

    public void addChooser(boolean onTeleop, String title, SendableChooser chooser, LayoutInfo layoutInfo) {
        ShuffleboardTab tab = onTeleop ? teleopTab : autoTab;
        tab.add(title, chooser).withWidget("ComboBox Chooser").withPosition(layoutInfo.x, layoutInfo.y)
            .withSize(layoutInfo.width, layoutInfo.height);
    }

    public void addAlertGroup(String title, Sendable sendable, LayoutInfo layoutInfo) {
        teleopTab.add(title, sendable).withWidget("Alerts").withPosition(layoutInfo.x, layoutInfo.y)
            .withSize(layoutInfo.width, layoutInfo.height);
    }
}
