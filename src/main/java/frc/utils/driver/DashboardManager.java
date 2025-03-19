package frc.utils.driver;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class DashboardManager {
    private ShuffleboardTab teleopTab = Shuffleboard.getTab("Teleoperated");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    private static DashboardManager instance;

    private DashboardManager() {
        teleopTab.addNumber("Match Time", () -> DriverStation.getMatchTime()).withWidget("Match Time");
    }

    public static DashboardManager getInstance() {
        if (instance == null) instance = new DashboardManager();
        return instance;
    }

    public void addBoolSupplier(boolean onTeleop, String title, BooleanSupplier supplier) {
        ShuffleboardTab tab = onTeleop ? teleopTab : autoTab;
        tab.addBoolean(title, supplier).withWidget("Boolean Box");
    }

    public void addCommand(boolean onTeleop, String title, Command command) {
        ShuffleboardTab tab = onTeleop ? teleopTab : autoTab;
        tab.add(title, command).withWidget("Command");
    }

    public void addAlertGroup(String title, Sendable sendable) {
        teleopTab.add(title, sendable).withWidget("Alerts");
    }
}
