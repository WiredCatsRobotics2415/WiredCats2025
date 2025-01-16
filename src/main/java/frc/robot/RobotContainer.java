package frc.robot;

import frc.subsystems.drive.CommandSwerveDrivetrain;

public class RobotContainer {
    private static RobotContainer instance;

    private CommandSwerveDrivetrain drive;

    private RobotContainer() {
        drive = CommandSwerveDrivetrain.getInstance();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
