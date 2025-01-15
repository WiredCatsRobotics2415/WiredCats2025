package frc.robot;

public class RobotContainer {
    private static RobotContainer instance;

    private RobotContainer() {}

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
