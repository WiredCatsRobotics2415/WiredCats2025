package frc.robot;

import frc.subsystems.drive.CommandSwerveDrivetrain;

public class RobotContainer {
    private static RobotContainer instance;
    private OI oi;

    private CommandSwerveDrivetrain drive;

    private RobotContainer() {
        drive = CommandSwerveDrivetrain.getInstance();
        oi = OI.getInstance();
    }

    public void teleopInit() {
        drive.setDefaultCommand(drive.applyRequest(() -> {
            double[] input = oi.getXY();
            return drive.drive.withVelocityX(input[0]).withVelocityY(input[1]).withRotationalRate(oi.getRotation());
        }));
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }
}
