package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.driver.DashboardManager;
import frc.utils.driver.DashboardManager.LayoutConstants;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private OI oi = OI.getInstance();

    private SendableChooser<Command> autoChooser;

    private RobotContainer() {
        setupAuto();
        neutralizeSubsystems();
        configureControls();
        configureTriggers();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void setupAuto() {
        //Put Auto named commands here
        autoChooser = AutoBuilder.buildAutoChooser("");
        DashboardManager.getInstance().addChooser(false, "Auto", autoChooser,
            LayoutConstants.AutoSelector);
    }

    public void neutralizeSubsystems() {
        drive.setDefaultCommand(Commands.idle());
    }

    private void configureControls() {
        drive.setDefaultCommand(drive.applyRequest(() -> {
            double[] input = oi.getXY();
            return drive.drive.withVelocityX(input[0]).withVelocityY(input[1]).withRotationalRate(oi.getRotation());
        }));
    }

    private void configureTriggers() {
        //Triggers that interact across multiple subsystems/utils should be defined here
    }
}
