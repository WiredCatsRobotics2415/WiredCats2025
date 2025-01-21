package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.elevator.Elevator;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Arm arm = Arm.getInstance();
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
        // Put Auto named commands here
        // autoChooser = AutoBuilder.buildAutoChooser("");
        // DashboardManager.getInstance().addChooser(false, "Auto", autoChooser, LayoutConstants.AutoSelector);
    }

    public void neutralizeSubsystems() {
        drive.setDefaultCommand(Commands.idle(drive));
    }

    private void configureControls() {
        drive.setDefaultCommand(drive.applyRequest(() -> {
            double[] input = oi.getXY();
            return drive.drive.withVelocityX(input[0]).withVelocityY(input[1]).withRotationalRate(oi.getRotation());
        }));

        oi.binds.get(OI.Bind.ManualElevatorUp).whileTrue(elevator.changeGoal(1));
        oi.binds.get(OI.Bind.ManualElevatorDown).whileTrue(elevator.changeGoal(-1));

        oi.binds.get(OI.Bind.ManualArmForward).whileTrue(arm.changeGoal(1));
        oi.binds.get(OI.Bind.ManualArmBack).whileTrue(arm.changeGoal(-1));
    }

    private void configureTriggers() {
        // Triggers that interact across multiple subsystems/utils should be defined here
    }
}
