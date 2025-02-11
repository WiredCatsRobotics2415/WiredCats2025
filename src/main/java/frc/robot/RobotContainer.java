package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.commands.ScoreCoral;
import frc.commands.ScoreCoral.Level;
import frc.commands.ScoreCoral.Side;
import frc.constants.Controls;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.driver.DashboardManager;
import frc.utils.driver.DashboardManager.LayoutConstants;
import lombok.Getter;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private SuperStructure superstructure = SuperStructure.getInstance();
    private @Getter OI oi = OI.getInstance();

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
        autoChooser = AutoBuilder.buildAutoChooser("");
        DashboardManager.getInstance().addChooser(false, "Auto", autoChooser, LayoutConstants.AutoSelector);
    }

    public void neutralizeSubsystems() {
        drive.setDefaultCommand(Commands.idle(drive));
        elevator.setGoal(Inches.of(0));
    }

    private void configureControls() {
        oi.binds.get(OI.Bind.SeedFieldCentric).onTrue(drive.resetRotationFromLimelightMT1().ignoringDisable(true));

        drive.setDefaultCommand(drive.applyRequest(() -> {
            double[] input = oi.getXY();
            return drive.driveOpenLoopRequest.withVelocityX(-input[1] * Controls.MaxDriveMeterS)
                .withVelocityY(-input[0] * Controls.MaxDriveMeterS)
                .withRotationalRate(-oi.getRotation() * Controls.MaxAngularRadS);
        }));

        oi.binds.get(OI.Bind.ManualElevatorUp).whileTrue(superstructure.changeElevatorGoalBy(Inches.of(1.0)));
        oi.binds.get(OI.Bind.ManualElevatorDown).whileTrue(superstructure.changeElevatorGoalBy(Inches.of(-1.0)));

        oi.binds.get(OI.Bind.ManualArmForward).whileTrue(superstructure.changeArmGoalBy(Degrees.of(1)));
        oi.binds.get(OI.Bind.ManualArmBack).whileTrue(superstructure.changeArmGoalBy(Degrees.of(-1)));

        oi.binds.get(OI.Bind.AutoScoreLeftL1).onTrue(new ScoreCoral(Side.Left, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreLeftL2).onTrue(new ScoreCoral(Side.Left, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreLeftL3).onTrue(new ScoreCoral(Side.Left, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreLeftL4).onTrue(new ScoreCoral(Side.Left, Level.L4));
        oi.binds.get(OI.Bind.AutoScoreRightL1).onTrue(new ScoreCoral(Side.Right, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreRightL2).onTrue(new ScoreCoral(Side.Right, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreRightL3).onTrue(new ScoreCoral(Side.Right, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreRightL4).onTrue(new ScoreCoral(Side.Right, Level.L4));
    }

    private void configureTriggers() {
        // Triggers that interact across multiple subsystems/utils should be defined here
    }

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}
