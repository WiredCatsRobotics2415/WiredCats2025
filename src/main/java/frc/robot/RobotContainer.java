package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.AutoNoteDetect;
import frc.commands.FixAll;
import frc.commands.ShootingCommands;
import frc.constants.Controls;
import frc.constants.TunerConstants;
import frc.robot.OIs.Bindings;
import frc.robot.OIs.OI;
import frc.subsystems.arm.Arm;
import frc.subsystems.claw.Claw;
import frc.subsystems.drive.SwerveDrive;
import frc.subsystems.flywheel.Flywheel;
import frc.subsystems.intake.Intake;
import frc.util.driver.DashboardManager;
import frc.util.driver.DashboardManager.LayoutConstants;
import frc.util.driver.DriverFeedback;
import lombok.Getter;

public class RobotContainer {
    // Subsystems
    private final SwerveDrive swerveDrive = TunerConstants.DriveTrain;
    private final Arm arm = Arm.getInstance();
    private final Flywheel flywheel = Flywheel.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Claw claw = Claw.getInstance();

    @Getter private OI selectedOI;

    private SendableChooser<Command> autoChooser;

    private static RobotContainer instance;

    private RobotContainer() {
        setupAuto();
        neutralizeSubsystems();
        configureStartupTriggers();
    }

    public static RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();
        return instance;
    }

    /** Schedules flywheel and intake off, sets arm goal to current position */
    public void neutralizeSubsystems() {
        flywheel.off().schedule();
        intake.off().schedule();
        arm.brake();
        arm.setGoal(0);
        swerveDrive.reset();
    }

    /** Triggers intended to be run on startup */
    private void configureStartupTriggers() {
        new Trigger(RobotController::getUserButton)
            .onTrue(new InstantCommand(() -> arm.coast(), arm).ignoringDisable(true));
    }

    private void setupAuto() {
        // Autonomous named commands
        NamedCommands.registerCommand("Intake", intake.intakeAndWaitForNote());
        NamedCommands.registerCommand("ShootSub", ShootingCommands.subwooferAuto());
        NamedCommands.registerCommand("ShootWhileMoving",
            ShootingCommands.shootWhileMoving());
        NamedCommands.registerCommand("ShootSlap", claw.fire());
        NamedCommands.registerCommand("FlywheelOn",
            flywheel.on(Controls.SubwooferShot.LeftFlywheelSpeed,
                Controls.SubwooferShot.RightFlywheelSpeed)); // Shoot next to subwoofer.
        NamedCommands.registerCommand("FlywheelOff", flywheel.off());
        NamedCommands.registerCommand("Amp",
            new InstantCommand(() -> arm.setGoal(Controls.AmpShot.ArmAngle)));
        NamedCommands.registerCommand("ShootMiddle", ShootingCommands.shootMiddle());
        NamedCommands.registerCommand("ShootBottom", ShootingCommands.shootBottom());
        NamedCommands.registerCommand("shootTop", ShootingCommands.shootTop());
        NamedCommands.registerCommand("ArmDown", new InstantCommand(() -> arm.setGoal(0)));
        NamedCommands.registerCommand("ArmUp",
            new InstantCommand(() -> arm.setGoal(Controls.FieldShotAngles.Bottom)));
        NamedCommands.registerCommand("ShootMiddleCorner",
            ShootingCommands.shootMiddleCorner());
        NamedCommands.registerCommand("ShootSubNoFly", ShootingCommands.shootSubNoFly());
        // NamedCommands.registerCommand("AutoAlign", new AutoNoteDetect());

        // Configure auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("Top_Slap");
        DashboardManager.getInstance().addChooser(false, "Auto", autoChooser,
            LayoutConstants.AutoSelector);
    }

    /**
     * Prepares the robot for teleoperated control. Gets the OI selected, configures all
     * binds, and calls any teleopInit methods on subsystems. CLEARS ALL ROBOT BUTTON
     * EVENTLOOP BINDS
     */
    public void teleopInit() {
        neutralizeSubsystems();
        prepareOI();
        configureButtonBindings();
        configureTriggers();
    }

    /**
     * Sets the selected OI variable to the smartdashboard selected OI. Intended to be run
     * in teleopInit.
     */
    private void prepareOI() {
        switch (OI.oiChooser.getSelected()) {
            case 0:
                selectedOI = new OIs.GulikitController();
                break;
            default:
                selectedOI = new OIs.GulikitController();
                break;
        }
    }

    /**
     * Adds all Commands to the Triggers in the selectedOI's binds map, clears previous
     * binds Intended to be run in teleopInit.
     */
    private void configureButtonBindings() {
        Robot.buttonEventLoop.clear();

        // Swerve
        swerveDrive.setDefaultCommand(swerveDrive.applyRequest(() -> {
            double[] input = selectedOI.getXY();
            return swerveDrive.drive.withVelocityX(-input[0] * Controls.MaxDriveMeterS)
                .withVelocityY(-input[1] * Controls.MaxDriveMeterS)
                .withRotationalRate(-selectedOI.getRotation() * Controls.MaxAngularRadS);
        }));
        selectedOI.binds.get(Bindings.PigeonReset).onTrue(new InstantCommand(() -> {
            swerveDrive.seedFieldRelative();
        }, swerveDrive));

        // Intake
        selectedOI.binds.get(Bindings.Intake).onTrue(intake.toggleIntake());
        // selectedOI.binds.get("Intake").onTrue(intake.intakeNote());
        selectedOI.binds.get(Bindings.ManualOuttake).onTrue(intake.out())
            .onFalse(intake.off());
        // selectedOI.binds.get("ManualIntake").onTrue(intake.in()).onFalse(intake.off());

        // Arm manual
        selectedOI.binds.get(Bindings.RaiseArm).whileTrue(arm.increaseGoal());
        selectedOI.binds.get(Bindings.LowerArm).whileTrue(arm.decreaseGoal());

        // Fire
        // selectedOI.binds.get("Shoot").onTrue(new ConditionalCommand(claw.fire(),
        // new InstantCommand(() -> {}), flywheel::withinSetGoal));
        selectedOI.binds.get(Bindings.Shoot).onTrue(claw.fire());
        // selectedOI.binds.get("ReverseClaw").whileTrue(new
        // RepeatCommand(claw.reverse()));
        selectedOI.binds.get(Bindings.ReverseClaw).onTrue(claw.reverse());

        // Flywheel
        // TODO: change call to onFromSmartDashboard to a call to on(flwyheelSppeds)
        selectedOI.binds.get(Bindings.ShootClose)
            .onTrue(flywheel.on(Controls.SubwooferShot.LeftFlywheelSpeed,
                Controls.SubwooferShot.RightFlywheelSpeed));
        selectedOI.binds.get(Bindings.SpinOff).onTrue(flywheel.off());
        selectedOI.binds.get(Bindings.SpinUpToAmp).onTrue(flywheel
            .on(Controls.AmpShot.LeftFlywheelSpeed, Controls.AmpShot.RightFlywheelSpeed));
        selectedOI.binds.get(Bindings.FixAll).whileTrue(new FixAll());
        // selectedOI.binds.get(Bindings.ArmAngle).onTrue(arm.moveToShotAngle());

        // Climber
        // selectedOI.binds.get("LeftClimberDown").onTrue(
        // climber.manualDown(Constants.Climber.ClimberSpeed, 0));
        // selectedOI.binds.get("LeftClimberUp").onTrue(
        // climber.manualUp(Constants.Climber.ClimberSpeed, 0));
        // selectedOI.binds.get("RightClimberDown").onTrue(
        // climber.manualDown(0, Constants.Climber.ClimberSpeed));
        // selectedOI.binds.get("RightClimberUp").onTrue(
        // climber.manualUp(0, Constants.Climber.ClimberSpeed));

        /*
         * selectedOI.binds.get("LeftClimberDown").onTrue(
         * climber.manualDown(Constants.Climber.ClimberSpeed, 0));
         * selectedOI.binds.get("LeftClimberUp").onTrue(
         * climber.manualUp(Constants.Climber.ClimberSpeed, 0));
         */
        // Presets

        selectedOI.binds.get(Bindings.Amp).onTrue(new InstantCommand(() -> {
            arm.setGoal(Controls.AmpShot.ArmAngle);
        }));
        selectedOI.binds.get(Bindings.ArmDrivePreset).onTrue(new InstantCommand(() -> {
            arm.setGoal(5);
        }));
        selectedOI.binds.get(Bindings.ArmIntakePosition).onTrue(new InstantCommand(() -> {
            arm.setGoal(0);
        }));
        // selectedOI.binds.get("ShuttleRotate").onTrue(shooterPre.shuttle());
        // selectedOI.binds.get("ShootClose").onTrue(flywheel.on(6000, 8000)); //
        // Subwoofer
        // selectedOI.binds.get("TargetHotspot").onTrue(new FixAll());

        selectedOI.binds.get(Bindings.AutoIntake).whileTrue(new AutoNoteDetect());

        selectedOI.binds.get(Bindings.FixArm).onTrue(new InstantCommand(() -> {
            arm.resetPotentiometerAndArm();
        }));
    }

    /** Adds all binds to triggers. Intended to be run in teleopInit. */
    private void configureTriggers() {
        new Trigger(intake::hasNote).onTrue(DriverFeedback.blinkInConfirmation());
    }

    public boolean isBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return true;
    }

    /**
     * @return autonomous command to be run from Robot.java
     */
    public Command getAutonomousCommand() {
        String chosenAuto = autoChooser.getSelected().getName();
        return new PathPlannerAuto(chosenAuto);
    }
}
