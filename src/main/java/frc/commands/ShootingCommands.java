package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.Controls;
import frc.subsystems.arm.Arm;
import frc.subsystems.claw.Claw;
import frc.subsystems.flywheel.Flywheel;

public class ShootingCommands {
    /** Fire next to subwoofer */
    public static final Command shootClose() {
        return new ParallelCommandGroup(
            new InstantCommand(
                () -> Arm.getInstance().setGoal(Controls.SubwooferShot.ArmAngle)),
            Flywheel.getInstance().on(Controls.SubwooferShot.LeftFlywheelSpeed,
                Controls.SubwooferShot.RightFlywheelSpeed));
    }

    /**
     * Fire next to subwoofer and then stop the Flywheel.getInstance() during autonomous.
     */
    public static final Command subwooferAuto() {
        return new SequentialCommandGroup(shootClose(), new WaitCommand(2),
            // new WaitUntilCommand(() -> Flywheel.getInstance().withinSetGoal()),
            Claw.getInstance().fire(), new WaitCommand(0.5), Flywheel.getInstance().off());
    }

    public static final Command shootWhileMoving() {
        return new SequentialCommandGroup(Claw.getInstance().fire(), new WaitCommand(0.5),
            Flywheel.getInstance().off());
    }

    public static final Command shootSubNoFly() {
        return new SequentialCommandGroup(Claw.getInstance().fire(), new WaitCommand(0.5));
    }

    public static final Command shootMiddle() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> Arm.getInstance().setGoal(Controls.FieldShotAngles.MiddleCenter)),
            new WaitUntilCommand(() -> Arm.getInstance().withinSetGoalTolerance()),
            Claw.getInstance().fire(), new WaitCommand(0.5));
    }

    public static final Command shootMiddleCorner() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> Arm.getInstance().setGoal(Controls.FieldShotAngles.MiddleCorner)),
            new WaitUntilCommand(() -> Arm.getInstance().withinSetGoalTolerance()),
            Claw.getInstance().fire(), new WaitCommand(0.5));
    }

    public static final Command shootTop() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> Arm.getInstance().setGoal(Controls.FieldShotAngles.Top)),
            Claw.getInstance().fire(), new WaitCommand(0.5));
    }

    public static final Command shootBottom() {
        return new SequentialCommandGroup(
            // new InstantCommand(() -> Arm.getInstance().setGoal(Settings.field.bottom)),
            Claw.getInstance().fire(), new WaitCommand(0.5));
    }
}
