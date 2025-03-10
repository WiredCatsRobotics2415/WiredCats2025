package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;
import frc.subsystems.leds.LEDStrip;
import lombok.Getter;

public class RobotStatus {
    private static LEDStrip leds = LEDStrip.getInstance();

    public static final Command flashingGreen = leds.flash(UseableColor.Green, Seconds.of(0.25), Seconds.of(0.25));

    @Getter private static RobotState robotState;

    public enum RobotState {
        Disabled(leds.set(UseableColor.BreathingGreen)), Enabled(leds.set(UseableColor.ChasingGreen)),
        AligningToHPS(leds.set(UseableColor.Pink)), AutoGroundIntaking(leds.set(UseableColor.Pink)),
        AligningToDeAlgae(leds.set(UseableColor.Pink)), Stow(leds.set(UseableColor.White)),
        AligningToScoreCoral(leds.set(UseableColor.Pink)), AligningToBarge(leds.set(UseableColor.Pink)),
        WaitingForCoralAtHPS(leds.set(UseableColor.Red)), WaitingToDeAlgae(leds.set(UseableColor.Red)),
        WaitingToScoreCoral(leds.set(UseableColor.Red)), WaitingToBargeAlgae(leds.set(UseableColor.Red));

        public Command ledStripCommand;

        private RobotState(Command ledStripCommand) {
            this.ledStripCommand = ledStripCommand;
        }
    }

    public static void setRobotState(RobotState newState) {
        robotState = newState;
        leds.setDefaultCommand(newState.ledStripCommand);
    }

    public static Command setRobotStateOnce(RobotState newState) {
        return new InstantCommand(() -> setRobotState(newState));
    }

    public static Command keepStateUntilInterrupted(RobotState newState) {
        return new RepeatCommand(setRobotStateOnce(newState));
    }
}
