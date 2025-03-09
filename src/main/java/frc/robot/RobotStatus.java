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

    private static Command flashingGreen = leds.flash(UseableColor.Green, Seconds.of(0.25), Seconds.of(0.25));

    @Getter private static RobotState robotState;

    public enum RobotState {
        Disabled(leds.set(UseableColor.BreathingGreen)), Enabled(leds.set(UseableColor.ChasingGreen)),
        AligningToCPS(leds.set(UseableColor.Gray)), AutoGroundIntaking(leds.set(UseableColor.Yellow)),
        AligningToDeAlgae(leds.set(UseableColor.SkyBlue)), ContainingCoral(flashingGreen),
        ContainingAlgaeInEE(flashingGreen), AligningToScoreCoral(leds.set(UseableColor.White)),
        AligningToBarge(leds.set(UseableColor.White)), WaitingForCoralAtCPS(leds.set(UseableColor.Red)),
        WaitingToDeAlgae(leds.set(UseableColor.Orange)), WaitingToScoreCoral(leds.set(UseableColor.Gray)),
        WaitingToBargeAlgae(leds.set(UseableColor.SkyBlue));

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
