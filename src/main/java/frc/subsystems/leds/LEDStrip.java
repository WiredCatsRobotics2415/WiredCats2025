package frc.subsystems.leds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;
import frc.robot.Robot;
import frc.utils.Util;
import lombok.Getter;

public class LEDStrip extends SubsystemBase {
    private LEDStripIO io;
    private static LEDStrip instance;

    @Getter private UseableColor currentUseable = UseableColor.BreathingGreen;
    @Getter private Color currentColor = currentUseable.color;

    private LEDStrip() {
        io = (LEDStripIO) Util.getIOImplementation(LEDStripIOBlinkin.class, LEDStripIOSim.class, new LEDStripIO() {});
    }

    public static LEDStrip getInstance() {
        if (instance == null) instance = new LEDStrip();
        return instance;
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) return;
    }

    public void setColor(UseableColor color) {
        if (!color.equals(currentUseable)) {
            currentColor = color.color;
            io.setColor(color);
        }
    }

    public Command set(UseableColor color) {
        return run(() -> setColor(color));
    }

    public Command flash(UseableColor color, Time offFor, Time onFor) {
        return new RepeatCommand(runOnce(() -> {
            setColor(color);
        }).andThen(new WaitCommand(offFor)).andThen(runOnce(() -> {
            setColor(UseableColor.Black);
        }).andThen(new WaitCommand(onFor))));
    }
}
