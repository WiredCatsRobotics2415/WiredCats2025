package frc.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.LEDStripConstants;
import frc.robot.Robot;

public class LEDStrip extends SubsystemBase {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer entireStripBuffer;

    private LEDPattern idlePattern = LEDPattern.solid(LEDStripConstants.WestminsterGreen).breathe(Seconds.of(2));

    public enum RobotStatus {
        Idle, Automating, Teleop
    }

    private static LEDStrip instance;

    private LEDStrip() {
        if (Robot.isSimulation()) return;
        ledStrip = new AddressableLED(LEDStripConstants.PortPWM);
        ledStrip.setLength(LEDStripConstants.Length);
        ledStrip.start();
        entireStripBuffer = new AddressableLEDBuffer(LEDStripConstants.Length);
        setDefaultCommand(runStatusLights(RobotStatus.Idle));
    }

    public static LEDStrip getInstance() {
        if (instance == null) instance = new LEDStrip();
        return instance;
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) return;
        ledStrip.setData(entireStripBuffer);
    }

    public Command runStatusLights(RobotStatus status) {
        return run(() -> {
            if (Robot.isSimulation()) return;
            switch (status) {
                case Idle:
                    idlePattern.applyTo(entireStripBuffer);
                    break;

                default:
                    break;
            }
        });
    }
}
