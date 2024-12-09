package frc.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.FlywheelConstants;
import frc.util.Utils;
import frc.util.driver.DashboardManager;
import frc.util.driver.DashboardManager.LayoutConstants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private double rightSetRPM = 0d;
    private double leftSetRPM = 0d;
    private boolean isOn = false;

    private FlywheelIO io;
    private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private static Flywheel instance;

    private Flywheel() {
        io = (FlywheelIO) Utils.getIOImplementation(FlywheelIOReal.class,
            FlywheelIOSim.class, FlywheelIO.class);

        DashboardManager.getInstance().addBoolSupplier(true, "Flywheel Status", () -> isOn,
            LayoutConstants.FlywheelStatus);
        DashboardManager.getInstance().addBoolSupplier(true, "Ready to Shoot",
            () -> withinSetGoal() && isOn, LayoutConstants.FlywheelUpToSpeed);
    }

    public static Flywheel getInstance() {
        if (instance == null) instance = new Flywheel();
        return instance;
    }

    /**
     * @return Command that spins up each motor to the specified RPM.
     */
    public Command on(double leftSpeed, double rightSpeed) {
        return runOnce(() -> {
            isOn = true;
            leftSetRPM = leftSpeed;
            rightSetRPM = rightSpeed;
            io.setRPM(leftSpeed, rightSpeed);
        });
    }

    /**
     * @return Command that sets both motor's RPM to 0. Note that the flywheels is in coast.
     */
    public Command off() {
        return runOnce(() -> {
            isOn = false;
            io.setRPM(0, 0);
        });
    }

    /**
     * True if the current speed of the left shooter motor is within + or - GoalToleranceRPM
     */
    public boolean withinSetGoal() {
        return MathUtil.isNear(leftSetRPM, inputs.leftVelocity,
            FlywheelConstants.GoalToleranceRPM);
    }

    public boolean isOn() { return isOn; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheels", inputs);
    }
}
