package frc.robot;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.RuntimeConstants;
import frc.sim.SimulatedEnvironment;
import lombok.Getter;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Getter private static SimulatedEnvironment simEnv;

    /**
     * The BUTTON ONLY event loop for the robot. Gets polled during robotPeriodic. Intended
     * for use in {@link OIs}'s binds maps, and is cleared before {@link RobotContainer}'s
     * configureButtonBinds is called.
     */
    public static final EventLoop buttonEventLoop = new EventLoop();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Set up data receivers & replay source
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            switch (RuntimeConstants.simMode) {
                case SIM:
                    // Running a physics simulator, log to NT
                    Logger.addDataReceiver(new NT4Publisher());
                    // Logger.addDataReceiver(new WPILOGWriter());
                    simEnv = new SimulatedEnvironment();
                    break;

                case REPLAY:
                    // Replaying a log, set up replay source
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog();
                    Logger.setReplaySource(new WPILOGReader(logPath));
                    Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                    break;
            }
        }
        Logger.start();
        robotContainer = RobotContainer.getInstance();
    }

    @Override
    public void robotPeriodic() {
        if (simEnv != null) simEnv.updateEnvironment();

        CommandScheduler.getInstance().run();
        buttonEventLoop.poll();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer}
     * class.
     */
    @Override
    public void autonomousInit() {
        if (simEnv != null) simEnv.resetField(true);

        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (simEnv != null) simEnv.resetField(false);

        robotContainer.teleopInit();
        if (autonomousCommand != null) autonomousCommand.cancel();
    }
}
