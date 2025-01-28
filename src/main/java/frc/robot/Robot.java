package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.RuntimeConstants;
import frc.utils.Visualizer;
import frc.utils.tuning.TuningModeTab;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    public Robot() {
        RobotContainer.getInstance();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (RuntimeConstants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        Logger.start();

        if (RuntimeConstants.TuningMode) {
            SignalLogger.start();
            TuningModeTab.enableTuningMode();
        }
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public void robotPeriodic() {
        if (RuntimeConstants.VisualizationEnabled) Visualizer.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        RobotContainer.getInstance().getAutonomousCommand().schedule();
    }
}
