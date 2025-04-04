package frc.robot;

import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.RuntimeConstants;
import frc.constants.RuntimeConstants.Mode;
import frc.robot.RobotStatus.RobotState;
import frc.utils.AllianceDependent;
import frc.utils.TorqueSafety;
import frc.utils.Visualizer;
import frc.utils.simulation.SimulationTab;
import frc.utils.tuning.TuneableBoolean;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuningModeTab;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    public Robot() {
        PowerDistribution pdh = new PowerDistribution();
        pdh.setSwitchableChannel(true);

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
        switch (RuntimeConstants.CurrentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                SignalLogger.setPath("/U/logs");
                SignalLogger.start();
                break;

            case SIM:
                // Running a physics simulator, log to NT
                SimulationTab.enableSimulationControls();
                SignalLogger.start();
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                if (!RuntimeConstants.HootFileName.equals("")) {
                    HootReplay.loadFile(RuntimeConstants.HootFileName);
                    System.out.println("Playing replay file " + RuntimeConstants.HootFileName);
                }
                break;
        }
        if (RuntimeConstants.TuningMode) {
            Logger.registerURCL(URCL.startExternal());
        }
        Logger.start();

        // DO NOT REMOVE THIS LINE
        // constructs the robot container, which constructs all subsystems
        RobotContainer.getInstance();

        if (RuntimeConstants.TuningMode) {
            TuningModeTab.enableTuningMode();
        }
        Notifier allianceUpdater = new Notifier(AllianceDependent::updateCurrentAlliance);
        allianceUpdater.startPeriodic(1);

        // From 6328, log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
            .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
            .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        if (RuntimeConstants.CurrentMode == Mode.REPLAY && !RuntimeConstants.HootFileName.equals("")) {
            HootReplay.play();
        }
    }

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public void robotPeriodic() {
        if (RuntimeConstants.TuningMode) {
            TuneableNumber.periodic();
            TuneableBoolean.periodic();
            TorqueSafety.getInstance().periodic();
        }
        RobotContainer.getInstance().periodic();
        CommandScheduler.getInstance().run();
        if (RuntimeConstants.VisualizationEnabled) Visualizer.update();
    }

    @Override
    public void teleopInit() {
        RobotContainer.getInstance().teleopEnable();
    }

    @Override
    public void autonomousInit() {
        if (Robot.isSimulation()) SimulatedArena.getInstance().placeGamePiecesOnField();
        RobotContainer.getInstance().getAutonomousCommand().schedule();
    }

    @Override
    public void disabledInit() {
        RobotStatus.setRobotState(RobotState.Disabled);
    }

    @Override
    public void disabledExit() {
        RobotStatus.setRobotState(RobotState.Enabled);
    }
}
