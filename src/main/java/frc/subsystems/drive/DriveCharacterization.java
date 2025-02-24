package frc.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.Controls;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.TunerConstants;
import frc.utils.tuning.Characterizer;
import frc.utils.tuning.TuningModeTab;
import java.util.stream.DoubleStream;

public class DriveCharacterization extends Characterizer {
    private static DriveCharacterization instance;
    private CommandSwerveDrivetrain driveSubsystem;

    /* Swerve requests to apply during SysId characterization */
    private static final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private static final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private static final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private DriveCharacterization(CommandSwerveDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                state -> SignalLogger.writeString("SysIdTranslationState", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> driveSubsystem.setControl(translationCharacterization.withVolts(output)), null,
                driveSubsystem));

        SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null,
                state -> SignalLogger.writeString("SysIdSteerState", state.toString())),
            new SysIdRoutine.Mechanism(volts -> driveSubsystem.setControl(steerCharacterization.withVolts(volts)), null,
                driveSubsystem));

        SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
                state -> SignalLogger.writeString("SysIdRotationState", state.toString())),
            new SysIdRoutine.Mechanism(output ->
            {
                driveSubsystem.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, driveSubsystem));

        commands.add(sysIdRoutineTranslation.dynamic(Direction.kForward).withName("Translation: Dynamic Forward"));
        commands.add(sysIdRoutineTranslation.dynamic(Direction.kReverse).withName("Translation: Dynamic Backward"));
        commands.add(sysIdRoutineTranslation.quasistatic(Direction.kForward).withName("Translation: Quasi Forward"));
        commands.add(sysIdRoutineTranslation.quasistatic(Direction.kReverse).withName("Translation: Quasi Backward"));

        commands.add(sysIdRoutineSteer.dynamic(Direction.kForward).withName("Steer: Dynamic Forward"));
        commands.add(sysIdRoutineSteer.dynamic(Direction.kReverse).withName("Steer: Dynamic Backward"));
        commands.add(sysIdRoutineSteer.quasistatic(Direction.kForward).withName("Steer: Quasi Forward"));
        commands.add(sysIdRoutineSteer.quasistatic(Direction.kReverse).withName("Steer: Quasi Backward"));

        commands.add(new WheelRadiusCharacterization(driveSubsystem, 10).withName("Wheel Radius Characterization"));
        commands.add(
            new CurrentLimitCharacterization(driveSubsystem, 20, 2.5, 0.5).withName("Slip Current Characterization"));

        commands.add(sysIdRoutineRotation.dynamic(Direction.kForward).withName("Rotation: Dynamic Forward"));
        commands.add(sysIdRoutineRotation.dynamic(Direction.kReverse).withName("Rotation: Dynamic Backward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kForward).withName("Rotation: Quasi Forward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kReverse).withName("Rotation: Quasi Backward"));

        TuningModeTab.getInstance().addCharacterizer("Swerve", this);
    }

    public class WheelRadiusCharacterization extends Command {
        double driveBaseRadius = RobotMeasurements.DriveTrainRadius.in(Inches);
        double wheelRadiusMeters = TunerConstants.kWheelRadius.in(Meters);
        CommandSwerveDrivetrain drive;

        double[] lastModuleDriveEncoderPositions = new double[4];
        double lastGyroDegrees;

        int runForSeconds;
        double[] radii;
        int execution = 0;

        public WheelRadiusCharacterization(CommandSwerveDrivetrain drive, int runtimeSeconds) {
            this.drive = drive;
            addRequirements(drive);
            runForSeconds = runtimeSeconds;
            radii = new double[runtimeSeconds * 20];
        }

        @Override
        public void initialize() {
            int i = 0;
            for (SwerveModule<TalonFX, TalonFX, CANcoder> m : drive.getModules()) {
                lastModuleDriveEncoderPositions[i] = Math.abs(
                    Units.rotationsToDegrees(m.getPosition(true).distanceMeters / (2 * Math.PI * wheelRadiusMeters)));
                i++;
            }
            lastGyroDegrees = drive.getPigeon2().getYaw().getValueAsDouble();
        }

        @Override
        public void execute() {
            double currentGyroDegrees = drive.getPigeon2().getYaw().getValueAsDouble();
            double currentModuleDriveEncoderPositions[] = new double[4];
            int i = 0;
            for (SwerveModule<TalonFX, TalonFX, CANcoder> m : drive.getModules()) {
                // rotationsToRadius(dist/(2*rInMeters*pi))
                currentModuleDriveEncoderPositions[i] = Math.abs(
                    Units.rotationsToDegrees(m.getPosition(true).distanceMeters / (2 * Math.PI * wheelRadiusMeters)));
                i++;
            }
            double deltaSum = 0;
            for (int j = 0; j < 4; j++) {
                deltaSum += Math.abs(currentModuleDriveEncoderPositions[j] - lastModuleDriveEncoderPositions[j]);
            }
            // inches = (degrees * inches)/degrees
            radii[execution] = ((currentGyroDegrees - lastGyroDegrees) * (driveBaseRadius)) / (deltaSum / 4);

            lastGyroDegrees = currentGyroDegrees;
            lastModuleDriveEncoderPositions = currentModuleDriveEncoderPositions;

            drive.setControl(new SwerveRequest.RobotCentric()
                .withRotationalRate(Controls.MaxAngularRadS * ((double) execution / (radii.length / 1.5)))
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(DriveRequestType.Velocity));

            execution++;
        }

        @Override
        public boolean isFinished() { return execution == radii.length; }

        @Override
        public void end(boolean interrupted) {
            drive.setControl(new SwerveRequest.SwerveDriveBrake());
            double average = DoubleStream.of(radii).filter(d -> Double.isFinite(d)).average().getAsDouble();
            System.out.println("CHARACTERIZED RADIUS (INCHES): " + average);
        }
    }

    public class CurrentLimitCharacterization extends Command {
        private CommandSwerveDrivetrain drive;
        private SwerveRequest.RobotCentric driveForwardAtFullEffort = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withVelocityX(TunerConstants.kSpeedAt12Volts);
        private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true);
        private Timer timer;
        private double secondsCounter;
        private double currentCounter = 20;
        private double currentStep = 5;
        private double timeStep;

        public CurrentLimitCharacterization(CommandSwerveDrivetrain drive, double startingCurrent, double currentStep,
            double timeStep) {
            this.drive = drive;
            addRequirements(drive);
            timer = new Timer();
            secondsCounter = 1;
            this.currentStep = currentStep;
            this.timeStep = timeStep;
            currentCounter = startingCurrent;
        }

        private void applyCurrentToAll(Current current) {
            for (SwerveModule<TalonFX, TalonFX, CANcoder> m : drive.getModules()) {
                m.getDriveMotor().getConfigurator().apply(currentLimitsConfigs.withStatorCurrentLimit(current));
            }
        }

        @Override
        public void initialize() {
            applyCurrentToAll(Amps.of(currentCounter));
            drive.setControl(driveForwardAtFullEffort);
            System.out.println("Starting at current Limit: " + currentCounter);
            timer.start();
            System.out.println("YOU must end the command when the wheels start turning");
        }

        @Override
        public void execute() {
            if (timer.hasElapsed(secondsCounter)) {
                currentCounter += currentStep;
                System.out.println("Current Limit: " + currentCounter);
                applyCurrentToAll(Amps.of(currentCounter));
                secondsCounter += timeStep;
            }
        }

        @Override
        public boolean isFinished() { return (currentCounter >= 80); }

        @Override
        public void end(boolean interrupted) {
            System.out.println("Current Characterization ended, current limit: " + currentCounter);
            System.out.println("Please restart robot code to apply correct current limit to swerve");
            timer.stop();
            timer.reset();
        }
    }

    public class SpeedAt12VCharacterization extends Command {
        private CommandSwerveDrivetrain drive;
        private SwerveRequest.RobotCentric driveForwardAtFullEffort = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo).withVelocityX(TunerConstants.kSpeedAt12Volts);

        public SpeedAt12VCharacterization(CommandSwerveDrivetrain drive) {
            this.drive = drive;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            drive.setControl(driveForwardAtFullEffort);
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("Ending m/s: " + drive.getState().Speeds.vxMetersPerSecond);
            System.out.println("Please double check the log to ensure this value is correct");
        }
    }

    public static void enable(CommandSwerveDrivetrain driveSubsystem) {
        if (instance == null) instance = new DriveCharacterization(driveSubsystem);
    }
}
