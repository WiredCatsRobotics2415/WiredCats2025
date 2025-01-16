package frc.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.TunerConstants;
import frc.utils.tuning.Characterizer;
import java.util.stream.DoubleStream;

public class DriveCharacterization extends Characterizer {
    private static DriveCharacterization instance;
    private CommandSwerveDrivetrain driveSubsystem;

    /* Swerve requests to apply during SysId characterization */
    private static final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private static final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private static final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private DriveCharacterization(CommandSwerveDrivetrain driveSubsystem) {
        super("Swerve");
        this.driveSubsystem = driveSubsystem;

        SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // (1 V/s)
            Volts.of(4), // Prevent brownout
            null, // (10 s)
            state -> SignalLogger.writeString("SysIdTranslationState", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> driveSubsystem.setControl(translationCharacterization.withVolts(output)), null,
                driveSubsystem));

        SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteerState", state.toString())),
            new SysIdRoutine.Mechanism(volts -> driveSubsystem.setControl(steerCharacterization.withVolts(volts)), null,
                driveSubsystem));

        SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI), null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotationState", state.toString())),
            new SysIdRoutine.Mechanism(output ->
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                driveSubsystem.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
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

        commands.add(sysIdRoutineRotation.dynamic(Direction.kForward).withName("Rotation: Dynamic Forward"));
        commands.add(sysIdRoutineRotation.dynamic(Direction.kReverse).withName("Rotation: Dynamic Backward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kForward).withName("Rotation: Quasi Forward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kReverse).withName("Rotation: Quasi Backward"));
    }

    public class WheelRadiusCharacterization extends Command {
        double driveBaseRadius = TunerConstants.DriveBaseRadius;
        CommandSwerveDrivetrain drive;

        double[] lastModuleDriveEncoderPositions = new double[4];
        double lastGyroRads;

        int runForSeconds;
        double[] radii;
        int execution = 0;

        public WheelRadiusCharacterization(CommandSwerveDrivetrain drive, int runtimeSeconds) {
            this.drive = drive;
            runForSeconds = runtimeSeconds;
            radii = new double[runtimeSeconds * 20];
        }

        @Override
        public void initialize() {
            int i = 0;
            for (SwerveModule<TalonFX, TalonFX, CANcoder> m : drive.getModules()) {
                lastModuleDriveEncoderPositions[i] = m.getPosition(true).distanceMeters;
                i++;
            }
            lastGyroRads = drive.getPigeon2().getAccumGyroY().getValueAsDouble();
            System.out.println("Initialization");
        }

        @Override
        public void execute() {
            double currentGyroRads = drive.getPigeon2().getAccumGyroY().getValueAsDouble();
            double currentModuleDriveEncoderPositions[] = new double[4];
            int i = 0;
            for (SwerveModule<TalonFX, TalonFX, CANcoder> m : drive.getModules()) {
                currentModuleDriveEncoderPositions[i] = m.getPosition(true).distanceMeters;
                i++;
            }
            double deltaSum = 0;
            for (int j = 0; j < 4; j++) {
                deltaSum += currentModuleDriveEncoderPositions[j] - lastModuleDriveEncoderPositions[j];
            }
            radii[execution] = ((currentGyroRads - lastGyroRads) * (driveBaseRadius)) / (deltaSum / 4);

            lastGyroRads = currentGyroRads;
            lastModuleDriveEncoderPositions = currentModuleDriveEncoderPositions;

            drive.setControl(rotationCharacterization.withRotationalRate(Math.PI * (execution / (runForSeconds * 20))));

            execution++;
            System.out.println("exection number " + execution + " out of " + radii.length);

            if (execution > radii.length) {
                end(false);
            }
        }

        @Override
        public void end(boolean interrupted) {
            double sum = DoubleStream.of(radii).sum();
            System.out.println("CHARACTERIZED RADIUS (INCHES): " + (sum / radii.length));
        }
    }

    public static void enable(CommandSwerveDrivetrain driveSubsystem) {
        if (instance == null) instance = new DriveCharacterization(driveSubsystem);
    }
}
