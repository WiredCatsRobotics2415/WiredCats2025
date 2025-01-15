package frc.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.utils.tuning.Characterizer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveCharacterization extends Characterizer {
    private static DriveCharacterization instance;
    private CommandSwerveDrivetrain driveSubsystem;

    /* Swerve requests to apply during SysId characterization */
    private static final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private static final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private static final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private DriveCharacterization(CommandSwerveDrivetrain driveSubsystem) {
        super();
        this.driveSubsystem = driveSubsystem;

        SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // (1 V/s)
                        Volts.of(4), // Prevent brownout
                        null, // (10 s)
                        state -> SignalLogger.writeString("SysIdTranslationState", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> setControl(translationCharacterization.withVolts(output)), null, driveSubsystem));

        SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteerState", state.toString())),
                new SysIdRoutine.Mechanism(volts -> setControl(steerCharacterization.withVolts(volts)), null, driveSubsystem));

        SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /* This is in radians per second², but SysId only supports "volts per second" */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotationState", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /* output is actually radians per second, but SysId only supports "volts" */
                            setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        driveSubsystem));
        
        commands.add(sysIdRoutineTranslation.dynamic(Direction.kForward).withName("Translation: Dynamic Forward"));
        commands.add(sysIdRoutineTranslation.dynamic(Direction.kReverse).withName("Translation: Dynamic Backward"));
        commands.add(sysIdRoutineTranslation.quasistatic(Direction.kForward).withName("Translation: Quasi Forward"));
        commands.add(sysIdRoutineTranslation.quasistatic(Direction.kReverse).withName("Translation: Quasi Backward"));

        commands.add(sysIdRoutineSteer.dynamic(Direction.kForward).withName("Steer: Dynamic Forward"));
        commands.add(sysIdRoutineSteer.dynamic(Direction.kReverse).withName("Steer: Dynamic Backward"));
        commands.add(sysIdRoutineSteer.quasistatic(Direction.kForward).withName("Steer: Quasi Forward"));
        commands.add(sysIdRoutineSteer.quasistatic(Direction.kReverse).withName("Steer: Quasi Backward"));

        commands.add(sysIdRoutineRotation.dynamic(Direction.kForward).withName("Rotation: Dynamic Forward"));
        commands.add(sysIdRoutineRotation.dynamic(Direction.kReverse).withName("Rotation: Dynamic Backward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kForward).withName("Rotation: Quasi Forward"));
        commands.add(sysIdRoutineRotation.quasistatic(Direction.kReverse).withName("Rotation: Quasi Backward"));
    }

    public void enable(CommandSwerveDrivetrain driveSubsystem) {
        if (instance == null)
            instance = new DriveCharacterization(driveSubsystem);
    }
}
