package frc.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ArmConstants;
import frc.utils.Util;
import lombok.Getter;

public class Arm extends SubsystemBase {
    @Getter private double goalDegrees = 0.0;

    private ArmFeedforward ff = new ArmFeedforward(ArmConstants.kS, ArmConstants.kV,
    ArmConstants.kG, ArmConstants.kA);
    private ProfiledPIDController pid = new ProfiledPIDController(ArmConstants.kP, 0, ArmConstants.kD,
        new TrapezoidProfile.Constraints(ArmConstants.VelocityMax, ArmConstants.AccelerationMax));

    private static Arm instance;
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Arm() {
        io = (ArmIO) Util.getIOImplementation(ArmIOReal.class, ArmIOSim.class, ArmIO.class);
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalDegrees) {
        if (goalDegrees > ArmConstants.MaxDegreesFront || goalDegrees < ArmConstants.MaxDegreesBack) return;
        this.goalDegrees = goalDegrees;
        pid.setGoal(goalDegrees);
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
        double voltOut = output + feedforward;
        io.setVoltage(voltOut);
    }

    public double getMeasurement() {
        return Util.linearMap(inputs.position, ArmConstants.ThroughboreMin,
        ArmConstants.ThroughboreMax, ArmConstants.MaxDegreesBack,
        ArmConstants.MaxDegreesFront);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        double measurement = getMeasurement();
        useOutput(pid.calculate(measurement), pid.getSetpoint());
    }
}
