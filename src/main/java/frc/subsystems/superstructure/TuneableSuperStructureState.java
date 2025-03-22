package frc.subsystems.superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.constants.Subsystems.CoralIntakeConstants;
import frc.utils.tuning.TuneableNumber;
import lombok.Getter;

public class TuneableSuperStructureState {
    @Getter private TuneableNumber height;
    @Getter private TuneableNumber arm;
    @Getter private TuneableNumber coralIntake;

    public TuneableSuperStructureState(Distance height, Angle arm, Angle coralIntake, String name) {
        this.height = new TuneableNumber(height, "SSState/" + name + "/height");
        this.arm = new TuneableNumber(arm, "SSState/" + name + "/arm");
        this.coralIntake = new TuneableNumber(coralIntake, "SSState/" + name + "/cIntake");
    }

    /**
     * Sets coral intake to default stow position
     */
    public TuneableSuperStructureState(Distance height, Angle arm, String name) {
        this.height = new TuneableNumber(height, "SSState/" + name + "/height");
        this.arm = new TuneableNumber(arm, "SSState/" + name + "/arm");
        this.coralIntake = new TuneableNumber(CoralIntakeConstants.StowAngle.angle(), "SSState/" + name + "/cIntake");
    }
}
