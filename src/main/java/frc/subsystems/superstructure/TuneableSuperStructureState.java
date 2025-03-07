package frc.subsystems.superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.utils.tuning.TuneableAngle;
import frc.utils.tuning.TuneableDistance;
import lombok.Getter;

public class TuneableSuperStructureState {
    @Getter private TuneableDistance height;
    @Getter private TuneableAngle arm;
    @Getter private TuneableAngle coralIntake;

    public TuneableSuperStructureState(Distance height, Angle arm, Angle coralIntake, String name) {
        this.height = new TuneableDistance(height, "SSState/" + name + "/height");
        this.arm = new TuneableAngle(arm, "SSState/" + name + "/arm");
        this.coralIntake = new TuneableAngle(coralIntake, "SSState/" + name + "/cIntake");
    }
}
