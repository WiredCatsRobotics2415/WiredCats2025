package frc.subsystems.algaeintake;

public class AlgaeIntake {
    private static AlgaeIntake instance;

    private AlgaeIntake() {

    }

    public static AlgaeIntake getInstance() {
        if (instance == null) instance = new AlgaeIntake();
        return instance;
    }
}
