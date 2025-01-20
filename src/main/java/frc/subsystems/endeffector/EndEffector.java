package frc.subsystems.endeffector;

public class EndEffector {
    private static EndEffector instance;

    private EndEffector() {

    }

    public static EndEffector getInstance() {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}
