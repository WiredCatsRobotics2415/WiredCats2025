package frc.util.math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ProjectileMotion3d {
    private double speedMPS;
    private Pose3d startPose;

    /**
     * Projectile equation in 3d given a speed and start pose, generates pose3ds at a time t
     * where each pose3d is where the object would be at that time if it started at speedMPS
     * and was shot forward on the startPose-relative forward
     */
    public ProjectileMotion3d(Pose3d start, double speedMPS) {
        this.speedMPS = speedMPS;
        this.startPose = start;
    }

    public Pose3d getPoseAtTime(double time) {
        double x = speedMPS * time;
        // 4.905 = 9.81/2
        double z = -4.905 * (time * time);
        return startPose.plus(new Transform3d(x, 0, -z, new Rotation3d()));
    }
}
