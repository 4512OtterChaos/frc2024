package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounting pose
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(7.25), Units.inchesToMeters(20)),
        new Rotation3d(0, Math.toRadians(-17), Math.toRadians(-5))
    );
    
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
