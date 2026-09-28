package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AimMath {
    private AimMath() {}

    public static Translation2d computeTurretPivotFieldPosition(
        Pose2d robotPose,
        Translation2d robotToTurretPivot
    ) {
        return robotPose.getTranslation().plus(robotToTurretPivot.rotateBy(robotPose.getRotation()));
    }

    public static Rotation2d computeDesiredTurretAngle(
        Pose2d robotPose,
        Translation2d robotToTurretPivot,
        Rotation2d turretZeroOffset,
        Translation2d hubFieldPoint
    ) {
        Translation2d turretPivotFieldPosition = computeTurretPivotFieldPosition(robotPose, robotToTurretPivot);
        Rotation2d desiredFieldYaw = computeDesiredFieldYaw(turretPivotFieldPosition, hubFieldPoint);
        return computeTurretRelativeYaw(desiredFieldYaw, robotPose.getRotation(), turretZeroOffset);
    }

    public static Rotation2d computeDesiredFieldYaw(
        Translation2d shooterFieldPosition,
        Translation2d targetFieldPosition
    ) {
        return targetFieldPosition.minus(shooterFieldPosition).getAngle();
    }

    public static Rotation2d computeTurretRelativeYaw(
        Rotation2d fieldYaw,
        Rotation2d robotHeading,
        Rotation2d turretZeroOffset
    ) {
        return fieldYaw
            .minus(robotHeading)
            .minus(turretZeroOffset);
    }
}
