package frc.robot.util;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShooterKinematics {
    private static final double GRAVITY_MPS2 = 9.81;
    private static final double MIN_FLIGHT_TIME_SEC = 0.05;
    private static final double MAX_FLIGHT_TIME_SEC = 3.0;
    private static final double FLIGHT_TIME_STEP_SEC = 0.005;
    private static final double EPSILON = 1e-9;

    private ShooterKinematics() {}

    public static Optional<MovingShotSolution> solveMovingShot(MovingShotRequest request) {
        return solveMovingShot(request, SelectionPreference.MIN_EXIT_SPEED);
    }

    public static Optional<MovingShotSolution> solveMovingShot(
        MovingShotRequest request,
        SelectionPreference selectionPreference
    ) {
        if (request == null
            || request.targetFieldPoint() == null
            || request.robotPose() == null
            || request.robotRelativeSpeeds() == null
            || request.robotToTurretPivot() == null
            || request.turretZeroOffset() == null
            || request.shooterCalibration() == null) {
            return Optional.empty();
        }

        if (request.targetFieldPoint().getZ() <= 0.0
            || request.shooterExitHeightMeters() < 0.0
            || request.maxExitSpeedMps() <= 0.0
            || request.maxHoodPositionRot() < request.minHoodPositionRot()) {
            return Optional.empty();
        }

        Translation2d shooterFieldPosition = AimMath.computeTurretPivotFieldPosition(
            request.robotPose(),
            request.robotToTurretPivot()
        );
        Translation3d shooterFieldPoint = new Translation3d(
            shooterFieldPosition.getX(),
            shooterFieldPosition.getY(),
            request.shooterExitHeightMeters()
        );
        Translation3d delta = request.targetFieldPoint().minus(shooterFieldPoint);
        double horizontalDistanceMeters = new Translation2d(delta.getX(), delta.getY()).getNorm();
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(
            shooterFieldPosition,
            new Translation2d(request.targetFieldPoint().getX(), request.targetFieldPoint().getY())
        );

        ChassisSpeeds shooterRelativeSpeeds = computeShooterRelativeSpeeds(
            request.robotRelativeSpeeds(),
            request.robotToTurretPivot()
        );
        ChassisSpeeds shooterFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            shooterRelativeSpeeds.vxMetersPerSecond,
            shooterRelativeSpeeds.vyMetersPerSecond,
            shooterRelativeSpeeds.omegaRadiansPerSecond,
            request.robotPose().getRotation()
        );

        MovingShotSolution bestSolution = null;

        for (double flightTimeSec = MIN_FLIGHT_TIME_SEC;
            flightTimeSec <= MAX_FLIGHT_TIME_SEC + EPSILON;
            flightTimeSec += FLIGHT_TIME_STEP_SEC) {
            Translation2d requiredHorizontalVelocity = new Translation2d(
                (delta.getX() / flightTimeSec) - shooterFieldSpeeds.vxMetersPerSecond,
                (delta.getY() / flightTimeSec) - shooterFieldSpeeds.vyMetersPerSecond
            );
            double requiredVerticalVelocity = (delta.getZ() / flightTimeSec) + (0.5 * GRAVITY_MPS2 * flightTimeSec);
            double horizontalSpeed = requiredHorizontalVelocity.getNorm();
            double exitSpeed = Math.hypot(horizontalSpeed, requiredVerticalVelocity);
            double launchPitchDeg = Math.toDegrees(Math.atan2(requiredVerticalVelocity, horizontalSpeed));
            OptionalDouble hoodPositionOpt = request.shooterCalibration().hoodPositionForLaunchAngleDeg(launchPitchDeg);

            if (exitSpeed > request.maxExitSpeedMps()
                || hoodPositionOpt.isEmpty()) {
                continue;
            }

            double hoodPositionRot = hoodPositionOpt.getAsDouble();
            double minAllowedHoodPositionRot = request.minHoodPositionRot();
            if (horizontalDistanceMeters > request.farShotDistanceMeters()) {
                minAllowedHoodPositionRot = Math.max(
                    minAllowedHoodPositionRot,
                    request.farShotMinHoodPositionRot()
                );
            }

            if (hoodPositionRot < minAllowedHoodPositionRot
                || hoodPositionRot > request.maxHoodPositionRot()) {
                continue;
            }

            Rotation2d desiredFieldYaw = horizontalSpeed > EPSILON
                ? new Rotation2d(requiredHorizontalVelocity.getX(), requiredHorizontalVelocity.getY())
                : stationaryFieldYaw;
            Rotation2d desiredTurretYaw = AimMath.computeTurretRelativeYaw(
                desiredFieldYaw,
                request.robotPose().getRotation(),
                request.turretZeroOffset()
            );
            MovingShotSolution candidate = new MovingShotSolution(
                desiredFieldYaw,
                desiredTurretYaw,
                Rotation2d.fromDegrees(launchPitchDeg),
                exitSpeed,
                flightTimeSec,
                desiredFieldYaw.minus(stationaryFieldYaw).getDegrees()
            );

            if (isBetterCandidate(selectionPreference, candidate, bestSolution)) {
                bestSolution = candidate;
            }
        }

        return Optional.ofNullable(bestSolution);
    }

    private static boolean isBetterCandidate(
        SelectionPreference selectionPreference,
        MovingShotSolution candidate,
        MovingShotSolution bestSolution
    ) {
        if (bestSolution == null) {
            return true;
        }

        if (selectionPreference == SelectionPreference.FLATTEST_SHOT) {
            double candidatePitchDeg = candidate.recommendedLaunchPitch().getDegrees();
            double bestPitchDeg = bestSolution.recommendedLaunchPitch().getDegrees();
            if (candidatePitchDeg + EPSILON < bestPitchDeg) {
                return true;
            }
            if (Math.abs(candidatePitchDeg - bestPitchDeg) <= EPSILON
                && candidate.flightTimeSec() + EPSILON < bestSolution.flightTimeSec()) {
                return true;
            }
            if (Math.abs(candidatePitchDeg - bestPitchDeg) <= EPSILON
                && Math.abs(candidate.flightTimeSec() - bestSolution.flightTimeSec()) <= EPSILON
                && candidate.requiredExitSpeedMps() + EPSILON < bestSolution.requiredExitSpeedMps()) {
                return true;
            }
            return false;
        }

        return candidate.requiredExitSpeedMps() + EPSILON < bestSolution.requiredExitSpeedMps()
            || (Math.abs(candidate.requiredExitSpeedMps() - bestSolution.requiredExitSpeedMps()) <= EPSILON
                && candidate.flightTimeSec() + EPSILON < bestSolution.flightTimeSec());
    }

    private static ChassisSpeeds computeShooterRelativeSpeeds(
        ChassisSpeeds robotRelativeSpeeds,
        Translation2d robotToTurretPivot
    ) {
        double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
        double pivotVx = robotRelativeSpeeds.vxMetersPerSecond - (omega * robotToTurretPivot.getY());
        double pivotVy = robotRelativeSpeeds.vyMetersPerSecond + (omega * robotToTurretPivot.getX());
        return new ChassisSpeeds(pivotVx, pivotVy, omega);
    }

    public record MovingShotRequest(
        Pose2d robotPose,
        ChassisSpeeds robotRelativeSpeeds,
        Translation2d robotToTurretPivot,
        double shooterExitHeightMeters,
        Translation3d targetFieldPoint,
        Rotation2d turretZeroOffset,
        ShooterCalibration shooterCalibration,
        double minHoodPositionRot,
        double maxHoodPositionRot,
        double farShotDistanceMeters,
        double farShotMinHoodPositionRot,
        double maxExitSpeedMps
    ) {}

    public record MovingShotSolution(
        Rotation2d desiredFieldYaw,
        Rotation2d desiredTurretYaw,
        Rotation2d recommendedLaunchPitch,
        double requiredExitSpeedMps,
        double flightTimeSec,
        double leadYawDeg
    ) {}

    public enum SelectionPreference {
        MIN_EXIT_SPEED,
        FLATTEST_SHOT
    }
}
