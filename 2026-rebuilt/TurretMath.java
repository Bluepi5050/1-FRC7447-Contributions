package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public final class TurretMath {
    private static final double FULL_TURRET_ROTATION = 1.0;
    private static final double EPSILON = 1e-9;

    private TurretMath() {}

    public static double convertTurretRotationsToMotorRotations(
        double turretRotations,
        double motorRotationsPerTurretRotation
    ) {
        return turretRotations * motorRotationsPerTurretRotation;
    }

    public static double convertMotorRotationsToTurretRotations(
        double motorRotations,
        double motorRotationsPerTurretRotation
    ) {
        return motorRotations / motorRotationsPerTurretRotation;
    }

    public static double convertTurretDegreesToMotorRotations(
        double turretDegrees,
        double motorRotationsPerTurretRotation
    ) {
        return convertTurretRotationsToMotorRotations(
            turretDegrees / 360.0,
            motorRotationsPerTurretRotation
        );
    }

    public static double convertMotorRotationsToTurretDegrees(
        double motorRotations,
        double motorRotationsPerTurretRotation
    ) {
        return convertMotorRotationsToTurretRotations(
            motorRotations,
            motorRotationsPerTurretRotation
        ) * 360.0;
    }

    public static double getNearestEquivalentLegalTurretRotation(
        double desiredTurretRotations,
        double currentTurretRotations,
        double legalLowTurretRotations,
        double legalHighTurretRotations
    ) {
        long minTurns = (long) Math.ceil(legalLowTurretRotations - desiredTurretRotations - EPSILON);
        long maxTurns = (long) Math.floor(legalHighTurretRotations - desiredTurretRotations + EPSILON);

        double bestCandidate = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (long turns = minTurns; turns <= maxTurns; turns++) {
            double candidate = desiredTurretRotations + (turns * FULL_TURRET_ROTATION);
            double distance = Math.abs(candidate - currentTurretRotations);
            if (distance + EPSILON < bestDistance) {
                bestCandidate = candidate;
                bestDistance = distance;
            }
        }

        if (!Double.isNaN(bestCandidate)) {
            return bestCandidate;
        }

        double nearestEquivalent = desiredTurretRotations
            + Math.rint((currentTurretRotations - desiredTurretRotations) / FULL_TURRET_ROTATION) * FULL_TURRET_ROTATION;
        return MathUtil.clamp(nearestEquivalent, legalLowTurretRotations, legalHighTurretRotations);
    }
}
