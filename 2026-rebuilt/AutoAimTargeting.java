package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public final class AutoAimTargeting {
    private AutoAimTargeting() {}

    public enum TargetMode {
        HUB,
        NEUTRAL_BACK,
        OPPONENT_PASS_BACK,
        HOLD,
        PARK
    }

    public record TargetSelection(
        TargetMode targetMode,
        boolean inAllianceZone,
        boolean inNeutralZone,
        Translation2d targetPoint,
        Translation3d targetPoint3d
    ) {}

    public static TargetSelection selectTarget(
        DriverStation.Alliance alliance,
        Translation2d robotTranslation,
        double robotPoseY
    ) {
        boolean inAllianceZone = PolygonUtil.contains(
            Constants.FieldTargets.getAllianceZone(alliance),
            robotTranslation
        );
        DriverStation.Alliance opponentAlliance =
            alliance == DriverStation.Alliance.Red ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red;
        boolean inOpponentAllianceZone = PolygonUtil.contains(
            Constants.FieldTargets.getAllianceZone(opponentAlliance),
            robotTranslation
        );
        boolean inNeutralZone = !inAllianceZone && !inOpponentAllianceZone;

        if (inAllianceZone) {
            Translation2d hubPoint = Constants.FieldTargets.getHubForAlliance(alliance);
            return new TargetSelection(
                TargetMode.HUB,
                true,
                false,
                hubPoint,
                Constants.FieldTargets.getHubAimPoint3d(alliance)
            );
        }

        if (inNeutralZone) {
            Translation2d backTarget = Constants.FieldTargets.getPreferredBackTargetForAlliance(alliance, robotPoseY);
            return new TargetSelection(
                TargetMode.NEUTRAL_BACK,
                false,
                true,
                backTarget,
                Constants.FieldTargets.getPreferredBackTarget3dForAlliance(alliance, robotPoseY)
            );
        }

        Translation2d opponentPassBackTarget = Constants.FieldTargets.getPreferredOpponentPassBackTarget(robotPoseY);
        return new TargetSelection(
            TargetMode.OPPONENT_PASS_BACK,
            false,
            false,
            opponentPassBackTarget,
            Constants.FieldTargets.getPreferredOpponentPassBackTarget3d(robotPoseY)
        );
    }
}
