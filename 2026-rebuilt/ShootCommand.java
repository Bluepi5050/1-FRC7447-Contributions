package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private static final double OUTTAKE_DELAY_AFTER_SHOOTER_READY_SEC = 0.25;
    private static final double INDEXER_DELAY_SEC = 0.1;

    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final Timer timer = new Timer();
    private boolean shooterReachedTargetSpeed = false;
    private double shooterReadyTimeSec = 0.0;

    public ShootCommand(
        ShooterSubsystem shooterSubsystem,
        IndexerSubsystem indexerSubsystem,
        OuttakeSubsystem outtakeSubsystem
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;

        addRequirements(shooterSubsystem, indexerSubsystem, outtakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
        shooterReachedTargetSpeed = false;
        shooterReadyTimeSec = 0.0;

        shooterSubsystem.enableRoller();
        outtakeSubsystem.disableRoller();
        indexerSubsystem.disableRoller();

        outtakeSubsystem.setRollerTargetVelocity(Constants.Outtake.ROLLER_VELOCITY);
        indexerSubsystem.setRollerTargetVelocity(Constants.Indexer.ROLLER_VELOCITY);
    }

    @Override
    public void execute() {
        if (!shooterReachedTargetSpeed && shooterSubsystem.atTargetSpeed()) {
            shooterReachedTargetSpeed = true;
            shooterReadyTimeSec = timer.get();
        }

        if (shooterReachedTargetSpeed
            && timer.get() - shooterReadyTimeSec >= OUTTAKE_DELAY_AFTER_SHOOTER_READY_SEC) {
            outtakeSubsystem.enableRoller();
        }

        if (shooterReachedTargetSpeed
            && timer.get() - shooterReadyTimeSec
                >= OUTTAKE_DELAY_AFTER_SHOOTER_READY_SEC + INDEXER_DELAY_SEC) {
            indexerSubsystem.enableRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooterSubsystem.disableRoller();
        outtakeSubsystem.disableRoller();
        indexerSubsystem.disableRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
