package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterSetTrackShot extends SequentialCommandGroup {

    public static class ShooterParams{
        public double limelightOffset = -9999.0;
        public int limelightPipeline;
        public boolean isLimelightActive = true;
        public double magazineRPM = Constants.MAGAZINE_SHOOT_RPM;
    }

    public ShooterSetTrackShot(Shooter shooter, Magazine magazine, Turret turret, ShooterParams params, Drive drive) {
        addCommands(
            new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
            new InstantCommand(()->Limelight.getInstance().setPipeline(params.limelightPipeline)),
            new ShooterSetReady(shooter,false),
            new TurretSetToGoalAngle(turret, drive),
            new TurretSetToTrackLimelightAngle(turret, params.limelightOffset, true),
            new ShooterSetLimelightTrackMode(shooter),
            new InstantCommand(()->magazine.setMagazineRPMValue(params.magazineRPM)),
            new ShooterSetReady(shooter, true)
        );
    }
}