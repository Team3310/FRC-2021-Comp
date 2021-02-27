package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class IntakeExtendAllAuto extends SequentialCommandGroup {

    public IntakeExtendAllAuto(Intake intake, Turret turret, Magazine magazine) {

        addCommands(
                new TurretSetAngle(turret, Constants.TURRET_INTAKE_ANGLE_DEGREES),
                new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                new MagazineSetRPM(magazine, 0)
        );
    }
}