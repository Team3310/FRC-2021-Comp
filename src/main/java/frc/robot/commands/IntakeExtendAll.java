package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class IntakeExtendAll extends SequentialCommandGroup {
    
    public IntakeExtendAll(Intake intake, Magazine magazine) {

        addCommands(
                new TurretSetAngle(Turret.getInstance(), Constants.TURRET_INTAKE_ANGLE_DEGREES),
                new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_INTAKE_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)

        );
    }
}