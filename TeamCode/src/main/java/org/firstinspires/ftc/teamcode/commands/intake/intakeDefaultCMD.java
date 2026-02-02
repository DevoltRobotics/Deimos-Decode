package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class intakeDefaultCMD extends CommandBase {

    private final IntakeSubsystem intake;
    private final SpindexSubsystem spindex;

    public intakeDefaultCMD(IntakeSubsystem intake, SpindexSubsystem spindex) {
        this.intake = intake;
        this.spindex = spindex;
        addRequirements(intake);
    }

    @Override
    public void execute() {

        if (spindex.getnBalls() == 3 && spindex.getExtraB() || spindex.isShooting && spindex.getExtraB()) {
            intake.Out();
        }
        else if (spindex.getBPresence() || spindex.getnBalls() == 3) {
            intake.Hold();
        }
        else {
            intake.In();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.Stop(); // recomendado para seguridad
    }

    @Override
    public boolean isFinished() {
        return false; // default command
    }
}
