package org.firstinspires.ftc.teamcode.commands.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TransferCMD extends CommandBase {
    private TransferSubsystem transferSubsystem;

    public TransferCMD(TransferSubsystem transferSubsystem){

        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);

    }
    @Override
    public void initialize(){
        transferSubsystem.In();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
