package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterShootCmd extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    DoubleSupplier targetvelocity;
    DoubleSupplier tragetHood;

    public ShooterShootCmd(ShooterSubsystem shooterSubsystem, DoubleSupplier targetvelocity,DoubleSupplier targetHood){
        this.shooterSubsystem = shooterSubsystem;
        this.targetvelocity = targetvelocity;
        this.tragetHood = targetHood;
        addRequirements(shooterSubsystem);
    }

    public ShooterShootCmd(ShooterSubsystem shooterSubsystem, double targetvelocity,double targetHood) {
        this(shooterSubsystem, () -> targetvelocity,()->targetHood);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetVelocity(targetvelocity.getAsDouble());
        shooterSubsystem.setHood(tragetHood.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.setTargetVelocity(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
