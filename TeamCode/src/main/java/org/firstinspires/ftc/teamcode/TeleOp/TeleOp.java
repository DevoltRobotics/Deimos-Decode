package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.light.SetAutoStatus;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.IntakeModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootAllCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexModeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;

public abstract class TeleOp extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;

    public TeleOp(Alliance alliance) {
        super(alliance);
    }


    @Override
    public void initialize() {
        pedroSubsystem.follower.setPose(PedroSubsystem.robotPose);
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);
        

        new TurretToPosCMD(turretSubsystem,0d).schedule();

        shooterSubsystem.setDefaultCommand(new ShooterAutoOdoCMD(shooterSubsystem,turretSubsystem));
        spindexSubsystem.setDefaultCommand(new SpindexModeDefaultCMD(spindexSubsystem));
        statusLightSubsystem.setDefaultCommand(new SetAutoStatus(statusLightSubsystem,spindexSubsystem));

        pedroSubsystem.setDefaultCommand(pedroSubsystem.fieldCentricCmd(gamepad1, alliance));



        intakeSubsystem.setDefaultCommand(new IntakeHoldCMD(intakeSubsystem));

        Button IntakeIn = new GamepadButton(
                Garra, GamepadKeys.Button.A
        );


        IntakeIn.whenHeld(new IntakeInCMD(intakeSubsystem));

        Button Turretauto = new GamepadButton(
                Garra, GamepadKeys.Button.X
        );


        Turretauto.toggleWhenPressed(
              new TurretAutoOdoCMD(turretSubsystem)
        );




        Button IntakeOut = new GamepadButton(
                Garra, GamepadKeys.Button.B
        );

        IntakeOut.whenHeld(new IntakeOutCMD(intakeSubsystem));







        Button lastB = new GamepadButton(
                Garra, GamepadKeys.Button.LEFT_BUMPER
        );

        lastB.whenPressed(new ShootAllCMD(spindexSubsystem));




        Button nextB = new GamepadButton(
                Garra, GamepadKeys.Button.RIGHT_BUMPER
        );

        nextB.whenActive(new NextPosSorterCMD(spindexSubsystem), false);

        Button intakemode = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_DOWN
        );

        intakemode.whenPressed(
                new IntakeModeCMD(spindexSubsystem)
        );

        Button AutoShoot = new GamepadButton(
                Garra, GamepadKeys.Button.Y
        );

        AutoShoot.whenPressed(
                new Shoot3BallsCMD(spindexSubsystem, intakeSubsystem, ()->spindexSubsystem.getPatternOffset())
        );












    }

    @Override
    public void run(){
        PedroSubsystem.robotPose = pedroSubsystem.follower.getPose();

    }
}
