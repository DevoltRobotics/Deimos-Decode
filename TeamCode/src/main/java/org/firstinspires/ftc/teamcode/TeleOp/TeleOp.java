package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpinSorterCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;

    @Override
    public void initialize() {
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);


        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem,new IntakeHoldCMD(intakeSubsystem));
        Button IntakeIn =new GamepadButton(
                Garra,GamepadKeys.Button.A
        );

       IntakeIn.whenHeld(new IntakeInCMD(intakeSubsystem));

       Button IntakeOut = new GamepadButton(
               Garra,GamepadKeys.Button.B
       );

       IntakeOut.whenHeld(new IntakeOutCMD(intakeSubsystem));

       Button Sorter1 = new GamepadButton(
               Garra,GamepadKeys.Button.DPAD_LEFT
       );

       Sorter1.whenHeld(new SpinSorterCMD(spindexSubsystem,spindexSubsystem.FPos));

        Button Sorter2 = new GamepadButton(
                Garra,GamepadKeys.Button.DPAD_UP
        );

        Sorter2.whenHeld(new SpinSorterCMD(spindexSubsystem,spindexSubsystem.SPos));

        Button Sorter3 = new GamepadButton(
                Garra,GamepadKeys.Button.DPAD_RIGHT
        );

        Sorter3.whenHeld(new SpinSorterCMD(spindexSubsystem,spindexSubsystem.TPos));



    }
}
