package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.LastPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;

@Config
@TeleOp (name = "spindex")
public class spindex_test extends OpModeCommand {
    GamepadEx gamepadEx1;

    public spindex_test() {
        super(Alliance.ANY);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setDefaultCommand(new IntakeHoldCMD(intakeSubsystem));

        new RunCommand(()->{
           telemetry.update();
        }).schedule();

        gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new NextPosSorterCMD(spindexSubsystem));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new LastPosSorterCMD(spindexSubsystem));




    }
}
