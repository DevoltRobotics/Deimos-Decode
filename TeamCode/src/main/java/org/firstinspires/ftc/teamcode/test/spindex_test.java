package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.commands.sorter.SpinSorterCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Config
@TeleOp
public class spindex_test extends OpModeCommand {
    GamepadEx gamepadEx1;

    @Override
    public void initialize() {

        new RunCommand(()->{
           telemetry.update();
        }).schedule();

        gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new SpinSorterCMD(spindexSubsystem,spindexSubsystem.FPosRad));

    }
}
