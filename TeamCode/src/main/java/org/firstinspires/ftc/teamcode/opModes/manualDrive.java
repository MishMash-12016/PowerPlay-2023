package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Gamepad;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.RobotController;

@TeleOp
public class manualDrive extends LinearOpMode{
    @Override
    public void runOpMode() {

        RobotController robotController = new RobotController(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        robotController.cycleController.start();
        robotController.elevatorController.start();
        robotController.driveController.start();

        while (opModeIsActive()){
            Gamepad.a = gamepad1.a;
            Gamepad.b = gamepad1.b;
            Gamepad.y = gamepad1.y;
            Gamepad.x = gamepad1.x;

            Gamepad.left_trigger = gamepad1.left_trigger;
            Gamepad.right_trigger = gamepad1.right_trigger;

            Gamepad.left_bumper = gamepad1.left_bumper;
            Gamepad.right_bumper = gamepad1.right_bumper;

            Gamepad.left_stick_y = gamepad1.left_stick_y;
            Gamepad.left_stick_x = gamepad1.left_stick_x;
            Gamepad.right_stick_x = gamepad1.right_stick_x;

        }

        robotController.terminate();
        telemetry.clearAll();
        telemetry.addLine("stop");
        telemetry.update();

    }
}