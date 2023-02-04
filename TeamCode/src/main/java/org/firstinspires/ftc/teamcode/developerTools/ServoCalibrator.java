package org.firstinspires.ftc.teamcode.developerTools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoCalibrator extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        String[] names = {
                "grabber",
                "puffer",
                "pufferRight\npufferLeft",
                "armRight\narmLeft",
                "grabberRight\ngrabberLeft"
        };


        int i = 0;

        double position = 0;
        double sensitivity = 1;

        while (opModeIsActive() && !gamepad1.a){
            if (B_Pressed()) {
                i++;
            }
            if (X_Pressed()){
                i--;
            }
            i = (i + names.length) % names.length;

            telemetry.addLine(names[i]);
            telemetry.addData("position", position - gamepad1.left_stick_y * sensitivity);
            telemetry.update();
        }

        sleep(500);

        String[] name = names[i].split("\n");

        if(name.length == 1){
            Servo s = hardwareMap.servo.get(name[0]);

            while (opModeIsActive()) {
                s.setPosition(position - gamepad1.left_stick_y * sensitivity);

                if (A_Pressed()){
                    position -= gamepad1.left_stick_y * sensitivity;
                    sensitivity /= 2;
                    sleep(500);
                }

                telemetry.addData("position", position - gamepad1.left_stick_y * sensitivity);
                telemetry.update();
            }
        }
        else {
            Servo s1 = hardwareMap.servo.get(name[0]);
            Servo s2 = hardwareMap.servo.get(name[1]);

            while (opModeIsActive()) {
                s1.setPosition(position - gamepad1.left_stick_y * sensitivity);
                s2.setPosition(position - gamepad1.left_stick_y * sensitivity);

                s1.setDirection(Servo.Direction.REVERSE);

                if (A_Pressed()) {
                    position -= gamepad1.left_stick_y * sensitivity;
                    sensitivity /= 2;
                    sleep(500);
                }

                telemetry.addData("position", position - gamepad1.left_stick_y * sensitivity);
                telemetry.update();

            }
        }
    }

    private boolean A_wasPressedLastTime = true;
    public boolean A_Pressed() {
        if (gamepad1.a) {
            if (A_wasPressedLastTime) {
                A_wasPressedLastTime = false;
                return true;
            }
        } else {
            A_wasPressedLastTime = true;
        }
        return false;
    }
    private boolean B_wasPressedLastTime = true;
    public boolean B_Pressed() {
        if (gamepad1.b) {
            if (B_wasPressedLastTime) {
                B_wasPressedLastTime = false;
                return true;
            }
        } else {
            B_wasPressedLastTime = true;
        }
        return false;
    }
    private boolean X_wasPressedLastTime = true;
    public boolean X_Pressed() {
        if (gamepad1.x) {
            if (X_wasPressedLastTime) {
                X_wasPressedLastTime = false;
                return true;
            }
        } else {
            X_wasPressedLastTime = true;
        }
        return false;
    }
}
