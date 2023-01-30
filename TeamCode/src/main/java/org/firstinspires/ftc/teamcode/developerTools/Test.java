package org.firstinspires.ftc.teamcode.developerTools;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        DcMotorEx left  = (DcMotorEx) hardwareMap.dcMotor.get("elevatorLeft" );
        DcMotorEx right = (DcMotorEx) hardwareMap.dcMotor.get("elevatorRight");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo pufferRight = hardwareMap.servo.get("pufferRight");
        Servo pufferLeft  = hardwareMap.servo.get("pufferLeft" );

        pufferRight.setDirection(Servo.Direction.REVERSE);

        pufferLeft.setPosition(0.5);
        pufferRight.setPosition(0.5);

        double lastP = 0;
        double p = 0;
        double d = 0;
        double power = 0;

        while (opModeIsActive() && left.getCurrentPosition() < 15000){
            d = p - lastP;
            lastP = p;
            p = left.getCurrentPosition();

            telemetry.addData("p", p);
            telemetry.addData("d", d);
            telemetry.update();
        }

        left.setPower(0.01);
        right.setPower(0.01);

        try {
            Thread.sleep(1000);

            left.setPower(0);
            right.setPower(0);

            Thread.sleep(20000);
        } catch (InterruptedException E){}
    }
}
