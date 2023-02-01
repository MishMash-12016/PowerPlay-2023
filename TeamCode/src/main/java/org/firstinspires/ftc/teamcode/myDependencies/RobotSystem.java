package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;

import java.util.concurrent.Callable;

public class RobotSystem {
    // region SYSTEM CONSTANTS
    public static final InterruptedException hardStopRequest = new InterruptedException("stop everything !");
    public static final InterruptedException softStopRequest = new InterruptedException("stop running safely");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static int currentConeHeight;
    // endregion

    // region INITIALIZATION
    public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);

        arm.initialize();
        puffer.initialize();
        grabber.initialize();
        elevator.initialize();
        driveTrain.initialize();
    }
    public static void resetAll(){
        RobotSystem.reset();

        led.reset();
        elevator.reset();
        driveTrain.reset();
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.reset();

        RobotSystem.hardwareMap = hardwareMap;
        RobotSystem.telemetry   = telemetry  ;
        RobotSystem.gamepad1    = gamepad1   ;
        RobotSystem.gamepad2    = gamepad2   ;
    }
    public static void reset(){
        isStopRequested = false;
        currentConeHeight = 0;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllControllers(){
        RobotSystem.cycleController.start();
        //driveTrain.controller.start();
        //elevator.controller.start();
    }

    public static void terminate(){
        isStopRequested = true;
        resetAll();

        telemetry.update();
    }
    // endregion

    // region GENERAL FUNCTIONALITY
    private static final ElapsedTime timePassed = new ElapsedTime();
    public static void sleep(int wantedSleepTime) throws InterruptedException{
        timePassed.reset();
        while(wantedSleepTime < timePassed.milliseconds()){
            if (isStopRequested) {
                throw hardStopRequest;
            }
            Thread.sleep(20);
        }
    }
    public static void await(Callable<Boolean> requirement) throws Exception{
        try {
            while (requirement.call() && !isStopRequested) {
                Thread.sleep(20);
                if (isStopRequested) {
                    throw hardStopRequest;
                }
            }
        } catch (Exception e){
            if (e != hardStopRequest && e != softStopRequest) {
                telemetry.addData("await got this exception", e);
                throw e;
            }
        }
    }

    public static class FirstPress{
        public static boolean a = true;
        public static boolean A(){
            if (gamepad1.a){
                if(a){
                    a = false;
                    return true;
                }
            } else a = true;
            return false;
        }

        public static boolean x = true;
        public static boolean X(){
            if (gamepad1.x){
                if(x){
                    x = false;
                    return true;
                }
            } else x = true;
            return false;
        }

        public static boolean y = true;
        public static boolean Y(){
            if (gamepad1.y){
                if(y){
                    y = false;
                    return true;
                }
            } else y = true;
            return false;
        }

        public static boolean b = true;
        public static boolean B(){
            if (gamepad1.b){
                if(b){
                    b = false;
                    return true;
                }
            } else b = true;
            return false;
        }
    }
    // endregion

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.cycleController.isInterrupted()) {
            if (!manual.asyncCollect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.collect();
            }

            if (!manual.asyncScore.isAlive()) {
                if (FirstPress.A()) { manual.score(elevator.highPosition); }
                else if (FirstPress.X()) { manual.score(elevator.middlePosition); }
                else if (FirstPress.Y()) { manual.score(elevator.lowPosition); }
            }
            else if (FirstPress.B()){ elevator.wantedPosition = elevator.bottomPosition; }
        }
    });

    static class manual{
        public static void collect() {
            asyncCollect.start();
        }
        public static void score(double height) {
            elevator.wantedPosition = height;
            asyncScore.start();
        }

        private static final Thread asyncCollect = new Thread(() -> {
            try {
                grabber.goToCone(currentConeHeight);
                grabber.fullRelease();
                while (!RobotSystem.manual.asyncCollect.isInterrupted() && !isStopRequested && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                    arm.goToRelativePosition(gamepad1.left_trigger);

                    if (grabber.coneIsInRange()) {
                        grabber.grab();
                    }
                }


                if (isStopRequested){
                    throw hardStopRequest;
                }

                if (RobotSystem.manual.asyncCollect.isInterrupted()){
                    throw softStopRequest;
                }

                if (grabber.hasCone()) {
                    arm.goToRelativePosition(0);
                    grabber.goToMid();

                    await(() -> !arm.isOut() && !elevator.isUp());

                    grabber.goToIn();

                    await(() -> !grabber.isOut());

                    grabber.midRelease();
                    puffer.grab();

                    sleep(300);

                    puffer.goToMid();

                    sleep(300);

                    puffer.release();

                    sleep(500);

                    puffer.grab();
                } else {
                    arm.goToRelativePosition(0);
                    grabber.midRelease();
                    grabber.goToIn();
                }

            }catch (Exception e){
                if (e == softStopRequest){
                    arm.goToRelativePosition(0);
                    grabber.midRelease();
                    grabber.goToIn();
                } else if (e != hardStopRequest){
                    telemetry.addData("stopped manual collect completely", e);
                }
            }
        });
        private static final Thread asyncScore = new Thread(()-> {
            try {
                await(elevator::reachedWantedPosition);
                puffer.goToOut();

                await(() -> gamepad1.right_trigger > 0 || manual.asyncScore.isInterrupted());

                if (manual.asyncScore.isInterrupted()){
                    throw softStopRequest;
                }

                puffer.release();

                await(() -> gamepad1.right_trigger == 0 || manual.asyncScore.isInterrupted());

                throw softStopRequest;
            }catch (InterruptedException e1){
                if (e1 == softStopRequest){
                    try {
                        puffer.goToIn();

                        sleep(200);

                        elevator.wantedPosition = elevator.bottomPosition;

                        await(() -> !elevator.isUp());

                        puffer.release();
                    } catch (Exception e2){
                        telemetry.addData("stopped manual score from safely stopping", e2);
                    }
                }
            } catch (Exception e){
                telemetry.addData("stopped manual score completely", e);
            }
        });
    }
    // endregion

    // region AUTONOMOUS FUNCTIONS
    static class auto{
        public static final Thread autoCycle = new Thread(() -> {

        });

        private void score(boolean prepareForNext, int nextConeNum){}
        private void collect(int coneNum){}
    }
    // endregion
}
