package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;

import java.util.concurrent.Callable;

public class System {
    // region SYSTEM CONSTANTS
    public enum DriveMode {}
    public enum ScoreState {}
    public enum CollectionState {}

    public static class LedColor {}

    public static final InterruptedException hardStopRequest = new InterruptedException("stop everything !");
    public static final InterruptedException softStopRequest = new InterruptedException("stop running safely");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion SYSTEM CONSTANTS

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static int currentConeHeight;
    // endregion SYSTEM VARIABLES

    // region INITIALIZATION
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        System.hardwareMap = hardwareMap;
        System.telemetry   = telemetry  ;
        System.gamepad1    = gamepad1   ;
        System.gamepad2    = gamepad2   ;

        isStopRequested = false;
        currentConeHeight = 0;

        arm.initialize();
        driveTrain.initialize();
        elevator.initialize();
        grabber.initialize();
        puffer.initialize();

    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllControllers(){
        driveTrain.controller.start();
        elevator.controller.start();
        cycleController.start();
    }

    public static void terminate(){
        isStopRequested = true;
        driveTrain.reset();
        elevator.reset();

        telemetry.update();
    }
    // endregion SYSTEM FUNCTIONALITY

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

    private static boolean a = true;
    private static boolean A_firstPress(){
        if (gamepad1.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }

    private static boolean x = true;
    private static boolean X_firstPress(){
        if (gamepad1.x){
            if(x){
                x = false;
                return true;
            }
        } else x = true;
        return false;
    }

    private static boolean y = true;
    private static boolean Y_firstPress(){
        if (gamepad1.y){
            if(y){
                y = false;
                return true;
            }
        } else y = true;
        return false;
    }

    private static boolean b = true;
    private static boolean B_firstPress(){
        if (gamepad1.b){
            if(b){
                b = false;
                return true;
            }
        } else b = true;
        return false;
    }
    // endregion GENERAL FUNCTIONALITY

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested) {
            if (!manual.collect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.collect();
            }

            if (A_firstPress()) {

                elevator.wantedPosition = elevator.highPosition;
                if (!manual.score.isAlive()) {
                    manual.score();
                }

            }
            else if (X_firstPress()) {

                elevator.wantedPosition = elevator.middlePosition;
                if (!manual.score.isAlive()) {
                    manual.score();
                }

            }
            else if (Y_firstPress()) {

                elevator.wantedPosition = elevator.lowPosition;
                if (!manual.score.isAlive()) {
                    manual.score();
                }

            }
            else if (B_firstPress()){
                elevator.wantedPosition = elevator.bottomPosition;
                manual.score.interrupt();
            }
        }
    });

    static class manual{
        public static void collect() {collect.start();}
        public static void score() {score.start();}

        private static final Thread collect = new Thread(() -> {
            try {
                grabber.goToCone(currentConeHeight);
                grabber.fullRelease();
                while (!System.manual.collect.isInterrupted() && !isStopRequested && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                    arm.goToRelativePosition(gamepad1.left_trigger);

                    if (grabber.coneIsInRange()) {
                        grabber.grab();
                    }
                }


                if (isStopRequested){
                    throw hardStopRequest;
                }

                if (System.manual.collect.isInterrupted()){
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
        private static final Thread score = new Thread(()-> {
            try {
                await(elevator::reachedWantedPosition);
                puffer.goToOut();

                await(() -> gamepad1.right_trigger > 0 || manual.score.isInterrupted());

                if (manual.score.isInterrupted()){
                    throw softStopRequest;
                }

                puffer.release();

                await(() -> gamepad1.right_trigger == 0  || manual.score.isInterrupted());

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
    // endregion DRIVER CONTROLLED FUNCTIONS

    // region AUTONOMOUS FUNCTIONS
    static class auto{
        public static final Thread autoCycle = new Thread(() -> {

        });

        private void score(boolean prepareForNext, int nextConeNum){}
        private void collect(int coneNum){}
    }
    // endregion AUTONOMOUS FUNCTIONS
}
