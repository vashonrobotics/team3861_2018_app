package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;


public class DriveTrainBoogaloo {



    /**
     * Created by Caz on 9/30/2017.
     */


        //    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
        public static void nonMecanum(ArrayList baseMotorArray, double power[]) {
        }

        /**
         *
         * @param baseMotorArray this is the motor array that is being used.
         * @param x desired change in position left to right.  Right is positive.
         * @param y value of movement front/back. Bot-specific. Forward is positive.
         * @param turn turn-power value. Right is positive. -1 to 1.
         * @param frontIsInTheDirectionOfTheWheels
         */
        public static void mecanum(ArrayList baseMotorArray, double x, double y, double turn, boolean frontIsInTheDirectionOfTheWheels){
            double power = maxUnit(Math.sqrt((x * x) + (y * y)));
//        double radianAngle = 0;
            double radianAngle = Math.atan2(y, x) - Math.PI * 1/4;
//
            if (Math.abs(power) + Math.abs(turn) > 1)
            {
                power = power /(Math.abs(power) + Math.abs(turn));
                turn = Math.signum(turn) * (1 - Math.abs(power));
            }

            double motorPower[] = {
                    (Math.cos(radianAngle) * power) + turn, // frontLeft
                    (Math.sin(radianAngle) * power) - turn*(frontIsInTheDirectionOfTheWheels ? 1:-1), // frontRight
                    (Math.sin(radianAngle) * power) + turn*(frontIsInTheDirectionOfTheWheels ? 1:-1), // backLeft
                    (Math.cos(radianAngle) * power) - turn  // backRight
            };

            for (int i = 0; i < baseMotorArray.size(); i++) {
                ((DcMotor) baseMotorArray.get(i)).setPower(maxUnit(motorPower[i]));
            }
        }

        private static double maxUnit(final double input) {
            return input > 1 ? 1 : input < -1 ? -1 : input;
        }
        public static void turn(ArrayList baseMotorArray, double angle, double wheelWidthBetweenWheels, double wheelHeightBetweenWheels) {
            // angle is in degrees
            // wheel distances are in mm
            // clockwise is positive
            double distanceToTravel = 2*Math.PI*Math.sqrt(Math.pow(wheelHeightBetweenWheels/2,2)+Math.pow(wheelWidthBetweenWheels/2,2))*angle/360;
            final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
            final double     DRIVE_GEAR_REDUCTION = 1 ;     // This is < 1.0 if geared UP
            final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
            final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_MM * 3.1415);
            for (int i = 0; i < baseMotorArray.size(); i++) {
                DcMotor motor = ((DcMotor) baseMotorArray.get(i));
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int sideMultiplier = i % 2 == 0 ? 1 : -1;
                motor.setPower(0.4*sideMultiplier*Math.signum(angle));
            }
            while (Math.abs((int) (distanceToTravel * COUNTS_PER_MM)) > Math.abs(((DcMotor) baseMotorArray.get(0)).getCurrentPosition())) {
                try {
                    Thread.sleep(10, 0);
                } catch (InterruptedException e) {
                }
            }
            mecanum(baseMotorArray,0,0,0,true);
        }
    }


