/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Depot Autonomous Op Mode", group="Linear Opmode")
public class DepotOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = null;
    private MineralDetector mineralDetector;
    private Navigation navigation;
    private LiftArm liftArm;
    private Collector collector;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        OdometryNavigation oNav = new OdometryNavigation(-12, 12,  3*PI/4);
        SimpleOutput output = new TelemetrySimpleOutput(telemetry);
        SimpleIMU simpleIMU = new BNO055SimpleIMU(hardwareMap);
        this.navigation = oNav;
        //driveTrain = new FourWheelDifferentialDriveTrain(hardwareMap, navigation,
        //     oNav, simpleIMU, output);
        //driveTrain = new DifferentialDriveTrain(hardwareMap, navigation, oNav, simpleIMU, output);
//        driveTrain = new MiniBotDriveTrain(hardwareMap, navigation,
//                oNav, simpleIMU, output);

        driveTrain = new MecanumDriveTrain(hardwareMap,
                new MecanumParams(11.75 / 2, 13.75 / 2, 1.96),
                oNav, oNav);

        collector = new Collector(hardwareMap, output);
        collector.init();

        driveTrain.init();
        mineralDetector = new OpenCVMineralDetector(hardwareMap);
        mineralDetector.init();

        liftArm = new LiftArm(hardwareMap, output);
        liftArm.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        liftArm.prepareToUnlatch();
        liftArm.landRobot();
        driveTrain.driveLeft(4);
        driveTrain.driveForward(4);
        driveTrain.driveLeft(-4);
        liftArm.retractLandingGear();
        driveTrain.driveForward(-4);

        telemetry.addData("Status", "Initial sequence");
        telemetry.update();

        driveTrain.lookAt(-36, 36);
        if(!mineralDetector.isGold()) {
            driveTrain.lookAt(-48,24);  // for the differential bot this is
            doSleep();
            if(mineralDetector.isGold()){
                driveTrain.driveTo(-36,21);
            }
            else{
                driveTrain.driveTo(-21,36);
            }
        } else {
            driveTrain.driveTo(-36,36);
        }

//        driveTrain.driveTo(-50,50);
//        collector.lowerAndWait();
//        collector.blow();
//        sleep(2000);
//        collector.raiseCollector();
//        driveTrain.driveTo(60,60);
    }

    public void doSleep() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            // discard
        }
    }
    }


