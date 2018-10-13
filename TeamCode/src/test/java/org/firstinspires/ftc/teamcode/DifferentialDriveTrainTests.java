package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static java.lang.Math.PI;
import static org.junit.Assert.*;

public class DifferentialDriveTrainTests {
    @Test
    public void odometryStartsInTheRightPlace() {
        TestDifferentialDriveTrain driveTrain = new TestDifferentialDriveTrain(0, 10, PI);
        assertEquals(TestDifferentialDriveTrain.getNav().getX(), 0, 0.001);
        assertEquals(TestDifferentialDriveTrain.getNav().getY(), 10, 0.001);
        assertEquals(TestDifferentialDriveTrain.getNav().getTheta(), PI, 0.001);
    }

    @Test
    public void lookAtCalculatesCorrectly() {
        TestDifferentialDriveTrain driveTrain = new TestDifferentialDriveTrain(0, 0, PI/2);
        double theta = driveTrain.calculateDirectionToLook(48, 0);
        assertEquals(0, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(48, 48);
        assertEquals(PI/4, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(0, 48);
        assertEquals(PI/2, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(-48, 48);
        assertEquals(3 * PI/4, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(-48, 0);
        assertEquals(PI, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(-48, -48);
        assertEquals(-3 * PI/4, theta, 0.001);

        theta = driveTrain.calculateDirectionToLook(0, -48);
        assertEquals(-PI/2, theta, 0.001);
    }

    @Test
    public void lookAtDoesntChangePosition() {
        TestDifferentialDriveTrain driveTrain = new TestDifferentialDriveTrain(0, 0, PI/2);
        driveTrain.lookAt(48, 48);
        assertEquals(TestDifferentialDriveTrain.getNav().getX(), 0, 0.001);
        assertEquals(TestDifferentialDriveTrain.getNav().getY(), 0, 0.001);
    }

    @Test
    public void relativeSquareEndsInCorrectPosition() {
        // drive a square using the relative commands and
        // make sure we ended up where we started.
        TestDifferentialDriveTrain driveTrain = new TestDifferentialDriveTrain(0, 0, 0);
        driveTrain.driveForward(100);

        driveTrain.turnRelative(PI/2);
        driveTrain.driveForward(100);

        driveTrain.turnRelative(PI/2);
        driveTrain.driveForward(100);

        driveTrain.turnRelative(PI/2);
        driveTrain.driveForward(100);

        assertEquals(TestDifferentialDriveTrain.getNav().getX(), 0, 0.001);
        assertEquals(TestDifferentialDriveTrain.getNav().getY(), 0, 0.001);
        assertEquals(TestDifferentialDriveTrain.getNav().getTheta(), 3 * PI/2, 0.001);
    }

    @Test
    public void rightTurnMovesRightWheelBackwards() {
        TestDifferentialDriveTrain driveTrain = new TestDifferentialDriveTrain(0, 0, 0);
        driveTrain.lookAt(0, -48);

        assertTrue("Right wheel turned the wrong way",
                driveTrain.getLastRightWheelSteps() < 0);

        assertTrue("Left wheel turned the wrong way",
                driveTrain.getLastLeftWheelSteps() > 0);

        assertEquals("Bot pointed in wrong direction",
                -PI/2, TestDifferentialDriveTrain.getNav().getTheta(), 0.001);
    }
}
