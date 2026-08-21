package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TurretSetpointTest {
    private static final double kEpsilon = 1e-9;

    @Test
    void selectsDirectReachableSetpoint() {
        Turret.SetpointSelection selection = Turret.selectSetpoint(0.2, 0.0, -0.5, 0.5);

        assertTrue(selection.reachable());
        assertEquals(0.2, selection.setpoint(), kEpsilon);
    }

    @Test
    void selectsEquivalentRotationNearestLastSetpoint() {
        Turret.SetpointSelection selection = Turret.selectSetpoint(-0.4, 0.55, -0.75, 0.75);

        assertTrue(selection.reachable());
        assertEquals(0.6, selection.setpoint(), kEpsilon);
    }

    @Test
    void supportsMechanismRangesBeyondOneRotation() {
        Turret.SetpointSelection selection = Turret.selectSetpoint(0.2, 2.1, 1.5, 2.5);

        assertTrue(selection.reachable());
        assertEquals(2.2, selection.setpoint(), kEpsilon);
    }

    @Test
    void holdsLastSafeSetpointWhenTargetIsUnreachable() {
        Turret.SetpointSelection selection = Turret.selectSetpoint(0.49, 0.2, -0.4, 0.4);

        assertFalse(selection.reachable());
        assertTrue(Double.isFinite(selection.setpoint()));
        assertEquals(0.2, selection.setpoint(), kEpsilon);
    }

    @Test
    void rejectsNonFiniteTargetWithoutReturningNaN() {
        Turret.SetpointSelection selection = Turret.selectSetpoint(Double.NaN, 0.1, -0.4, 0.4);

        assertFalse(selection.reachable());
        assertTrue(Double.isFinite(selection.setpoint()));
        assertEquals(0.1, selection.setpoint(), kEpsilon);
    }
}
