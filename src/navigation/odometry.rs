
use core::f32;

use crate::units::MilliMeter;
use crate::navigation::{Coord, WheelsConstants};

#[allow(unused_imports)]
use libm::F32Ext;

/// Contient la position du robot et peut se mettre à jour en
/// fonction des informations provenant des roues codeuses
#[derive(Debug)]
pub struct Odometry {
    /// ticks à gauche
    left_ticks: i64,
    /// ticks à droite
    right_ticks: i64,
    /// Coordonnees du robot
    robot_pos: Coord,
    /// Angle du robot en millirad
    angle: i64
}

impl Odometry {
    /// Crée une nouvelle odometrie. La position du robot et des
    /// roues codeuses est initialisée à 0.
    pub fn new() -> Self {
        Odometry {
            left_ticks:0,
            right_ticks:0,
            robot_pos: Coord {
                x: MilliMeter(0),
                y: MilliMeter(0),
            },
            angle: 0
        }
    }

    /// Définit les informations de position du robot.
    pub fn set_position(&mut self, new_pos: Coord, new_angle: i64) {
        self.robot_pos = new_pos;
        self.angle = new_angle;
    }

    /// Met à jour l'odometrie à partir de la variation des ticks
    /// de chaque roue codeuse
    pub fn update(&mut self, left_ticks: i64,
                  right_ticks: i64,
                  constants: &WheelsConstants) {

        let distance_per_wheel_turn =
            constants.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;

        let dist_left = (left_ticks - self.left_ticks) as f32 * distance_per_wheel_turn / 1024.0;
        let dist_right = (right_ticks - self.right_ticks) as f32 * distance_per_wheel_turn / 1024.0;

        let dist_diff = (dist_left + dist_right) / 2.0;
        let angle_diff = dist_left - dist_right;

        let anglef = self.angle as f32 / 1000.0;
        let (sin, cos) = anglef.sin_cos();
        let dxf = dist_diff * cos;
        let dyf = dist_diff * sin;
        self.robot_pos = Coord {
            x: MilliMeter(self.robot_pos.x.as_millimeters() + dxf.round() as i64),
            y: MilliMeter(self.robot_pos.y.as_millimeters() + dyf.round() as i64),
        };
        self.angle += angle_diff.round() as i64;

        self.left_ticks = left_ticks;
        self.right_ticks = right_ticks;
    }
}


#[cfg(test)]
mod test {

    use crate::units::MilliMeter;

    use crate::navigation::odometry::*;
    use crate::navigation::{WheelsConstants, Coord};

    #[test]
    fn odom_forward() {
        let mut odom = Odometry::new();

        let constants = WheelsConstants {
            coder_radius: MilliMeter(31),
            inter_axial_length: MilliMeter(223),
        };

        odom.update(1024, 1024, &constants);

        assert_eq!(odom.robot_pos.x, MilliMeter(195));
        assert_eq!(odom.robot_pos.y, MilliMeter(0));
    }

    #[test]
    fn odom_backward() {
        let mut odom = Odometry::new();

        let constants = WheelsConstants {
            coder_radius: MilliMeter(31),
            inter_axial_length: MilliMeter(223),
        };

        odom.update(-1024, -1024, &constants);

        assert_eq!(odom.robot_pos.x, MilliMeter(-195));
        assert_eq!(odom.robot_pos.y, MilliMeter(0));
    }

    #[test]
    fn odom_angle_custom() {
        let mut odom = Odometry::new();

        let constants = WheelsConstants {
            coder_radius: MilliMeter(31),
            inter_axial_length: MilliMeter(223),
        };

        odom.set_position(Coord {x:MilliMeter(0), y:MilliMeter(0)}, 3141/4 );
        odom.update(1024, 1024, &constants);

        assert_eq!(odom.robot_pos.x, MilliMeter(138));
        assert_eq!(odom.robot_pos.y, MilliMeter(138));
    }

    // #[test]
    fn odom_complex_navigation() {
        let mut odom = Odometry::new();

        let constants = WheelsConstants {
            coder_radius: MilliMeter(31),
            inter_axial_length: MilliMeter(223),
        };

        odom.update(1024, 1024, &constants);
        odom.update(-1024, 1024, &constants);
        odom.update(1024, 1024, &constants);

        // assert_eq!(odom.robot_pos.x, MilliMeter(-195));
        // assert_eq!(odom.robot_pos.y, MilliMeter(0));
        // assert_eq!(odom.angle, ...);
    }
}
