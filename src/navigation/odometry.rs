use core::f32;

use crate::navigation::{Coord, PIDParameters};
use crate::units::MilliMeter;

#[allow(unused_imports)]
use micromath::F32Ext;

/// Contient la position du robot et peut se mettre à jour en
/// fonction des informations provenant des roues codeuses
#[derive(Debug)]
pub(crate) struct Odometry {
    /// ticks à gauche
    left_ticks: i64,
    /// ticks à droite
    right_ticks: i64,
    /// Coordonnee en x du robot en mm
    x: f32,
    /// Coordonnee en y du robot en mm
    y: f32,
    /// Angle du robot en radians
    angle: f32,
}

impl Odometry {
    /// Crée une nouvelle odometrie. La position du robot et des
    /// roues codeuses est initialisée à 0.
    pub(crate) fn new() -> Self {
        Odometry {
            left_ticks: 0,
            right_ticks: 0,
            x: 0.,
            y: 0.,
            angle: 0.,
        }
    }

    /// Définit les informations de position du robot.
    /// `new_pos` est exprimé en millimètres, `new_angle` est exprimé
    /// en milliradians
    pub(crate) fn set_position_and_angle(&mut self, new_pos: Coord, new_angle: i64) {
        self.x = new_pos.x.as_millimeters() as f32;
        self.y = new_pos.y.as_millimeters() as f32;
        self.angle = new_angle as f32 / 1000.0;
    }

    pub(crate) fn get_position(&self) -> Coord {
        Coord {
            x: MilliMeter(self.x as i64),
            y: MilliMeter(self.y as i64),
        }
    }

    /// Retourne l'angle du robot en milliradians
    pub(crate) fn get_angle(&self) -> i64 {
        (self.angle * 1000.0) as i64
    }

    /// Met à jour l'odometrie à partir de la variation des ticks
    /// de chaque roue codeuse
    pub(crate) fn update(&mut self, left_ticks: i64, right_ticks: i64, params: &PIDParameters) {
        let (dist_left, dist_right) =
            params.ticks_to_distance(left_ticks - self.left_ticks, right_ticks - self.right_ticks);

        let dist_diff = (dist_left + dist_right) / 2.0;
        let angle_diff = (dist_right - dist_left) / params.inter_axial_length;

        let sin = self.angle.sin();
        let cos = self.angle.cos();
        let dxf = dist_diff * cos;
        let dyf = dist_diff * sin;
        self.x += dxf;
        self.y += dyf;
        self.angle += angle_diff;

        self.left_ticks = left_ticks;
        self.right_ticks = right_ticks;
    }
}

#[cfg(test)]
mod test {

    use crate::units::MilliMeter;

    use crate::navigation::odometry::*;
    use crate::navigation::{Coord, PIDParameters};

    #[test]
    fn odom_forward() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        for i in 0..1025 {
            odom.update(i, i, &params);
        }
        let robot_pos = odom.get_position();

        assert_eq!(robot_pos.x, MilliMeter(195));
        assert_eq!(robot_pos.y, MilliMeter(0));
    }

    #[test]
    fn odom_backward() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        for i in 0..1025 {
            odom.update(-i, -i, &params);
        }
        let robot_pos = odom.get_position();

        assert_eq!(robot_pos.x, MilliMeter(-195));
        assert_eq!(robot_pos.y, MilliMeter(0));
    }

    #[test]
    fn odom_forward_with_start_angle() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        odom.set_position_and_angle(
            Coord {
                x: MilliMeter(0),
                y: MilliMeter(0),
            },
            3141 / 4,
        );

        for i in 0..1025 {
            odom.update(i, i, &params);
        }
        let robot_pos = odom.get_position();

        assert_eq!(robot_pos.x, MilliMeter(138));
        assert_eq!(robot_pos.y, MilliMeter(138));
        assert_eq!(odom.get_angle(), 3141 / 4);
    }

    #[test]
    fn odom_turn_self() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        for i in 0..921 {
            odom.update(i, -i, &params);
        }

        let robot_pos = odom.get_position();

        assert!(
            (robot_pos.x.as_millimeters() - 0).abs() <= 1,
            "{} should be {}",
            robot_pos.x.as_millimeters(),
            0
        );
        assert!(
            (robot_pos.y.as_millimeters() - 0).abs() <= 1,
            "{} should be {}",
            robot_pos.y.as_millimeters(),
            0
        );
        assert!(
            (odom.get_angle() + 1571).abs() <= 3,
            "{} should be {}",
            odom.get_angle(),
            -1571
        );
    }

    #[test]
    fn odom_turn_right() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        for i in 0..1843 {
            odom.update(i, 0, &params);
        }

        let robot_pos = odom.get_position();

        assert!(
            (robot_pos.x.as_millimeters() - 111).abs() <= 1,
            "{} should be {}",
            robot_pos.x.as_millimeters(),
            111
        );
        assert!(
            (robot_pos.y.as_millimeters() + 111).abs() <= 1,
            "{} should be {}",
            robot_pos.y.as_millimeters(),
            -111
        );
    }

    #[test]
    fn odom_front_backward_angle() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        odom.set_position_and_angle(
            Coord {
                x: MilliMeter(0),
                y: MilliMeter(0),
            },
            0,
        );

        for i in 0..1025 {
            odom.update(i, i, &params);
        }
        let robot_pos = odom.get_position();

        assert_eq!(robot_pos.x, MilliMeter(195));
        assert_eq!(robot_pos.y, MilliMeter(0));
        assert_eq!(odom.get_angle(), 0);

        let val_before = odom.get_position().x.as_millimeters();
        println!("angle before: {}", odom.get_angle());
        for i in 1..4000 {
            odom.update(i + 1025, 1025, &params);
        }
        let angle_after = odom.get_angle();
        println!("angle after: {}", angle_after);

        for i in 1..1025 {
            println!("{} {} {}", odom.get_position().x.as_millimeters(), odom.get_position().y.as_millimeters(), odom.get_angle());
            odom.update(i + 4000 + 1025, i + 1025, &params);
        }
        // assert_eq!(angle_after, odom.get_angle());
        let val_after = odom.get_position().x.as_millimeters();

        assert!(val_before > val_after, "{} > {} for {}", val_before, val_after, odom.get_angle());
    }

    // #[test]
    fn odom_complex_navigation() {
        let mut odom = Odometry::new();

        let params = PIDParameters {
            coder_radius: 31.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 223.0,
            ..Default::default()
        };

        odom.update(1024, 1024, &params);
        odom.update(-1024, 1024, &params);
        odom.update(1024, 1024, &params);

        // assert_eq!(odom.robot_pos.x, MilliMeter(-195));
        // assert_eq!(odom.robot_pos.y, MilliMeter(0));
        // assert_eq!(odom.angle, ...);
    }
}
