use core::f32;
#[allow(unused_imports)]
use libm::F32Ext;

use crate::navigation::motor::Command;

/// Module permettant de detecter si le robot est bloqué. Le robot est
/// considéré bloqué s'il reçoit une commande non nulle mais ne bouge pas.
///
/// Plus précisément, si au moins une roue devrait avancer mais n'avance
/// pas, le robot est considéré bloqué, sauf si l'autre roue est en train
/// d'avancer.
pub struct Blocking {
    command_threshold: u16,
    distance_threshold: f32,

    last_dist: (f32, f32),
    blocked: bool,
}

impl Blocking {
    /// `command_threshold`: Si une commande reçue par un des moteurs est
    /// supérieure à cette valeur on considère que le robot reçoit une commande.
    ///
    /// `distance_threshold`: Si la distance parcourue par le robot est inférieure
    /// à cette valeur on considère que le robot n'a pas changé de position. En mm.
    pub fn new(command_threshold: u16, distance_threshold: f32) -> Self {
        Blocking {
            command_threshold,
            distance_threshold,
            last_dist: (0.0, 0.0),
            blocked: false,
        }
    }

    /// Reset internal tracking data
    pub fn reset(&mut self) {
        self.last_dist = (0.0, 0.0);
        self.blocked = false;
    }

    /// Met à jour l'état de bloquage du robot. Cette fonction doit être appelée
    /// periodiquement avec une période suffisamment longue (par exemple 100ms).
    ///
    /// `command` La commande actuellement envoyée au moteur
    /// `dist` La distance totale parcourue par les codeurs, en mm
    pub fn update(&mut self, command: (Command, Command), dist: (f32, f32)) {
        let (left_diff, right_diff) = (dist.0 - self.last_dist.0, dist.1 - self.last_dist.1);
        self.last_dist = dist;
        let (left_command, right_command) = command;

        self.blocked = if left_command.get_value() > self.command_threshold {
            match left_command {
                Command::Front(_) => left_diff < self.distance_threshold,
                Command::Back(_) => left_diff > -self.distance_threshold,
            }
        } else if right_command.get_value() > self.command_threshold {
            match right_command {
                Command::Front(_) => right_diff < self.distance_threshold,
                Command::Back(_) => right_diff > -self.distance_threshold,
            }
        } else {
            false
        }
    }

    /// Renvoie l'état de bloquage du robot.
    pub fn blocked(&self) -> bool {
        self.blocked
    }
}

#[cfg(test)]
mod test {
    use crate::navigation::blocking::Blocking;
    use crate::navigation::Command;

    #[test]
    fn test_blocking() {
        let mut blocking = Blocking::new(100, 0.1);

        // Forward
        blocking.update((Command::Front(12), Command::Front(12)), (0.05, 0.05));
        assert!(!blocking.blocked());
        blocking.reset();

        blocking.update((Command::Front(120), Command::Front(12)), (0.05, 0.05));
        assert!(blocking.blocked());
        blocking.reset();

        blocking.update((Command::Front(120), Command::Front(120)), (14.0, 0.05));
        assert!(!blocking.blocked());
        blocking.reset();

        // Backward
        blocking.update((Command::Back(12), Command::Back(12)), (-0.05, -0.05));
        assert!(!blocking.blocked());
        blocking.reset();

        blocking.update((Command::Back(120), Command::Back(120)), (-0.14, -0.05));
        assert!(!blocking.blocked());
        blocking.reset();

        blocking.update((Command::Back(120), Command::Back(120)), (-0.14, -0.05));
        assert!(!blocking.blocked());
        blocking.reset();

        // Backward / forward
        blocking.update((Command::Back(120), Command::Back(120)), (0.14, 0.14));
        assert!(blocking.blocked());
        blocking.reset();

        blocking.update((Command::Back(120), Command::Front(120)), (0.14, -0.14));
        assert!(blocking.blocked());
        blocking.reset();
    }
}
