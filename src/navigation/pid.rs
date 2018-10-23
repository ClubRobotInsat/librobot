use navigation::ms::*;

use core::fmt::{Display, Formatter, Result};

use qei::QeiManager;

use embedded_hal::digital::OutputPin;
use embedded_hal::{PwmPin, Qei};

/// Un moteur avec ses deux broches : vitesse et direction.
pub struct Motor<MOT, DIR> {
    pwm: MOT,
    dir: DIR,
}

impl<MOT, DIR> Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    /// Crée une nouvelle structure de gestion moteur à partir d'une broche de direction et d'une
    /// broche de PWM :
    /// * la PWM commande la vitesse : son duty cycle est proportionnel à la vitesse voulue du
    /// moteur
    /// * la broche d'entrée sortie controle la direction du moteur
    pub fn new(pwm: MOT, dir: DIR) -> Self {
        Motor { pwm, dir }
    }

    /// Applique la commande de direction et de vitesse aux moteurs :
    /// * avancer correspond à un état bas sur la broche de direction
    /// * reculer correspond à un état haut sur la broche de direction
    pub fn apply_command(&mut self, cmd: Command) {
        match cmd {
            Command::Front(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_low();
            }
            Command::Back(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_high();
            }
        }
    }
}

/// Une commande pour un moteur : une direction et une vitesse sur 16 bits (0 : vitesse nulle).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Command {
    Front(u16),
    Back(u16),
}

impl Display for Command {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Command::Front(val) => write!(f, "Forward : {}", val),
            Command::Back(val) => write!(f, "Backward {}", val),
        }
    }
}

/// Le PID du robot
pub struct Pid<L, R> {
    old_left_count: i64,
    old_right_count: i64,
    left_qei: QeiManager<L>,
    right_qei: QeiManager<R>,
    pos_kp: i64,
    pos_kd: i64,
    orient_kp: i64,
    orient_kd: i64,
    // Le multiplicateur interne pour augmenter la précision des calculs
    internal_multiplier: i64,
    // La valeur maximale de la commande en sortie
    cap: u16,
    // La consigne de position du robot exprimée en nombre de tick de roue codeuse
    position_consigne: i64,
    // La consigne d'angle exprimée en différence de tick de chaque roue codeuse
    orientation_consigne: i64,
}

// Implémentation du PID
impl<L, R> Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    /// Crée un nouveau PID à partir :
    /// * des coefficients de l'asservissement
    /// * d'une valeur maximale de sortie (la valeur du registre ARR du timer qui gère la PWM par
    /// exemple)
    /// * deux roues codeuses
    pub fn new(
        pos_kp: i64,
        pos_kd: i64,
        orient_kp: i64,
        orient_kd: i64,
        internal_multiplier: i64,
        cap: u16,
        left_qei: QeiManager<L>,
        right_qei: QeiManager<R>,
    ) -> Self {
        Pid {
            old_left_count: 0,
            old_right_count: 0,
            left_qei,
            right_qei,
            pos_kp,
            pos_kd,
            orient_kp,
            orient_kd,
            internal_multiplier,
            cap,
            position_consigne: 0,
            orientation_consigne: 0,
        }
    }

    /// Mets à jour la consigne en position en terme de tick de roues codeuses
    pub fn set_position_goal(&mut self, pos: i64) {
        self.position_consigne = pos;
    }

    /// Mets à jour la consigne en orientation en terme de tick de roues codeuses.
    pub fn set_orientation_goal(&mut self, orientation: i64) {
        self.orientation_consigne = orientation;
    }

    /// Renvoie la nouvelle consigne à appliquer aux deux roues pour atteindre la commande en position
    fn update_position_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> i64 {
        let dist = (left_count + right_count) / 2;
        let speed = (left_speed + right_speed) / 2;
        let diff = self.position_consigne as i64 - dist;
        let cmd = (diff * self.pos_kp) - self.pos_kd * speed;
        cmd
    }

    /// Renvoie les nouvelles consignes à appliquer aux deux roues pour atteindre la commande en orientation
    fn update_orientation_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> (i64, i64) {
        let orientation = right_count - left_count;
        let speed = right_speed - left_speed;
        let diff = self.orientation_consigne - orientation;
        let cmd = (diff * self.orient_kp) - self.orient_kd * speed;
        (-cmd, cmd)
    }

    fn truncate(&self, val: i64) -> Command {
        if val.is_positive() {
            if val > self.cap as i64 {
                Command::Front(self.cap)
            } else {
                Command::Front(val as u16)
            }
        } else {
            if -val > self.cap as i64 {
                Command::Back(self.cap)
            } else {
                Command::Back((-val) as u16)
            }
        }
    }

    /// Renvoie la nouvelle consigne à appliquer aux roues pour le pid
    /// L'algorithme est issue de [cette](https://www.rcva.fr/10-ans-dexperience/9/) page internet.
    pub fn update(&mut self) -> (Command, Command) {
        // Mise à jour des QEI QEI
        self.left_qei.sample_unwrap();
        self.right_qei.sample_unwrap();

        // Mise à jour de la mémoire du PID
        let (new_left_count, new_right_count) = (self.left_qei.count(), self.right_qei.count());
        let (left_speed, right_speed) = (
            new_left_count - self.old_left_count,
            new_right_count - self.old_right_count,
        );
        self.old_left_count = new_left_count;
        self.old_right_count = new_right_count;
        // Calcul du PID
        let position_cmd =
            self.update_position_command(new_left_count, new_right_count, left_speed, right_speed);
        let (orientation_cmd_left, orientation_cmd_right) = self.update_orientation_command(
            new_left_count,
            new_right_count,
            left_speed,
            right_speed,
        );

        let left_cmd = position_cmd + orientation_cmd_left;
        let right_cmd = position_cmd + orientation_cmd_right;
        // Truncate resul
        (self.truncate(left_cmd), self.truncate(right_cmd))
    }
}

#[cfg(test)]
mod test {

    use std::cell::Cell;
    use std::rc::Rc;

    use embedded_hal::Qei;
    use navigation::pid::{Command, Pid};
    use qei::QeiManager;

    #[derive(Clone, Copy)]
    enum Direction {
        Front,
        Back,
    }


    #[derive(Clone)]
    struct DummyMotor {
        speed: Rc<Cell<u16>>,
        real_position: Rc<Cell<i64>>,
        wrapped_position: Rc<Cell<u16>>,
        dir: Rc<Cell<Direction>>,
    }

    impl DummyMotor {
        fn new() -> Self {
            DummyMotor {
                speed: Rc::new(Cell::new(0)),
                real_position: Rc::new(Cell::new(0)),
                wrapped_position: Rc::new(Cell::new(0)),
                dir: Rc::new(Cell::new(Direction::Front)),
            }
        }

        fn update(&mut self) {
            match self.dir.get(){
                Direction::Front => {
                    self.real_position.replace(self.real_position.get() + self.speed.get() as i64);
                    self.wrapped_position.replace(self.wrapped_position.get().wrapping_add(self
                        .speed.get()));
                }
                Direction::Back => {
                    self.real_position.replace(self.real_position.get() - self.speed.get() as i64);
                    self.wrapped_position.replace(self.wrapped_position.get().wrapping_sub(self
                        .speed.get()));

                }
            }
        }

        fn apply_command(&mut self, command: Command) {
            match command {
                Command::Front(speed) => {
                    self.speed.replace(speed/5);
                    self.dir.replace(Direction::Front);
                }
                Command::Back(speed) => {
                    self.speed.replace(speed/5);
                    self.dir.replace(Direction::Back);
                }
            }
        }

        pub fn get_real_position(&self) -> i64 {
            self.real_position.get()
        }
    }

    impl Qei for DummyMotor {
        type Count = u16;

        /// Returns the current pulse count of the encoder
        fn count(&self) -> Self::Count {
            self.wrapped_position.get() as u16
        }

        /// Returns the count direction
        fn direction(&self) -> embedded_hal::Direction {
            embedded_hal::Direction::Upcounting
        }
    }

    #[test]
    fn pid_forward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1, 1, 1, 1, 1, 800, qei_left, qei_right);

        pid.set_position_goal(9000);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!((motor_left.get_real_position() - 9000).abs()<= 9);
        assert!((motor_right.get_real_position() - 9000).abs()<= 9);
    }

    #[test]
    fn pid_backward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1, 1, 1, 1, 1, 800, qei_left, qei_right);

        pid.set_position_goal(-9137);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!((motor_left.get_real_position() + 9137).abs()<= 9);
        assert!((motor_right.get_real_position() + 9137).abs()<= 9);
    }

}
