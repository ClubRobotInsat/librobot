use core::fmt::{Debug, Display, Formatter, Result};

use embedded_hal::digital::OutputPin;
use embedded_hal::PwmPin;

/// Une commande pour un moteur : une direction et une vitesse sur 16 bits (0 : vitesse nulle).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Command {
    /// Le moteur doit avancer à la vitesse fournie
    Front(u16),
    /// Le moteur doit reculer à la vitesse fournie
    Back(u16),
}

impl Command {
    /// Renvoie l'intensité de la commande
    pub fn get_value(&self) -> u16 {
        match self {
            Command::Front(val) => *val,
            Command::Back(val) => *val,
        }
    }
}

impl Display for Command {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Command::Front(val) => write!(f, "Forward : {}", val),
            Command::Back(val) => write!(f, "Backward {}", val),
        }
    }
}

/// Un moteur avec ses deux broches : vitesse et direction.
pub struct Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    pwm: MOT,
    dir: DIR,
}

impl<MOT, DIR> Debug for Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(
            f,
            "Motor {{ Dir : Unknown, Pwm : {} }}",
            self.pwm.get_duty()
        )
    }
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
                self.dir.set_high();
            }
            Command::Back(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_low();
            }
        }
    }
}

#[cfg(test)]
pub mod test {
    use std::cell::Cell;
    use std::rc::Rc;

    use embedded_hal::Qei;

    use super::Command;

    #[derive(Debug, Clone, Copy)]
    enum Direction {
        Front,
        Back,
    }

    #[derive(Debug, Clone)]
    pub(crate) struct DummyMotor {
        speed: Rc<Cell<u16>>,
        real_position: Rc<Cell<i64>>,
        wrapped_position: Rc<Cell<u16>>,
        dir: Rc<Cell<Direction>>,
    }

    impl DummyMotor {
        pub(crate) fn new() -> Self {
            DummyMotor {
                speed: Rc::new(Cell::new(0)),
                real_position: Rc::new(Cell::new(0)),
                wrapped_position: Rc::new(Cell::new(0)),
                dir: Rc::new(Cell::new(Direction::Front)),
            }
        }

        pub(crate) fn update(&mut self) {
            match self.dir.get() {
                Direction::Front => {
                    self.real_position
                        .replace(self.real_position.get() + self.speed.get() as i64);
                    self.wrapped_position
                        .replace(self.wrapped_position.get().wrapping_add(self.speed.get()));
                }
                Direction::Back => {
                    self.real_position
                        .replace(self.real_position.get() - self.speed.get() as i64);
                    self.wrapped_position
                        .replace(self.wrapped_position.get().wrapping_sub(self.speed.get()));
                }
            }
        }

        pub(crate) fn apply_command(&mut self, command: Command) {
            match command {
                Command::Front(speed) => {
                    self.speed.replace(speed / 5);
                    self.dir.replace(Direction::Front);
                }
                Command::Back(speed) => {
                    self.speed.replace(speed / 5);
                    self.dir.replace(Direction::Back);
                }
            }
        }

        pub(crate) fn get_real_position(&self) -> i64 {
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
}
