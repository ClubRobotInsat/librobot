//! Ce module contiens du code permettant de gérer les unités de longeurs

use core::fmt::{Display, Formatter, Result};
use core::ops::{Add, Div, Mul, Sub};

/// Une longueur exprimée en millimètre
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MilliMeter(pub i64);

impl MilliMeter {
    /// Récupère la valeur en mètre
    pub fn as_meters(&self) -> i64 {
        self.as_millimeters() / 1000
    }

    /// Récupère la valeur en centimètre
    pub fn as_centimeters(&self) -> i64 {
        self.as_millimeters() / 10
    }

    /// Récupère la valeur en millimètre
    pub fn as_millimeters(&self) -> i64 {
        self.0
    }
}

impl Display for MilliMeter {
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(f, "{} mm", self.as_millimeters())
    }
}

impl Add for MilliMeter {
    type Output = MilliMeter;
    fn add(self, rhs: MilliMeter) -> Self::Output {
        MilliMeter(self.as_millimeters() + rhs.as_millimeters())
    }
}

impl Div for MilliMeter {
    type Output = MilliMeter;
    fn div(self, rhs: MilliMeter) -> Self::Output {
        MilliMeter(self.as_millimeters() / rhs.as_millimeters())
    }
}

impl Mul for MilliMeter {
    type Output = MilliMeter;
    fn mul(self, rhs: MilliMeter) -> Self::Output {
        MilliMeter(self.as_millimeters() * rhs.as_millimeters())
    }
}

impl Sub for MilliMeter {
    type Output = MilliMeter;
    fn sub(self, rhs: MilliMeter) -> Self::Output {
        MilliMeter(self.as_millimeters() - rhs.as_millimeters())
    }
}

#[cfg(test)]
mod test {

    use units::MilliMeter;

    #[test]
    fn mm_to_meter() {
        let x = MilliMeter(56000);
        assert_eq!(x.as_meters(), 56);
    }

    #[test]
    fn mm_to_centimeter() {
        let x = MilliMeter(560);
        assert_eq!(x.as_centimeters(), 56);
    }

    #[test]
    fn mm_arithmetic_operations() {
        let x = MilliMeter(43);
        let y = MilliMeter(5);
        assert_eq!(x + y, MilliMeter(48));
        assert_eq!(x - y, MilliMeter(38));
        assert_eq!(x / y, MilliMeter(8));
        assert_eq!(x * y, MilliMeter(215));
    }

}
