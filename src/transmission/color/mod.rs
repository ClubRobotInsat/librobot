use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
pub enum Color {
    Red,
    Green,
    Blue,
}

impl Jsonizable for Color {
    fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>,
    {
        to_string(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::consts::U2048;

    #[test]
    fn color_ser() {
        let expected_value = "\"Red\"";
        let color = Color::Red;
        assert_eq!(color.to_string::<U2048>().unwrap(), expected_value);
        let expected_value = "\"Blue\"";
        let color = Color::Blue;
        assert_eq!(color.to_string::<U2048>().unwrap(), expected_value);
        let expected_value = "\"Green\"";
        let color = Color::Green;
        assert_eq!(color.to_string::<U2048>().unwrap(), expected_value);
    }

}
