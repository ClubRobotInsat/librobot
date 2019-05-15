use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

/// Trame contenant les paramètres de la navigation, pour permettre un
/// changement en direct des paramètres du robot (concernant l'odométrie,
/// les coefficients du PID, etc)
#[derive(Debug, Copy, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct NavigationParametersFrame {
    /// Le rayon d'une roue codeuse en dixièmes de mm
    pub coder_radius: u16,
    /// Coefficient de correction de la roue codeuse droite en fixé 16 bits
    pub right_wheel_coef: u32,
    /// La distance entre les roues codeuses en dixièmes de mm
    pub inter_axial_length: u16,
    /// Le coefficient proportionnel sur la position en fixé 16 bits
    pub pos_kp: u32,
    /// Le coefficient dérivé sur la position en fixé 16 bits
    pub pos_kd: u32,
    /// Le coefficient proportionnel sur l'orientation en fixé 16 bits
    pub orient_kp: u32,
    /// Le coefficient dérivée sur l'orientation en fixé 16 bits
    pub orient_kd: u32,
}

impl Jsonizable for NavigationParametersFrame {
    /// Construit une trame a partir d'un flux de donnees json.
    fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    /// Construit une chaine de caractère en json à partir de cette trame
    fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>,
    {
        to_string(self)
    }
}
