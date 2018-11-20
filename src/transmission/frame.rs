use transmission::{MessageKind,Message};

/// La structure de donnée qui est utilisée pour la communication en electronique.
/// Pour la création d'une trame il vaut mieux utiliser la macro [frame!][macro@frame].
///
/// # Exemple
///
/// Création et conversion d'une [Frame] pour l'envoi de données :
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::transmission::*;
/// # fn main() {
/// let t = frame!(MessageKind::Servo,[0x55,0x66]);
/// let arr: arrayvec::ArrayVec<[u8; 256]> = t.into();
/// assert_eq!(&[0x4,
///             0x55,
///             0x66],
///             &arr[0..3])
/// # }
/// ```
///
#[derive(Clone, Debug, Eq)]
pub struct Frame {
    /// L'identifiant d'une trame.
    pub kind: MessageKind,
    /// Les données de la trame.
    pub data: Message,
}

impl PartialEq for Frame {
    fn eq(&self, rhs: &Frame) -> bool {
        self.kind == rhs.kind && self.data == rhs.data
    }
}

/// Macro permettant de créer une nouvelle [Frame] facilement. La gestion des données fait appel à
/// [Frame::push][Frame::push], donc si il y a plus de 8 données elles seront ignorées.
///
/// # Exemple
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::transmission::*;
/// # fn main() {
/// // Une trame avec seulement un ID.
/// let t1 = frame!(MessageKind::Servo);
///
/// // Une trame avec un ID, une commande et 8 données : [1,2,3,4,5,6,7,8].
/// // Les données en trop sont ignorées !
/// let t2 = frame!(MessageKind::Servo, [1,2,3,4,5,6,7,8]);
///
/// assert_eq!(t1, Frame{kind:MessageKind::Servo,data: Message::new()});
///
/// let mut array = arrayvec::ArrayVec::<[u8; 256]>::new();
/// for i in 1..9 {
///     array.push(i);
/// }
/// assert_eq!(t2, Frame{kind:MessageKind::Servo,
///                      data: array,
///                      });
/// # }
/// ```
///
/// # Limitations
///
/// La macro ne permet pas de gérer le numéro de paquet.
///
#[macro_export]
macro_rules! frame {
    ($kind:expr) => {
        Frame::new($kind,Message::new())
    };

    ($id:expr, $arr:expr) => {{
        let mut t = frame!($id);
        for i in $arr.iter() {
            let _ = t.push(*i);
        }
        t
    }};
}

impl Frame {
    /// Permet de créer une nouvelle trame.
    ///
    /// # Exemple
    ///
    /// Création d'une [Frame] :
    ///
    /// ```
    ///  # use librobot::transmission::*;
    ///  let t1 = Frame::new(MessageKind::Servo, arrayvec::ArrayVec::<[u8; 256]>::new());
    ///  let t2 = Frame{ kind: MessageKind::Servo,
    ///                  data : arrayvec::ArrayVec::<[u8; 256]>::new()};
    ///  assert_eq!(t1,t2);
    /// ```
    ///
    /// # Notes
    ///
    /// Il vaut mieux utiliser la macro [frame!][macro@frame] pour construire des trames.
    ///
    pub fn new(kind: MessageKind, data: Message) -> Frame {
        Frame {
            kind,
            data
        }
    }

    /// Rajoute un octet de donnée dans la trame.
    /// Renvoi `Err<()>` quand la trame a déjà 8 données.
    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.data.len() < 255 {
            self.data.push(data);
            Ok(())
        } else {
            Err(())
        }
    }
}

impl Into<Message> for Frame {
    fn into(self) -> Message {
        let mut arr = Message::new();
        arr.push(self.kind.into());
        for byte in self.data.iter() {
            arr.push(*byte);
        }
        arr
    }
}

#[cfg(test)]
mod test {
    use transmission::*;
    use transmission::MessageKind;

    #[test]
    fn frame_macro() {
        let t = frame!(MessageKind::Servo, [1, 2, 3]);
        let mut array = Message::new();
        array.push(1);
        array.push(2);
        array.push(3);
        assert_eq!(
            t,
            Frame {
                kind: MessageKind::Servo,
                data: array,
            }
        );
        assert_eq!(3, t.data.len());
    }

    #[test]
    fn frame_conversion() {
        let t = frame!(MessageKind::Servo, [0x55, 0x66, 0x1, 2, 3, 4, 5, 6]);
        let mut expected_result = Message::new();
        expected_result.push(0x4);
        expected_result.push(0x55);
        expected_result.push(0x66);
        expected_result.push(0x1);
        expected_result.push(2);
        expected_result.push(3);
        expected_result.push(4);
        expected_result.push(5);
        expected_result.push(6);

        let bytes: Message = t.clone().into();
        assert_eq!(bytes, expected_result);
    }
}
