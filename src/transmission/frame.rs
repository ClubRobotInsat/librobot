use transmission::Message;

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
/// let t = frame!(0xFF,[0x55,0x66]);
/// let arr: arrayvec::ArrayVec<[u8; 256]> = t.into();
/// assert_eq!(&[0xAC,
///             0xDC,
///             0xAB,
///             0xBA,
///             3,
///             0xFF,
///             0x55,
///             0x66],
///             &arr[0..8])
/// # }
/// ```
///
#[derive(Clone, Debug, Default, Eq)]
pub struct Frame {
    /// L'identifiant d'une trame.
    pub id: u8,
    /// Les données de la trame.
    pub data: Message,
}

impl PartialEq for Frame {
    fn eq(&self, rhs: &Frame) -> bool {
        self.id == rhs.id && self.data == rhs.data
    }
}

/// Macro permettant de créer une nouvelle [Frame] facilement. La gestion des données fait appel à
/// [`Frame::push`][Frame::push], donc si il y a plus de 8 données elles seront ignorées.
///
/// # Exemple
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::transmission::*;
/// # fn main() {
/// // Une trame avec seulement un ID.
/// let t1 = frame!(0xAA);
///
/// // Une trame avec un ID, une commande et 8 données : [1,2,3,4,5,6,7,8].
/// // Les données en trop sont ignorées !
/// let t2 = frame!(0xAA, [1,2,3,4,5,6,7,8]);
///
/// assert_eq!(t1, Frame{id:0xAA,..Frame::default()});
///
/// let mut array = arrayvec::ArrayVec::<[u8; 256]>::new();
/// for i in 1..9 {
///     array.push(i);
/// }
/// assert_eq!(t2, Frame{id:0xAA,
///                      data: array,
///                      ..Frame::default()});
/// # }
/// ```
///
/// # Limitations
///
/// La macro ne permet pas de gérer le numéro de paquet.
///
#[macro_export]
macro_rules! frame {
    () => {
        Frame::default()
    };

    ($id:expr) => {{
        let mut frame = Frame::default();
        frame.id = $id;
        frame
    }};

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
    ///  let t1 = Frame::new(0x80, arrayvec::ArrayVec::<[u8; 256]>::new());
    ///  let t2 = Frame{ id: 0x80,
    ///                  data : arrayvec::ArrayVec::<[u8; 256]>::new()};
    ///  assert_eq!(t1,t2);
    /// ```
    ///
    /// # Notes
    ///
    /// Il vaut mieux utiliser la macro [frame!][macro@frame] pour construire des trames.
    ///
    pub fn new(id: u8, data: Message) -> Frame {
        let mut t: Frame = Default::default();
        t.id = id;
        t.data = data;
        t
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
        arr.push(0xAC);
        arr.push(0xDC);
        arr.push(0xAB);
        arr.push(0xBA);
        arr.push(1 + self.data.len() as u8);
        arr.push(self.id);
        for byte in self.data.iter() {
            arr.push(*byte);
        }
        arr
    }
}

#[cfg(test)]
mod test {
    use transmission::*;

    #[test]
    fn frame_macro() {
        let t = frame!(0x1, [1, 2, 3]);
        let mut array = Message::new();
        array.push(1);
        array.push(2);
        array.push(3);
        assert_eq!(
            t,
            Frame {
                id: 0x01,
                data: array,
            }
        );
        assert_eq!(3, t.data.len());
    }

    #[test]
    fn frame_conversion() {
        let t = frame!(0xFF, [0x55, 0x66, 0x1, 2, 3, 4, 5, 6]);
        let bytes: Message = t.clone().into();
        let mut expected_result = Message::new();
        expected_result.push(0xAC);
        expected_result.push(0xDC);
        expected_result.push(0xAB);
        expected_result.push(0xBA);
        expected_result.push(9);
        expected_result.push(0xFF);
        expected_result.push(0x55);
        expected_result.push(0x66);
        expected_result.push(0x1);
        expected_result.push(2);
        expected_result.push(3);
        expected_result.push(4);
        expected_result.push(5);
        expected_result.push(6);
        assert_eq!(bytes, expected_result);
    }
}
