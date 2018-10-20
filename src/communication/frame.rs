//! Les trames sont le moyen de communication entre l'éléctronique et l'informatique. Un Frame
//! contiens plusieurs champs :
//! * Un identifiant `id`
//! * Une commande `cmd`
//! * Un numéro de paquet optionnel `pnum`
//! * Des données `data` et un nombre de donnée `data_length`
//!
//! | Champs                                    | Valeur            |
//! |-------------------------------------------|-------------------|
//! | H1                                        | 0xAC              |
//! | H2                                        | 0xDC              |
//! | H3                                        | 0xAB              |
//! | Type                                      | 0xAB - 0xBA       |
//! |  Pnum (optionnel) sur 1 octet             | ?                 |
//! |  Id sur 1 octet                           | ?                 |
//! |  Cmd sur 1 cotet                          | ?                 |
//! |   Taille Donnée sur 1 octet Max : 8 octet | ?                 |
//! | Donnée (max 8 octet)                      | [?,?,?,?,?,?,?,?] |
//!
//! # Exemple
//!
//! Voici un exemple de communication PC-Elec :
//! ```c++
//! +----------------+           +--------------------+
//! |                |           |                    |
//! |  Raspberry PI  |           |  Microcontrolleur  |
//! |                |           |                    |
//! +--------+-------+           +----------+---------+
//!          |                              |
//!          |  AC DC AB BA 01 05 06 00     |
//!          | +--------------------------> |
//!          |                              |
//!          |  AC DC AB BB 01 00 00        |
//!          | <--------------------------+ |
//!          |                              |
//!          |  AC DC AB BA 05 46 02 11 77  |
//!          | <--------------------------+ |
//!          |                              |
//!          v(t)                           v(t)
//! ```
//!
//! La première trame comporte un header `AC DC AB`, le type de trame (normal) `0xBA` puis le numéro de paquet `01`, un
//! identifiant `05` une commande `06` et `00` données.
//!
//! Le microcontrolleur réponds alors avec une trame d'acquitement, son type est `BB` et il est
//! suivi du numéro de paquet `01` et de deux octets nuls `00 00`.
//!
//! Enfin, après avoir traité la trame informatique, le microcontrolleur réponds à la commande `06`
//! avec un message d'id `05`, de commande `46` et `02` données `11` et `77`.

//use utils::*;
use arrayvec::ArrayVec;

/// La structure de donnée qui est utilisée pour la communication en electronique.
/// Pour la création d'une trame il vaut mieux utiliser la macro [frame!][macro@frame].
///
/// # Exemple
///
/// Création et conversion d'une [Frame] pour l'envoi de données :
///
/// ```ignore
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::frame::*;
/// # fn main() {
/// let t = frame!(0xFF,[0x55,0x66]);
/// let (arr,size) = t.into();
/// assert_eq!(&[0xAC,
///             0xDC,
///             0xAB,
///             0xBA,
///             0xFF,
///             0x11,
///             0x02,
///             0x55,
///             0x66],
///             &arr[0..size])
/// # }
/// ```
///
#[derive(Clone, Debug, Default, Eq)]
pub struct Frame {
    /// L'identifiant d'une trame.
    pub id: u8,
    /// Le numéro de commande d'une trame.
    //pub cmd: u8,
    /// Le numéro de paquet optionnel d'une trame.
    //pub pnum: Option<u8>,
    /// Le nombre de donnée dans la trame.
    //pub data_length: u8,
    /// Les données de la trame.
    pub data: ArrayVec<[u8; 256]>,
}

/*impl Default for Frame {
    fn default() -> Self {
        Frame {
            id: 0,
            data_length: 0,
            data: [0; 256],
        }
    }
}*/

impl PartialEq for Frame {
    fn eq(&self, rhs: &Frame) -> bool {
        self.id == rhs.id
            //&& self.cmd == rhs.cmd
            //&& self.pnum == rhs.pnum
            //&& self.data_length == rhs.data_length
            && self.data == rhs.data
    }
}

/// Macro permettant de créer une nouvelle [Frame] facilement. La gestion des données fait appel à
/// [Frame::push][frame::Frame::push], donc si il y a plus de 8 données elles seront ignorées.
///
/// # Exemple
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::frame::*;
/// # use librobot::frame::Frame;
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

    ($id:expr/*, $cmd:expr*/) => {{
        let mut frame = Frame::default();
        frame.id = $id;
        //frame.cmd = $cmd;
        frame
    }};

    ($id:expr/*, $cmd:expr*/, $arr:expr) => {{
        let mut t = frame!(/*, $cmd*/ $id);
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
    ///  # use librobot::frame::Frame;
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
    pub fn new(
        id: u8,
        //cmd: u8,
        //pnum: T,
        //data_length: u8,
        data: ArrayVec<[u8; 256]>,
    ) -> Frame {
        let mut t: Frame = Default::default();
        t.id = id;
        //t.cmd = cmd;
        //t.pnum = pnum.into();
        //t.data_length = data_length;
        t.data = data;
        t
    }

    /// Crée une nouvelle trame à partir des données fournies. Si `data` contiens plus de 8 données,
    /// celles-ci sont ignorées. `data` peut contenir moins de 8 données.
    ///
    /// # Exemple
    /// ```ignore
    /// # use librobot::frame::Frame;
    /// let t1 = Frame::new_from_slice(0x80,0xAA, None, &[0,1,2,3,4,5]);
    /// let t2 = Frame{id : 0x80,
    ///                cmd : 0xAA,
    ///                pnum : None,
    ///                data_length : 6,
    ///                data : [0,1,2,3,4,5,0,0]};
    /// /* Comme la longueur des données est de 6, les autres données seront ignorées */
    /// assert_eq!(t1,t2);
    /// ```
    /// Il vaut mieux utiliser la macro [frame!][macro@frame] pour construire des trames.
    ///
    /*pub fn new_from_slice<T: Into<Option<u8>>>(id: u8, data: &[u8]) -> Frame {
        let (data_copy, size) = slice_to_array_8(data);
        Frame::new(id, size, data_copy)
    }*/

    /// Crée une nouvelle trame de pong :
    /// * `id` : la valeur passée en argument
    /// * `cmd` : `0x00`
    /// * pnum : la valeur passée en argument
    /// * data_length : `1`
    /// * `data` : `[0xAA]`
    /*pub fn new_pong<T: Into<Option<u8>>>(id: u8, pnum: T) -> Frame {
        Frame::new(id, 0, pnum, 1, [0xAA, 0, 0, 0, 0, 0, 0, 0])
    }*/

    /// Crée une nouvelle trame d'acquitement.
    /// * `id` : `0`
    /// * `cmd` : `0`
    /// * `pnum` : `pnum`
    /// * `data_length` : `0`
    /// * `data` : `[]`
    /*pub fn new_ack(pnum: u8) -> Frame {
        let mut t = frame!();
        t.pnum = Some(pnum);
        t
    }*/

    /// Renvoie vrai si il s'agit d'une trame de ping.
    /// C'est à dire que :
    /// * `cmd == 0`
    /// * `data_length == 1`
    /// * `data[0] == 0x55`
    /*pub fn is_ping(self) -> bool {
        self.cmd == 0 && self.data_length == 1 && self.data[0] == 0x55
    }*/

    /// Renvoie vrai si il s'agit d'une trame de pong.
    /*pub fn is_pong(self) -> bool {
        self.cmd == 0 && self.data_length == 1 && self.data[0] == 0xAA
    }*/

    /// Rajoute un octet de donnée dans la trame.
    /// Renvoi `Err<()>` quand la trame a déjà 8 données.
    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.data.len() < 255 {
            self.data.push(data);
            //self.data_length += 1;
            Ok(())
        } else {
            Err(())
        }
    }
}

/// Multiplex l'ID et la commande pour la transmission. Le premier bit doit être écris en premier.
/*pub fn multiplex_id_cmd(id: u8, cmd: u8) -> (u8, u8) {
    let first = id.wrapping_shr(4) & 0x0F;
    let second = (cmd & 0x0F) + id.wrapping_shl(4);
    (first, second)
}*/

/// Demultiplex l'ID et la commande d'une trame que l'on viens de recevoir. Il faut passer
/// en premier les bits de poids forts (ceux qu'on a lu en premier).
/*pub fn demultiplex_id_cmd(first: u8, second: u8) -> (u8, u8) {
    let data = make_u16(first, second);
    let id: u8 = (data.wrapping_shr(4)) as u8;
    let cmd: u8 = (data as u8) & 0x0F;
    (id, cmd)
}*/

fn make_u16(high: u8, low: u8) -> u16 {
    low as u16 + ((high as u16).wrapping_shr(8))
}

impl Into<ArrayVec<[u8; 256]>> for Frame {
    fn into(self) -> ArrayVec<[u8; 256]> {
        // Taille du tableau : 3 octet de header
        //                   + 1 octet de type
        //                   + 1 octet d'id
        //                   + 1 octet de commande
        //                   + 1 octet pour la taille des données
        //                   + `data_length` octet
        //                   ---------------------
        //                   = 7 + data_length octet
        //                   = 7 + 8 au plus
        //                   --------------------
        //                   = 15 au plus
        let mut arr = ArrayVec::<[u8; 256]>::new();
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
    use frame::*;

    #[test]
    fn frame_macro() {
        let t = frame!(0x1, /*0x11, */ [1, 2, 3]);
        let mut array = ArrayVec::<[u8; 256]>::new();
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
        let bytes: ArrayVec<[u8; 256]> = t.clone().into();
        let mut expected_result = ArrayVec::<[u8; 256]>::new();
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
