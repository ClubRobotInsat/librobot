//! Les trames sont le moyen de communication entre l'éléctronique et l'informatique. Un Trame
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

use utils::*;

/// La structure de donnée qui est utilisée pour la communication en electronique.
/// Pour la création d'une trame il vaut mieux utiliser la macro [trame!][macro@trame].
///
/// # Exemple
///
/// Création et conversion d'une [Trame] pour l'envoi de données :
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::trame::*;
/// # fn main() {
/// let t = trame!(0xFF,0x11,[0x55,0x66]);
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
#[derive(Copy, Clone, Debug, Default, Eq)]
pub struct Trame {
    /// L'identifiant d'une trame.
    pub id: u8,
    /// Le numéro de commande d'une trame.
    pub cmd: u8,
    /// Le numéro de paquet optionnel d'une trame.
    pub pnum: Option<u8>,
    /// Le nombre de donnée dans la trame.
    pub data_length: u8,
    /// Les données de la trame.
    pub data: [u8; 8],
}

impl PartialEq for Trame {
    fn eq(&self, rhs: &Trame) -> bool {
        self.id == rhs.id
            && self.cmd == rhs.cmd
            && self.pnum == rhs.pnum
            && self.data_length == rhs.data_length
            && self.data[0..self.data_length as usize] == rhs.data[0..rhs.data_length as usize]
    }
}

/// Macro permettant de créer une nouvelle [Trame] facilement. La gestion des données fait appel à
/// [Trame::push][trame::Trame::push], donc si il y a plus de 8 données elles seront ignorées.
///
/// # Exemple
///
/// ```
/// # #[macro_use]
/// # extern crate librobot;
/// # use librobot::trame::*;
/// # use librobot::trame::Trame;
/// # fn main() {
/// // Une trame avec seulement un ID et une commande.
/// let t1 = trame!(0xAA,0xBC);
///
/// // Une trame avec un ID, une commande et 8 données : [1,2,3,4,5,6,7,8].
/// // Les données en trop sont ignorées !
/// let t2 = trame!(0xAA,0xBC,[1,2,3,4,5,6,7,8,9,10,11]);
///
/// // Une trame avec un ID, une commande et 5 données : [1,2,3,4,5]
/// let t3 = trame!(0xAA,0xBC,[1,2,3,4,5]);
///
/// assert_eq!(t1, Trame{id:0xAA,cmd:0xBC,..Trame::default()});
///
/// assert_eq!(t2, Trame{id:0xAA,
///                      cmd:0xBC,
///                      data_length:8,
///                      data:[1,2,3,4,5,6,7,8],
///                      ..Trame::default()});
/// assert_eq!(t3, Trame{id:0xAA,
///                      cmd:0xBC,
///                      data_length:5,
///                      data:[1,2,3,4,5,0,0,0],
///                      ..Trame::default()});
///
/// # }
/// ```
///
/// # Limitations
///
/// La macro ne permet pas de gérer le numéro de paquet.
///
#[macro_export]
macro_rules! trame {
    () => {
        Trame::default()
    };

    ($id:expr, $cmd:expr) => {{
        let mut trame = Trame::default();
        trame.id = $id;
        trame.cmd = $cmd;
        trame
    }};

    ($id:expr, $cmd:expr, $arr:expr) => {{
        let mut t = trame!($id, $cmd);
        for i in $arr.iter() {
            let _ = t.push(*i);
        }
        t
    }};
}

impl Trame {
    /// Permet de créer une nouvelle trame.
    ///
    /// # Exemple
    ///
    /// Création d'une [Trame] :
    ///
    /// ```
    ///  # use librobot::trame::Trame;
    ///  let t1 = Trame::new(0x80,0xAA, None,0,[0,0,0,0,0,0,0,0]);
    ///  let t2 = Trame{ id: 0x80,
    ///                  cmd : 0xAA,
    ///                  pnum : None,
    ///                  data_length : 0,
    ///                  data : [0,0,0,0,0,0,0,0]};
    ///  assert_eq!(t1,t2);
    /// ```
    ///
    /// # Notes
    ///
    /// Il vaut mieux utiliser la macro [trame!][macro@trame] pour construire des trames.
    ///
    pub fn new<T: Into<Option<u8>>>(
        id: u8,
        cmd: u8,
        pnum: T,
        data_length: u8,
        data: [u8; 8],
    ) -> Trame {
        let mut t: Trame = Default::default();
        t.id = id;
        t.cmd = cmd;
        t.pnum = pnum.into();
        t.data_length = data_length;
        t.data = data;
        t
    }

    /// Crée une nouvelle trame à partir des données fournies. Si `data` contiens plus de 8 données,
    /// celles-ci sont ignorées. `data` peut contenir moins de 8 données.
    ///
    /// # Exemple
    /// ```
    /// # use librobot::trame::Trame;
    /// let t1 = Trame::new_from_slice(0x80,0xAA, None, &[0,1,2,3,4,5]);
    /// let t2 = Trame{id : 0x80,
    ///                cmd : 0xAA,
    ///                pnum : None,
    ///                data_length : 6,
    ///                data : [0,1,2,3,4,5,0,0]};
    /// /* Comme la longueur des données est de 6, les autres données seront ignorées */
    /// assert_eq!(t1,t2);
    /// ```
    /// Il vaut mieux utiliser la macro [trame!][macro@trame] pour construire des trames.
    ///
    pub fn new_from_slice<T: Into<Option<u8>>>(id: u8, cmd: u8, pnum: T, data: &[u8]) -> Trame {
        let (data_copy, size) = slice_to_array_8(data);
        Trame::new(id, cmd, pnum, size, data_copy)
    }

    /// Crée une nouvelle trame de pong :
    /// * `id` : la valeur passée en argument
    /// * `cmd` : `0x00`
    /// * pnum : la valeur passée en argument
    /// * data_length : `1`
    /// * `data` : `[0xAA]`
    pub fn new_pong<T: Into<Option<u8>>>(id: u8, pnum: T) -> Trame {
        Trame::new(id, 0, pnum, 1, [0xAA, 0, 0, 0, 0, 0, 0, 0])
    }

    /// Crée une nouvelle trame d'acquitement.
    /// * `id` : `0`
    /// * `cmd` : `0`
    /// * `pnum` : `pnum`
    /// * `data_length` : `0`
    /// * `data` : `[]`
    pub fn new_ack(pnum: u8) -> Trame {
        let mut t = trame!();
        t.pnum = Some(pnum);
        t
    }

    /// Renvoie vrai si il s'agit d'une trame de ping.
    /// C'est à dire que :
    /// * `cmd == 0`
    /// * `data_length == 1`
    /// * `data[0] == 0x55`
    pub fn is_ping(self) -> bool {
        self.cmd == 0 && self.data_length == 1 && self.data[0] == 0x55
    }

    /// Renvoie vrai si il s'agit d'une trame de pong.
    pub fn is_pong(self) -> bool {
        self.cmd == 0 && self.data_length == 1 && self.data[0] == 0xAA
    }

    /// Rajoute un octet de donnée dans la trame.
    /// Renvoies `Err<()>` quand la trame a déjà 8 données.
    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.data_length < 8 {
            self.data[self.data_length as usize] = data;
            self.data_length += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    /// Permet de modifier facilement le numéro de paquet d'une [Trame].
    ///
    /// # Exemple
    /// ```
    /// # #[macro_use]
    /// # extern crate librobot;
    /// # use librobot::trame::Trame;
    /// # fn main() {
    /// let mut trame = trame!();
    /// // Cette ligne est équivalente ...
    /// trame.set_pnum(0x96);
    /// // à celle là
    /// trame.pnum = Some(0x96);
    /// # }
    /// ```
    pub fn set_pnum<T: Into<Option<u8>>>(&mut self, val: T) {
        self.pnum = val.into();
    }
}

/// Multiplex l'ID et la commande pour la transmission. Le premier bit doit être écris en premier.
pub fn multiplex_id_cmd(id: u8, cmd: u8) -> (u8, u8) {
    let first = id.wrapping_shr(4) & 0x0F;
    let second = (cmd & 0x0F) + id.wrapping_shl(4);
    (first, second)
}

/// Demultiplex l'ID et la commande d'une trame que l'on viens de recevoir. Il faut passer
/// en premier les bits de poids forts (ceux qu'on a lu en premier).
pub fn demultiplex_id_cmd(first: u8, second: u8) -> (u8, u8) {
    let data = make_u16(first, second);
    let id: u8 = (data.wrapping_shr(4)) as u8;
    let cmd: u8 = (data as u8) & 0x0F;
    (id, cmd)
}

fn make_u16(high: u8, low: u8) -> u16 {
    low as u16 + ((high as u16).wrapping_shr(8))
}

impl Into<([u8; 15], usize)> for Trame {
    fn into(self) -> ([u8; 15], usize) {
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
        let arr = [
            0xAC,
            0xDC,
            0xAB,
            0xBA,
            self.id,
            self.cmd,
            self.data_length,
            self.data[0],
            self.data[1],
            self.data[2],
            self.data[3],
            self.data[4],
            self.data[5],
            self.data[6],
            self.data[7],
        ];
        (arr, self.data_length as usize + 7)
    }
}

#[cfg(test)]
mod test {
    use trame::*;

    #[test]
    fn trame_macro() {
        let t = trame!(0x1, 0x11, [0, 0, 0]);
        assert_eq!(
            t,
            Trame {
                id: 0x01,
                cmd: 0x11,
                pnum: None,
                data_length: 3,
                data: [0, 0, 0, 0, 0, 0, 0, 0]
            }
        );
    }

    #[test]
    fn trame_pong() {
        assert!(Trame::new_pong(5, None).is_pong());
    }

    #[test]
    fn trame_ack() {
        let mut result = trame!();
        result.set_pnum(0x96);
        assert_eq!(Trame::new_ack(0x96), result);
    }

    #[test]
    fn trame_multiplex_id_cmd() {
        let (id, cmd) = (6, 9);
        let (m_id, m_cmd) = multiplex_id_cmd(id, cmd);
        let (c_id, c_cmd) = demultiplex_id_cmd(m_id, m_cmd);
        assert_eq!((id, cmd), (c_id, c_cmd));
    }

    #[test]
    fn trame_conversion() {
        let t = trame!(0xFF, 0x11, [0x55, 0x66, 0x1, 2, 3, 4, 5, 6]);
        let (arr, size) = t.into();
        assert_eq!(
            &[0xAC, 0xDC, 0xAB, 0xBA, 0xFF, 0x11, 8, 0x55, 0x66, 0x1, 2, 3, 4, 5, 6],
            &arr[0..size]
        );
        let t = trame!(0xDD,0xCC);
        let (arr, size) = t.into();
        assert_eq!(&[0xAC, 0xDC, 0xAB, 0xBA, 0xDD, 0xCC, 0],
                   &arr[0..size]);
    }

}
