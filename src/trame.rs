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
///
#[derive(Debug, Default, Eq)]
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
        self.id == rhs.id && self.cmd == rhs.cmd && self.pnum == rhs.pnum
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
        let t = trame!();
        t.pnum = pnum;
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
}

#[cfg(test)]
mod test {
    use trame::Trame;

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

}
