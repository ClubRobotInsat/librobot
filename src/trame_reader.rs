//! Une machine à état finis permettant de lire des [Trames](struct.Trame.html) depuis un flux d'octet.

use trame::Trame;

use arrayvec::ArrayVec;

/// La taille du buffer interne dans lesquels sont stockés les [Trame]s lues par tous les
/// [TrameReader].
pub const TRAME_READER_INTERNAL_BUFFER_SIZE: usize = 2048;

#[derive(Debug)]
pub(crate) enum TrameReaderState {
    H1,
    H2,
    H3,
    TypeTrame,
    Id {
        pnum: u8,
    },
    Cmd {
        id: u8,
        pnum: u8,
    },
    Pnum,
    DataLength {
        id: u8,
        cmd: u8,
        pnum: u8,
    },
    Data {
        id: u8,
        cmd: u8,
        pnum: u8,
        data_length: u8,
        data: [u8; 8],
        current_index: u8,
    },
}

/// Déserialise des [Trame] depuis un flux d'octet.
/// types `T` implémentant le trait.
///
/// Les trames lus sont stockés dans un buffer de taille [TRAME_READER_INTERNAL_BUFFER_SIZE].
///
#[derive(Debug)]
pub struct TrameReader {
    pub(crate) state: TrameReaderState,
    buffer: ArrayVec<[Trame; TRAME_READER_INTERNAL_BUFFER_SIZE]>,
}

impl TrameReader {
    /// Crée une nouvelle machine à état s'appuyant sur `reader` pour lire des trames.
    /// La taille du buffer est fixée à la compilation, cf [TRAME_READER_INTERNAL_BUFFER_SIZE].
    pub fn new() -> TrameReader {
        TrameReader {
            state: TrameReaderState::H1,
            buffer: ArrayVec::new(),
        }
    }

    /// Renvoie la plus vieille trame non lue et la supprime du buffer.
    ///
    /// # Notes
    ///
    /// Si aucune [Trame] n'est présente dans le buffer, renvoie `None`
    pub fn pop_trame(&mut self) -> Option<Trame> {
        self.buffer.pop()
    }

    /// Renvoie le nombre de trames dans le buffer.
    pub fn get_buffer_size(&mut self) -> usize {
        self.buffer.len()
    }

    /// Fais avancer la machine à état en lui donnant en entrée tous les octets dans le buffer
    /// `buf`.
    /// ```
    /// # #[macro_use]
    /// # extern crate librobot;
    /// # use librobot::trame::*;
    /// # use librobot::trame_reader::*;
    /// # fn main() {
    /// let mut reader = TrameReader::new();
    /// let trame : [u8;13] = [0xAC, // Header 1
    ///                        0xDC, // Header 2
    ///                        0xAB, // Header 3
    ///                        0xBA, // Type de trame
    ///                        0x66, // Numéro de paquet
    ///                        0xAA, // ID
    ///                        0x01, // CMD
    ///                        0x05, // Data Length
    ///                        1,    // Data 1
    ///                        2,    // Data 2
    ///                        3,    // Data 3
    ///                        4,    // Data 4
    ///                        5];   // Data 5
    /// reader.parse(&trame);
    /// let t1 = trame!(0xAA, 0x01, 0x66, [1,2,3,4,5]);
    /// assert_eq!(t1, reader.pop_trame().unwrap());
    /// assert_eq!(reader.get_buffer_size(),0);
    /// # }
    /// ```
    ///
    pub fn parse(&mut self, buf: &[u8]) {
        for byte in buf {
            self.step(*byte);
        }
    }

    fn step(&mut self, byte: u8) {
        use trame_reader::TrameReaderState::*;
        match self.state {
            H1 if byte == 0xAC => self.state = H2,
            H2 if byte == 0xDC => self.state = H3,
            H3 if byte == 0xAB => self.state = TypeTrame,

            TypeTrame if byte == 0xBA => self.state = Pnum,

            Pnum => self.state = Id { pnum: byte },

            Id { pnum } => {
                self.state = Cmd {
                    id: byte,
                    pnum: pnum,
                }
            }

            Cmd { id, pnum } => {
                self.state = DataLength {
                    id: id,
                    cmd: byte,
                    pnum: pnum,
                };
            }

            DataLength { id, cmd, pnum } if byte > 0 && byte <= 8 => {
                self.state = Data {
                    id: id,
                    cmd: cmd,
                    pnum: pnum,
                    data_length: byte,
                    data: [0; 8],
                    current_index: 0,
                };
            }

            DataLength { id, cmd, pnum } if byte == 0 => {
                self.state = H1;
                let t = Trame::new(id, cmd, Some(pnum), 0, [0, 0, 0, 0, 0, 0, 0, 0]);
                self.buffer.push(t);
            }

            Data {
                id,
                cmd,
                pnum,
                data_length,
                mut data,
                current_index,
            } if current_index < data_length - 1 =>
            {
                data[current_index as usize] = byte;
                self.state = Data {
                    id: id,
                    cmd: cmd,
                    pnum: pnum,
                    data_length: data_length,
                    current_index: current_index + 1,
                    data: data,
                };
            }

            Data {
                id,
                cmd,
                pnum,
                data_length,
                mut data,
                current_index,
            } if current_index == data_length - 1 =>
            {
                data[current_index as usize] = byte;
                let t: Trame = Trame::new(id, cmd, Some(pnum), data_length, data);
                self.buffer.push(t);
                self.state = H1;
            }

            _ => {
                self.state = H1;
            }
        }
    }
}

#[cfg(test)]
mod test {

    use trame::*;
    use trame_reader::*;

    fn trame_to_u8_with_pnum(t: Trame, pnum: u8) -> [u8; 16] {
        let mut result = [0; 16];
        let (arr, size) = t.into();
        for i in 0..16 {
            match i {
                0...3 => result[i] = arr[i],
                5...16 if i < size + 1 => result[i] = arr[i - 1],
                4 => result[i] = pnum,
                _ => {}
            }
        }
        result
    }

    #[test]
    fn trame_reader_buffer() {
        let mut reader = TrameReader::new();
        assert_eq!(reader.pop_trame(), None);
    }

    #[test]
    fn trame_reader_standard_trame() {
        let mut reader = TrameReader::new();
        {
            let t1 = trame!(0xAA, 0xBB, 0x99, [5, 6, 7, 8, 9, 10]);
            let mut arr = trame_to_u8_with_pnum(t1, t1.pnum.unwrap());
            reader.parse(&arr);
            assert_eq!(reader.pop_trame().unwrap(), t1);
            assert_eq!(reader.get_buffer_size(), 0);
            let t2 = trame!(0xFF, 0x00, 0x00, []);
            arr = trame_to_u8_with_pnum(t2, t2.pnum.unwrap());
            reader.parse(&arr);
            assert_eq!(reader.pop_trame().unwrap(), t2);
            assert_eq!(reader.get_buffer_size(), 0);
            let t3 = trame!(0x00, 0x00, 0xFF, [1, 2, 3, 4, 5, 6, 7, 8]);
            arr = trame_to_u8_with_pnum(t3, t3.pnum.unwrap());
            reader.parse(&arr);
            assert_eq!(reader.pop_trame().unwrap(), t3);
            assert_eq!(reader.get_buffer_size(), 0);
        }
    }

    #[test]
    fn trame_reader_special_trame() {
        let mut reader = TrameReader::new();
        let arr_header = [0xAC, 0xDC, 0xAB, 0xBA, 0x00, 0xFF, 0xFF, 0xFF, 0x9];
        let mut data: [u8; 107] = [0; 107];
        data[0..8].clone_from_slice(&arr_header[0..8]);
        reader.parse(&data);
        assert_eq!(reader.pop_trame(), None);
        assert_eq!(reader.get_buffer_size(), 0);
    }
}
