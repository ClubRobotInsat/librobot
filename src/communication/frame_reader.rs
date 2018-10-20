//! Une machine à état finis permettant de lire des [Frames](struct.Frame.html) depuis un flux d'octet.

use frame::Frame;

use arrayvec::ArrayVec;

/// La taille du buffer interne dans lesquels sont stockés les [Frame]s lues par tous les
/// [FrameReader].
pub const FRAME_READER_INTERNAL_BUFFER_SIZE: usize = 256;
/// Taille maximale du message véhiculé par la trame
pub const FRAME_MAX_SIZE: usize = FRAME_READER_INTERNAL_BUFFER_SIZE /* - 6*/;

#[derive(Debug, Clone)]
pub(crate) enum FrameReaderState {
    H1,
    H2,
    H3,
    FrameType,
    BeginFrame,
    DataLength {
        length: u8,
    },
    Data {
        data_length: u8,
        id: u8,
        data: ArrayVec<[u8; FRAME_MAX_SIZE]>,
    },
    /*Id {
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
    },*/
}

/// Déserialise des [Frame] depuis un flux d'octet.
///
/// Les trames lues sont stockés dans un buffer de taille [FRAME_READER_INTERNAL_BUFFER_SIZE].
///
#[derive(Debug, Clone)]
pub struct FrameReader {
    state: FrameStateMachine,
    buffer: ArrayVec<[Frame; FRAME_READER_INTERNAL_BUFFER_SIZE]>,
}

/// Machine à état de la désérialisation du flux d'octets.
#[derive(Debug, Clone)]
pub struct FrameStateMachine {
    state: FrameReaderState,
}

impl FrameReader {
    /// Crée une nouvelle machine à état s'appuyant sur `reader` pour lire des trames.
    /// La taille du buffer est fixée à la compilation, cf [FRAME_READER_INTERNAL_BUFFER_SIZE].
    pub fn new() -> FrameReader {
        FrameReader {
            state: FrameStateMachine::new(),
            buffer: ArrayVec::new(),
        }
    }

    /// Renvoie la plus vieille trame non lue et la supprime du buffer.
    ///
    /// # Notes
    ///
    /// Si aucune [Frame] n'est présente dans le buffer, renvoie `None`
    pub fn pop_frame(&mut self) -> Option<Frame> {
        self.buffer.pop()
    }

    /// Renvoie le nombre de trames dans le buffer.
    pub fn get_buffer_size(&mut self) -> usize {
        self.buffer.len()
    }

    // TODO : update comments
    /// Fais avancer la machine à état en lui donnant en entrée tous les octets dans le buffer
    /// `buf`.
    /// ```ignore
    /// # #[macro_use]
    /// # extern crate librobot;
    /// # use librobot::frame::*;
    /// # use librobot::frame_reader::*;
    /// # fn main() {
    /// let mut reader = FrameStateMachine::new();
    /// let frame : [u8;13] = [0xAC, // Header 1
    ///                        0xDC, // Header 2
    ///                        0xAB, // Header 3
    ///                        0xBA, // Type de trame
    ///                        0x08, // Data Length
    ///                        0x05, // ID
    ///                        1,    // Data 1
    ///                        2,    // Data 2
    ///                        3,    // Data 3
    ///                        4,    // Data 4
    ///                        5,    // Data 5
    ///                        6,    // Data 6
    ///                        7];   // Data 7
    /// reader.parse(&frame);
    /// let t1 = frame!(0x05, [1,2,3,4,5,6,7]);
    /// assert_eq!(t1, reader.pop_frame().unwrap());
    /// assert_eq!(reader.get_buffer_size(),0);
    /// # }
    /// ```
    ///
    pub fn parse(&mut self, buf: &[u8]) {
        for byte in buf {
            self.step(*byte);
        }
    }

    /// Fais avancer la machine à état en fonction de l'octet lu suivant
    pub fn step(&mut self, byte: u8) {
        let (state, opt_frame) = self.state.clone().step(byte); // FIXME
        self.state = state;
        if let Some(frame) = opt_frame {
            self.buffer.push(frame);
        }
    }
}

impl FrameStateMachine {
    pub(crate) fn new() -> Self {
        FrameStateMachine {
            state: FrameReaderState::H1,
        }
    }

    /// Fais avancer la machine à état d'un octet.
    pub fn step(mut self, byte: u8) -> (Self, Option<Frame>) {
        use frame_reader::FrameReaderState::*;
        let mut result = None;
        (
            FrameStateMachine {
                state: match self.state {
                    H1 => {
                        if byte == 0xAC {
                            H2
                        } else {
                            H1
                        }
                    }
                    H2 => {
                        if byte == 0xDC {
                            H3
                        } else {
                            H1
                        }
                    }
                    H3 => {
                        if byte == 0xAB {
                            FrameType
                        } else {
                            H1
                        }
                    }

                    FrameType => {
                        if byte == 0xBA {
                            BeginFrame
                        } else {
                            H1
                        }
                    }

                    BeginFrame => {
                        // Length == 0 ; l'ID n'est même pas communiqué donc rejet de la trame
                        if byte == 0 {
                            H1
                        }
                        // Trop de données arrivent
                        else if byte as usize > FRAME_MAX_SIZE {
                            H1
                        } else if byte as usize <= FRAME_MAX_SIZE {
                            DataLength {
                                // DataLength représente la taille des données utiles, sans compter l'ID
                                length: byte - 1,
                            }
                        } else {
                            // normalement on n'arrive pas ici
                            //asm::bkpt();
                            H1
                        }
                    }

                    DataLength { length } => {
                        Data {
                            data_length: length,
                            id: byte,
                            data: ArrayVec::new()
                        }
                    }

                    Data {
                        data_length,
                        id,
                        ref mut data,
                    } => {
                        if data.len() < (data_length - 1) as usize {
                            data.push(byte);
                            Data {
                                data_length,
                                id,
                                data: data.clone(), //FIXME
                            }
                        } else if data.len() == (data_length - 1) as usize {
                            data.push(byte);
                            result = Some(Frame::new(id, data.clone()));
                            H1
                        } else {
                            // Rejet de la trame trop longue mais normalement on n'arrive pas ici
                            //asm::bkpt();
                            H1
                        }
                    }

                    //_ => H1,
                },
            },
            result,
        )
    }
}

#[cfg(test)]
mod test {

    use frame::*;
    use frame_reader::*;

    #[test]
    fn frame_reader_buffer() {
        let mut reader: FrameReader = FrameReader::new();
        assert_eq!(reader.pop_frame(), None);
    }

    #[test]
    fn frame_reader_standard_frame() {
        let mut reader = FrameReader::new();
        {
            let t1 = frame!(0xAA, [5, 6, 7, 8, 9, 10]);
            let bytes: ArrayVec<[u8; 256]> = t1.clone().into();
            reader.parse(&bytes);
            assert_eq!(reader.pop_frame().expect("I should have read a frame"), t1);

            /*let mut arr = trame_to_u8_with_pnum(t1, t1.pnum.unwrap());
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
            assert_eq!(reader.get_buffer_size(), 0);*/
        }
    }

    /*
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
    }*/
}
