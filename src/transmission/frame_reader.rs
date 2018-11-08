//! Une machine à état finis permettant de lire des [Frames](struct.Frame.html) depuis un flux d'octet.

use transmission::Frame;

use arrayvec::ArrayVec;
use transmission::{Message, FRAME_MAX_SIZE};

/// La taille du buffer interne dans lesquels sont stockés les [Frame]s lues par tous les
/// [`FrameReader`].
pub const FRAME_READER_INTERNAL_BUFFER_SIZE: usize = 10;

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub(crate) enum FrameReaderState {
    H1,
    H2,
    H3,
    FrameType,
    BeginFrame,
    DataLength {
        data_length: u8,
    },
    Data {
        data_length: u8,
        id: u8,
        data: Message,
    },
}

/// Déserialise des [Frame] depuis un flux d'octet.
///
/// Les trames lues sont stockés dans un buffer de taille [`FRAME_READER_INTERNAL_BUFFER_SIZE`].
///
#[derive(Debug, Clone)]
pub struct FrameReader {
    state: FrameStateMachine,
    buffer: ArrayVec<[Frame; FRAME_READER_INTERNAL_BUFFER_SIZE]>,
}

impl Default for FrameReader {
    fn default() -> FrameReader {
        FrameReader {
            state: FrameStateMachine::new(),
            buffer: ArrayVec::new(),
        }
    }
}

/// Machine à état de la désérialisation du flux d'octets.
#[derive(Debug, Clone)]
struct FrameStateMachine {
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
    /// ```
    /// # #[macro_use]
    /// # extern crate librobot;
    /// # use librobot::transmission::*;
    /// # use librobot::transmission::*;
    /// # fn main() {
    /// let mut reader = FrameReader::new();
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
        let opt_frame = self.state.step(byte);
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
    pub fn step(&mut self, byte: u8) -> Option<Frame> {
        use transmission::FrameReaderState::*;
        let mut result = None;
        let new_state = match self.state {
                    H1 => {
                        if byte == 0xAC {
                            Some(H2)
                        } else {
                            Some(H1)
                        }
                    }
                    H2 => {
                        if byte == 0xDC {
                            Some(H3)
                        } else {
                            Some(H1)
                        }
                    }
                    H3 => {
                        if byte == 0xAB {
                            Some(FrameType)
                        } else {
                            Some(H1)
                        }
                    }

                    FrameType => {
                        if byte == 0xBA {
                            Some(BeginFrame)
                        } else {
                            Some(H1)
                        }
                    }

                    BeginFrame => {
                        // Length == 0 ; l'ID n'est même pas communiqué donc rejet de la trame
                        // byte > FRAME_MAX_SIZE : frame trop grande
                        if byte == 0 || byte as usize > FRAME_MAX_SIZE {
                            Some(H1)
                        } else {
                            Some(DataLength {
                                // DataLength représente la taille des données utiles, sans compter l'ID
                                data_length: byte - 1,
                            })
                        }
                    }

                    DataLength { data_length } => {
                        if data_length == 0 {
                            // Le message véhiculé est vide
                            result = Some(Frame::new(byte, Message::new()));
                            Some(H1)
                        } else {
                            Some(Data {
                                data_length,
                                id: byte,
                                data: Message::new()
                            })
                        }
                    }

                    Data {
                        data_length,
                        id,
                        ref mut data,
                    } => {
                        if data.len() < (data_length - 1) as usize {
                            data.push(byte);
                            None
                        } else if data.len() == (data_length - 1) as usize {
                            data.push(byte);
                            result = Some(Frame::new(id, data.clone()));
                            Some(H1)
                        } else {
                            Some(H1)
                        }
                    }

                    //_ => H1,
                };
        if let Some(new_state) = new_state {
            self.state = new_state;
        }
        result
    }
}

#[cfg(test)]
mod test {

    use transmission::*;

    #[test]
    fn frame_reader_buffer() {
        let mut reader: FrameReader = FrameReader::new();
        assert_eq!(reader.pop_frame(), None);
    }

    #[test]
    fn frame_reader_standard_frame() {
        let mut reader = FrameReader::new();
        {
            // Trame bien formée+
            let t1 = frame!(0xAA, [5, 6, 7, 8, 9, 10]);
            let bytes1: Message = t1.clone().into();
            reader.parse(&bytes1);
            assert_eq!(reader.pop_frame().expect("I should have read a frame."), t1);
            assert_eq!(reader.get_buffer_size(), 0);

            // Message véhiculé vide
            let t2 = frame!(0xDF, []);
            let bytes2: Message = t2.clone().into();
            reader.parse(&bytes2);
            assert_eq!(reader.pop_frame().unwrap(), t2);
            assert_eq!(reader.get_buffer_size(), 0);

            // Trame découpée en plusieurs morceaux
            let mut bytes3: Message = bytes1;
            // suppression de [8, 9, 10]
            bytes3.truncate(9);
            reader.parse(&bytes3);
            assert_eq!(reader.get_buffer_size(), 0);
            bytes3.clear();
            bytes3.push(8);
            bytes3.push(9);
            bytes3.push(10);
            reader.parse(&bytes3);
            assert_eq!(reader.pop_frame().expect("I should have read a frame."), t1);
            assert_eq!(reader.get_buffer_size(), 0);
        }
    }

    #[test]
    fn frame_reader_struct_size() {
        use std;
        assert!(
            std::mem::size_of::<FrameReader>() < 4096,
            format!(
                "FrameReader has a big size unsuitable for µ-controller. It is {} bytes.",
                std::mem::size_of::<FrameReader>()
            )
        );
    }
}
