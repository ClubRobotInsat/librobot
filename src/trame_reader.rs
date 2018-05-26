//! Une machine à état finis permettant de lire des [Trame]s depuis tous les types qui implémentent
//! `Read`.

use trame::Trame;

use embedded_hal::serial::Read;

use arrayvec::ArrayVec;

/// La taille du buffer interne dans lesquels sont stockés les [Trame]s lues par tous les
/// [TrameReader].
pub const TRAME_READER_INTERNAL_BUFFER_SIZE: usize = 2048;

#[derive(Debug)]
enum TrameReaderState {
    H1,
    H2,
    H3,
    TypeTrame,
    Id,
    Cmd {
        id: u8,
    },
    Pnum {
        id: u8,
        cmd: u8,
    },
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
pub struct TrameReader<T>
where
    T: Read<u8>,
{
    state: TrameReaderState,
    reader: T,
    buffer: ArrayVec<[Trame; TRAME_READER_INTERNAL_BUFFER_SIZE]>,
}

impl<T> TrameReader<T>
where
    T: Read<u8>,
{
    /// Crée une nouvelle machine à état s'appuyant sur `reader` pour lire des trames.
    /// La taille du buffer est fixée à la compilation, cf [TRAME_READER_INTERNAL_BUFFER_SIZE].
    pub fn new(reader: T) -> TrameReader<T> {
        TrameReader {
            state: TrameReaderState::H1,
            reader: reader,
            buffer: ArrayVec::new(),
        }
    }

    /// Renvoie la plus vieille trame non lue.
    pub fn get_trame(&mut self) -> Option<Trame> {
        self.buffer.pop()
    }

    fn read_byte(&mut self) {}

    fn parse_byte(&mut self, _byte: u8) {}
}

#[cfg(test)]
mod test {

    use embedded_hal::serial::Read;
    use nb::Result;
    use trame_reader::*;

    struct TestReader {
        buf: ArrayVec<[u8; 2048]>,
    }

    impl TestReader {
        pub fn push_bytes(&mut self, bytes: &[u8]) {
            for b in bytes {
                self.buf.push(*b);
            }
        }

        fn new() -> Self {
            TestReader {
                buf: ArrayVec::new(),
            }
        }
    }

    impl Read<u8> for TestReader {
        type Error = ();
        fn read(&mut self) -> Result<u8, ()> {
            Ok(0)
        }
    }

    #[test]
    fn trame_reader_buffer() {
        let mut reader = TrameReader::new(TestReader::new());
        assert_eq!(reader.get_trame(), None);
    }

}
