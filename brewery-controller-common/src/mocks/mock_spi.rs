use std::collections::VecDeque;
use embedded_hal::spi::{SpiBus, ErrorType, Error, ErrorKind, SpiBusFlush, SpiBusRead, SpiBusWrite};

struct ReadWrite {
    bytes_in: Vec<u8>,
    bytes_out: Vec<u8>,
    ret: Result<(), MockError>,
}

/// A mock of the embedded_hal::i2c::I2c trait for testing purposes.
pub struct MockSpi {
    transfers_in_place: VecDeque<ReadWrite>,
}

impl MockSpi {
    pub fn new() -> Self {
        Self {
            transfers_in_place: VecDeque::new(),
        }
    }

    pub fn expect_transfer_in_place(&mut self, bytes_in: &[u8], bytes_out: &[u8]) {
        assert_eq!(bytes_in.len(), bytes_out.len());

        self.transfers_in_place.push_back(ReadWrite {
            bytes_in: bytes_in.to_vec(),
            bytes_out: bytes_out.to_vec(),
            ret: Ok(())
        });
    }

    pub fn expect_transfer_in_place_err(&mut self, err: MockError) {
        self.transfers_in_place.push_back(ReadWrite {
            bytes_in: vec!(),
            bytes_out: vec!(),
            ret: Err(err)
        });
    }

    pub fn verify(&self) {
        assert_eq!(self.transfers_in_place.len(), 0, "less transfers were performed than expected");
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum MockError {
    Bad,
}

impl Error for MockError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for MockSpi {
    type Error = MockError;
}

impl SpiBusFlush for MockSpi {
    fn flush(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

impl SpiBusRead for MockSpi {
    fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

impl SpiBusWrite for MockSpi {
    fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

impl SpiBus for MockSpi {
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if let Some(transfer) = self.transfers_in_place.pop_front() {
            
            if transfer.ret.is_ok() {
                assert_eq!(words.len(), transfer.bytes_in.len());
                
                for i in 0..words.len() {
                    words[i] = transfer.bytes_out[i];
                }
            }
            
            transfer.ret
        } else {
            panic!("Unexpected transfer_in_place({:x?})", words);
        }
    }

    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }
}

mod tests {
    use super::*;

    #[test]
    fn test_transfer_in_place() {
        let mut spi = MockSpi::new();
        let bytes_in = [1, 2, 0, 0];
        let bytes_out = [0, 0, 3, 4];
        let mut buffer = [1, 2, 0, 0];

        spi.expect_transfer_in_place(&bytes_in, &bytes_out);
        spi.transfer_in_place(&mut buffer).unwrap();
        
        assert_eq!(buffer, bytes_out);
        spi.verify();
    }

    #[test]
    fn test_read_err() {
        let mut spi = MockSpi::new();

        spi.expect_transfer_in_place_err(MockError::Bad);
        assert!(spi.transfer_in_place(&mut [0]).is_err());
        
        spi.verify();
    }

    #[test]
    fn test_multiple_calls() {
        let mut spi = MockSpi::new();
        
        spi.expect_transfer_in_place(&[0], &[0]);
        spi.expect_transfer_in_place(&[1], &[0]);
        spi.expect_transfer_in_place(&[2], &[0]);
        spi.transfer_in_place(&mut [0]).unwrap();
        spi.transfer_in_place(&mut [1]).unwrap();
        spi.transfer_in_place(&mut [2]).unwrap();

        spi.verify();
    }

    #[test]
    #[should_panic]
    fn test_multiple_calls_fail() {
        let mut spi = MockSpi::new();
        
        spi.expect_transfer_in_place(&[0], &[0]);
        spi.expect_transfer_in_place(&[1], &[0]);
        spi.expect_transfer_in_place(&[2], &[0]);
        spi.transfer_in_place(&mut [0]).unwrap();
        spi.transfer_in_place(&mut [1]).unwrap();

        spi.verify();
    }
}