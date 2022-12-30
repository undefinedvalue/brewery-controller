use std::collections::VecDeque;
use embedded_hal::i2c::{Error, ErrorKind, ErrorType, I2c, Operation};

struct ReadWrite {
    address: u8,
    bytes_in: Vec<u8>,
    bytes_out: Vec<u8>,
    ret: Result<(), MockError>,
}

/// A mock of the embedded_hal::i2c::I2c trait for testing purposes.
pub struct MockI2c {
    reads: VecDeque<ReadWrite>,
    writes: VecDeque<ReadWrite>,
    write_reads: VecDeque<ReadWrite>,
}

impl MockI2c {
    pub fn new() -> Self {
        Self {
            writes: VecDeque::new(),
            reads: VecDeque::new(),
            write_reads: VecDeque::new(),
        }
    }

    pub fn expect_read(&mut self, address: u8, bytes_out: &[u8]) {
        self.reads.push_back(ReadWrite {
            address,
            bytes_in: vec!(),
            bytes_out: bytes_out.to_vec(),
            ret: Ok(())
        });
    }

    pub fn expect_read_err(&mut self, address: u8, err: MockError) {
        self.reads.push_back(ReadWrite {
            address,
            bytes_in: vec!(),
            bytes_out: vec!(),
            ret: Err(err)
        });
    }

    pub fn expect_write(&mut self, address: u8, bytes: &[u8]) {
        self.writes.push_back(ReadWrite {
            address,
            bytes_in: bytes.to_vec(),
            bytes_out: vec!(),
            ret: Ok(())
        });
    }

    pub fn expect_write_err(&mut self, address: u8, err: MockError) {
        self.writes.push_back(ReadWrite {
            address,
            bytes_in: vec!(),
            bytes_out: vec!(),
            ret: Err(err)
        });
    }

    pub fn expect_write_read(&mut self, address: u8, bytes: &[u8], buffer: &[u8]) {
        self.write_reads.push_back(ReadWrite {
            address,
            bytes_in: bytes.to_vec(),
            bytes_out: buffer.to_vec(),
            ret: Ok(())
        });
    }

    pub fn expect_write_read_err(&mut self, address: u8, err: MockError) {
        self.write_reads.push_back(ReadWrite {
            address,
            bytes_in: vec!(),
            bytes_out: vec!(),
            ret: Err(err)
        });
    }

    pub fn verify(&self) {
        assert_eq!(self.reads.len(), 0, "less reads were performed than expected");
        assert_eq!(self.writes.len(), 0, "less reads were performed than expected");
        assert_eq!(self.write_reads.len(), 0, "less write_reads were performed than expected");
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

impl ErrorType for MockI2c {
    type Error = MockError;
}

#[allow(unused)]
impl I2c for MockI2c {
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        if let Some(read) = self.reads.pop_front() {
            assert_eq!(address, read.address, "Unexpected address");
            
            if read.ret.is_ok() {
                assert_eq!(buffer.len(), read.bytes_out.len());
                
                for i in 0..buffer.len() {
                    buffer[i] = read.bytes_out[i];
                }
            }
            
            read.ret
        } else {
            panic!("Unexpected read({:x})", address);
        }
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        if let Some(write) = self.writes.pop_front() {
            assert_eq!(address, write.address, "Unexpected address");
            
            if write.ret.is_ok() {
                assert_eq!(bytes, write.bytes_in, "Unexpected write bytes: {:x?}", bytes);
            }
            
            write.ret
        } else {
            panic!("Unexpected write({:x}, {:x?})", address, bytes);
        }
    }

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        if let Some(read) = self.write_reads.pop_front() {
            assert_eq!(address, read.address, "Unexpected address");
            
            if read.ret.is_ok() {
                assert_eq!(bytes, read.bytes_in);
                assert_eq!(buffer.len(), read.bytes_out.len());
                
                for i in 0..buffer.len() {
                    buffer[i] = read.bytes_out[i];
                }
            }
            
            read.ret
        } else {
            panic!("Unexpected write_read({:x})", address);
        }
    }

    fn write_iter<B>(&mut self, _address: u8, _bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        unimplemented!()
    }

    fn write_iter_read<B>(&mut self, address: u8, bytes: B, buffer: &mut [u8]) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        unimplemented!()
    }

    fn transaction<'a>(&mut self, _address: u8, _operations: &mut [Operation<'a>]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    fn transaction_iter<'a, O>(&mut self, _address: u8, _operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = Operation<'a>>,
    {
        unimplemented!()
    }
}

mod tests {
    use super::*;

    #[test]
    fn test_read() {
        let mut i2c = MockI2c::new();
        let address = 0x70;
        let bytes = [1, 2, 3, 4];
        let mut buffer = [0u8; 4];

        i2c.expect_read(address, &bytes);
        i2c.read(address, &mut buffer).unwrap();
        
        assert_eq!(buffer, bytes);
        i2c.verify();
    }

    #[test]
    fn test_read_err() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_read_err(address, MockError::Bad);
        assert!(i2c.read(address, &mut [0]).is_err());
        
        i2c.verify();
    }

    #[test]
    #[should_panic]
    fn test_read_fail_address() {
        let mut i2c = MockI2c::new();
        let bytes = [1, 2, 3, 4];
        let mut buffer = [0u8; 4];

        i2c.expect_read(0x70, &bytes);
        i2c.read(0x71, &mut buffer).unwrap();
    }

    #[test]
    fn test_write() {
        let mut i2c = MockI2c::new();
        let address = 0x70;
        let write_bytes = [1, 2, 3, 4];

        i2c.expect_write(address, &write_bytes);
        i2c.write(address, &write_bytes).unwrap();
        
        i2c.verify();
    }

    #[test]
    fn test_write_err() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_write_err(address, MockError::Bad);
        assert!(i2c.write(address, &[1]).is_err());
        
        i2c.verify();
    }

    #[test]
    #[should_panic]
    fn test_write_fail() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_write(address, &[1, 2, 3, 4]);
        i2c.write(address, &[1, 2, 4, 3]).unwrap();
        
        i2c.verify();
    }

    #[test]
    fn test_write_read() {
        let mut i2c = MockI2c::new();
        let address = 0x70;
        let bytes_in = [1, 2, 3, 4];
        let bytes_out = [5, 6, 7, 8];
        let mut buffer = [1, 2, 3, 4];

        i2c.expect_write_read(address, &bytes_in, &bytes_out);
        i2c.write_read(address, &bytes_in, &mut buffer).unwrap();
        
        assert_eq!(buffer, bytes_out);
        i2c.verify();
    }

    #[test]
    fn test_write_read_err() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_write_read_err(address, MockError::Bad);
        assert!(i2c.write_read(address, &[0], &mut [0]).is_err());
        
        i2c.verify();
    }

    #[test]
    #[should_panic]
    fn test_write_read_fail_address() {
        let mut i2c = MockI2c::new();
        let bytes_in = [1, 2, 3, 4];
        let bytes_out = [5, 6, 7, 8];

        i2c.expect_write_read(0x70, &bytes_in, &bytes_out);
        i2c.write_read(0x71, &bytes_in, &mut [0u8; 4]).unwrap();
    }

    #[test]
    #[should_panic]
    fn test_write_read_fail_bytes_in() {
        let mut i2c = MockI2c::new();
        let address= 0x70;
        let bytes_in = [1, 2, 3, 4];
        let bytes_out = [5, 6, 7, 8];

        i2c.expect_write_read(address, &bytes_in, &bytes_out);
        i2c.write_read(address, &[2, 2, 3, 4], &mut [0u8; 4]).unwrap();
    }

    #[test]
    fn test_multiple_calls() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_write(address, &[1]);
        i2c.expect_write(address, &[2]);
        i2c.expect_write(address, &[3]);
        i2c.write(address, &[1]).unwrap();
        i2c.write(address, &[2]).unwrap();
        i2c.write(address, &[3]).unwrap();
        
        i2c.verify();
    }

    #[test]
    #[should_panic]
    fn test_multiple_calls_fail() {
        let mut i2c = MockI2c::new();
        let address = 0x70;

        i2c.expect_write(address, &[1]);
        i2c.expect_write(address, &[2]);
        i2c.expect_write(address, &[3]);
        i2c.write(address, &[1]).unwrap();
        i2c.write(address, &[3]).unwrap();
        
        i2c.verify();
    }
}