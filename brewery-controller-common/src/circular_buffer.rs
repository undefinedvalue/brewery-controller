/// A fixed-size buffer that overwrites the oldest value when it is full.
/// It also tracks the sum and average of the values it currently contains.
pub struct CircularBuffer<const N: usize> {
    buffer: [f32; N],
    index: usize,
    sum: f32,
}

impl<const N: usize> CircularBuffer<N> {
    pub fn new(initial_value: f32) -> Self {
        Self {
            buffer: [initial_value; N],
            index: 0,
            sum: initial_value * N as f32,
        }
    }

    pub const fn len(&self) -> usize {
        N
    }

    /// Adds the `n` to the buffer, overwriting the oldest value.
    pub fn add(&mut self, n: f32) {
        self.sum -= self.buffer[self.index];
        self.sum += n;

        self.buffer[self.index] = n;
        self.index = (self.index + 1) % N;
    }

    /// Returns the value `di` positions before the current position (`di` = 0).
    pub fn pastget(&self, di: usize) -> f32 {
        assert!(di < N, "di out of range");

        if di >= self.index {
            self.buffer[N + self.index - 1 - di]
        } else {
            self.buffer[self.index - 1 - di]
        }
    }

    /// Returns the sum of the values currently in the buffer, unless it has
    /// not been filled yet, in which case it returns None.
    pub fn sum(&self) -> f32 {
        self.sum
    }

    /// Returns the average of the values currently in the buffer, unless it has
    /// not been filled yet, in which case it returns None.
    pub fn avg(&self) -> f32 {
        self.sum() / N as f32
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let buffer = CircularBuffer::<5>::new(1.0);

        assert_eq!(buffer.sum(), 5.0);
        assert_eq!(buffer.avg(), 1.0);
    }

    #[test]
    fn test_pastget() {
        let mut buffer = CircularBuffer::<5>::new(0.0);

        for n in 0..4 {
            buffer.add(n as f32);
        }

        assert_eq!(buffer.pastget(0), 3.0);
        assert_eq!(buffer.pastget(1), 2.0);
        assert_eq!(buffer.pastget(2), 1.0);
        assert_eq!(buffer.pastget(3), 0.0);
    }

    #[test]
    fn test_pastget_wraps() {
        let mut buffer = CircularBuffer::<5>::new(0.0);

        for n in 0..7 {
            buffer.add(n as f32);
        }

        assert_eq!(buffer.pastget(0), 6.0);
        assert_eq!(buffer.pastget(1), 5.0);
        assert_eq!(buffer.pastget(2), 4.0);
        assert_eq!(buffer.pastget(3), 3.0);
        assert_eq!(buffer.pastget(4), 2.0);
    }



    #[test]
    fn test_sumavg() {
        let mut buffer = CircularBuffer::<5>::new(0.0);

        for n in 1..6 {
            buffer.add(n as f32);
        }

        assert_eq!(buffer.sum(), 15.0);
        assert_eq!(buffer.avg(), 3.0);
    }

    #[test]
    fn test_sumavg_removes_overwritten_values() {
        let mut buffer = CircularBuffer::<5>::new(0.0);

        for n in 1..11 {
            buffer.add(n as f32);
        }

        assert_eq!(buffer.sum(), 40.0);
        assert_eq!(buffer.avg(), 8.0);
    }

}