use core::convert::Infallible;

use gpt_disk_io::{
    BlockIo,
    gpt_disk_types::{BlockSize, Lba},
};

use crate::SdMmc;

impl BlockIo for SdMmc {
    type Error = Infallible;

    fn block_size(&self) -> BlockSize {
        BlockSize::BS_512
    }

    fn num_blocks(&mut self) -> Result<u64, Self::Error> {
        Ok(self.num_blocks)
    }

    fn read_blocks(&mut self, start_lba: Lba, dst: &mut [u8]) -> Result<(), Self::Error> {
        let (chunks, remainder) = dst.as_chunks_mut::<512>();
        assert_eq!(remainder.len(), 0, "dst must be a multiple of block size");

        for (i, chunk) in chunks.iter_mut().enumerate() {
            let block = start_lba.0 + i as u64;
            self.read_block(block as u32, chunk);
        }

        Ok(())
    }

    fn write_blocks(&mut self, start_lba: Lba, src: &[u8]) -> Result<(), Self::Error> {
        let (chunks, remainder) = src.as_chunks::<512>();
        assert_eq!(remainder.len(), 0, "src must be a multiple of block size");

        for (i, chunk) in chunks.iter().enumerate() {
            let block = start_lba.0 + i as u64;
            self.write_block(block as u32, chunk);
        }

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
