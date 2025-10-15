use core::ptr::NonNull;

use log::{debug, info, trace};
use volatile::VolatilePtr;

use crate::{
    cmd::{Command, DataXfer},
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2},
};

fn wait_until<F>(mut f: F)
where
    F: FnMut() -> bool,
{
    // TODO: yield?
    while !f() {
        core::hint::spin_loop();
    }
}
enum CardType {  
    Sd,  
    Emmc,  
}  

/// SD/MMC driver.
pub struct SdMmc {  
    regs: VolatilePtr<'static, RegisterBlock>,  
    num_blocks: u64,  
    card_type: CardType,  // 新增字段  
}

impl SdMmc {
    const FIFO: usize = 0x200;

    /// Creates a new `SdMmc` instance from the given base address.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` is a valid pointer to the SD/MMC controller's
    /// register block and that no other code is concurrently accessing the same hardware.
    pub unsafe fn new(base: usize) -> Self {
        let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut _)) };

        let mut this = Self {
            regs,
            num_blocks: 0,
            card_type: CardType::Sd,
        };
        this.init();
        this
    }

    /// Creates a new `SdMmc` instance specifically for eMMC from the given base address.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` is a valid pointer to the SD/MMC controller's
    /// register block and that no other code is concurrently accessing the same hardware.
    pub unsafe fn new_emmc(base: usize) -> Self {
        let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut _)) };

        let mut this = Self {
            regs,
            num_blocks: 0,
            card_type: CardType::Emmc,
        };
        this.init_emmc_only();
        this
    }

    fn can_send_cmd(&self) -> bool {
        !self.regs.cmd().read().start_cmd()
    }

    fn can_send_data(&self) -> bool {
        !self.regs.status().read().data_busy()
    }

    fn has_response(&self) -> bool {
        self.regs.rintsts().read().command_done()
    }

    fn fifo_cnt(&self) -> usize {
        self.regs.status().read().fifo_count() as usize
    }

    fn set_transaction_size(&self, blk_size: u16, byte_cnt: u32) {
        self.regs.blksiz().update(|r| r.with_block_size(blk_size));
        self.regs.bytcnt().write(byte_cnt);
    }

    fn send_cmd(&self, command: Command<'_>) -> Option<[u32; 4]> {
        trace!("send_cmd {command:#x?}");

        let (cmd, arg, xfer) = command.build();
        assert_eq!(cmd.data_expected(), xfer.is_some());

        trace!("send_cmd {cmd:?} {arg:#x?}");

        wait_until(|| self.can_send_cmd());
        if cmd.data_expected() {
            wait_until(|| self.can_send_data());
        }

        self.regs.cmdarg().write(arg);
        self.regs.cmd().write(cmd);

        wait_until(|| self.can_send_cmd());
        trace!("cmd {} sent", cmd.cmd_index());

        if cmd.response_expect() {
            wait_until(|| self.has_response());
            trace!("cmd {} received response", cmd.cmd_index());
        }

        if let Some(xfer) = xfer {
            let fifo_base = unsafe { self.regs.as_raw_ptr().byte_add(Self::FIFO) }.cast::<u64>();
            let mut offset = 0;
            match xfer {
                DataXfer::Read(buf) => {
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.receive_fifo_data_request() {
                            trace!("rxdr");
                            while self.fifo_cnt() >= 2 {
                                let data = unsafe { fifo_base.byte_add(offset).read_volatile() };
                                buf[offset..offset + 8].copy_from_slice(&data.to_le_bytes());
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error()
                    });
                    trace!("received {offset} bytes");
                }
                DataXfer::Write(buf) => {
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.transmit_fifo_data_request() {
                            trace!("txdr");
                            // Hard coded FIFO depth
                            while self.fifo_cnt() < 120 && offset < buf.len() {
                                let data =
                                    u64::from_le_bytes(buf[offset..offset + 8].try_into().unwrap());
                                unsafe { fifo_base.byte_add(offset).write_volatile(data) };
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error()
                    });
                    trace!("sent {offset} bytes");
                }
            }
        }

        let resp = self.regs.resp().read();

        let rintsts = self.regs.rintsts().read();
        // clear interrupt status
        self.regs.rintsts().write(rintsts);

        if rintsts.error() {
            trace!("cmd {} error: {rintsts:?} resp: {resp:?}", cmd.cmd_index());
            return None;
        }
        Some(resp)
    }

    fn init(&mut self) {
        info!("Initializing SD/MMC driver at {:?}", self.regs);

        trace!("ctrl: {:?}", self.regs.ctrl().read());
        trace!("pwren: {:?}", self.regs.pwren().read());
        trace!("clkdiv: {:?}", self.regs.clkdiv().read());
        trace!("clksrc: {:?}", self.regs.clksrc().read());
        trace!("clkena: {:?}", self.regs.clkena().read());
        trace!("tmout: {:?}", self.regs.tmout().read());
        trace!("ctype: {:?}", self.regs.ctype().read());
        trace!("cdetect: {:?}", self.regs.cdetect().read());
        trace!("wrtprt: {:?}", self.regs.wrtprt().read());
        trace!("usrid: {:?}", self.regs.usrid().read());
        trace!("verid: {:?}", self.regs.verid().read());
        trace!("hcon: {:?}", self.regs.hcon().read());
        trace!("uhs: {:?}", self.regs.uhs().read());
        trace!("bmod: {:?}", self.regs.bmod().read());
        trace!("dbaddr: {:?}", self.regs.dbaddr().read());

        // 基本硬件初始化
        if !self.basic_hw_init() {
            info!("Basic hardware initialization failed. Aborting card detection.");
            return;
        }

        // 尝试检测卡类型
        if self.try_init_emmc() {
            self.card_type = CardType::Emmc;
            info!("eMMC card detected and initialized");
        } else if self.init_sd_card() {
            self.card_type = CardType::Sd;
            info!("SD card detected and initialized");
        } else {
            info!("No card detected or unsupported card type.");
            // 可以选择在这里进行错误处理或者将 card_type 设置为未知类型
            // 例如：self.card_type = CardType::Unknown;
        }
    }

    fn try_init_emmc(&mut self) -> bool {
        // 发送CMD1 (SEND_OP_COND) 来检测eMMC
        // eMMC会响应CMD1，而SD卡不会
        if let Some(resp) = self.send_cmd(Command::EmmcSendOpCond(0x40FF8000)) {
            let ocr = resp[0];
            if ocr & 0x8000_0000 != 0 {
                // eMMC ready
                debug!("eMMC OCR: {:#x}", ocr);
                
                // 获取CID
                let resp = self.send_cmd(Command::AllSendCid).unwrap();
                let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) };
                info!("eMMC CID: {cid:?}");

                // 设置RCA (对于eMMC，我们可以设置任意值)
                let rca = 0x0001;
                self.send_cmd(Command::SetRelativeAddr(rca << 16)).unwrap();
                debug!("eMMC RCA set to: {rca:#x}");

                // 获取CSD
                let resp = self.send_cmd(Command::SendCsd(rca << 16)).unwrap();
                let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
                debug!("eMMC CSD: {csd:?}");

                self.num_blocks = csd.num_blocks();
                info!("eMMC capacity: {:#x} blocks", self.num_blocks);

                // 选择卡
                self.send_cmd(Command::SelectCard(rca << 16)).unwrap();

                return true;
            }
        }
        false
    }

    fn init_sd_card(&mut self) {

        let resp = self.send_cmd(Command::SendIfCond(0x1aa)).unwrap();
        assert_eq!(resp[0] & 0xff, 0xaa, "unsupported version");

        loop {
            self.send_cmd(Command::AppCmd(0));
            let resp = self.send_cmd(Command::SdSendOpCond(0x41FF_8000)).unwrap();
            let ocr = resp[0];
            if ocr & 0x8000_0000 != 0 {
                info!("SD card is ready");
                if ocr & 0x4000_0000 != 0 {
                    debug!("SD card supports high capacity");
                } else {
                    debug!("SD card is standard capacity");
                }
                break;
            }

            trace!("SD card not ready, ocr: {ocr:x}");
            core::hint::spin_loop();
        }

        let resp = self.send_cmd(Command::AllSendCid).unwrap();
        let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) };
        info!("cid: {cid:?}");

        let resp = self.send_cmd(Command::SendRelativeAddr).unwrap();
        let rca = (resp[0] >> 16) & 0xffff;
        debug!("rca: {rca:#x}");

        let resp = self.send_cmd(Command::SendCsd(rca << 16)).unwrap();
        let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
        debug!("csd: {csd:?}");

        self.num_blocks = csd.num_blocks();
        info!("SD card capacity: {:#x} blocks", self.num_blocks);

        self.send_cmd(Command::SelectCard(rca << 16)).unwrap();

        self.send_cmd(Command::AppCmd(rca << 16)).unwrap();

        self.set_transaction_size(8, 8);
        let mut buf = [0u8; 512];
        self.send_cmd(Command::SendScr(&mut buf)).unwrap();

        trace!("fifo count: {}", self.fifo_cnt());
        let resp = unsafe {
            self.regs
                .as_raw_ptr()
                .byte_add(Self::FIFO)
                .cast::<u64>()
                .read_volatile()
        };
        debug!("Bus width supported: {:#x?}", (resp >> 8) & 0xf);
        trace!("fifo count: {}", self.fifo_cnt());

        trace!("ctrl: {:?}", self.regs.ctrl().read());
        let rintsts = self.regs.rintsts().read();
        trace!("rintsts: {rintsts:?}");
        self.regs.rintsts().write(rintsts); // clear interrupt status
    }

    fn init_emmc_only(&mut self) {
        info!("Initializing eMMC driver at {:?}", self.regs);

        // 基本的硬件初始化（与SD卡相同）
        if !self.basic_hw_init() {
            info!("Basic hardware initialization failed for eMMC. Aborting eMMC initialization.");
            return;
        }

        // 直接初始化eMMC
        self.init_emmc_card();
        info!("eMMC driver initialized");
    }

    fn basic_hw_init(&mut self) -> bool {
        // reset clock
        self.regs.clkena().write(ClkEna::new());
        if self.send_cmd(Command::ResetClock).is_none() {
            info!("Failed to send ResetClock command during basic hardware initialization.");
            return false;
        }

        // set clock divider to 400kHz (low)
        self.regs.clkdiv().write(ClkDiv::new().with_clk_divider0(4));

        // enable clock
        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        if self.send_cmd(Command::ResetClock).is_none() {
            info!("Failed to send ResetClock command after enabling clock.");
            return false;
        }

        trace!("clock reset");

        // set data width -> 1bit
        self.regs.ctype().write(0.into());

        // reset dma
        self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
        self.regs
            .ctrl()
            .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));

        trace!("dma reset");

        if self.send_cmd(Command::GoIdleState).is_none() {
            info!("Failed to send GoIdleState command during basic hardware initialization.");
            return false;
        }
        trace!("idle state set");
        true
    }

    fn init_emmc_card(&mut self) {
        // 发送CMD1直到eMMC准备就绪
        loop {
            if let Some(resp) = self.send_cmd(Command::EmmcSendOpCond(0x40FF8000)) {
                let ocr = resp[0];
                if ocr & 0x8000_0000 != 0 {
                    info!("eMMC is ready, OCR: {:#x}", ocr);
                    if ocr & 0x4000_0000 != 0 {
                        debug!("eMMC supports high capacity");
                    } else {
                        debug!("eMMC is standard capacity");
                    }
                    break;
                }
                trace!("eMMC not ready, ocr: {ocr:x}");
            }
            core::hint::spin_loop();
        }

        // 获取CID
        let resp = self.send_cmd(Command::AllSendCid).unwrap();
        let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) };
        info!("eMMC CID: {cid:?}");

        // 设置RCA (对于eMMC，我们可以设置任意值)
        let rca = 0x0001;
        self.send_cmd(Command::SetRelativeAddr(rca << 16)).unwrap();
        debug!("eMMC RCA set to: {rca:#x}");

        // 获取CSD
        let resp = self.send_cmd(Command::SendCsd(rca << 16)).unwrap();
        let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
        debug!("eMMC CSD: {csd:?}");

        self.num_blocks = csd.num_blocks();
        info!("eMMC capacity: {:#x} blocks", self.num_blocks);

        // 选择卡
        self.send_cmd(Command::SelectCard(rca << 16)).unwrap();
        info!("SD/MMC driver initialized");
    }   

    

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.set_transaction_size(512, 512);
        self.send_cmd(Command::ReadSingleBlock(block, buf)).unwrap();
        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) {
        self.set_transaction_size(512, 512);
        self.send_cmd(Command::WriteSingleBlock(block, buf))
            .unwrap();
        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Returns the card type (SD or eMMC).
    pub fn card_type(&self) -> &str {
        match self.card_type {
            CardType::Sd => "SD",
            CardType::Emmc => "eMMC",
        }
    }

    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        self.num_blocks
    }

    /// The size of a block in bytes.
    pub const BLOCK_SIZE: usize = 512;
}

unsafe impl Send for SdMmc {}
unsafe impl Sync for SdMmc {}
