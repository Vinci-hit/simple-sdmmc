use core::fmt;

use crate::regs::Cmd;

pub enum Command<'a> {
    GoIdleState,
    AllSendCid,
    SendRelativeAddr,
    SetRelativeAddr(u32), // eMMC专用：设置RCA
    SelectCard(u32),
    SendIfCond(u32),
    SendCsd(u32),
    ReadSingleBlock(u32, &'a mut [u8]),
    WriteSingleBlock(u32, &'a [u8]),
    SdSendOpCond(u32),
    EmmcSendOpCond(u32), // eMMC专用：CMD1
    SendScr(&'a mut [u8]),
    AppCmd(u32),
    /// Psuedo-command to reset the clock
    ResetClock,
}

impl fmt::Debug for Command<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Command::GoIdleState => write!(f, "GoIdleState"),
            Command::AllSendCid => write!(f, "AllSendCid"),
            Command::SendRelativeAddr => write!(f, "SendRelativeAddr"),
            Command::SetRelativeAddr(arg) => write!(f, "SetRelativeAddr({arg})"),
            Command::SelectCard(arg) => write!(f, "SelectCard({arg})"),
            Command::SendIfCond(arg) => write!(f, "SendIfCond({arg})"),
            Command::SendCsd(rca) => write!(f, "SendCsd({rca})"),
            Command::ReadSingleBlock(block, _) => write!(f, "ReadSingleBlock({block})"),
            Command::WriteSingleBlock(block, _) => write!(f, "WriteSingleBlock({block})"),
            Command::SdSendOpCond(arg) => write!(f, "SdSendOpCond({arg})"),
            Command::EmmcSendOpCond(arg) => write!(f, "EmmcSendOpCond({arg})"),
            Command::SendScr(_) => write!(f, "SendScr"),
            Command::AppCmd(arg) => write!(f, "AppCmd({arg})"),
            Command::ResetClock => write!(f, "ResetClock"),
        }
    }
}

pub enum DataXfer<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

impl<'a> Command<'a> {
    fn cmd_index(&self) -> u8 {
        match self {
            Command::GoIdleState => 0,
            Command::EmmcSendOpCond(_) => 1, // eMMC CMD1
            Command::AllSendCid => 2,
            Command::SendRelativeAddr => 3,
            Command::SetRelativeAddr(_) => 3, // eMMC CMD3 (设置RCA)
            Command::SelectCard(_) => 7,
            Command::SendIfCond(_) => 8,
            Command::SendCsd(_) => 9,
            Command::ReadSingleBlock(..) => 17,
            Command::WriteSingleBlock(..) => 24,
            Command::SdSendOpCond(_) => 41,
            Command::SendScr(_) => 51,
            Command::AppCmd(_) => 55,

            Command::ResetClock => 0, // Special case, not a real command
        }
    }

    pub(crate) fn build(self) -> (Cmd, u32, Option<DataXfer<'a>>) {
        let cmd = Cmd::default()
            .with_use_hold_reg(true)
            .with_response_expect(true)
            .with_cmd_index(self.cmd_index());
        let cmd_crc = cmd.with_check_response_crc(true);

        match self {
            Command::GoIdleState => (cmd_crc.with_send_initialization(true), 0, None),
            Command::SendRelativeAddr => (cmd_crc, 0, None),
            Command::SetRelativeAddr(arg) => (cmd_crc, arg, None), // eMMC设置RCA
            Command::SelectCard(arg) => (cmd_crc, arg, None),
            Command::SendIfCond(arg) | Command::AppCmd(arg) => (cmd_crc, arg, None),

            Command::AllSendCid => (cmd.with_response_length(true), 0, None),
            Command::SendCsd(arg) => (cmd.with_response_length(true), arg, None),
            Command::SdSendOpCond(arg) => (cmd, arg, None),
            Command::EmmcSendOpCond(arg) => (cmd, arg, None), // eMMC CMD1

            Command::ReadSingleBlock(block, buf) => (
                cmd_crc.with_data_expected(true),
                block,
                Some(DataXfer::Read(buf)),
            ),
            Command::SendScr(buf) => (
                cmd_crc.with_data_expected(true),
                0,
                Some(DataXfer::Read(buf)),
            ),
            Command::WriteSingleBlock(block, buf) => (
                cmd_crc.with_data_expected(true).with_read_write(true),
                block,
                Some(DataXfer::Write(buf)),
            ),

            Command::ResetClock => (
                Cmd::default().with_update_clock_registers_only(true),
                0,
                None,
            ),
        }
    }
}
