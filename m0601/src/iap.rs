//! In-Application Programming (IAP) firmware flashing over RS485.
//!
//! Compatible with the official bootloader of the M0601 / DDT M0601C-111 motor
//! (Linko LKS32MC08x / LKS32MC05x MCU) and vendor tools (`untitled.exe`).
//!
//! # Protocol Lifecycle
//!
//! 1. **Beacon detection**: Motor emits `AA 55 FF` upon power-on / reset.
//!    (Or host sends `AA 55 BF CF` to reboot a running motor).
//! 2. **Handshake**: Host responds with `AA 55 64 C8` within ~1–2 seconds.
//! 3. **Handshake ACK**: Motor replies with `AA 55 AA`.
//! 4. **Chunk Flashing**: Firmware binary is sent in 2048-byte chunks:
//!    `[0xAA, 0x55, chunk_data..., crc8_maxim(chunk_data)]`.
//! 5. **Chunk ACK**: Motor flashes chunk into on-chip flash and replies `AA 55 AA`.
//! 6. **Reboot**: Once all chunks are flashed, motor reboots into new firmware.

use std::time::{Duration, Instant};

use crate::error::{Error, Result};
use crate::protocol::{
    IAP_ACK, IAP_BEACON, IAP_CHUNK_SIZE, IAP_HANDSHAKE, frame_enter_iap, frame_iap_chunk,
};
use crate::transport::Transport;

/// Progress statistics reported during firmware flashing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IapProgress {
    /// 1-based index of current chunk being flashed.
    pub chunk_index: usize,
    /// Total number of chunks in the firmware binary.
    pub total_chunks: usize,
    /// Total bytes sent so far.
    pub bytes_sent: usize,
    /// Total size of the firmware binary in bytes.
    pub total_bytes: usize,
}

impl IapProgress {
    /// Percentage completed (0.0 ..= 100.0).
    pub fn percent(&self) -> f32 {
        if self.total_bytes == 0 {
            100.0
        } else {
            ((self.bytes_sent as f64 / self.total_bytes as f64) * 100.0).min(100.0) as f32
        }
    }
}

/// Options configuring an IAP firmware update run.
#[derive(Debug, Clone)]
pub struct IapOptions {
    /// Chunk size in bytes (defaults to 2048).
    pub chunk_size: usize,
    /// Maximum time to wait for bootloader beacon / handshake ACK (defaults to 10s).
    pub beacon_timeout: Duration,
    /// Timeout per chunk write ACK (defaults to 3s).
    pub chunk_timeout: Duration,
    /// Maximum retries per chunk on timeout or transmission error (defaults to 3).
    pub max_retries: usize,
    /// Whether to send software reset trigger (`AA 55 BF CF`) to enter bootloader.
    pub trigger_reboot: bool,
}

impl Default for IapOptions {
    fn default() -> Self {
        Self {
            chunk_size: IAP_CHUNK_SIZE,
            beacon_timeout: Duration::from_secs(10),
            chunk_timeout: Duration::from_millis(3000),
            max_retries: 3,
            trigger_reboot: false,
        }
    }
}

/// RS485 IAP Firmware Flasher.
pub struct IapFlashing<T: Transport> {
    transport: T,
    options: IapOptions,
}

impl<T: Transport> IapFlashing<T> {
    /// Create a new IAP flasher over an open transport.
    pub fn new(transport: T, options: IapOptions) -> Self {
        Self { transport, options }
    }

    /// Access the underlying transport.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume flasher and return underlying transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    /// Flash firmware binary with progress reporting.
    pub fn flash(
        &mut self,
        firmware_bytes: &[u8],
        mut on_progress: impl FnMut(IapProgress),
    ) -> Result<()> {
        if firmware_bytes.is_empty() {
            return Err(Error::Iap("firmware binary is empty".to_string()));
        }

        // 1. Enter Bootloader / Wait for Beacon & Complete Handshake
        self.establish_bootloader_connection()?;

        // 2. Transmit Chunks
        let chunk_size = if self.options.chunk_size == 0 {
            IAP_CHUNK_SIZE
        } else {
            self.options.chunk_size
        };

        let total_bytes = firmware_bytes.len();
        let total_chunks = (total_bytes + chunk_size - 1) / chunk_size;

        for (i, chunk_data) in firmware_bytes.chunks(chunk_size).enumerate() {
            let chunk_index = i + 1;
            let packet = frame_iap_chunk(chunk_data);

            let mut flashed = false;
            let mut last_err = None;

            for retry in 0..=self.options.max_retries {
                if retry > 0 {
                    std::thread::sleep(Duration::from_millis(150));
                }

                // Send chunk packet: [0xAA, 0x55, chunk_data..., crc]
                self.transport.send(&packet)?;

                // Wait for ACK AA 55 AA
                match self.wait_for_ack(self.options.chunk_timeout) {
                    Ok(true) => {
                        flashed = true;
                        break;
                    }
                    Ok(false) => {
                        last_err = Some(format!(
                            "chunk {chunk_index}/{total_chunks} timed out waiting for ACK (retry {retry})"
                        ));
                    }
                    Err(e) => {
                        last_err = Some(format!(
                            "chunk {chunk_index}/{total_chunks} I/O error: {e}"
                        ));
                    }
                }
            }

            if !flashed {
                let msg = last_err.unwrap_or_else(|| {
                    format!("chunk {chunk_index}/{total_chunks} failed after retries")
                });
                return Err(Error::Iap(msg));
            }

            let bytes_sent = (chunk_index * chunk_size).min(total_bytes);
            on_progress(IapProgress {
                chunk_index,
                total_chunks,
                bytes_sent,
                total_bytes,
            });
        }

        Ok(())
    }

    /// Wait for motor beacon (AA 55 FF) and establish handshake (AA 55 64 C8 -> AA 55 AA).
    fn establish_bootloader_connection(&mut self) -> Result<()> {
        let deadline = Instant::now() + self.options.beacon_timeout;
        let mut last_trigger = Instant::now() - Duration::from_secs(1);
        let mut rx_buf = Vec::new();

        while Instant::now() < deadline {
            if self.options.trigger_reboot && last_trigger.elapsed() >= Duration::from_millis(500) {
                let trigger = frame_enter_iap();
                let _ = self.transport.send(&trigger);
                last_trigger = Instant::now();
            }

            let chunk = self.transport.recv(Duration::from_millis(30))?;
            if !chunk.is_empty() {
                rx_buf.extend_from_slice(&chunk);

                // Check for beacon AA 55 FF
                if contains_subslice(&rx_buf, &IAP_BEACON) {
                    rx_buf.clear();
                    // Send handshake AA 55 64 C8
                    self.transport.send(&IAP_HANDSHAKE)?;
                    // Wait for ACK AA 55 AA
                    if self.wait_for_ack(Duration::from_millis(2000))? {
                        return Ok(());
                    }
                } else if contains_subslice(&rx_buf, &IAP_ACK) {
                    // Motor is already waiting in ACK state
                    return Ok(());
                }

                // Bound buffer size
                if rx_buf.len() > 1024 {
                    let drain_len = rx_buf.len() - 256;
                    rx_buf.drain(..drain_len);
                }
            }
        }

        Err(Error::Iap(
            "timed out waiting for bootloader beacon (AA 55 FF) or handshake ACK (AA 55 AA). Please power-cycle motor.".to_string(),
        ))
    }

    /// Wait until pattern AA 55 AA arrives or timeout.
    fn wait_for_ack(&mut self, timeout: Duration) -> Result<bool> {
        let deadline = Instant::now() + timeout;
        let mut rx_buf = Vec::new();

        while Instant::now() < deadline {
            let chunk = self.transport.recv(Duration::from_millis(20))?;
            if !chunk.is_empty() {
                rx_buf.extend_from_slice(&chunk);
                if contains_subslice(&rx_buf, &IAP_ACK) {
                    return Ok(true);
                }
                if rx_buf.len() > 512 {
                    let drain_len = rx_buf.len() - 128;
                    rx_buf.drain(..drain_len);
                }
            }
        }

        Ok(false)
    }
}

/// Search for needle inside haystack.
pub fn contains_subslice(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.is_empty() {
        return true;
    }
    haystack.windows(needle.len()).any(|w| w == needle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MockTransport;

    #[test]
    fn iap_flashing_success_flow() {
        // Script replies:
        // 1. Beacon from motor: AA 55 FF
        // 2. Handshake ACK from motor: AA 55 AA
        // 3. Chunk 1 ACK from motor: AA 55 AA
        // 4. Chunk 2 ACK from motor: AA 55 AA
        let mock = MockTransport::with_replies(vec![
            vec![0xAA, 0x55, 0xFF],
            vec![0xAA, 0x55, 0xAA],
            vec![0xAA, 0x55, 0xAA],
            vec![0xAA, 0x55, 0xAA],
        ]);

        let options = IapOptions {
            chunk_size: 16,
            beacon_timeout: Duration::from_millis(500),
            chunk_timeout: Duration::from_millis(500),
            max_retries: 1,
            trigger_reboot: false,
        };

        let mut flasher = IapFlashing::new(mock, options);
        let firmware = vec![0x42u8; 30]; // 2 chunks: 16 bytes + 14 bytes

        let mut progress_reports = Vec::new();
        let res = flasher.flash(&firmware, |p| {
            progress_reports.push(p);
        });

        assert!(res.is_ok(), "flashing should succeed: {res:?}");
        assert_eq!(progress_reports.len(), 2);
        assert_eq!(progress_reports[0].chunk_index, 1);
        assert_eq!(progress_reports[0].total_chunks, 2);
        assert_eq!(progress_reports[1].chunk_index, 2);
        assert_eq!(progress_reports[1].bytes_sent, 30);
    }

    #[test]
    fn iap_empty_firmware_fails() {
        let mock = MockTransport::default();
        let mut flasher = IapFlashing::new(mock, IapOptions::default());
        assert!(flasher.flash(&[], |_| {}).is_err());
    }

    #[test]
    fn iap_subslice_matching() {
        assert!(contains_subslice(&[1, 2, 3, 4], &[2, 3]));
        assert!(!contains_subslice(&[1, 2, 3, 4], &[2, 4]));
        assert!(contains_subslice(&[0xAA, 0x55, 0xFF], &IAP_BEACON));
        assert!(contains_subslice(&[0xAA, 0x55, 0xAA], &IAP_ACK));
    }
}
