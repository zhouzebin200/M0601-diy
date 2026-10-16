//! `m0601 update` — flash firmware over RS485 using the M0601 IAP bootloader.

use std::fs;
use std::io::Write;
use std::path::Path;
use std::process::ExitCode;
use std::time::Duration;

use m0601::iap::{IapFlashing, IapOptions, IapProgress};
use m0601::transport::SerialTransport;
use m0601::{Error, Result};

/// Run the firmware update workflow.
pub fn run(
    port: &str,
    timeout: Duration,
    bin_path: &Path,
    trigger: bool,
    chunk_size: usize,
) -> Result<ExitCode> {
    if !bin_path.exists() {
        return Err(Error::Iap(format!(
            "firmware file {:?} does not exist",
            bin_path.display()
        )));
    }

    let firmware_bytes = fs::read(bin_path).map_err(Error::Io)?;
    let total_bytes = firmware_bytes.len();
    if total_bytes == 0 {
        return Err(Error::Iap("firmware binary is 0 bytes".to_string()));
    }
    if total_bytes > 256 * 1024 {
        return Err(Error::Iap(format!(
            "firmware binary too large ({total_bytes} bytes; max 256 KB)"
        )));
    }

    let chunk_size = if chunk_size == 0 { 2048 } else { chunk_size };
    let total_chunks = (total_bytes + chunk_size - 1) / chunk_size;

    println!("[i] Firmware: {} ({total_bytes} bytes, {total_chunks} chunks)", bin_path.display());
    println!("[i] Serial port: {port} @ 115200 8N1");
    if trigger {
        println!("[i] Sending reboot trigger (AA 55 BF CF) to enter bootloader...");
    } else {
        println!("[i] Waiting for motor bootloader beacon (AA 55 FF)...");
        println!("    -> Please power-cycle the motor now (18V DC).");
    }

    let transport = SerialTransport::open(port, timeout)?;
    let options = IapOptions {
        chunk_size,
        beacon_timeout: Duration::from_secs(15),
        chunk_timeout: Duration::from_millis(3000),
        max_retries: 3,
        trigger_reboot: trigger,
    };

    let mut flasher = IapFlashing::new(transport, options);

    println!("[+] Connected! Starting firmware flashing...");

    flasher.flash(&firmware_bytes, |p: IapProgress| {
        let bar_width = 30;
        let filled = (p.percent() / 100.0 * bar_width as f32).round() as usize;
        let filled = filled.min(bar_width);
        let empty = bar_width.saturating_sub(filled);

        let bar: String = "=".repeat(filled) + &" ".repeat(empty);
        print!(
            "\r[{bar}] Chunk {}/{} ({:.0}%) — {}/{} bytes",
            p.chunk_index, p.total_chunks, p.percent(), p.bytes_sent, p.total_bytes
        );
        let _ = std::io::stdout().flush();
    })?;

    println!("\n[+] Firmware update completed successfully!");
    println!("[i] Power-cycle the motor to boot into the newly flashed firmware.");

    Ok(ExitCode::SUCCESS)
}
