use std::process::ExitCode;
use std::time::Duration;

use m0601::protocol::{BAUD, CUR_MAX, CUR_MIN, POS_MAX, RPM_MAX, RPM_MIN};
use m0601::{FirmwareProfile, M0601};

pub fn run(
    port: &str,
    id: u8,
    timeout: Duration,
    profile: FirmwareProfile,
) -> m0601::Result<ExitCode> {
    let mut motor = M0601::open(port, id, timeout)?.with_profile(profile);

    let bar = "=".repeat(48);
    println!("{bar}");
    println!("  M0601 Configuration");
    println!("{bar}");
    println!("  Port          : {port}");
    println!("  Baud / format : {BAUD} 8N1 (RS485 half-duplex)");
    println!("  Motor ID      : 0x{id:02X} ({id})");
    println!("  Profile       : {profile}");
    println!("  Velocity range: {RPM_MIN}..{RPM_MAX} RPM");
    println!("  Current range : {CUR_MIN}..{CUR_MAX} (~-8..+8 A)");
    println!("  Position range: 0..{POS_MAX} (0..360 deg)");
    println!("{}", "-".repeat(48));

    let Some(fb) = motor.query()? else {
        // Diagnostics go to stderr so `m0601 info > log.txt` captures only the
        // real readout, not this failure notice.
        eprintln!("  Live readout  : no valid response.");
        eprintln!("  Check 18V power, wiring (brown->GND), A/B polarity, and --id.");
        return Ok(ExitCode::FAILURE);
    };

    println!("  Mode          : {}", fb.mode_name());
    println!("  Speed         : {:+} RPM", fb.speed_rpm);
    println!("  Current       : {:+.3} A", fb.current_a);
    println!("  Position      : {:.1} deg", fb.position_deg);
    if fb.profile == FirmwareProfile::NoTemperature {
        println!("  Winding temp  : uncalibrated (no_temperature profile)");
    } else {
        match fb.temp_c {
            Some(t) => println!("  Winding temp  : {t} C"),
            None => println!("  Winding temp  : --"),
        }
    }
    let eff = fb.effective_faults();
    let status = if eff.is_ok() {
        "OK".to_owned()
    } else {
        format!("FAULT ({eff})")
    };
    println!("  Error         : 0x{:02X}  {status}", eff.0);
    println!("  Raw frame     : {}", fb.raw_hex());
    println!("{bar}");
    Ok(ExitCode::SUCCESS)
}
