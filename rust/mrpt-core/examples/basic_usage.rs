//! Examples demonstrating the MRPT Rust core library

use mrpt_core::clock::{Clock, ClockSource};
use mrpt_core::exceptions::MrptResult;
use mrpt_core::mrpt_assert;
use mrpt_core::format::format_float;
use mrpt_core::bits::{ReverseBytesExt, PopCount};

fn main() {
    println!("MRPT Rust Core Library Examples\n");
    
    clock_example();
    exceptions_example();
    format_example();
    bits_example();
}

fn clock_example() {
    println!("=== Clock Example ===");
    
    // Get current time
    let now = Clock::now();
    println!("Current time (raw): {}", now);
    
    // Get as double
    let now_double = Clock::now_double();
    println!("Current time (double): {:.6}", now_double);
    
    // Convert between formats
    let timestamp = Clock::from_double(1234567.890);
    println!("From double 1234567.890: {}", timestamp);
    let back = Clock::to_double(timestamp);
    println!("Back to double: {:.6}", back);
    
    // Use simulated time
    println!("\nSwitching to simulated time...");
    Clock::set_simulated_time(100_000_000);
    Clock::set_active_clock(ClockSource::Simulated);
    let sim_time = Clock::now();
    println!("Simulated time: {}", sim_time);
    
    // Restore realtime
    Clock::set_active_clock(ClockSource::Realtime);
    println!("Restored to realtime\n");
}

fn exceptions_example() {
    println!("=== Exceptions Example ===");
    
    // Successful call
    match validate_positive(42) {
        Ok(_) => println!("validate_positive(42): OK"),
        Err(e) => println!("Error: {}", e),
    }
    
    // Failed call
    match validate_positive(-5) {
        Ok(_) => println!("validate_positive(-5): OK"),
        Err(e) => println!("Error: {}", e),
    }
    
    // Assertion example
    match check_range(50) {
        Ok(_) => println!("check_range(50): OK"),
        Err(e) => println!("Error: {}", e),
    }
    
    match check_range(150) {
        Ok(_) => println!("check_range(150): OK"),
        Err(e) => println!("Error: {}", e),
    }
    
    println!();
}

fn validate_positive(value: i32) -> MrptResult<()> {
    mrpt_assert!(value > 0, "Value must be positive");
    Ok(())
}

fn check_range(value: i32) -> MrptResult<()> {
    mrpt_assert!(value >= 0 && value <= 100, "Value must be in range [0, 100]");
    Ok(())
}

fn format_example() {
    println!("=== Format Example ===");
    
    let pi = std::f64::consts::PI;
    println!("Pi with 2 decimals: {}", format_float(pi, 2));
    println!("Pi with 6 decimals: {}", format_float(pi, 6));
    
    let values = vec![1.1, 2.2, 3.3, 4.4];
    let formatted = mrpt_core::format::format_vec(&values, " | ");
    println!("Vector formatted: {}", formatted);
    
    println!();
}

fn bits_example() {
    println!("=== Bits Example ===");
    
    // Byte reversal
    let value = 0x12345678u32;
    println!("Original: 0x{:08X}", value);
    println!("Reversed: 0x{:08X}", value.reverse_bytes());
    
    // Population count
    let bits = 0b11010110u8;
    println!("\nBits: {:08b}", bits);
    println!("Number of 1s: {}", bits.pop_count());
    
    // Power of 2
    for n in [0, 1, 5, 16, 17] {
        let rounded = mrpt_core::bits::round_up_to_power_of_2(n);
        let is_pow2 = mrpt_core::bits::is_power_of_2(n);
        println!("{}: is_power_of_2={}, round_up={}", n, is_pow2, rounded);
    }
    
    println!();
}
