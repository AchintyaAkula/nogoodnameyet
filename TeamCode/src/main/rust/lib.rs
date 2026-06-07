use android_logger::Config;
use log::{error, info, LevelFilter};
use jni::JNIEnv;
use jni::objects::{JClass, JByteBuffer};
use jni::sys::jint;
use std::panic;
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicI16, AtomicI32, Ordering};
use std::thread;
use std::time::{Duration, Instant};


const ATOMIC_I32_INIT: AtomicI32 = AtomicI32::new(0);
const ATOMIC_I16_INIT: AtomicI16 = AtomicI16::new(0);

static mut TIME_SUM_W: u128 = 0;
static mut LOOP_COUNT_W: u32 = 0;

static mut TIME_SUM_R: u128 = 0;
static mut LOOP_COUNT_R: u32 = 0;

static PIPELINE_THREAD_RUNNING: AtomicBool = AtomicBool::new(false);
static COMMAND_QUEUE: [AtomicI16; 20] = [ATOMIC_I16_INIT; 20];
static ENCODER_POS_READINGS: [AtomicI32; 8] = [ATOMIC_I32_INIT; 8];
static ENCODER_VEL_READINGS: [AtomicI16; 8] = [ATOMIC_I16_INIT; 8];
static DIGITAL_VALUES: AtomicU16 = AtomicU16::new(0);
static ANALOG_VALUES: [AtomicI16; 8] = [ATOMIC_I16_INIT; 8];

// Initialize all threads and setup everything
#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_org_firstinspires_ftc_teamcode_Hubs_initializeRustPipeline(
    _env: JNIEnv,
    _class: JClass,
    motor_usages: jint,
    servo_usages: jint
) {
    // Check if pipeline running, and make it to true if it isn't
    if PIPELINE_THREAD_RUNNING.swap(true, Ordering::SeqCst) { return; }

    // Add logger
    android_logger::init_once(
        Config::default()
            .with_max_level(LevelFilter::Debug)
            .with_tag("RustSDK")
    );
	
	panic::set_hook(Box::new(|err| {
		if let Some(loc) = err.location() {
			if let Some(err_msg) = err.payload().downcast_ref::<&str>() {
				error!("Panic, Message: {:?}, Location: {}", err_msg, loc)
			} else if let Some(err_msg) = err.payload().downcast_ref::<String>() {
				error!("Panic, Message: {:?}, Location: {}", err_msg, loc)
			} else {
				error!("Unknown panic at Location: {}", loc)
			}
		} else {
			if let Some(err_msg) = err.payload().downcast_ref::<&str>() {
				error!("Panic, Message: {:?}, Location: NOT FOUND", err_msg, )
			} else if let Some(err_msg) = err.payload().downcast_ref::<String>() {
				error!("Panic, Message: {:?}, Location: NOT FOUND", err_msg)
			} else {
				error!("BRO WHY TF IS THERE NO INFO FOR THE PANIC")
			}
		}
	}));
	
    // Open the serial port
    if let Ok(serial_port)
        = serialport::new("/dev/ttyS0", 460_800)
            .timeout(Duration::from_millis(15))
            .open() {
        // Clone it into two ports for dual-threading
        if let (Ok(mut read_port), Ok(mut write_port)) = (serial_port.try_clone(), serial_port.try_clone()) {

            // Thread 1 - Write: Handles all outbound commands
            thread::spawn(
                move || {
                    let mut packet: [u8; 258] = [0u8; 258];
                    let mut curr_packet_id: u8 = 0;

                    let mut active_actuator_ids: Vec<usize> = Vec::with_capacity(20);
                    let mut byte_counter: usize = 0;

                    // Handle packet template creation for motor commands
                    for m_port in 0..8 {
                        // If the motor is declared as used, then create a template
                        if motor_usages & (1 << m_port) != 0 {
                            let motor_rel_id: u8 = (m_port % 4) as u8;
                            let module_id: u8 = (
                                if m_port < 4 { 1 }
                                else { 2 }
                            ) as u8;

                            active_actuator_ids.push(m_port);

                            packet[byte_counter..=byte_counter+3].copy_from_slice(&[
                                0x44,            // D
                                0x4B,            // K
                                0x00,            // Source id: 0 since its from c-hub code
                                module_id        // Module id: 1 or 2 based on destination hub
                            ]);
                            packet[byte_counter+5..=byte_counter+7].copy_from_slice(&[
                                0x00,            // Payload size: 00
                                0x0b,            // Payload size: 11
                                0x01,            // DC Motor Module Interface
                                motor_rel_id     // Motor port ID: 1-4
                            ]);

                            byte_counter += 12;
                        }
                    }

                    // Handle packet template creation for servo commands
                    for s_port in 0..12 {
                        // If servo is declared as used, then create a template
                        if (servo_usages & (1 << s_port)) != 0 {
                            let servo_rel_id: u8 = (s_port % 6) as u8;
                            let module_id: u8 = (
                                if s_port < 6 { 1 }
                                else { 2 }
                            ) as u8;

                            active_actuator_ids.push(8 + s_port);

                            packet[byte_counter..=byte_counter+3].copy_from_slice(&[
                                0x44,           // D
                                0x4B,           // K
                                0x00,           // Source id: 0 since its from c-hub code
                                module_id       // Module id: 1 or 2 based on destination hub
                            ]);
                                                // Packet id: Wrapping Counter
                            packet[byte_counter+5..=byte_counter+8].copy_from_slice(&[
                                0x00,           // Payload size: 00
                                0x0B,           // Payload size: 11
                                0x11,           // PWM Servo Interface
                                servo_rel_id    // Servo port ID: 1-6
                            ]);
                                                // Payload data: byte 1
                                                // Payload data: byte 2
                                                // Checksum
                            byte_counter += 12; // Packet size: 12 bytes
                        }
                    }

                    for hub in 1..=2 {
                        let module_id: u8 = hub as u8;
                        packet[byte_counter..=byte_counter+3].copy_from_slice(&[
                            0x44,           // D
                            0x4B,           // K
                            0x00,           // Source id: 0 since its from c-hub code
                            module_id       // Module id: 1 or 2 based on destination hub
                        ]);
                        packet[byte_counter+5..=byte_counter+7].copy_from_slice(&[
                            0x00,           // Payload size??
                            0x01,           // Payload size??
                            0x7F,           // Bulk Data Request
                        ]);
                        byte_counter += 9;  // Packet size: 12 bytes
                    }

                    let true_packet_size: usize = byte_counter;

                    // While everything is still running and opmode not stopped
                    while PIPELINE_THREAD_RUNNING.load(Ordering::Relaxed) {
                        let start_time = Instant::now();

                        // Iterate through actuators and inject packet data
                        let mut bulk_read_start: usize = active_actuator_ids.len() * 12;
                        for i in 0..active_actuator_ids.len() {
                            let start = i * 12;
                            let actuator_id = active_actuator_ids[i];

                            let payload_val = COMMAND_QUEUE[actuator_id].load(Ordering::Relaxed);
                            let payload_bytes = payload_val.to_le_bytes();

                            let mut checksum_counter: u8 = 0;

                            curr_packet_id = curr_packet_id.wrapping_add(1);

                            packet[start+4] = curr_packet_id;    // Packet ID
                            packet[start+9] = payload_bytes[0];  // Payload byte 1
                            packet[start+10] = payload_bytes[1]; // Payload byte 2

                            for byte in &packet[start..=start+10] {
                                checksum_counter = checksum_counter.wrapping_add(*byte);
                            }
                            packet[start+11] = checksum_counter; // Checksum
                        }

                        // Iterate through hubs and inject bulk read packet data
                        for _ in 0..2 {
                            let mut checksum_counter: u8 = 0;

                            curr_packet_id = curr_packet_id.wrapping_add(1);
                            packet[bulk_read_start+4] = curr_packet_id;   // Packet ID

                            for byte in &packet[bulk_read_start..=bulk_read_start+7] {
                                checksum_counter = checksum_counter.wrapping_add(*byte);
                            }
                            packet[bulk_read_start+8] = checksum_counter; // Checksum

                            bulk_read_start += 9;
                        }

                        // Write to port and break/end opmode if error
                        if write_port.write_all(&packet[0..true_packet_size]).is_err() {
                            PIPELINE_THREAD_RUNNING.store(false, Ordering::Relaxed);
                            break;
                        }

                        thread::sleep(Duration::from_micros(1000));
                        let loop_time = start_time.elapsed().as_micros();
                        TIME_SUM_W += loop_time;
                        LOOP_COUNT_W += 1;
                        if LOOP_COUNT_W == 500 {
                            info!("Write Thread Looptime report: {} us", (TIME_SUM_W/(LOOP_COUNT_W as u128)));
                        }

                    }
                }

            );

            // Thread 2 - Read: Handles all reading (Bulk Read data)
            thread::spawn(
                move || {
                    let mut read_buffer: [u8; 1024] = [0u8; 1024];

                    // While opmode is running
                    while PIPELINE_THREAD_RUNNING.load(Ordering::Relaxed) {
                        let start_time = Instant::now();

                        // Get the data from the port
                        if let Ok(data_size) = read_port.read(&mut read_buffer) {
                            let mut pk_start = 0;

                            // While there is still data left
                            while (pk_start +4) < data_size {

                                // Check if start is "D", "K"
                                if (read_buffer[pk_start] == 0x44) && (read_buffer[pk_start+1] == 0x4B) {

                                    // Hub ID
                                    let hub_id: u8 = read_buffer[pk_start+3];

                                    // Make sure valid hub_id
                                    if (hub_id == 1) || (hub_id == 2) {

                                        // Make sure all data has been recieved
                                        if pk_start + 39 <= data_size {
                                            let offset: usize = ((hub_id - 1) * 4) as usize;

                                            // Parse and store digital values
                                            let digital_values: u8 = read_buffer[pk_start + 5] as u8;
                                            if hub_id == 1 {
                                                // Update the first half
                                                let _ = DIGITAL_VALUES.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |old| {
                                                    Some((old & 0xFF00) | (digital_values as u16))
                                                });
                                            } else if hub_id == 2 {
                                                // Update the second half
                                                let _ = DIGITAL_VALUES.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |old| {
                                                    Some((old & 0x00FF) | ((digital_values as u16) << 8))
                                                });
                                            }

                                            // Parse and store motor encoder pos data
                                            for m_port in 0..4 {
                                                let start_pos = pk_start + 6 + (m_port * 4);
                                                let encoder_value = i32::from_le_bytes([
                                                    read_buffer[start_pos],
                                                    read_buffer[start_pos+1],
                                                    read_buffer[start_pos+2],
                                                    read_buffer[start_pos+3]
                                                ]);
                                                ENCODER_POS_READINGS[m_port + offset].store(encoder_value, Ordering::Relaxed);
                                            }

                                            // Parse and store motor encoder vel data
                                            for m_port in 0..4 {
                                                let start_pos = pk_start + 22 + (m_port * 2);
                                                let encoder_value = i16::from_le_bytes([
                                                    read_buffer[start_pos],
                                                    read_buffer[start_pos+1]
                                                ]);
                                                ENCODER_VEL_READINGS[m_port + offset].store(encoder_value, Ordering::Relaxed);
                                            }

                                            // Parse and store analog data
                                            for a_port in 0..4 {
                                                let start_pos = pk_start + 30 + (a_port * 2);
                                                let analog_value = i16::from_le_bytes([
                                                    read_buffer[start_pos],
                                                    read_buffer[start_pos+1]
                                                ]);
                                                ANALOG_VALUES[a_port + offset].store(analog_value, Ordering::Relaxed);
                                            }
                                            // Advance to next packet
                                            pk_start += 39;
                                            continue;
                                        } else {
                                            break;
                                        }
                                    } else {
                                        break;
                                    }
                                }
                                pk_start += 1;
                            }
                        }
                        let loop_time = start_time.elapsed().as_micros();
                        TIME_SUM_R += loop_time;
                        LOOP_COUNT_R += 1;
                        if LOOP_COUNT_R == 500 {
                            info!("Read Thread Looptime report: {} us", (TIME_SUM_R/(LOOP_COUNT_R as u128)));
                        }
                    }
                }
            );
        }
    }
    info!("Pipeline initialization finished")
}


// Close the pipeline and end everything when the opmode is finished
#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_org_firstinspires_ftc_teamcode_Hubs_shutdownRustPipeline(
    _env: JNIEnv,
    _class: JClass
) {
    PIPELINE_THREAD_RUNNING.store(false, Ordering::SeqCst);
}

// Send all motor power and servo pos data to update the command queue
#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_org_firstinspires_ftc_teamcode_Hubs_internalUpdate(
    _env: JNIEnv,
    _class: JClass,
    c_m_0: jint, c_m_1: jint, c_m_2: jint, c_m_3: jint,
    e_m_0: jint, e_m_1: jint, e_m_2: jint, e_m_3: jint,
    c_s_0: jint, c_s_1: jint, c_s_2: jint, c_s_3: jint, c_s_4: jint, c_s_5: jint,
    e_s_0: jint, e_s_1: jint, e_s_2: jint, e_s_3: jint, e_s_4: jint, e_s_5: jint
) {
    // Manually update everything in the command queue
    COMMAND_QUEUE[0].store(c_m_0 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[1].store(c_m_1 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[2].store(c_m_2 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[3].store(c_m_3 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[4].store(e_m_0 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[5].store(e_m_1 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[6].store(e_m_2 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[7].store(e_m_3 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_0 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_1 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_2 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_3 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_4 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(c_s_5 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_0 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_1 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_2 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_3 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_4 as i16, Ordering::Relaxed);
    COMMAND_QUEUE[0].store(e_s_5 as i16, Ordering::Relaxed);
}

// Send all the data in bulk to Java
#[unsafe(no_mangle)]
pub unsafe extern "system" fn Java_org_firstinspires_ftc_teamcode_Hubs_getAllData(
    env: JNIEnv,
    _class: JClass,
    buffer: JByteBuffer
) {
    if let Ok(data_addr) = env.get_direct_buffer_address(&buffer) {
        let shared_data = std::slice::from_raw_parts_mut(data_addr as *mut i32, 25);

        // Encoder Pos
        for i in 0..8 {
            shared_data[i] = ENCODER_POS_READINGS[i].load(Ordering::Relaxed);
        }
        // Encoder Vel
        for i in 0..8 {
            shared_data[i+8] = ENCODER_VEL_READINGS[i].load(Ordering::Relaxed) as i32;
        }
        // Analog
        for i in 0..8 {
            shared_data[i+16] = ANALOG_VALUES[i].load(Ordering::Relaxed) as i32;
        }
        // Digital
        shared_data[24] = DIGITAL_VALUES.load(Ordering::Relaxed) as i32;
    }
}