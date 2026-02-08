use std::process;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use r2r::geometry_msgs::msg::Pose;
use r2r::geometry_msgs::msg::PoseWithCovariance;
use r2r::nav_msgs::msg::Odometry;

use futures::future;
use futures::stream::StreamExt;

use r2r::{builtin_interfaces, QosProfile};
// use sentireader_rust::dvl_nucleus1000_parser::ExtendedDVLMessage;
use tokio::task;

use sentireader_rust::{
    dvl_nucleus1000_parser::{
        parse_magnetometer_data, parse_track_data, parse_ahrs_data, DataID, ExtendedDVLMessage,
    },
    sentireader,
    stim300_parser::{self, IMUMessage},
};

use coning_and_sculling::coning_and_sculling::ConingAndSculling;

extern crate nalgebra as na;
use na::{Quaternion, SMatrix, UnitQuaternion, Vector3};

const G_UNIT_SCALING: f32 = 9.80665;
// const ROT_IMU_TO_FRD: Quaternion<f32> = Quaternion::<f32>::new(1.0, 1.0, 0.0, 0.0); // 90 deg pos. rotation around x-axis
const ROT_IMU_TO_FRD: Quaternion<f32> = Quaternion::<f32>::new(0.0, 1.0, 0.0, 0.0);

pub enum SensorID {
    STIM300,
    DVL_NUCLEUS,
    DVL_A50,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "sentireader_node", "")?;

    let imu_pub = node.create_publisher::<r2r::sensor_msgs::msg::Imu>(
        "/bluerov/imu",
        QosProfile::sensor_data(),
    )?;

    let dvl_pub = node.create_publisher::<r2r::bluerov_interfaces::msg::DVL>(
        "/bluerov/dvl",
        QosProfile::sensor_data(),
    )?;
    let mag_pub = node.create_publisher::<r2r::sensor_msgs::msg::MagneticField>(
        "/bluerov/mag",
        QosProfile::sensor_data(),
    )?;

    let ahrs_pub = node.create_publisher::<r2r::bluerov_interfaces::msg::NucleusAHRS>(
      "/bluerov/ahrs",
      QosProfile::sensor_data(),
    )?;

    // let altimeter_pub = node.create_publisher::<r2r::geometry_msgs::msg::Point>("/estimates", QosProfile::default())?;

    let arc_node = Arc::new(Mutex::new(node));

    task::spawn(async move { senti_reader(imu_pub, dvl_pub, mag_pub, ahrs_pub).await });

    // let an = arc_node.clone();
    // task::spawn(async move { imu_subscriber(an, kf_imu, est_pub).await.unwrap() });

    let handle = tokio::task::spawn_blocking(move || loop {
        {
            arc_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_millis(10));
        }
        std::thread::sleep(std::time::Duration::from_millis(100))
    });

    handle.await?;

    Ok(())
}

async fn senti_reader(
    imu_pub: r2r::Publisher<r2r::sensor_msgs::msg::Imu>,
    dvl_pub: r2r::Publisher<r2r::bluerov_interfaces::msg::DVL>,
    mag_pub: r2r::Publisher<r2r::sensor_msgs::msg::MagneticField>,
    ahrs_pub: r2r::Publisher<r2r::bluerov_interfaces::msg::NucleusAHRS>,
) -> ! {
    let mut sentireader = sentireader::SentiReader::new("/dev/ttyACM1".to_string(), 115200)
        .unwrap_or_else(|e| {
            eprintln!("Failed to open Sentiboard serial port: {e}");
            process::exit(1);
        });

    let t_0 = Instant::now();
    let mut t_prev = t_0;
    let decimation_factor: u32 = 10;

    // let dt = Duration::from_secs_f64(dt);

    let mut coning_and_sculling = ConingAndSculling::new(decimation_factor, t_0);

    // let mut counter = 0;
    // let mut t_count = Instant::now();

    loop {
        let sentiboard_msg = sentireader.read_package();

        let Ok(sentiboard_msg) = sentiboard_msg else {
          continue;
        };
        let sensor_data = sentiboard_msg.sensor_data.unwrap();

        let sensor_id = get_sensor_id(sentiboard_msg.sensor_id.unwrap()).unwrap();

        // if sensor_id == SENTIBOARD_MSG_ID_NUCLEUS as u8 {
        // let dvl_msg: dvl_nucleus1000_parser::ExtendedDVLMessage;
        // let altimeter_msg: dvl_nucleus1000_parser::AltimeterMessage;

        match sensor_id {
            SensorID::DVL_NUCLEUS => {
                // let (data_id, dvl_msg, _, magnetometer_data) = sentireader_rust::dvl_nucleus1000_parser::parse_nucleus_data(&sensor_data);
                let data_id = sentireader_rust::dvl_nucleus1000_parser::get_data_id(&sensor_data);

                match data_id {
                    DataID::BottomTrackData => {
                        let dvl_msg = parse_track_data(&sensor_data, data_id);

                        let ros_dvl_msg = create_dvl_msg(&dvl_msg);
                        dvl_pub.publish(&ros_dvl_msg).unwrap();
                    }
                    DataID::MagnetometerData => {
                        let mag_data = parse_magnetometer_data(&sensor_data);
                        //println!("Magnetometer data: {}", mag_data);

                        let t_now = std::time::SystemTime::now();
                        let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

                        let header_msg = r2r::std_msgs::msg::Header {
                            stamp: builtin_interfaces::msg::Time {
                                sec: since_epoch.as_secs() as i32,
                                nanosec: since_epoch.subsec_nanos(),
                            },
                            frame_id: "".to_string(),
                        };

                        let mag_msg = r2r::sensor_msgs::msg::MagneticField {
                            header: header_msg,
                            magnetic_field: r2r::geometry_msgs::msg::Vector3 {
                                x: mag_data[0] as f64,
                                y: mag_data[1] as f64,
                                z: mag_data[2] as f64,
                            },
                            ..Default::default()
                        };

                        mag_pub.publish(&mag_msg).unwrap();
                    }
                    DataID::AHRSData => {
                      let ahrs_data = parse_ahrs_data(&sensor_data);

                      let quat = ahrs_data.orientation;
                      let (roll, pitch, yaw) = quat.
                       euler_angles();

                      let heading = ahrs_data.heading;

                      let t_now = std::time::SystemTime::now();
                      let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

                      let header_msg = r2r::std_msgs::msg::Header {
                        stamp: builtin_interfaces::msg::Time {
                            sec: since_epoch.as_secs() as i32,
                            nanosec: since_epoch.subsec_nanos(),
                        },
                        frame_id: "".to_string(),
                    };

                      let ahrs_msg = r2r::bluerov_interfaces::msg::NucleusAHRS {
                        header: header_msg,
                        orientation: r2r::geometry_msgs::msg::Quaternion {
                          w: quat.w as f64,
                          x: quat.i as f64,
                          y: quat.j as f64,
                          z: quat.k as f64,
                      },
                      heading: heading as f64,
                      };

                      ahrs_pub.publish(&ahrs_msg).unwrap();

                    }
                    _ => continue,
                }
            }
            SensorID::STIM300 => {
                let imu_msg = match stim300_parser::parse_stim300_data(&sensor_data) {
                    Ok(imu_msg) => imu_msg,
                    // Err(e) => IMUMessage {
                    //     ..Default::default()
                    // },
                    Err(e) => continue,
                };

                let mut ang_vel = imu_msg.angular_velocity.unwrap(); // in deg/s
                let mut lin_accel = imu_msg.acceleration.unwrap(); // in units of g

                ang_vel
                    .iter_mut()
                    .for_each(|x| *x *= core::f32::consts::PI / 180.0);
                lin_accel.iter_mut().for_each(|x| *x *= G_UNIT_SCALING);

                // println!("lin_accel: {:?}", lin_accel);

                let t_now = Instant::now();
                let (vel_imu, rot_vec_imu) =
                    match coning_and_sculling.update(t_now, ang_vel, lin_accel) {
                        Some((vel_imu, rot_vec_imu)) => (vel_imu, rot_vec_imu),
                        None => continue,
                    };

                let dt = (t_now - t_prev).as_secs_f32();
                t_prev = Instant::now();

                // let R_IMU_FRD = UnitQuaternion::new_normalize(ROT_IMU_TO_FRD);
                #[allow(non_snake_case)]
                let R_IMU_FRD = UnitQuaternion::from_axis_angle(
                    &Vector3::x_axis(),
                    -core::f32::consts::PI / 2.0,
                );
                // println!("rot_imu_frd: {}", R_IMU_FRD.to_rotation_matrix().matrix());

                let ang_vel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix() * rot_vec_imu;
                let lin_accel = 1.0 / dt * R_IMU_FRD.to_rotation_matrix().matrix() * vel_imu;
                // println!("vel_imu: {}", (1.0 / dt * vel_imu));

                // let ang_vel = R_IMU_FRD.to_rotation_matrix().matrix()
                //     * Vector3::new(ang_vel[0], ang_vel[1], ang_vel[2]);
                // let lin_accel = R_IMU_FRD.to_rotation_matrix().matrix()
                //     * Vector3::new(lin_accel[0], lin_accel[1], lin_accel[2]);

                let ros_imu_msg = create_imu_ros_msg(&ang_vel, &lin_accel);

                imu_pub.publish(&ros_imu_msg).unwrap();
            }
            _ => continue,
        }
    }
}

fn create_imu_ros_msg(
    ang_vel: &Vector3<f32>,
    lin_accel: &Vector3<f32>,
) -> r2r::sensor_msgs::msg::Imu {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header_msg = r2r::std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    r2r::sensor_msgs::msg::Imu {
        header: header_msg,
        angular_velocity: r2r::geometry_msgs::msg::Vector3 {
            x: ang_vel[0] as f64,
            y: ang_vel[1] as f64,
            z: ang_vel[2] as f64,
        },
        linear_acceleration: r2r::geometry_msgs::msg::Vector3 {
          x: lin_accel[0] as f64,
            y: lin_accel[1] as f64,
            z: lin_accel[2] as f64,
        },
        ..Default::default()
    }
}

fn create_dvl_msg(dvl_msg: &ExtendedDVLMessage) -> r2r::bluerov_interfaces::msg::DVL {
    let t_now = std::time::SystemTime::now();
    let since_epoch = t_now.duration_since(std::time::UNIX_EPOCH).unwrap();

    let header_msg = r2r::std_msgs::msg::Header {
        stamp: builtin_interfaces::msg::Time {
            sec: since_epoch.as_secs() as i32,
            nanosec: since_epoch.subsec_nanos(),
        },
        frame_id: "".to_string(),
    };

    r2r::bluerov_interfaces::msg::DVL {
        header: header_msg,
        vel_body: r2r::geometry_msgs::msg::Vector3 {
            x: dvl_msg.velocity[0] as f64,
            y: dvl_msg.velocity[1] as f64,
            z: dvl_msg.velocity[2] as f64,
        },
        uncertainty_vel: r2r::geometry_msgs::msg::Vector3 {
            x: dvl_msg.uncertainty_vel[0] as f64,
            y: dvl_msg.uncertainty_vel[1] as f64,
            z: dvl_msg.uncertainty_vel[2] as f64,
        },
        vel_beam1: dvl_msg.vel_beams[0] as f64,
        vel_beam2: dvl_msg.vel_beams[1] as f64,
        vel_beam3: dvl_msg.vel_beams[2] as f64,
        uncertainty_beam1: dvl_msg.uncertainty_beams[0] as f64,
        uncertainty_beam2: dvl_msg.uncertainty_beams[1] as f64,
        uncertainty_beam3: dvl_msg.uncertainty_beams[2] as f64,
        pressure: dvl_msg.pressure as f64,
        temperature: dvl_msg.temperature as f64,
        vel_valid: dvl_msg.velocity_valid,
        ..Default::default()
    }
}

const SENTIBOARD_MSG_ID_NUCLEUS: usize = 1;
const SENTIBOARD_MSG_ID_STIM: usize = 2;
fn get_sensor_id(id: u8) -> Result<SensorID, &'static str> {
    match id {
        1 => Ok(SensorID::DVL_NUCLEUS),
        2 => Ok(SensorID::STIM300),
        4 => Ok(SensorID::DVL_A50),
        _ => Err("This Sensor ID is not supported.")?,
    }
}
