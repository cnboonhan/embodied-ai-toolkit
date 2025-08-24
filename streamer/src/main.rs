use clap::Parser;
use anyhow::Result;
use std::time::{Duration, Instant};
use tokio::time::sleep;
use rerun;
use nokhwa::pixel_format::RgbFormat;
use nokhwa::utils::{RequestedFormat, RequestedFormatType, CameraIndex, Resolution, CameraFormat, FrameFormat};
use nokhwa::Camera;
use image::{RgbImage, DynamicImage};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Camera index (e.g., 0 for first camera)
    #[arg(long, default_value_t = 0)]
    camera_index: u32,

    /// Project name for the stream
    #[arg(long)]
    project_name: String,

    /// Label for the stream
    #[arg(long)]
    label: String,

    /// Episode name for the stream. Make sure its consistent with the joint state stream.
    #[arg(long)]
    episode_name: String,

    /// Frames per second
    #[arg(long, default_value_t = 30)]
    fps: i32,

    /// Image width in pixels
    #[arg(long, default_value_t = 640)]
    width: u32,

    /// Image height in pixels
    #[arg(long, default_value_t = 480)]
    height: u32,

    #[arg(long, default_value = "rerun+http://localhost:5050/proxy")]
    rerun_endpoint: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    println!("Starting webcam stream:");
    println!("  Camera Index: {}", args.camera_index);
    println!("  Project: {}", args.project_name);
    println!("  Label: {}", args.label);
    println!("  FPS: {}", args.fps);
    println!("  Dimensions: {}x{}", args.width, args.height);
    println!("  Rerun endpoint: {}", args.rerun_endpoint);

    let rec = rerun::RecordingStreamBuilder::new(args.project_name.clone())
        .recording_id(args.episode_name.clone())
        .connect_grpc_opts(args.rerun_endpoint.clone(), None)?;

    // Initialize camera with nokhwa
    let resolution = Resolution::new(args.width, args.height);
    let camera_format = CameraFormat::new(resolution, FrameFormat::MJPEG, 30); // 30 fps default
    let requested = RequestedFormat::new::<RgbFormat>(RequestedFormatType::Exact(camera_format));
    let camera_index = CameraIndex::Index(args.camera_index);
    
    let mut camera = Camera::new(camera_index, requested)?;
    
    // Start the camera stream
    camera.open_stream()?;
    
    println!("Camera opened successfully!");
    println!("Starting stream...");
    println!("Press Ctrl+C to stop");

    let frame_interval = Duration::from_millis((1000 / args.fps) as u64);
    let mut frame_count = 0;
    let start_time = Instant::now();

    loop {
        let frame_start = Instant::now();
        
        // Capture frame using nokhwa
        match camera.frame() {
            Ok(frame) => {
                // Convert frame to RGB image
                let rgb_frame = frame.decode_image::<RgbFormat>()?;
                let (width, height) = (rgb_frame.width(), rgb_frame.height());
                
                // Create image from raw RGB data
                let img = RgbImage::from_raw(width, height, rgb_frame.into_raw())
                    .ok_or_else(|| anyhow::anyhow!("Failed to create image from frame data"))?;
                
                // Convert to dynamic image
                let dynamic_img = DynamicImage::ImageRgb8(img);
                
                // Convert to rerun format and stream
                rec.log(args.label.clone(), &rerun::Image::from_dynamic_image(dynamic_img)?)?;
                
                frame_count += 1;
                let elapsed = start_time.elapsed();
                let fps_actual = frame_count as f64 / elapsed.as_secs_f64();
                
                println!("Frame {} streamed to rerun - Actual FPS: {:.2}", frame_count, fps_actual);
            }
            Err(e) => {
                eprintln!("Failed to capture frame: {}", e);
                // Don't break, just continue trying
            }
        }

        // Wait for next frame
        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            sleep(frame_interval - elapsed).await;
        }
    }
}
